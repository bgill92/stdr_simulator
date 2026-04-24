#pragma once

/** @file Lock-free ring buffer for the sim-to-plot data path.
 *
 *  Intended for a single-producer (sim thread) and one or more independent
 *  consumers (GUI thread plotters, each with its own cursor).
 *
 *  ## Concurrency contract
 *
 *  This ring is safe when the producer and consumer do not overlap in time on
 *  any given slot.  In the intended deployment:
 *
 *  - The sim thread pushes at most once per physics tick (default 10 Hz).
 *  - The GUI thread drains at most once per rendered frame (≤ 60 Hz).
 *  - The ring capacity (default 256) guarantees the producer cannot lap a
 *    consumer that drains every 100 ms.
 *
 *  The producer may overwrite slots containing unconsumed entries when the
 *  ring is full (oldest-drop policy).  In that case `drain()` reports the
 *  overwritten count via `DrainResult::dropped`.
 *
 *  @warning The `std::span` returned by `drain()` points into the ring's
 *           internal buffer.  It is valid **only until the next `push()` call
 *           from the producer**.  Callers must copy or process the span before
 *           yielding back to the sim thread.  Do not hold the span across a
 *           frame boundary.
 *
 *  @warning NOT safe for concurrent producers.  Exactly one thread may call
 *           `push()`; multiple threads may each call `drain()` with their own
 *           cursor, provided each cursor is accessed by at most one thread. */

#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

namespace stdr::plot_data
{

/** Result of a drain() call. */
template <typename T>
struct DrainResult
{
  /** Elements produced since the last drain, in order from oldest to newest.
   *  Valid until the next push() call (the ring may overwrite the slots). */
  std::span<const T> entries;

  /** Number of entries the producer wrote but the consumer never saw because
   *  the ring wrapped.  Zero in the common (non-overrun) case. */
  std::size_t dropped{ 0 };
};

/** Lock-free ring buffer with per-consumer cursors and oldest-drop semantics.
 *
 *  Capacity is fixed at construction so different sensor types can be sized
 *  independently at runtime.
 *
 *  @tparam T Element type.  Must be DefaultConstructible (the internal buffer
 *            is allocated as `make_unique<T[]>(capacity)`) and either
 *            MoveAssignable or CopyAssignable (elements are written via
 *            assignment in push()). */
template <typename T>
class SpscRing
{
  static_assert(std::is_default_constructible_v<T>, "SpscRing requires DefaultConstructible T (uses make_unique<T[]>)");
  static_assert(std::is_move_assignable_v<T> || std::is_copy_assignable_v<T>,
                "SpscRing requires move- or copy-assignable T");

public:
  /** @param capacity Maximum number of entries the ring holds.  Must be > 1.
   *         When full the oldest entry is overwritten by the next push(). */
  explicit SpscRing(std::size_t capacity)
    : capacity_{ capacity }, buf_{ std::make_unique<T[]>(capacity) }, write_count_{ 0 }
  {
    assert(capacity > 1 && "SpscRing capacity must be > 1");
  }

  // Non-copyable, non-movable — atomics are not copyable.
  SpscRing(const SpscRing&) = delete;
  SpscRing& operator=(const SpscRing&) = delete;
  SpscRing(SpscRing&&) = delete;
  SpscRing& operator=(SpscRing&&) = delete;

  ~SpscRing() = default;

  /** Push an element.  Called by the producer (sim thread) only.
   *
   *  Stores the element in the next available slot (overwriting the oldest
   *  entry if the ring is full) and increments the write counter. */
  void push(T value)
  {
    // Relaxed: write_count_ is only written by this thread, so the load is
    // not racing with any other store.
    const std::uint64_t seq = write_count_.load(std::memory_order_relaxed);
    const std::size_t slot = static_cast<std::size_t>(seq % capacity_);

    buf_[slot] = std::move(value);
    // Release: the payload write above is visible to consumers that
    // subsequently acquire write_count_ and derive the same slot.
    write_count_.store(seq + 1, std::memory_order_release);
  }

  /** Drain all entries produced since `cursor`.
   *
   *  Called by the consumer only.  The cursor is owned by the consumer; pass
   *  the same cursor across calls to receive only new entries.
   *
   *  @param[in,out] cursor  On entry: the consumer's current read position
   *                         (sequence number of the next unread entry).
   *                         Initialise to 0 before the first call.
   *                         On exit: advanced past all returned entries and
   *                         any detected drops.
   *  @return DrainResult with a span of entries and a dropped count.
   *
   *  @note The returned span points into the ring's internal buffer and is
   *        valid only until the next push() call.  When entries wrap the
   *        physical end of the buffer, this call returns the first contiguous
   *        segment; call drain() again to retrieve the remainder. */
  [[nodiscard]] DrainResult<T> drain(std::uint64_t& cursor) const
  {
    // Acquire: ensures we see all payload writes that the producer released
    // before storing write_count_.
    const std::uint64_t head = write_count_.load(std::memory_order_acquire);

    if (head == cursor)
    {
      return DrainResult<T>{ std::span<const T>{}, 0 };
    }

    // How many entries ahead of cursor has the producer committed?
    const std::uint64_t total_written = head - cursor;

    std::size_t dropped = 0;

    // If the producer is more than capacity_ ahead, the oldest entries have
    // been overwritten.  Jump cursor to the oldest surviving entry.
    if (total_written > static_cast<std::uint64_t>(capacity_))
    {
      dropped = static_cast<std::size_t>(total_written - capacity_);
      cursor += dropped;
    }

    // Return the first contiguous physical segment starting at cursor.
    const std::size_t start_slot = static_cast<std::size_t>(cursor % capacity_);
    const std::uint64_t remaining = head - cursor;
    const std::size_t segment_len = std::min(static_cast<std::size_t>(remaining), capacity_ - start_slot);

    const std::span<const T> result{ buf_.get() + start_slot, segment_len };
    cursor += static_cast<std::uint64_t>(segment_len);
    return DrainResult<T>{ result, dropped };
  }

  /** @return Total number of entries committed by the producer so far.
   *
   *  A consumer can compare its cursor against this value to determine how
   *  far behind it is. */
  [[nodiscard]] std::uint64_t write_count() const noexcept
  {
    return write_count_.load(std::memory_order_acquire);
  }

  /** @return Maximum number of entries the ring can hold. */
  [[nodiscard]] std::size_t capacity() const noexcept
  {
    return capacity_;
  }

private:
  std::size_t capacity_;
  std::unique_ptr<T[]> buf_;

  // Monotonically increasing count of entries committed by the producer.
  // The next entry is written at slot (write_count_ % capacity_).
  alignas(64) std::atomic<std::uint64_t> write_count_;
};

/** Default capacity for sensor event logs: 256 entries ≈ 25 s at 10 Hz.
 *  Chosen to survive typical pause/resume cycles while keeping memory
 *  bounded (~360 KB per laser sensor for a 180-ray scan). */
inline constexpr std::size_t kDefaultSensorLogCapacity = 256;

}  // namespace stdr::plot_data
