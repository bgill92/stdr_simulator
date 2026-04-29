#pragma once

/** @file Best-effort lock-free single-producer / single-consumer latest-value
 *  cell.  `std::atomic<std::shared_ptr<T>>` is lock-free on platforms with
 *  16-byte CAS (e.g. x86-64); other platforms fall back to an internal mutex.
 *
 *  The producer (sim thread) atomically publishes a new `shared_ptr<const T>`;
 *  the consumer (GUI thread) swaps in the latest value without blocking.
 *  This formalises the double-buffered snapshot pattern already used in
 *  `StandaloneBackend::get_snapshot()` into a reusable typed wrapper.
 *
 *  ## Memory ordering rationale
 *  `std::atomic<std::shared_ptr<T>>` uses seq-cst by default, which is correct
 *  but slightly heavier than necessary.  We use `release` on publish and
 *  `acquire` on read to establish the required happens-before relationship:
 *  any writes the producer makes to the T object before calling publish() are
 *  visible to the consumer after exchange().
 *
 *  @warning NOT thread-safe for multiple concurrent producers.  Exactly one
 *  producer thread and one consumer thread must be used. */

#include <atomic>
#include <memory>

namespace stdr::plot_data
{

/** Best-effort lock-free latest-value cell.  `std::atomic<std::shared_ptr<T>>`
 *  is lock-free on platforms with 16-byte CAS (e.g. x86-64); other platforms
 *  fall back to an internal mutex.  Stores at most one value at a time; each
 *  publish() replaces the previous value.  Callers holding the old
 *  `shared_ptr` keep it alive after a new value is published.
 *
 *  @tparam T The value type.  Instances are always held through
 *            `shared_ptr<const T>` so the consumer gets an immutable view
 *            and ownership is reference-counted. */
template <typename T>
class LatestSnapshot
{
public:
  LatestSnapshot() = default;

  // Non-copyable, non-movable — atomics are not copyable.
  LatestSnapshot(const LatestSnapshot&) = delete;
  LatestSnapshot& operator=(const LatestSnapshot&) = delete;
  LatestSnapshot(LatestSnapshot&&) = delete;
  LatestSnapshot& operator=(LatestSnapshot&&) = delete;

  ~LatestSnapshot() = default;

  /** Publish a new snapshot.  Called by the producer (sim thread) only.
   *
   *  The old snapshot (if any) is released here unless the consumer is
   *  currently holding a reference to it.  If the consumer called read()
   *  before this publish(), it keeps its `shared_ptr` alive — the refcount
   *  drops to zero only when both the cell and the consumer release it.
   *
   *  @note Passing `nullptr` is valid and clears the cell; a subsequent
   *        read() returns `nullptr` until a non-null value is published. */
  void publish(std::shared_ptr<const T> value)
  {
    // release: the T object constructed before this call is visible to the
    // consumer after it reads with acquire.
    latest_.store(std::move(value), std::memory_order_release);
  }

  /** Read the latest snapshot.  Called by the consumer (GUI thread) only.
   *
   *  @return The most recently published value, or nullptr if no value has
   *          been published yet.  The returned `shared_ptr` keeps the object
   *          alive even if the producer calls publish() again immediately. */
  [[nodiscard]] std::shared_ptr<const T> read() const
  {
    // acquire: pairs with the release in publish(), ensuring the T payload
    // is visible.
    return latest_.load(std::memory_order_acquire);
  }

private:
  // std::atomic<std::shared_ptr<T>> is lock-free on platforms that support it
  // (x86-64 with 16-byte CAS, or when the implementation uses an internal
  // lock).  The interface contract is still correct in either case.
  std::atomic<std::shared_ptr<const T>> latest_;
};

}  // namespace stdr::plot_data
