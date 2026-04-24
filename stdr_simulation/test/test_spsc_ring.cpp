#include <stdr_simulation/plot_data/spsc_ring.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>
#include <vector>

namespace stdr::plot_data
{
namespace
{

using ::testing::Eq;
using ::testing::SizeIs;

// Helper: drain all currently available entries from `ring` into `out`,
// accumulating the total drop count.  Handles the segment-at-a-time API
// automatically.  Returns the number of drops detected this call.
template <typename T>
std::size_t drain_all(const SpscRing<T>& ring, std::uint64_t& cursor, std::vector<T>& out)
{
  std::size_t total_dropped = 0;
  while (true)
  {
    const DrainResult<T> result = ring.drain(cursor);
    if (result.entries.empty() && result.dropped == 0)
    {
      break;
    }
    total_dropped += result.dropped;
    for (const T& v : result.entries)
    {
      out.push_back(v);
    }
  }
  return total_dropped;
}

// ---------------------------------------------------------------------------
// Basic enqueue / drain
// ---------------------------------------------------------------------------

TEST(SpscRingTest, EmptyRingDrainReturnsEmptySpanAndZeroDrops)
{
  const SpscRing<int> ring{ 8 };
  std::uint64_t cursor = 0;
  const DrainResult<int> result = ring.drain(cursor);
  EXPECT_THAT(result.entries, SizeIs(0));
  EXPECT_THAT(result.dropped, Eq(std::size_t{ 0 }));
}

TEST(SpscRingTest, SinglePushDrainReturnsOneElement)
{
  SpscRing<int> ring{ 8 };
  ring.push(42);

  std::uint64_t cursor = 0;
  const DrainResult<int> result = ring.drain(cursor);
  ASSERT_THAT(result.entries, SizeIs(1));
  EXPECT_THAT(result.entries[0], Eq(42));
  EXPECT_THAT(result.dropped, Eq(std::size_t{ 0 }));
}

TEST(SpscRingTest, MultiplePushesDrainedInOrder)
{
  SpscRing<int> ring{ 8 };
  ring.push(1);
  ring.push(2);
  ring.push(3);

  std::uint64_t cursor = 0;
  std::vector<int> collected;
  const std::size_t dropped = drain_all(ring, cursor, collected);

  EXPECT_THAT(dropped, Eq(std::size_t{ 0 }));
  ASSERT_THAT(collected, SizeIs(3));
  EXPECT_THAT(collected[0], Eq(1));
  EXPECT_THAT(collected[1], Eq(2));
  EXPECT_THAT(collected[2], Eq(3));
}

// ---------------------------------------------------------------------------
// Cursor advance — second drain returns only new elements
// ---------------------------------------------------------------------------

TEST(SpscRingTest, SecondDrainReturnsOnlyNewEntries)
{
  SpscRing<int> ring{ 8 };
  ring.push(10);
  ring.push(20);

  std::uint64_t cursor = 0;
  std::vector<int> ignored;
  drain_all(ring, cursor, ignored);

  // Push one more and verify only the new one comes back.
  ring.push(30);
  std::vector<int> collected;
  const std::size_t dropped = drain_all(ring, cursor, collected);

  EXPECT_THAT(dropped, Eq(std::size_t{ 0 }));
  ASSERT_THAT(collected, SizeIs(1));
  EXPECT_THAT(collected[0], Eq(30));
}

// ---------------------------------------------------------------------------
// Wraparound (oldest-drop policy)
// ---------------------------------------------------------------------------

TEST(SpscRingTest, WraparoundOldestDropPolicyReportsDropCount)
{
  // Capacity 4.  Push 5 items without consuming — the oldest must be dropped.
  SpscRing<int> ring{ 4 };
  ring.push(1);
  ring.push(2);
  ring.push(3);
  ring.push(4);
  ring.push(5);  // Wraps: 1 is overwritten.

  std::uint64_t cursor = 0;
  std::vector<int> collected;
  const std::size_t total_dropped = drain_all(ring, cursor, collected);

  EXPECT_THAT(total_dropped, Eq(std::size_t{ 1 }));
  // The surviving elements are 2, 3, 4, 5 (order preserved).
  ASSERT_THAT(collected, SizeIs(4));
  EXPECT_THAT(collected[0], Eq(2));
  EXPECT_THAT(collected[3], Eq(5));
}

TEST(SpscRingTest, ProducerLapsConsumerMultipleTimesDropCountAccumulates)
{
  // Capacity 4.  Push 9 items without consuming — 5 drops total:
  // entries 1–5 are overwritten, leaving 6–9 (4 entries survive).
  SpscRing<int> ring{ 4 };
  for (int i = 1; i <= 9; ++i)
  {
    ring.push(i);
  }

  std::uint64_t cursor = 0;
  std::vector<int> collected;
  const std::size_t total_dropped = drain_all(ring, cursor, collected);

  // Total items = total_dropped + total_seen (invariant).
  EXPECT_THAT(total_dropped + collected.size(), Eq(std::size_t{ 9 }));
  EXPECT_THAT(total_dropped, Eq(std::size_t{ 5 }));
  ASSERT_THAT(collected, SizeIs(4));
}

// ---------------------------------------------------------------------------
// Multi-drain consistency
// ---------------------------------------------------------------------------

TEST(SpscRingTest, InterleavedPushAndDrainDeliverAllElements)
{
  SpscRing<int> ring{ 8 };
  std::uint64_t cursor = 0;
  std::vector<int> collected;

  // Three rounds of push-then-drain, verifying no duplication or loss.
  for (int round = 0; round < 3; ++round)
  {
    ring.push(round * 2);
    ring.push(round * 2 + 1);
    drain_all(ring, cursor, collected);
  }

  ASSERT_THAT(collected, SizeIs(6));
  for (int i = 0; i < 6; ++i)
  {
    EXPECT_THAT(collected[static_cast<std::size_t>(i)], Eq(i));
  }
}

// ---------------------------------------------------------------------------
// Independent cursors — two consumers see independent views of the same ring
// ---------------------------------------------------------------------------

TEST(SpscRingTest, TwoIndependentCursorsGetIndependentViews)
{
  SpscRing<int> ring{ 8 };
  ring.push(1);
  ring.push(2);

  std::uint64_t cursor_a = 0;
  std::uint64_t cursor_b = 0;

  // Consumer A drains all.
  std::vector<int> a_entries;
  drain_all(ring, cursor_a, a_entries);

  // Consumer B drains all (gets the same entries independently).
  std::vector<int> b_entries;
  drain_all(ring, cursor_b, b_entries);

  ASSERT_THAT(a_entries, SizeIs(2));
  ASSERT_THAT(b_entries, SizeIs(2));
  EXPECT_THAT(a_entries[0], Eq(1));
  EXPECT_THAT(b_entries[0], Eq(1));

  // Push one more — A and B each get only the new entry.
  ring.push(3);
  drain_all(ring, cursor_a, a_entries);
  drain_all(ring, cursor_b, b_entries);

  ASSERT_THAT(a_entries, SizeIs(3));
  ASSERT_THAT(b_entries, SizeIs(3));
  EXPECT_THAT(a_entries[2], Eq(3));
  EXPECT_THAT(b_entries[2], Eq(3));
}

// ---------------------------------------------------------------------------
// Stress test: producer and consumer on separate threads.
//
// Verifies liveness (the ring terminates without deadlock) and that the
// cursor accounting is complete: after the producer finishes and the consumer
// drains all remaining entries, the cursor equals write_count().
//
// Value correctness under concurrent writes is NOT asserted here because
// the API contract (spans are valid only until the next push()) permits
// the producer to overwrite slots that the consumer has not yet read.  The
// single-threaded tests above prove value correctness in the intended usage
// model (push happens-before drain within the same "tick–frame" cycle).
// ---------------------------------------------------------------------------

TEST(SpscRingTest, StressProducerConsumerLivenessAndCursorAccountingIsCorrect)
{
  constexpr std::size_t kCapacity = 64;
  constexpr std::size_t kTotalItems = 10'000;

  SpscRing<std::uint64_t> ring{ kCapacity };

  std::atomic<bool> producer_done{ false };
  std::uint64_t cursor = 0;

  // Producer runs on a jthread; consumer runs on this thread.
  {
    std::jthread producer{ [&ring, &producer_done]() {
      for (std::size_t i = 0; i < kTotalItems; ++i)
      {
        ring.push(static_cast<std::uint64_t>(i));
      }
      producer_done.store(true, std::memory_order_release);
    } };

    // Consumer drains while the producer is running.  Yield to avoid
    // burning 100% CPU on a time-sharing OS while the producer is slow.
    while (!producer_done.load(std::memory_order_acquire))
    {
      const DrainResult<std::uint64_t> result = ring.drain(cursor);
      if (result.entries.empty() && result.dropped == 0)
      {
        std::this_thread::yield();
      }
    }
  }  // jthread joins here — producer is guaranteed done.

  // Final drain: collect everything the producer wrote before signalling done.
  std::vector<std::uint64_t> unused;
  drain_all(ring, cursor, unused);

  // The cursor must now equal write_count() — every committed entry is
  // accounted for (seen or dropped).
  EXPECT_THAT(cursor, Eq(ring.write_count()));
  EXPECT_THAT(ring.write_count(), Eq(static_cast<std::uint64_t>(kTotalItems)));
}

// ---------------------------------------------------------------------------
// Concurrent value correctness — ring sized to kTotalItems so the producer
// never laps the consumer.  Verifies that acquire/release ordering actually
// delivers correct payload values across threads (not just liveness).
// ---------------------------------------------------------------------------

TEST(SpscRingTest, StressProducerConsumerNoLapValuesAreCorrect)
{
  // Ring capacity == item count so the producer can never overwrite an entry
  // the consumer has not yet read.  Under this invariant we can assert exact
  // values without any dropped entries.
  constexpr std::size_t kTotalItems = 1'000;
  constexpr std::size_t kCapacity = kTotalItems;

  SpscRing<std::uint64_t> ring{ kCapacity };

  std::uint64_t cursor = 0;
  std::vector<std::uint64_t> received;
  received.reserve(kTotalItems);

  {
    std::jthread producer{ [&ring]() {
      for (std::size_t i = 0; i < kTotalItems; ++i)
      {
        ring.push(static_cast<std::uint64_t>(i));
      }
    } };

    // Consumer drains until all kTotalItems values have arrived.
    while (received.size() < kTotalItems)
    {
      const DrainResult<std::uint64_t> result = ring.drain(cursor);
      if (result.entries.empty() && result.dropped == 0)
      {
        std::this_thread::yield();
        continue;
      }
      // No drops expected: ring is large enough to hold every entry.
      EXPECT_THAT(result.dropped, Eq(std::size_t{ 0 }));
      for (const std::uint64_t v : result.entries)
      {
        received.push_back(v);
      }
    }
  }  // jthread joins here — producer is guaranteed done.

  ASSERT_THAT(received, SizeIs(kTotalItems));
  for (std::size_t i = 0; i < kTotalItems; ++i)
  {
    EXPECT_THAT(received[i], Eq(static_cast<std::uint64_t>(i)));
  }
}

}  // namespace
}  // namespace stdr::plot_data
