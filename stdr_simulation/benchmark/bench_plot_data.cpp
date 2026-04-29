#include <stdr_simulation/plot_data/latest_snapshot.hpp>
#include <stdr_simulation/plot_data/spsc_ring.hpp>

#include <benchmark/benchmark.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

namespace stdr::plot_data
{
namespace
{

// ---------------------------------------------------------------------------
// SpscRing throughput vs mutex-guarded std::deque
// ---------------------------------------------------------------------------

/** Push N items then drain them from a fresh SpscRing.  Measures combined
 *  producer + consumer throughput in a single-threaded setting (no contention).
 *  Real-world throughput with two threads is higher (parallel execution), but
 *  single-thread measures the pure data-structure cost. */
static void BmSpscRingThroughput(benchmark::State& state)
{
  const std::size_t count = static_cast<std::size_t>(state.range(0));
  SpscRing<std::size_t> ring{ 256 };

  for (auto _ : state)
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      ring.push(i);
    }
    std::uint64_t cursor = 0;
    std::size_t sum = 0;
    while (true)
    {
      const DrainResult<std::size_t> result = ring.drain(cursor);
      if (result.entries.empty() && result.dropped == 0)
      {
        break;
      }
      for (const std::size_t v : result.entries)
      {
        sum += v;
      }
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) * static_cast<std::int64_t>(count));
}
BENCHMARK(BmSpscRingThroughput)->Arg(64)->Arg(256)->Arg(1024);

/** Equivalent benchmark using a mutex-guarded std::deque as baseline. */
static void BmMutexDequeThroughput(benchmark::State& state)
{
  const std::size_t count = static_cast<std::size_t>(state.range(0));
  std::deque<std::size_t> q;
  std::mutex mtx;

  for (auto _ : state)
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      const std::lock_guard<std::mutex> lock{ mtx };
      q.push_back(i);
    }
    std::size_t sum = 0;
    while (true)
    {
      const std::lock_guard<std::mutex> lock{ mtx };
      if (q.empty())
      {
        break;
      }
      sum += q.front();
      q.pop_front();
    }
    benchmark::DoNotOptimize(sum);
  }
  state.SetItemsProcessed(static_cast<std::int64_t>(state.iterations()) * static_cast<std::int64_t>(count));
}
BENCHMARK(BmMutexDequeThroughput)->Arg(64)->Arg(256)->Arg(1024);

// ---------------------------------------------------------------------------
// LatestSnapshot read latency vs mutex+shared_ptr baseline
// ---------------------------------------------------------------------------

/** Measure the latency of a single LatestSnapshot::read() call when a value
 *  has already been published (the common GUI-frame hot-path). */
static void BmLatestSnapshotReadLatency(benchmark::State& state)
{
  LatestSnapshot<std::size_t> cell;
  cell.publish(std::make_shared<const std::size_t>(42));

  for (auto _ : state)
  {
    const std::shared_ptr<const std::size_t> got = cell.read();
    benchmark::DoNotOptimize(got);
  }
}
BENCHMARK(BmLatestSnapshotReadLatency);

/** Equivalent benchmark: mutex-guarded shared_ptr read. */
static void BmMutexSharedPtrReadLatency(benchmark::State& state)
{
  std::shared_ptr<const std::size_t> ptr = std::make_shared<const std::size_t>(42);
  std::mutex mtx;

  for (auto _ : state)
  {
    std::shared_ptr<const std::size_t> got;
    {
      const std::lock_guard<std::mutex> lock{ mtx };
      got = ptr;
    }
    benchmark::DoNotOptimize(got);
  }
}
BENCHMARK(BmMutexSharedPtrReadLatency);

}  // namespace
}  // namespace stdr::plot_data

BENCHMARK_MAIN();
