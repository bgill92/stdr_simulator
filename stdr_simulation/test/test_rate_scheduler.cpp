#include <stdr_simulation/rate_scheduler.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace stdr_simulation
{
namespace
{

using ::testing::IsEmpty;
using ::testing::SizeIs;

// Convenience: collect the flat list of (kind, index) pairs that fired.
struct FiredSet
{
  std::vector<StreamEvent> events;

  bool contains(StreamKind kind, std::size_t index) const
  {
    for (const StreamEvent& e : events)
    {
      if (e.kind == kind && e.index == index)
      {
        return true;
      }
    }
    return false;
  }
};

// ---------------------------------------------------------------------------
// Tests ordered simplest → most complex per project convention.
// Error validation tests come first.
// ---------------------------------------------------------------------------

// Tick counter starts at 0 before the first tick().  The first tick()
// increments it to 1, so tick N refers to the N-th call (1-based).

TEST(RateSchedulerTest, ConstructorRejectsNonPositiveStepDt)
{
  EXPECT_THROW(RateScheduler(0.0), std::invalid_argument);
  EXPECT_THROW(RateScheduler(-1.0), std::invalid_argument);
}

TEST(RateSchedulerTest, SetStepDtRejectsNonPositive)
{
  RateScheduler sched(0.1);
  EXPECT_THROW(sched.set_step_dt(0.0), std::invalid_argument);
  EXPECT_THROW(sched.set_step_dt(-0.5), std::invalid_argument);
}

TEST(RateSchedulerTest, UnregisteredStreamReturnsZero)
{
  const RateScheduler sched(0.1);
  EXPECT_EQ(sched.effective_rate(StreamKind::Laser, 0), 0.0);
  EXPECT_EQ(sched.period_ticks(StreamKind::Laser, 0), 0U);
}

TEST(RateSchedulerTest, DefaultZeroFreqFiresEveryTick)
{
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, 0.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Odom, 0), 1U);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Odom, 0)) << "Expected Odom to fire on tick " << (i + 1);
  }
}

TEST(RateSchedulerTest, NegativeFreqTreatedAsZero)
{
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, -5.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Odom, 0), 1U);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Odom, 0)) << "Expected Odom to fire on tick " << (i + 1);
  }
}

TEST(RateSchedulerTest, FreqExactlyMatchingSimRate)
{
  // freq=10 Hz, dt=0.1 s → raw_period = 1/( 10 * 0.1 ) = 1.0 → period_ticks=1
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Tf, 0, 10.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Tf, 0), 1U);
  EXPECT_DOUBLE_EQ(sched.effective_rate(StreamKind::Tf, 0), 10.0);
}

TEST(RateSchedulerTest, FreqHalfSimRate)
{
  // freq=5 Hz, dt=0.1 s → raw_period = 1/(5 * 0.1) = 2.0 → period_ticks=2
  // Fires on ticks 2, 4, 6 (1-based); NOT on ticks 1, 3, 5.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, 5.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Odom, 0), 2U);

  for (int tick = 1; tick <= 6; ++tick)
  {
    const FiredSet fired{ sched.tick() };
    const bool expected_fire = (tick % 2 == 0);
    EXPECT_EQ(fired.contains(StreamKind::Odom, 0), expected_fire)
        << "Tick " << tick << ": expected fire=" << expected_fire;
  }
}

TEST(RateSchedulerTest, VeryLowFreq)
{
  // freq=0.5 Hz, dt=0.1 s → raw_period = 1/(0.5 * 0.1) = 20.0 → period_ticks=20
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Sonar, 0, 0.5);

  EXPECT_EQ(sched.period_ticks(StreamKind::Sonar, 0), 20U);
}

TEST(RateSchedulerTest, FreqExceedsSimRateClamped)
{
  // freq=100 Hz, dt=0.1 s → raw_period = 1/(100 * 0.1) = 0.1 → rounds to 0 →
  // clamped to 1 (cannot fire faster than sim rate).
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 100.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Laser, 0), 1U);
  EXPECT_DOUBLE_EQ(sched.effective_rate(StreamKind::Laser, 0), 10.0);
}

TEST(RateSchedulerTest, RoundsToNearestPeriod)
{
  // freq=7 Hz, dt=0.1 s → raw_period = 1/(7 * 0.1) ≈ 1.4286
  // std::round(1.4286) = 1, so period_ticks=1.
  // Trade-off: round over floor so targets slightly below an exact multiple
  // snap to the closer (faster) period rather than always rounding down.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 7.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Laser, 0), 1U);

  // Effective rate = 1/(1 * 0.1) = 10 Hz.
  EXPECT_DOUBLE_EQ(sched.effective_rate(StreamKind::Laser, 0), 10.0);
}

TEST(RateSchedulerTest, ChangingStepDtRecomputesPeriods)
{
  // At dt=0.1: 5 Hz → period=2.  After set_step_dt(0.05): 5 Hz → period=4.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 5.0);
  sched.set_rate(StreamKind::Odom, 0, 10.0);

  EXPECT_EQ(sched.period_ticks(StreamKind::Laser, 0), 2U);
  EXPECT_EQ(sched.period_ticks(StreamKind::Odom, 0), 1U);

  sched.set_step_dt(0.05);

  // 5 Hz × 0.05 s/tick → raw_period = 4.0 → period_ticks=4
  EXPECT_EQ(sched.period_ticks(StreamKind::Laser, 0), 4U);
  // 10 Hz × 0.05 s/tick → raw_period = 2.0 → period_ticks=2
  EXPECT_EQ(sched.period_ticks(StreamKind::Odom, 0), 2U);
}

TEST(RateSchedulerTest, MultipleStreamsIndependent)
{
  // laser-0 @ 5 Hz (period=2), laser-1 @ 10 Hz (period=1)
  // Over 4 ticks: laser-0 fires on ticks 2, 4 → 2 times.
  //               laser-1 fires on ticks 1, 2, 3, 4 → 4 times.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 5.0);
  sched.set_rate(StreamKind::Laser, 1, 10.0);

  std::size_t count0 = 0;
  std::size_t count1 = 0;
  for (int i = 0; i < 4; ++i)
  {
    const FiredSet fired{ sched.tick() };
    if (fired.contains(StreamKind::Laser, 0))
    {
      ++count0;
    }
    if (fired.contains(StreamKind::Laser, 1))
    {
      ++count1;
    }
  }

  EXPECT_EQ(count0, 2U);
  EXPECT_EQ(count1, 4U);
}

// ---------------------------------------------------------------------------
// Parameterized test: effective_rate == 1 / (period_ticks * dt) for various
// (freq_hz, dt) combinations.
// ---------------------------------------------------------------------------

struct RateConfig
{
  double freq_hz;
  double dt;
};

class EffectiveRateTest : public ::testing::TestWithParam<RateConfig>
{
};

TEST_P(EffectiveRateTest, MatchesInverseOfPeriodTimesDt)
{
  const RateConfig cfg = GetParam();
  RateScheduler sched(cfg.dt);
  sched.set_rate(StreamKind::Laser, 0, cfg.freq_hz);

  const std::size_t period = sched.period_ticks(StreamKind::Laser, 0);
  const double expected = 1.0 / (static_cast<double>(period) * cfg.dt);
  EXPECT_NEAR(sched.effective_rate(StreamKind::Laser, 0), expected, 1e-12)
      << "freq=" << cfg.freq_hz << " dt=" << cfg.dt;
}

INSTANTIATE_TEST_SUITE_P(FreqDtCombinations, EffectiveRateTest,
                         ::testing::Values(RateConfig{ 10.0, 0.1 }, RateConfig{ 5.0, 0.1 }, RateConfig{ 20.0, 0.05 },
                                           RateConfig{ 1.0, 0.1 }, RateConfig{ 0.5, 0.1 }),
                         [](const ::testing::TestParamInfo<RateConfig>& info) {
                           // Build a readable name like "Freq10_Dt100ms" from the parameter values.
                           // Frequency is formatted as an integer Hz; dt is converted to ms.
                           const int freq_int = static_cast<int>(info.param.freq_hz);
                           const int dt_ms = static_cast<int>(info.param.dt * 1000.0);
                           return "Freq" + std::to_string(freq_int) + "_Dt" + std::to_string(dt_ms) + "ms";
                         });

TEST(RateSchedulerTest, ClearRemovesAllEntries)
{
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Tf, 0, 50.0);
  sched.set_rate(StreamKind::Odom, 0, 10.0);
  sched.set_rate(StreamKind::Laser, 0, 5.0);

  sched.clear();

  const std::vector<StreamEvent> events = sched.tick();
  EXPECT_THAT(events, IsEmpty());

  // Also verify the query API returns zero for cleared streams.
  EXPECT_EQ(sched.period_ticks(StreamKind::Tf, 0), 0U);
  EXPECT_EQ(sched.effective_rate(StreamKind::Odom, 0), 0.0);
}

TEST(RateSchedulerTest, TickReturnsStreamsInStableOrder)
{
  // Register streams out of natural enum order; expect output sorted by
  // (StreamKind enum value, index).
  // Enum values: Tf=0, Odom=1, Laser=2, Sonar=3
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Sonar, 1, 10.0);
  sched.set_rate(StreamKind::Laser, 0, 10.0);
  sched.set_rate(StreamKind::Tf, 0, 10.0);
  sched.set_rate(StreamKind::Sonar, 0, 10.0);
  sched.set_rate(StreamKind::Odom, 0, 10.0);

  // All streams have period=1, so all fire on tick 1.
  const std::vector<StreamEvent> events = sched.tick();

  ASSERT_THAT(events, SizeIs(5U));
  EXPECT_EQ(events[0].kind, StreamKind::Tf);
  EXPECT_EQ(events[0].index, 0U);
  EXPECT_EQ(events[1].kind, StreamKind::Odom);
  EXPECT_EQ(events[1].index, 0U);
  EXPECT_EQ(events[2].kind, StreamKind::Laser);
  EXPECT_EQ(events[2].index, 0U);
  EXPECT_EQ(events[3].kind, StreamKind::Sonar);
  EXPECT_EQ(events[3].index, 0U);
  EXPECT_EQ(events[4].kind, StreamKind::Sonar);
  EXPECT_EQ(events[4].index, 1U);
}

// ---------------------------------------------------------------------------
// Accumulator mode tests.  Ordered simplest → complex per project convention.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Mode query API
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, DefaultModeIsSnapToMultiple)
{
  // set_rate with the 3-arg overload must default to SnapToMultiple.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 10.0);
  EXPECT_EQ(sched.mode(StreamKind::Laser, 0), SchedulingMode::SnapToMultiple);
}

TEST(AccumulatorModeTest, UnregisteredStreamReturnsSnapMode)
{
  // Consistent with period_ticks() returning 0 for unregistered streams.
  const RateScheduler sched(0.1);
  EXPECT_EQ(sched.mode(StreamKind::Sonar, 42), SchedulingMode::SnapToMultiple);
}

// ---------------------------------------------------------------------------
// Accumulator edge cases: zero / negative freq behave like snap "every tick"
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, AccumulatorZeroFreqFiresEveryTick)
{
  // freq=0.0 → fire every tick regardless of mode.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, 0.0, SchedulingMode::Accumulator);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Odom, 0)) << "Expected Odom to fire on tick " << (i + 1);
  }
}

TEST(AccumulatorModeTest, AccumulatorNegativeFreqFiresEveryTick)
{
  // Non-positive freq → fire every tick, same as SnapToMultiple convention.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, -5.0, SchedulingMode::Accumulator);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Odom, 0)) << "Expected Odom to fire on tick " << (i + 1);
  }
}

// ---------------------------------------------------------------------------
// Accumulator clean-ratio cases: should behave the same as snap
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, AccumulatorExactMatchFiresEveryTick)
{
  // freq=10 Hz, dt=0.1 s → period_seconds=0.1 == step_dt.
  // Each tick: accum += 0.1, fires at 0.1 >= 0.1, accum resets to 0.0.
  // So it fires every tick, matching snap behaviour.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Tf, 0, 10.0, SchedulingMode::Accumulator);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Tf, 0)) << "Expected Tf to fire on tick " << (i + 1);
  }
}

TEST(AccumulatorModeTest, AccumulatorHalfSimRateFiresAlternate)
{
  // freq=5 Hz, dt=0.1 s → period_seconds=0.2.
  // Tick 1: accum=0.1, no fire.  Tick 2: accum=0.2 >= 0.2, fire, accum=0.0.
  // Fires on ticks 2, 4, 6 — same as snap for this clean ratio.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, 5.0, SchedulingMode::Accumulator);

  for (int tick = 1; tick <= 6; ++tick)
  {
    const FiredSet fired{ sched.tick() };
    const bool expected = (tick % 2 == 0);
    EXPECT_EQ(fired.contains(StreamKind::Odom, 0), expected) << "Tick " << tick << ": expected fire=" << expected;
  }
}

TEST(AccumulatorModeTest, AccumulatorClampedAtSimRate)
{
  // freq=100 Hz, dt=0.1 s → period_seconds=0.01 < step_dt.
  // Each tick: accum += 0.1, which exceeds 0.01 immediately → fire every tick.
  // At most one fire per tick (no looping), capping effective rate at sim rate.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 100.0, SchedulingMode::Accumulator);

  for (int i = 0; i < 5; ++i)
  {
    const FiredSet fired{ sched.tick() };
    EXPECT_TRUE(fired.contains(StreamKind::Laser, 0)) << "Expected Laser to fire on tick " << (i + 1);
  }
}

TEST(AccumulatorModeTest, AccumulatorResidualBoundedAtHighFreq)
{
  // freq=100 Hz, dt=0.1 s: period_seconds=0.01 < step_dt.  Without fmod,
  // the residual would grow by (0.1 - 0.01) = 0.09 s per tick.  After
  // 1000 ticks that would be 90 s.  With fmod, residual stays in [0, 0.01).
  // We verify by checking the fire count is exactly 1000 (one per tick)
  // over 1000 ticks — if the residual had drifted, we'd lose ticks.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 100.0, SchedulingMode::Accumulator);

  std::size_t fire_count = 0;
  for (int i = 0; i < 1000; ++i)
  {
    const FiredSet fired{ sched.tick() };
    if (fired.contains(StreamKind::Laser, 0))
    {
      ++fire_count;
    }
  }
  EXPECT_EQ(fire_count, 1000U);
}

// ---------------------------------------------------------------------------
// Accumulator effective_rate() — must report the true target, not snapped value
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, AccumulatorEffectiveRateMatchesTargetForCleanRatio)
{
  // 5 Hz, dt=0.1 s → target is cleanly representable below sim rate (10 Hz).
  // effective_rate() must return 5.0, not the snapped 1/(period_ticks*dt).
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 5.0, SchedulingMode::Accumulator);

  EXPECT_DOUBLE_EQ(sched.effective_rate(StreamKind::Laser, 0), 5.0);
}

TEST(AccumulatorModeTest, AccumulatorEffectiveRateClampedAtSimRate)
{
  // 100 Hz, dt=0.1 s → target exceeds sim rate (10 Hz); clamped to 10.0.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 100.0, SchedulingMode::Accumulator);

  EXPECT_DOUBLE_EQ(sched.effective_rate(StreamKind::Laser, 0), 10.0);
}

// ---------------------------------------------------------------------------
// Accumulator headline tests: average rate matches target over time
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, AccumulatorMatchesTargetRateOverTime)
{
  // 7 Hz, dt=0.1 s, 100 ticks = 10 s of sim time.
  // Expected fires = 7 * 10 = 70.  Accumulator hits this exactly because
  // 10 s / (1/7 s period) = nominally exactly 70.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 7.0, SchedulingMode::Accumulator);

  std::size_t count = 0;
  for (int i = 0; i < 100; ++i)
  {
    const FiredSet fired{ sched.tick() };
    if (fired.contains(StreamKind::Laser, 0))
    {
      ++count;
    }
  }

  // Over 10 s the accumulated residual is (10.0 - 70 * (1/7)) = 0, so count
  // should be exactly 70.  Use EXPECT_NEAR in case of FP edge cases.
  EXPECT_NEAR(static_cast<int>(count), 70, 1);
}

TEST(AccumulatorModeTest, AccumulatorNonIntegerRatioMatchesTarget)
{
  // 3 Hz, dt=0.1 s, 100 ticks = 10 s.  Expected fires = 3 * 10 = 30.
  // 1/3 s period; over 10 s residual = 10.0 - 30 * (1/3) = 0.0, exact.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Sonar, 0, 3.0, SchedulingMode::Accumulator);

  std::size_t count = 0;
  for (int i = 0; i < 100; ++i)
  {
    const FiredSet fired{ sched.tick() };
    if (fired.contains(StreamKind::Sonar, 0))
    {
      ++count;
    }
  }

  EXPECT_NEAR(static_cast<int>(count), 30, 1);
}

// ---------------------------------------------------------------------------
// Mixed-mode test: snap and accumulator streams coexist independently
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, AccumulatorIndependentOfSnap)
{
  // snap @ 5 Hz (period=2 ticks) → fires 50 times in 100 ticks.
  // accum @ 7 Hz → fires ≈70 times in 100 ticks.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Odom, 0, 5.0, SchedulingMode::SnapToMultiple);
  sched.set_rate(StreamKind::Laser, 0, 7.0, SchedulingMode::Accumulator);

  std::size_t snap_count = 0;
  std::size_t accum_count = 0;
  for (int i = 0; i < 100; ++i)
  {
    const FiredSet fired{ sched.tick() };
    if (fired.contains(StreamKind::Odom, 0))
    {
      ++snap_count;
    }
    if (fired.contains(StreamKind::Laser, 0))
    {
      ++accum_count;
    }
  }

  EXPECT_EQ(snap_count, 50U);
  EXPECT_NEAR(static_cast<int>(accum_count), 70, 1);
}

// ---------------------------------------------------------------------------
// set_step_dt must not reset accumulator residual
// ---------------------------------------------------------------------------

TEST(AccumulatorModeTest, SetStepDtPreservesAccumulatorState)
{
  // 4 Hz accumulator at dt=0.1 s → period_seconds=0.25.
  // Tick 1: accum=0.1 (no fire).  Tick 2: accum=0.2 (no fire).
  // Now change dt to 0.2.  period_seconds is unchanged (dt-independent).
  // Tick 3: accum = 0.2 + 0.2 = 0.4 >= 0.25 → fire, accum=0.15.
  // Tick 4: accum = 0.15 + 0.2 = 0.35 >= 0.25 → fire, accum=0.10.
  // If residual were reset to 0 at set_step_dt, tick 3 accum=0.2 → no fire.
  RateScheduler sched(0.1);
  sched.set_rate(StreamKind::Laser, 0, 4.0, SchedulingMode::Accumulator);

  // Tick 1 and 2 at dt=0.1 — accumulate residual, no fires expected.
  std::ignore = sched.tick();
  std::ignore = sched.tick();

  // Switch dt; residual must survive.
  sched.set_step_dt(0.2);

  // Tick 3: should fire because accumulated residual (0.2) + new dt (0.2) = 0.4 >= 0.25.
  const FiredSet tick3{ sched.tick() };
  EXPECT_TRUE(tick3.contains(StreamKind::Laser, 0)) << "Tick 3 must fire after dt change preserves residual";
}

}  // namespace
}  // namespace stdr_simulation
