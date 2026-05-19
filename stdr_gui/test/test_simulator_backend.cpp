/** @file Tests for SimulatorBackend default effective-rate helpers.
 *
 *  The base-class default impls for get_effective_tf_rate() and
 *  get_effective_odom_rate() derive the effective rate from the configured
 *  target and step_dt using the same rounding rule as RateScheduler.  These
 *  tests verify that rounding and the zero/edge-case guards work correctly. */

#include <stdr_gui/simulator_backend.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace stdr_gui
{
namespace
{

// ---------------------------------------------------------------------------
// Minimal concrete backend — only get_step_dt / get_tf_rate / get_odom_rate
// are exercised by the tests in this file.  All other pure virtuals get safe
// no-op stubs.
// ---------------------------------------------------------------------------

class StubBackendForRateTest : public SimulatorBackend
{
public:
  double step_dt{ 0.1 };
  double tf_rate{ 0.0 };
  double odom_rate{ 0.0 };

  [[nodiscard]] tl::expected<void, std::string> load_map(const std::string& /*path*/) override
  {
    return {};
  }

  [[nodiscard]] tl::expected<std::string, std::string> spawn_robot(const std::string& /*path*/,
                                                                   const stdr_simulation::Pose2D& /*pose*/) override
  {
    return std::string("robot0");
  }

  void delete_robot(const std::string& /*name*/) override
  {
  }

  void start() override
  {
  }

  void pause() override
  {
  }

  void reset() override
  {
  }

  void set_speed(double /*multiplier*/) override
  {
  }

  void set_step_dt(double seconds) override
  {
    step_dt = seconds;
  }

  [[nodiscard]] double get_step_dt() const override
  {
    return step_dt;
  }

  void set_tf_rate(double hz) override
  {
    tf_rate = hz;
  }

  [[nodiscard]] double get_tf_rate() const override
  {
    return tf_rate;
  }

  void set_odom_rate(double hz) override
  {
    odom_rate = hz;
  }

  [[nodiscard]] double get_odom_rate() const override
  {
    return odom_rate;
  }

  void set_robot_pose(const std::string& /*name*/, const stdr_simulation::Pose2D& /*pose*/) override
  {
  }

  void set_cmd_vel(const std::string& /*name*/, const stdr_simulation::Twist2D& /*cmd*/) override
  {
  }

  [[nodiscard]] std::shared_ptr<const SimulationSnapshot> get_snapshot() const override
  {
    return nullptr;
  }

  [[nodiscard]] std::vector<std::string> poll_messages() override
  {
    return {};
  }
};

// ---------------------------------------------------------------------------
// Tests for get_effective_tf_rate() — uses default base-class impl.
// ---------------------------------------------------------------------------

// step_dt=0.1, target=5 Hz → period_ticks = round(1/(5*0.1)) = round(2) = 2
// → effective = 1/(2*0.1) = 5.0 Hz.
TEST(SimulatorBackendEffectiveTfRate, MatchesTargetWhenWithinSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 5.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 5.0);
}

// step_dt=0.1, target=50 Hz → period_ticks = round(1/(50*0.1)) = round(0.2) = 0
// → clamped to 1 → effective = 1/(1*0.1) = 10.0 Hz.
TEST(SimulatorBackendEffectiveTfRate, ClampedWhenAboveSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 50.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 10.0);
}

// step_dt=0.1, target=10 Hz → period_ticks = round(1/(10*0.1)) = round(1) = 1
// → effective = 1/(1*0.1) = 10.0 Hz.  Exact match at the sim rate boundary.
TEST(SimulatorBackendEffectiveTfRate, ExactlyAtSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 10.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 10.0);
}

// target=0 → effective is 0 (guard condition; "every tick" semantics are
// handled elsewhere; the GUI should show 0 when the target is unset).
TEST(SimulatorBackendEffectiveTfRate, ZeroWhenTargetIsZero)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 0.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 0.0);
}

// step_dt=0 → effective is 0 (guard against division by zero).
TEST(SimulatorBackendEffectiveTfRate, ZeroWhenStepDtIsZero)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.0;
  backend.tf_rate = 50.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 0.0);
}

// ---------------------------------------------------------------------------
// Tests for get_effective_odom_rate() — mirrors tf_rate logic.
// ---------------------------------------------------------------------------

TEST(SimulatorBackendEffectiveOdomRate, MatchesTargetWhenWithinSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 5.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 5.0);
}

TEST(SimulatorBackendEffectiveOdomRate, ClampedWhenAboveSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 50.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 10.0);
}

// step_dt=0.1, target=10 Hz → period_ticks = round(1/(10*0.1)) = round(1) = 1
// → effective = 1/(1*0.1) = 10.0 Hz.  Exact match at the sim rate boundary.
TEST(SimulatorBackendEffectiveOdomRate, ExactlyAtSimRate)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 10.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 10.0);
}

TEST(SimulatorBackendEffectiveOdomRate, ZeroWhenTargetIsZero)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 0.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 0.0);
}

TEST(SimulatorBackendEffectiveOdomRate, ZeroWhenStepDtIsZero)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.0;
  backend.odom_rate = 10.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 0.0);
}

// ---------------------------------------------------------------------------
// Test for publishes_ros_topics() — base-class default must return false.
// ---------------------------------------------------------------------------

TEST(SimulatorBackendPublishesRosTopics, DefaultIsFalse)
{
  StubBackendForRateTest backend;
  EXPECT_FALSE(backend.publishes_ros_topics());
}

// ---------------------------------------------------------------------------
// Stub that overrides get_scheduling_mode() to return Accumulator so the
// default effective-rate helpers take the accumulator branch.
// ---------------------------------------------------------------------------

class AccumulatorStubBackend : public StubBackendForRateTest
{
public:
  [[nodiscard]] stdr_simulation::SchedulingMode get_scheduling_mode() const override
  {
    return stdr_simulation::SchedulingMode::Accumulator;
  }
};

// ---------------------------------------------------------------------------
// Accumulator mode: effective rate = min(target, 1/dt).
// ---------------------------------------------------------------------------

// target=7 Hz, dt=0.1 s → sim_rate=10 Hz → effective=min(7, 10)=7 Hz.
TEST(SimulatorBackendEffectiveTfRateAccumulator, BelowSimRateReturnsTarget)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 7.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 7.0);
}

// target=50 Hz, dt=0.1 s → sim_rate=10 Hz → effective=min(50, 10)=10 Hz.
TEST(SimulatorBackendEffectiveTfRateAccumulator, AboveSimRateClampsToSimRate)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 50.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 10.0);
}

// target=0 → guard fires before mode branch; returns 0.
TEST(SimulatorBackendEffectiveTfRateAccumulator, ZeroWhenTargetIsZero)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 0.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 0.0);
}

// target=7 Hz, dt=0.1 s → effective=7 Hz for odom too.
TEST(SimulatorBackendEffectiveOdomRateAccumulator, BelowSimRateReturnsTarget)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 7.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 7.0);
}

// target=50 Hz, dt=0.1 s → effective=10 Hz.
TEST(SimulatorBackendEffectiveOdomRateAccumulator, AboveSimRateClampsToSimRate)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 50.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 10.0);
}

// target=0 → guard fires before mode branch; returns 0.
TEST(SimulatorBackendEffectiveOdomRateAccumulator, ZeroWhenTargetIsZero)
{
  AccumulatorStubBackend backend;
  backend.step_dt = 0.1;
  backend.odom_rate = 0.0;

  EXPECT_DOUBLE_EQ(backend.get_effective_odom_rate(), 0.0);
}

// ---------------------------------------------------------------------------
// Verify SnapToMultiple mode still returns the snapped value (not the target)
// when target does not land on a tick boundary.
// ---------------------------------------------------------------------------

// step_dt=0.1, target=7 Hz → round(1/(7*0.1))=round(1.43)=1 → effective=10 Hz.
// Confirms SnapToMultiple does NOT return 7.0 (unlike Accumulator).
TEST(SimulatorBackendEffectiveTfRateSnapToMultiple, NonBoundaryTargetSnaps)
{
  StubBackendForRateTest backend;
  backend.step_dt = 0.1;
  backend.tf_rate = 7.0;

  // Snapped period = round(1.43) = 1 tick → 10 Hz, NOT 7 Hz.
  EXPECT_DOUBLE_EQ(backend.get_effective_tf_rate(), 10.0);
}

}  // namespace
}  // namespace stdr_gui
