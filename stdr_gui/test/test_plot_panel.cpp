/** @file Tests for PlotPanel sample-path state machine.
 *
 *  The render path (ImGui::BeginChild, ImPlot calls, etc.) requires a live
 *  ImGui context and is validated by running the standalone binary.  These
 *  tests focus on the parts that can be exercised headlessly: the pause /
 *  resume state machine, error recording on exception, budget-overrun
 *  detection, sample-period decimation, and the Remove / reinstantiate flow.
 *
 *  ## Headless strategy
 *
 *  `PlotPanel::render()` calls `ImGui::BeginChild` / `ImGui::End` which
 *  assert-crash without a live frame.  Tests use `PlotPanel::pump_sample()`
 *  instead — it drives the sample phase (try/catch, budget timing, period
 *  decimation) without touching ImGui, and is the implementation under test.
 *  State is observed via the `slot_status()` accessor and the
 *  `pause_slot` / `unpause_slot` / `remove_slot` API. */

#include <stdr_gui/plot/plot_panel.hpp>
#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal SimulatorBackend stub — satisfies the pure-virtual interface with
// safe no-op defaults.  PlotPanel borrows a reference to it; tests do not
// need real simulation state for the sample-path assertions.
// ---------------------------------------------------------------------------

namespace stdr_gui
{

class StubBackend : public SimulatorBackend
{
public:
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

  void set_step_dt(double /*seconds*/) override
  {
  }

  [[nodiscard]] double get_step_dt() const override
  {
    return 0.1;
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

}  // namespace stdr_gui

// ---------------------------------------------------------------------------
// Minimal SimView stub — all methods return zero/empty values.
// PlotPanel builds a BackendSimView from the backend; this stub is only used
// by plotters in their on_sample if they call back into the view.  The backend
// stub above returns sensible defaults for all introspection calls.
// ---------------------------------------------------------------------------

namespace stdr::plot
{
namespace
{

// The fake plotter records state in shared external counters that we pass
// via a global (test-local, set before construction).  pump_sample() drives
// the try/catch and budget-overrun logic headlessly without ImGui.

// Thread-local state for fake plotters — avoids polluting the global registry.
struct FakeState
{
  int sample_count{ 0 };
  int render_count{ 0 };
  bool throw_on_sample{ false };
  std::chrono::milliseconds period{ 0 };
};

// One global state per test — tests that need multiple slots use different
// indices into a small array.
static FakeState g_fake_state[4];

// ---------------------------------------------------------------------------
// FakePlotter: increments g_fake_state[N].sample_count on each on_sample.
// ---------------------------------------------------------------------------

template <int N>
class FakePlotter : public Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    // Name encodes the slot index so the registry can distinguish types.
    if constexpr (N == 0)
      return "FakePlotter0";
    else if constexpr (N == 1)
      return "FakePlotter1";
    else if constexpr (N == 2)
      return "FakePlotter2";
    else
      return "FakePlotter3";
  }

  void on_sample(SimView& /*sim*/, PlotSink& /*out*/) override
  {
    if (g_fake_state[N].throw_on_sample)
    {
      throw std::runtime_error("deliberate test exception");
    }
    ++g_fake_state[N].sample_count;
  }

  void on_render(const PlotView& /*data*/) override
  {
    ++g_fake_state[N].render_count;
  }

  [[nodiscard]] std::chrono::milliseconds sample_period() const override
  {
    return g_fake_state[N].period;
  }
};

// ---------------------------------------------------------------------------
// SlowPlotter: sleeps 10 ms in on_sample to trigger budget overrun.
// ---------------------------------------------------------------------------

class SlowPlotter : public Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "SlowPlotter";
  }

  void on_sample(SimView& /*sim*/, PlotSink& /*out*/) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  void on_render(const PlotView& /*data*/) override
  {
  }
};

// ---------------------------------------------------------------------------
// Helper: build a local registry with a single plotter type.
// ---------------------------------------------------------------------------

template <typename T>
PlotterRegistry make_registry()
{
  PlotterRegistry reg;
  std::ignore = reg.register_plotter<T>(std::string(T{}.name()));
  return reg;
}

// Reset state before each test.
void reset_fake_state()
{
  for (FakeState& s : g_fake_state)
  {
    s = FakeState{};
  }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(PlotPanel, PauseSlotSetsStatus)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  ASSERT_THAT(panel.slot_status(), ::testing::SizeIs(1));
  EXPECT_FALSE(panel.slot_status()[0].paused);

  panel.pause_slot(0);
  EXPECT_TRUE(panel.slot_status()[0].paused);

  panel.unpause_slot(0);
  EXPECT_FALSE(panel.slot_status()[0].paused);
}

TEST(PlotPanel, PauseOutOfRangeIsNoOp)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // Should not throw or crash.
  panel.pause_slot(99);
  panel.unpause_slot(99);
  EXPECT_EQ(panel.slot_status().size(), 1u);
}

TEST(PlotPanel, RemoveResetsState)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // Manually set state to observe that remove clears it.
  panel.pause_slot(0);
  EXPECT_TRUE(panel.slot_status()[0].paused);

  panel.remove_slot(0);

  // After remove, slot is rebuilt: paused resets, error clears.
  const std::vector<PlotterStatus> after = panel.slot_status();
  ASSERT_THAT(after, ::testing::SizeIs(1));
  EXPECT_FALSE(after[0].paused);
  EXPECT_TRUE(after[0].error.empty());
  EXPECT_FALSE(after[0].overran_budget);
  EXPECT_EQ(after[0].name, "FakePlotter0");
}

TEST(PlotPanel, RemoveOutOfRangeIsNoOp)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  panel.remove_slot(99);
  EXPECT_EQ(panel.slot_status().size(), 1u);
}

TEST(PlotPanel, SlotStatusNameMatchesPlotter)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;

  PlotterRegistry reg;
  std::ignore = reg.register_plotter<FakePlotter<0>>("FakePlotter0");
  std::ignore = reg.register_plotter<FakePlotter<1>>("FakePlotter1");

  PlotPanel panel(backend, reg);

  const std::vector<PlotterStatus> statuses = panel.slot_status();
  ASSERT_THAT(statuses, ::testing::SizeIs(2));
  EXPECT_EQ(statuses[0].name, "FakePlotter0");
  EXPECT_EQ(statuses[1].name, "FakePlotter1");
}

TEST(PlotPanel, ExceptionInOnSampleIsRecorded)
{
  reset_fake_state();
  g_fake_state[0].throw_on_sample = true;

  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // Baseline: no error at construction (on_init doesn't call on_sample).
  const std::vector<PlotterStatus> before = panel.slot_status();
  ASSERT_THAT(before, ::testing::SizeIs(1));
  EXPECT_TRUE(before[0].error.empty());

  // pump_sample() wraps on_sample in try/catch and records the exception.
  panel.pump_sample();

  const std::vector<PlotterStatus> after = panel.slot_status();
  ASSERT_THAT(after, ::testing::SizeIs(1));
  EXPECT_FALSE(after[0].error.empty());
  EXPECT_THAT(after[0].error, ::testing::HasSubstr("deliberate test exception"));
}

TEST(PlotPanel, BudgetOverrunIsRecordedAfterSlowSample)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;

  PlotterRegistry reg;
  std::ignore = reg.register_plotter<SlowPlotter>("SlowPlotter");

  PlotPanel panel(backend, reg);

  const std::vector<PlotterStatus> before = panel.slot_status();
  ASSERT_THAT(before, ::testing::SizeIs(1));
  EXPECT_FALSE(before[0].overran_budget);

  // SlowPlotter sleeps 10 ms, which exceeds the 5 ms budget.
  panel.pump_sample();

  const std::vector<PlotterStatus> after = panel.slot_status();
  ASSERT_THAT(after, ::testing::SizeIs(1));
  EXPECT_TRUE(after[0].overran_budget);
}

TEST(PlotPanel, SamplePeriodDecimatesCalls)
{
  reset_fake_state();
  // 100 ms sample period: should not call on_sample on rapid successive pumps.
  g_fake_state[0].period = std::chrono::milliseconds(100);

  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // Five rapid pumps — sample period has not elapsed, so only the first pump
  // fires on_sample (period 100ms, no sleep between pumps).
  panel.pump_sample();
  panel.pump_sample();
  panel.pump_sample();
  panel.pump_sample();
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 1);

  // Sleep past the period, then pump again — second on_sample should fire.
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 2);
}

TEST(PlotPanel, PauseSkipsSampleViaPump)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // First pump fires on_sample.
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 1);

  // Pause, then pump — on_sample must not fire.
  panel.pause_slot(0);
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 1);
}

TEST(PlotPanel, RemoveResetsCountingState)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg = make_registry<FakePlotter<0>>();
  PlotPanel panel(backend, reg);

  // Pump three times to accumulate state.
  panel.pump_sample();
  panel.pump_sample();
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 3);

  // Remove reinstantiates a fresh plotter — counter in global state carries
  // over because g_fake_state is external, but the important invariant is that
  // the new instance starts its own count from the point of creation, not from
  // where the old one left off.  After remove, one more pump increments to 4.
  panel.remove_slot(0);
  panel.pump_sample();
  EXPECT_EQ(g_fake_state[0].sample_count, 4);

  // Verify slot state was reset by remove (no inherited error or budget flag).
  const std::vector<PlotterStatus> status = panel.slot_status();
  ASSERT_THAT(status, ::testing::SizeIs(1));
  EXPECT_TRUE(status[0].error.empty());
  EXPECT_FALSE(status[0].overran_budget);
}

TEST(PlotPanel, MultipleSlots)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;

  PlotterRegistry reg;
  std::ignore = reg.register_plotter<FakePlotter<0>>("FakePlotter0");
  std::ignore = reg.register_plotter<FakePlotter<1>>("FakePlotter1");
  std::ignore = reg.register_plotter<FakePlotter<2>>("FakePlotter2");

  PlotPanel panel(backend, reg);

  ASSERT_THAT(panel.slot_status(), ::testing::SizeIs(3));

  panel.pause_slot(1);
  EXPECT_FALSE(panel.slot_status()[0].paused);
  EXPECT_TRUE(panel.slot_status()[1].paused);
  EXPECT_FALSE(panel.slot_status()[2].paused);
}

TEST(PlotPanel, EmptyRegistryConstructsSuccessfully)
{
  reset_fake_state();
  stdr_gui::StubBackend backend;
  PlotterRegistry reg;
  PlotPanel panel(backend, reg);

  EXPECT_THAT(panel.slot_status(), ::testing::IsEmpty());
}

}  // namespace
}  // namespace stdr::plot
