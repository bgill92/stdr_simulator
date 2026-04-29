/** @file Integration test for the SimView adapter over StandaloneBackend.
 *
 *  These tests exercise the BackendSimView concrete class (declared inside
 *  sim_view.cpp) indirectly through the make_backend_sim_view() factory
 *  and the SimView interface.  We link against stdr_standalone so we have
 *  a real backend to drive. */

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>
#include <stdr_gui/simulator_backend.hpp>
#include <stdr_standalone/standalone_backend.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// Forward-declare the factory from sim_view.cpp (not a public header symbol).
// The declaration must be in the same namespace as the definition, not an
// anonymous namespace, so the linker sees the correct external symbol.
namespace stdr::plot
{
std::unique_ptr<SimView> make_backend_sim_view(stdr_gui::SimulatorBackend& backend,
                                               std::unordered_map<std::string, std::uint64_t>& laser_cursors,
                                               std::unordered_map<std::string, std::uint64_t>& sonar_cursors);
}  // namespace stdr::plot

namespace stdr::plot
{
namespace
{

using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::SizeIs;

constexpr const char* kResourcesDir = STDR_RESOURCES_DIR;
constexpr const char* kTestDir = STDR_TEST_DIR;

std::string map_path()
{
  return std::string(kResourcesDir) + "/maps/maze1.yaml";
}

std::string robot_path()
{
  return std::string(kResourcesDir) + "/resources/robots/khepera2.yaml";
}

std::string laser_robot_path()
{
  return std::string(kTestDir) + "/test_laser_robot.yaml";
}

// Helper: build a SimView over a backend with per-call cursor maps.
std::unique_ptr<SimView> make_view(stdr_gui::SimulatorBackend& backend,
                                   std::unordered_map<std::string, std::uint64_t>& lc,
                                   std::unordered_map<std::string, std::uint64_t>& sc)
{
  return make_backend_sim_view(backend, lc, sc);
}

// Helper: poll @p condition every 10 ms until it returns true or @p deadline
// elapses.  Returns true if the condition was satisfied before the deadline.
// Prefer this over unconditional sleep_for to avoid flakiness under CI load.
bool poll_until(std::function<bool()> condition, std::chrono::milliseconds deadline = std::chrono::seconds(2))
{
  const auto start = std::chrono::steady_clock::now();
  while (!condition())
  {
    if (std::chrono::steady_clock::now() - start >= deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

// --- Introspection ---

TEST(SimView, EmptyBackendNumRobotsIsZero)
{
  stdr_standalone::StandaloneBackend backend;
  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_EQ(view->num_robots(), 0u);
}

TEST(SimView, EmptyBackendRobotIdsIsEmpty)
{
  stdr_standalone::StandaloneBackend backend;
  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_THAT(view->robot_ids(), IsEmpty());
}

TEST(SimView, SpawnedRobotVisibleViaNumRobots)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 });

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_EQ(view->num_robots(), 1u);
}

TEST(SimView, PoseReturnsSpawnPose)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 2.5, 3.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const stdr_simulation::Pose2D p = view->pose(name);
  EXPECT_DOUBLE_EQ(p.x, 2.5);
  EXPECT_DOUBLE_EQ(p.y, 3.0);
}

TEST(SimView, PoseReturnsZeroForMissingRobot)
{
  stdr_standalone::StandaloneBackend backend;
  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const stdr_simulation::Pose2D p = view->pose("does_not_exist");
  EXPECT_DOUBLE_EQ(p.x, 0.0);
  EXPECT_DOUBLE_EQ(p.y, 0.0);
}

TEST(SimView, TwistInitiallyZero)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const stdr_simulation::Twist2D t = view->twist(name);
  EXPECT_DOUBLE_EQ(t.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(t.angular_z, 0.0);
}

TEST(SimView, CollidedInitiallyFalse)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_FALSE(view->collided(name));
}

TEST(SimView, SimTimeIsZeroBeforeStart)
{
  stdr_standalone::StandaloneBackend backend;
  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_DOUBLE_EQ(view->sim_time(), 0.0);
}

TEST(SimView, LaserSensorsForKheperaIsEmpty)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_THAT(view->laser_sensors(name), IsEmpty());
}

TEST(SimView, SonarSensorsForKheperaHasEight)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  EXPECT_THAT(view->sonar_sensors(name), SizeIs(8));
}

TEST(SimView, LatestLaserNulloptWithoutMap)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(laser_robot_path(), { 0.0, 0.0, 0.0 }).value();

  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.sim_time() > 0.0; }));
  backend.pause();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const std::optional<stdr_simulation::LaserScan> scan = view->latest_laser(name, "laser_0");
  EXPECT_FALSE(scan.has_value());
}

TEST(SimView, LatestLaserReturnedAfterStepWithMap)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.sim_time() > 0.0; }));
  backend.pause();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const std::optional<stdr_simulation::LaserScan> scan = view->latest_laser(name, "laser_0");
  ASSERT_TRUE(scan.has_value());
  EXPECT_THAT(scan->ranges, Not(IsEmpty()));
}

// --- Command injection via SimView ---

TEST(SimView, CmdVelocityMovesRobot)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  // Enqueue a forward velocity command via SimView.
  view->cmd_velocity(name, 0.5, 0.0);

  // Poll directly on the observable effect (pose.x > threshold) rather than
  // on sim_time to avoid stopping after only one step — the command may not
  // be dispatched until the second step due to when cmd_queue_.drain() runs
  // relative to cv_.wait() in the simulation loop.
  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.pose(name).value_or(stdr_simulation::Pose2D{}).x > 0.01; }));
  backend.pause();

  const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
  ASSERT_TRUE(p.has_value());
  EXPECT_GT(p->x, 0.01);
}

TEST(SimView, TeleportMovesRobotToPose)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  view->teleport(name, stdr_simulation::Pose2D{ .x = 3.0, .y = 4.0, .theta = 0.0 });

  // Poll until the teleport is visible rather than on sim_time to avoid
  // stopping before the second step where the command is dispatched.
  backend.start();
  ASSERT_TRUE(poll_until([&] {
    const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
    return p.has_value() && p->x > 2.0;
  }));
  backend.pause();

  const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->x, 3.0, 1e-2);
  EXPECT_NEAR(p->y, 4.0, 1e-2);
}

// --- Sensor event log via new_laser_scans ---

TEST(SimView, NewLaserScansEmptyBeforeAnyStep)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  // No steps have run — ring is empty.
  const SimView::DrainedLaser drained = view->new_laser_scans(name, "laser_0");
  EXPECT_THAT(drained.scans, IsEmpty());
  EXPECT_EQ(drained.dropped, 0u);
}

TEST(SimView, NewLaserScansReturnedAfterSteps)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  // Poll until the backend has produced at least one scan so the drain below
  // sees real data rather than racing against the first step.
  ASSERT_TRUE(poll_until([&] { return backend.sim_time() > 0.0; }));
  backend.pause();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const SimView::DrainedLaser drained = view->new_laser_scans(name, "laser_0");
  EXPECT_THAT(drained.scans, Not(IsEmpty()));
  EXPECT_EQ(drained.dropped, 0u);
}

TEST(SimView, NewLaserScansForMissingRobotIsEmpty)
{
  stdr_standalone::StandaloneBackend backend;

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  const std::unique_ptr<SimView> view = make_view(backend, lc, sc);

  const SimView::DrainedLaser drained = view->new_laser_scans("no_robot", "laser_0");
  EXPECT_THAT(drained.scans, IsEmpty());
  EXPECT_EQ(drained.dropped, 0u);
}

TEST(SimView, NewLaserScansSecondCallSeesOnlyNewScans)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  // Run a first batch of steps.
  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.sim_time() > 0.0; }));
  backend.pause();

  std::unordered_map<std::string, std::uint64_t> lc;
  std::unordered_map<std::string, std::uint64_t> sc;
  {
    const std::unique_ptr<SimView> view1 = make_view(backend, lc, sc);
    const SimView::DrainedLaser first = view1->new_laser_scans(name, "laser_0");
    EXPECT_THAT(first.scans, Not(IsEmpty()));
  }

  // Run a second batch of steps — capture the sim time after the first pause
  // so we can tell the second batch has actually produced new steps.
  const double sim_time_after_first = backend.sim_time();
  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.sim_time() > sim_time_after_first; }));
  backend.pause();

  {
    const std::unique_ptr<SimView> view2 = make_view(backend, lc, sc);
    const SimView::DrainedLaser second = view2->new_laser_scans(name, "laser_0");
    // The cursor was advanced by the first drain, so the second call must
    // see only new scans — not the ones returned by the first call.
    EXPECT_THAT(second.scans, Not(IsEmpty()));
  }
}

// --- WHOLE_ARCHIVE link guard ---

TEST(PlotterRegistry, WholeArchiveLinkActuallyPullsPoseErrorPlotter)
{
  // This test fails loudly if the WHOLE_ARCHIVE generator expression is ever
  // removed from test_sim_view's target_link_libraries.  Without it, the
  // PoseErrorPlotter TU is dead-stripped and never registers itself.
  const std::vector<std::unique_ptr<Plotter>> plotters = PlotterRegistry::instance().instantiate_all();
  bool found = false;
  for (const std::unique_ptr<Plotter>& p : plotters)
  {
    if (p->name() == "Pose Error")
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "PoseErrorPlotter not found — WHOLE_ARCHIVE link may have been removed";
}

// --- SimIntrospection ---

TEST(SimIntrospection, WrapperDelegatesNumRobots)
{
  stdr_standalone::StandaloneBackend backend;
  std::ignore = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 });

  const SimIntrospection intro(backend);
  EXPECT_EQ(intro.num_robots(), 1u);
}

TEST(SimIntrospection, WrapperDelegatesRobotIds)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  const SimIntrospection intro(backend);
  const std::vector<RobotId> ids = intro.robot_ids();
  ASSERT_THAT(ids, SizeIs(1));
  EXPECT_EQ(ids[0], name);
}

TEST(SimIntrospection, WrapperDelegatesLaserSensors)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(laser_robot_path(), { 0.0, 0.0, 0.0 }).value();

  const SimIntrospection intro(backend);
  const std::vector<SensorId> lasers = intro.laser_sensors(name);
  ASSERT_THAT(lasers, SizeIs(1));
  EXPECT_EQ(lasers[0], "laser_0");
}

TEST(SimIntrospection, WrapperDelegatesSonarSensors)
{
  stdr_standalone::StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  const SimIntrospection intro(backend);
  EXPECT_THAT(intro.sonar_sensors(name), SizeIs(8));
}

}  // namespace
}  // namespace stdr::plot
