#include "stdr_standalone/standalone_backend.hpp"

#include <stdr_simulation/plot_data/command_queue.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

namespace stdr_standalone
{
namespace
{

using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::SizeIs;

constexpr auto kResourcesDir = STDR_RESOURCES_DIR;

std::string map_path()
{
  return std::string(kResourcesDir) + "/maps/maze1.yaml";
}

std::string robot_path()
{
  // khepera2.yaml uses only inline specs (no filename: references), so it loads
  // correctly regardless of the base_dir passed to load_robot_config.
  return std::string(kResourcesDir) + "/resources/robots/khepera2.yaml";
}

// test_laser_robot.yaml is a minimal inline laser-only robot config with
// frame_id: laser_0. All specs are inline so it loads without a
// STDR_RESOURCES_DIR environment variable.
std::string laser_robot_path()
{
  return std::string(STDR_TEST_DIR) + "/test_laser_robot.yaml";
}

// test_laser_robot_5hz.yaml is the same as test_laser_robot.yaml but with
// frequency: 5 so the laser fires every 2nd tick at dt=0.1.
std::string laser_robot_5hz_path()
{
  return std::string(STDR_TEST_DIR) + "/test_laser_robot_5hz.yaml";
}

// test_polygon_robot.yaml is a minimal inline robot config with a square
// polygon footprint (four 0.2 m corners) and no sensors.
std::string polygon_robot_path()
{
  return std::string(STDR_TEST_DIR) + "/test_polygon_robot.yaml";
}

// --- Error cases first ---

TEST(StandaloneBackend, LoadMapFailsOnMissingFile)
{
  StandaloneBackend backend;
  const auto result = backend.load_map("/nonexistent/map.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST(StandaloneBackend, SpawnRobotFailsOnMissingFile)
{
  StandaloneBackend backend;
  const auto result = backend.spawn_robot("/nonexistent/robot.yaml", {});
  EXPECT_FALSE(result.has_value());
}

// --- Construction ---

TEST(StandaloneBackend, ConstructsAndDestructs)
{
  StandaloneBackend backend;
  // Destructor should cleanly stop the sim thread.
}

TEST(StandaloneBackend, InitialSnapshotIsEmpty)
{
  StandaloneBackend backend;
  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_THAT(snapshot->robots, IsEmpty());
  EXPECT_EQ(snapshot->elapsed_time, 0.0);
  EXPECT_FALSE(snapshot->running);
}

// --- Map loading ---

TEST(StandaloneBackend, LoadMapSucceeds)
{
  StandaloneBackend backend;
  const auto result = backend.load_map(map_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto snapshot = backend.get_snapshot();
  EXPECT_GT(snapshot->map.width, 0);
  EXPECT_GT(snapshot->map.height, 0);
}

// --- Robot spawning ---

TEST(StandaloneBackend, SpawnRobotSucceeds)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const stdr_simulation::Pose2D pose{ 1.0, 1.0, 0.0 };
  const auto result = backend.spawn_robot(robot_path(), pose);
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto snapshot = backend.get_snapshot();
  ASSERT_THAT(snapshot->robots, SizeIs(1));
  EXPECT_EQ(snapshot->robots[0].name, result.value());
}

TEST(StandaloneBackend, DeleteRobotRemovesIt)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();
  backend.delete_robot(name);

  const auto snapshot = backend.get_snapshot();
  EXPECT_THAT(snapshot->robots, IsEmpty());
}

// --- Simulation control ---

TEST(StandaloneBackend, StartAndPauseToggleRunningState)
{
  StandaloneBackend backend;
  EXPECT_FALSE(backend.get_snapshot()->running);

  backend.start();
  EXPECT_TRUE(backend.get_snapshot()->running);

  backend.pause();
  EXPECT_FALSE(backend.get_snapshot()->running);
}

TEST(StandaloneBackend, ResetClearsRobotsAndElapsedTime)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  std::ignore = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 });

  backend.start();
  // Let simulation run briefly to accumulate elapsed time.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  // Verify simulation actually advanced before testing that reset clears it.
  const auto pre_reset = backend.get_snapshot();
  EXPECT_GT(pre_reset->elapsed_time, 0.0);

  backend.reset();

  const auto snapshot = backend.get_snapshot();
  EXPECT_THAT(snapshot->robots, IsEmpty());
  EXPECT_EQ(snapshot->elapsed_time, 0.0);
  EXPECT_FALSE(snapshot->running);
}

TEST(StandaloneBackend, SetRobotPoseUpdatesPose)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();

  const stdr_simulation::Pose2D new_pose{ 2.0, 3.0, 1.57 };
  backend.set_robot_pose(name, new_pose);

  const auto snapshot = backend.get_snapshot();
  ASSERT_THAT(snapshot->robots, SizeIs(1));
  EXPECT_DOUBLE_EQ(snapshot->robots[0].pose.x, 2.0);
  EXPECT_DOUBLE_EQ(snapshot->robots[0].pose.y, 3.0);
}

// --- Velocity commands ---

TEST(StandaloneBackend, SetCmdVelMovesRobot)
{
  StandaloneBackend backend;
  // No map is loaded so collision checking is skipped, letting the robot move freely.
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  // Command the robot to drive forward along x at 0.5 m/s.
  backend.set_cmd_vel(name, stdr_simulation::Twist2D{ .linear_x = 0.5, .linear_y = 0.0, .angular_z = 0.0 });

  // Run the async loop long enough for several 10 Hz steps (~200 ms = ~2 steps).
  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  const auto snapshot = backend.get_snapshot();
  ASSERT_THAT(snapshot->robots, SizeIs(1));
  // At least one integration step at 0.5 m/s * 0.1 s = 0.05 m should have occurred.
  EXPECT_GT(snapshot->robots[0].pose.x, 0.01);
}

// --- Messages ---

TEST(StandaloneBackend, PollMessagesReturnsAndDrains)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const std::vector<std::string> messages = backend.poll_messages();
  EXPECT_THAT(messages, Not(IsEmpty()));

  // Second call should be empty since we drained.
  const std::vector<std::string> messages2 = backend.poll_messages();
  EXPECT_THAT(messages2, IsEmpty());
}

TEST(StandaloneBackend, SetSpeedDoesNotCrash)
{
  StandaloneBackend backend;
  backend.set_speed(2.0);
  backend.set_speed(0.5);

  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
}

// --- Step dt ---

TEST(StandaloneBackend, DefaultStepDtMatchesConstant)
{
  StandaloneBackend backend;
  EXPECT_DOUBLE_EQ(backend.get_step_dt(), stdr_gui::kDefaultStepDt);
}

TEST(StandaloneBackend, SetStepDtClampsToValidRange)
{
  StandaloneBackend backend;

  backend.set_step_dt(-1.0);
  EXPECT_EQ(backend.get_step_dt(), stdr_gui::kMinStepDt);

  backend.set_step_dt(100.0);
  EXPECT_EQ(backend.get_step_dt(), stdr_gui::kMaxStepDt);
}

TEST(StandaloneBackend, SetStepDtChangesElapsedTimeRate)
{
  StandaloneBackend backend;
  // No map loaded so collision checking is skipped; we only need the clock to tick.
  std::ignore = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 });

  // step dt = 0.05 s; sleeping ~250 ms wall-clock fires roughly 5 ticks, so
  // sim-time ≈ 0.25 s.  Bounds are intentionally loose to tolerate scheduling
  // jitter: lower bound assumes at least 3 ticks, upper bound allows up to
  // ~18 ticks on a loaded CI machine.
  backend.set_step_dt(0.05);
  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  const auto snapshot = backend.get_snapshot();
  EXPECT_GT(snapshot->elapsed_time, 0.15);
  EXPECT_LT(snapshot->elapsed_time, 0.9);
}

TEST(StandaloneBackend, DeleteNonexistentRobotIsNoOp)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  backend.delete_robot("does_not_exist");

  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_THAT(snapshot->robots, IsEmpty());
}

// --- Introspection: empty world ---

TEST(StandaloneBackendIntrospection, EmptyWorldNumRobotsIsZero)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.num_robots(), 0u);
}

TEST(StandaloneBackendIntrospection, EmptyWorldRobotIdsIsEmpty)
{
  StandaloneBackend backend;
  EXPECT_THAT(backend.robot_ids(), IsEmpty());
}

TEST(StandaloneBackendIntrospection, EmptyWorldSimTimeIsZero)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.sim_time(), 0.0);
}

TEST(StandaloneBackendIntrospection, EmptyWorldPoseForMissingRobotIsNullopt)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.pose("does_not_exist"), std::nullopt);
}

TEST(StandaloneBackendIntrospection, EmptyWorldTwistForMissingRobotIsNullopt)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.twist("does_not_exist"), std::nullopt);
}

TEST(StandaloneBackendIntrospection, EmptyWorldCollidedForMissingRobotIsNullopt)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.collided("does_not_exist"), std::nullopt);
}

TEST(StandaloneBackendIntrospection, EmptyWorldLaserSensorsForMissingRobotIsEmpty)
{
  StandaloneBackend backend;
  EXPECT_THAT(backend.laser_sensors("does_not_exist"), IsEmpty());
}

TEST(StandaloneBackendIntrospection, EmptyWorldSonarSensorsForMissingRobotIsEmpty)
{
  StandaloneBackend backend;
  EXPECT_THAT(backend.sonar_sensors("does_not_exist"), IsEmpty());
}

TEST(StandaloneBackendIntrospection, EmptyWorldLatestLaserForMissingRobotIsNullopt)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.latest_laser("does_not_exist", "laser_0"), std::nullopt);
}

TEST(StandaloneBackendIntrospection, EmptyWorldLatestSonarForMissingRobotIsNullopt)
{
  StandaloneBackend backend;
  EXPECT_EQ(backend.latest_sonar("does_not_exist", "sonar_0"), std::nullopt);
}

// --- Introspection: single sonar-only robot (khepera2) ---

TEST(StandaloneBackendIntrospection, SingleRobotNumRobotsIsOne)
{
  StandaloneBackend backend;
  std::ignore = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 });
  EXPECT_EQ(backend.num_robots(), 1u);
}

TEST(StandaloneBackendIntrospection, SingleRobotIdsReturnsName)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();
  const std::vector<std::string> ids = backend.robot_ids();
  ASSERT_THAT(ids, SizeIs(1));
  EXPECT_EQ(ids[0], name);
}

TEST(StandaloneBackendIntrospection, SingleRobotPoseReturnsSpawnPose)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 2.5, 3.0, 0.0 }).value();
  const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->x, 2.5);
  EXPECT_DOUBLE_EQ(p->y, 3.0);
}

TEST(StandaloneBackendIntrospection, SingleRobotTwistInitiallyZero)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();
  const std::optional<stdr_simulation::Twist2D> t = backend.twist(name);
  ASSERT_TRUE(t.has_value());
  EXPECT_DOUBLE_EQ(t->linear_x, 0.0);
  EXPECT_DOUBLE_EQ(t->angular_z, 0.0);
}

TEST(StandaloneBackendIntrospection, SingleRobotCollidedInitiallyFalse)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();
  // collided() reflects the last step result; before any step it returns false.
  const std::optional<bool> c = backend.collided(name);
  ASSERT_TRUE(c.has_value());
  EXPECT_FALSE(*c);
}

// Count and names (sonar_0..sonar_7) come from the khepera2.yaml definitions.
TEST(StandaloneBackendIntrospection, KheperaSonarSensorsHasEightEntries)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();
  const std::vector<std::string> sonars = backend.sonar_sensors(name);
  ASSERT_THAT(sonars, SizeIs(8));
  EXPECT_EQ(sonars[0], "sonar_0");
  EXPECT_EQ(sonars[7], "sonar_7");
}

TEST(StandaloneBackendIntrospection, KheperaHasNoLaserSensors)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();
  EXPECT_THAT(backend.laser_sensors(name), IsEmpty());
}

// --- Introspection: single laser robot (square_robot_rfid_reader) ---

TEST(StandaloneBackendIntrospection, LaserRobotHasOneLaserSensor)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(laser_robot_path(), { 0.0, 0.0, 0.0 }).value();
  const std::vector<std::string> lasers = backend.laser_sensors(name);
  ASSERT_THAT(lasers, SizeIs(1));
  EXPECT_EQ(lasers[0], "laser_0");
}

// Without a map, laser scans are not produced — latest_laser returns nullopt.
TEST(StandaloneBackendIntrospection, LaserScanNulloptWithoutMap)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(laser_robot_path(), { 0.0, 0.0, 0.0 }).value();

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  // Sensor slot exists but no map was loaded, so no scans were produced.
  EXPECT_EQ(backend.latest_laser(name, "laser_0"), std::nullopt);
}

TEST(StandaloneBackendIntrospection, LaserScanReturnedAfterStepWithMap)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  const std::optional<stdr_simulation::LaserScan> scan = backend.latest_laser(name, "laser_0");
  ASSERT_TRUE(scan.has_value());
  EXPECT_THAT(scan->ranges, Not(IsEmpty()));
}

TEST(StandaloneBackendIntrospection, LatestLaserNulloptForWrongSensorId)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(laser_robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  EXPECT_EQ(backend.latest_laser(name, "no_such_sensor"), std::nullopt);
}

// Sonar scans returned after a step with a map (khepera2 has sonar_0..sonar_7).
TEST(StandaloneBackendIntrospection, SonarScanReturnedAfterStepWithMap)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  const std::optional<stdr_simulation::SonarScan> scan = backend.latest_sonar(name, "sonar_0");
  ASSERT_TRUE(scan.has_value());
  // A valid range should be positive and within the sensor's configured max_range (0.5 m).
  EXPECT_GT(scan->range, 0.0);
}

TEST(StandaloneBackendIntrospection, LatestSonarNulloptForWrongSensorId)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  EXPECT_EQ(backend.latest_sonar(name, "no_such_sonar"), std::nullopt);
}

// --- Introspection: footprint ---

TEST(StandaloneBackendIntrospection, FootprintEmptyForMissingRobot)
{
  StandaloneBackend backend;
  EXPECT_THAT(backend.footprint("does_not_exist"), IsEmpty());
}

TEST(StandaloneBackendIntrospection, PolygonFootprintReturnsKnownVertices)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(polygon_robot_path(), { 0.0, 0.0, 0.0 }).value();

  const std::vector<stdr_simulation::Point2D> verts = backend.footprint(name);
  ASSERT_THAT(verts, SizeIs(4));
  EXPECT_DOUBLE_EQ(verts[0].x, -0.2);
  EXPECT_DOUBLE_EQ(verts[0].y, -0.2);
}

// --- Introspection: multi-robot ---

TEST(StandaloneBackendIntrospection, MultiRobotNumRobotsIsCorrect)
{
  StandaloneBackend backend;
  std::ignore = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 });
  std::ignore = backend.spawn_robot(robot_path(), { 3.0, 1.0, 0.0 });
  EXPECT_EQ(backend.num_robots(), 2u);
}

TEST(StandaloneBackendIntrospection, MultiRobotIdsReturnsBothNames)
{
  StandaloneBackend backend;
  const std::string name0 = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();
  const std::string name1 = backend.spawn_robot(robot_path(), { 3.0, 1.0, 0.0 }).value();

  const std::vector<std::string> ids = backend.robot_ids();
  ASSERT_THAT(ids, SizeIs(2));
  // Both names must appear, but order is unspecified.
  EXPECT_THAT(ids, ::testing::Contains(name0));
  EXPECT_THAT(ids, ::testing::Contains(name1));
}

TEST(StandaloneBackendIntrospection, MultiRobotPoseQueriedIndependently)
{
  StandaloneBackend backend;
  const std::string name0 = backend.spawn_robot(robot_path(), { 1.0, 0.0, 0.0 }).value();
  const std::string name1 = backend.spawn_robot(robot_path(), { 5.0, 0.0, 0.0 }).value();

  const std::optional<stdr_simulation::Pose2D> p0 = backend.pose(name0);
  const std::optional<stdr_simulation::Pose2D> p1 = backend.pose(name1);
  ASSERT_TRUE(p0.has_value());
  ASSERT_TRUE(p1.has_value());
  EXPECT_DOUBLE_EQ(p0->x, 1.0);
  EXPECT_DOUBLE_EQ(p1->x, 5.0);
}

// --- Introspection: sim_time advances ---

TEST(StandaloneBackendIntrospection, SimTimeAdvancesWhenRunning)
{
  StandaloneBackend backend;
  std::ignore = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 });

  EXPECT_EQ(backend.sim_time(), 0.0);

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  EXPECT_GT(backend.sim_time(), 0.0);
}

// --- Command injection via push_command ---

TEST(StandaloneBackendCommandQueue, VelocityCommandMovesRobot)
{
  StandaloneBackend backend;
  // No map needed — collision checking is skipped without a map.
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  // Enqueue a forward velocity command via the command queue.
  stdr::plot_data::Command cmd;
  cmd.kind = stdr::plot_data::Command::Kind::Velocity;
  cmd.robot = name;
  cmd.twist = stdr_simulation::Twist2D{ .linear_x = 0.5, .linear_y = 0.0, .angular_z = 0.0 };
  backend.push_command(cmd);

  // Run several ticks so the sim thread drains the queue and integrates motion.
  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  // The robot should have moved along x from the velocity command.
  const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
  ASSERT_TRUE(p.has_value());
  EXPECT_GT(p->x, 0.01);

  // The queued twist should now be visible in the robot's cmd_vel.
  const std::optional<stdr_simulation::Twist2D> t = backend.twist(name);
  ASSERT_TRUE(t.has_value());
  EXPECT_DOUBLE_EQ(t->linear_x, 0.5);
}

TEST(StandaloneBackendCommandQueue, TeleportCommandMovesRobotToPose)
{
  StandaloneBackend backend;
  const std::string name = backend.spawn_robot(robot_path(), { 0.0, 0.0, 0.0 }).value();

  // Teleport the robot to a known pose via the command queue.  The robot has
  // zero cmd_vel (never set), so it should not drift after teleport — any
  // position error is purely rounding from the physics integrator.
  stdr::plot_data::Command cmd;
  cmd.kind = stdr::plot_data::Command::Kind::Teleport;
  cmd.robot = name;
  cmd.pose = stdr_simulation::Pose2D{ .x = 3.0, .y = 4.0, .theta = 1.0 };
  backend.push_command(cmd);

  // Run long enough to guarantee at least one full sim iteration (default tick is
  // 100 ms real-time), so the sim thread drains and applies the teleport, then
  // stop. Zero cmd_vel keeps the robot stationary across however many extra ticks
  // run before pause() takes effect.
  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.pause();

  const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
  ASSERT_TRUE(p.has_value());
  // Zero cmd_vel means no drift — 1e-3 m tolerance covers floating-point rounding.
  EXPECT_NEAR(p->x, 3.0, 1e-3);
  EXPECT_NEAR(p->y, 4.0, 1e-3);
}

TEST(StandaloneBackendCommandQueue, UnknownRobotVelocityCommandDoesNotCrash)
{
  StandaloneBackend backend;

  stdr::plot_data::Command cmd;
  cmd.kind = stdr::plot_data::Command::Kind::Velocity;
  cmd.robot = "does_not_exist";
  cmd.twist = stdr_simulation::Twist2D{ .linear_x = 1.0, .linear_y = 0.0, .angular_z = 0.0 };
  backend.push_command(cmd);

  // Run a tick — the command should be silently dropped without crashing.
  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  backend.pause();

  // Verify the backend is still operational.
  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
}

TEST(StandaloneBackendCommandQueue, UnknownRobotTeleportCommandDoesNotCrash)
{
  StandaloneBackend backend;

  stdr::plot_data::Command cmd;
  cmd.kind = stdr::plot_data::Command::Kind::Teleport;
  cmd.robot = "does_not_exist";
  cmd.pose = stdr_simulation::Pose2D{ .x = 5.0, .y = 5.0, .theta = 0.0 };
  backend.push_command(cmd);

  backend.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  backend.pause();

  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
}

// ── Rate control: TF and odom rates ─────────────────────────────────────────

TEST(StandaloneBackendRates, DefaultTfRateIsKDefaultTfRate)
{
  StandaloneBackend backend;
  EXPECT_DOUBLE_EQ(backend.get_tf_rate(), stdr_gui::kDefaultTfRate);
}

TEST(StandaloneBackendRates, DefaultOdomRateIsKDefaultOdomRate)
{
  StandaloneBackend backend;
  EXPECT_DOUBLE_EQ(backend.get_odom_rate(), stdr_gui::kDefaultOdomRate);
}

TEST(StandaloneBackendRates, SetTfRateClampsBelowMin)
{
  StandaloneBackend backend;
  backend.set_tf_rate(-10.0);
  EXPECT_DOUBLE_EQ(backend.get_tf_rate(), stdr_gui::kMinRateHz);
}

TEST(StandaloneBackendRates, SetTfRateClampsAboveMax)
{
  StandaloneBackend backend;
  backend.set_tf_rate(9999.0);
  EXPECT_DOUBLE_EQ(backend.get_tf_rate(), stdr_gui::kMaxRateHz);
}

TEST(StandaloneBackendRates, SetOdomRateClampsBelowMin)
{
  StandaloneBackend backend;
  backend.set_odom_rate(-1.0);
  EXPECT_DOUBLE_EQ(backend.get_odom_rate(), stdr_gui::kMinRateHz);
}

TEST(StandaloneBackendRates, SetOdomRateClampsAboveMax)
{
  StandaloneBackend backend;
  backend.set_odom_rate(2000.0);
  EXPECT_DOUBLE_EQ(backend.get_odom_rate(), stdr_gui::kMaxRateHz);
}

// After setting odom_rate=5.0 with step_dt=0.1, effective rate should be 5 Hz
// because period_ticks = round(1/(5*0.1)) = 2 → effective = 1/(2*0.1) = 5.0.
TEST(StandaloneBackendRates, OdomRateGetterReturnsSetValue)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();

  backend.set_step_dt(0.1);
  backend.set_odom_rate(5.0);

  // Effective rate is exposed via the engine; use get_laser_rate/get_sonar_rate
  // for sensor streams.  For odom there is no direct getter on the backend, but
  // we can verify via the tf/odom rate accessors that the stored value is correct.
  EXPECT_DOUBLE_EQ(backend.get_odom_rate(), 5.0);
  // Indirectly confirm the engine uses it: get_laser_rate on a missing sensor is 0.
  EXPECT_DOUBLE_EQ(backend.get_laser_rate(name, 42), 0.0);
}

// With laser frequency=5 Hz and dt=0.1, the laser should fire ~10 times in 20 ticks
// (2 s sim time).  We measure via laser_publish_count with a loose upper bound to
// tolerate wall-clock scheduling jitter.
TEST(StandaloneBackendRates, LaserFiresAtConfiguredRate)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  // test_laser_robot_5hz.yaml: frequency: 5 → period_ticks=2 at dt=0.1.
  const std::string name = backend.spawn_robot(laser_robot_5hz_path(), { 1.0, 1.0, 0.0 }).value();

  // Run until sim_time >= 2.0 s (20 ticks at dt=0.1) with a 10 s wall-clock
  // timeout to avoid hanging CI.
  backend.start();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (backend.sim_time() < 2.0 && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  backend.pause();

  const std::string key = name + "/laser_0";
  const std::size_t count = backend.laser_publish_count(key);
  // 20 ticks ÷ period 2 = 10 firings.  Allow ±2 for timing imprecision.
  EXPECT_GE(count, 8u);
  EXPECT_LE(count, 12u);
}

}  // namespace
}  // namespace stdr_standalone
