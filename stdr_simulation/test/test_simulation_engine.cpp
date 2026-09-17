#include <stdr_simulation/motion/ideal_motion_model.hpp>
#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>

namespace stdr_simulation
{
namespace
{

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::SizeIs;

// A minimal robot config with no sensors and a circular footprint.
RobotConfig minimal_robot()
{
  RobotConfig cfg;
  cfg.kinematic_model.type = "ideal";
  cfg.footprint.radius = 0.05;
  return cfg;
}

// A robot config with a single laser sensor covering 180° at 0.5 m range.
RobotConfig robot_with_laser()
{
  RobotConfig cfg = minimal_robot();
  LaserConfig laser;
  laser.min_angle = -std::numbers::pi / 2.0;
  laser.max_angle = std::numbers::pi / 2.0;
  laser.min_range = 0.05;
  laser.max_range = 0.5;
  laser.num_rays = 5;
  cfg.laser_sensors.push_back(laser);
  return cfg;
}

// A robot config with a single laser at 5 Hz — fires every 2nd tick at dt=0.1.
RobotConfig robot_with_laser_5hz()
{
  RobotConfig cfg = robot_with_laser();
  cfg.laser_sensors[0].frequency = 5.0;
  return cfg;
}

// Builds a 20×20 free occupancy grid (0.1 m/cell, no obstacles).
OccupancyGrid free_map()
{
  OccupancyGrid map;
  map.width = 20;
  map.height = 20;
  map.resolution = 0.1;
  map.origin = { 0.0, 0.0, 0.0 };
  map.data.assign(static_cast<std::size_t>(20 * 20), 0);
  return map;
}

// Builds on free_map() by occupying an entire column (x in [0.6, 0.7)) as a
// vertical wall spanning the full height of the grid.
OccupancyGrid wall_map()
{
  OccupancyGrid map = free_map();
  constexpr int kWallCol = 6;
  for (int row = 0; row < map.height; ++row)
  {
    map.data[static_cast<std::size_t>(row * map.width + kWallCol)] = 100;
  }
  return map;
}

// ---- Simple cases -----------------------------------------------------------

TEST(SimulationEngineTest, SpawnRobotReturnsName)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 1.0, 1.0, 0.0 });
  EXPECT_EQ(name, "robot0");
}

TEST(SimulationEngineTest, DeleteRobot)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 1.0, 1.0, 0.0 });
  engine.step(0.1);
  ASSERT_THAT(engine.get_sensor_data(name), NotNull());

  engine.delete_robot(name);
  EXPECT_THAT(engine.get_sensor_data(name), IsNull());
}

TEST(SimulationEngineTest, ClearSensorDataUnknownRobotIsNoop)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  engine.clear_sensor_data("robot0");
  EXPECT_THAT(engine.get_sensor_data("robot0"), IsNull());
}

// ---- Complex cases ----------------------------------------------------------

TEST(SimulationEngineTest, StepUpdatesPose)
{
  world::WorldModel world;
  world.set_map(free_map());
  SimulationEngine engine{ world };

  const Pose2D start{ 1.0, 1.0, 0.0 };
  const std::string name = engine.spawn_robot(minimal_robot(), start);

  // Drive forward along +x at 1 m/s.
  engine.set_cmd_vel(name, Twist2D{ 1.0, 0.0, 0.0 });
  engine.step(0.1);

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  // After 0.1 s at 1 m/s the robot should have moved forward.
  EXPECT_GT(state->pose.x, start.x);
  EXPECT_DOUBLE_EQ(state->pose.y, start.y);
}

TEST(SimulationEngineTest, StepWithMapRunsSensors)
{
  world::WorldModel world;
  world.set_map(free_map());
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(robot_with_laser(), Pose2D{ 1.0, 1.0, 0.0 });
  engine.step(0.1);

  const RobotSensorData* data = engine.get_sensor_data(name);
  ASSERT_THAT(data, NotNull());
  // The robot has one laser; step() must populate exactly one scan.
  ASSERT_THAT(data->laser_scans, SizeIs(1));
  // Each scan has the configured number of rays.
  EXPECT_THAT(data->laser_scans[0].ranges, SizeIs(5));
}

TEST(SimulationEngineTest, ClearSensorDataResetsToFreshlySpawnedState)
{
  world::WorldModel world;
  world.set_map(free_map());
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(robot_with_laser(), Pose2D{ 1.0, 1.0, 0.0 });
  engine.step(0.1);

  const RobotSensorData* populated = engine.get_sensor_data(name);
  ASSERT_THAT(populated, NotNull());
  ASSERT_THAT(populated->laser_scans, SizeIs(1));
  // Confirm the scan actually got populated by the step above, so clearing it
  // below is a meaningful assertion rather than a no-op check.
  EXPECT_THAT(populated->laser_scans[0].ranges, SizeIs(5));

  engine.clear_sensor_data(name);

  const RobotSensorData* cleared = engine.get_sensor_data(name);
  ASSERT_THAT(cleared, NotNull());
  EXPECT_FALSE(cleared->collided);
  ASSERT_THAT(cleared->laser_scans, SizeIs(1));
  // Value-initialized: the slot exists (so event.index stays valid) but holds
  // no scan data, exactly as at spawn.
  EXPECT_THAT(cleared->laser_scans[0].ranges, SizeIs(0));
}

TEST(SimulationEngineTest, StepWithoutMapSkipsRaycastSensors)
{
  world::WorldModel world;
  // Intentionally no map set — raycast sensors cannot fire.
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(robot_with_laser(), Pose2D{ 0.5, 0.5, 0.0 });
  engine.step(0.1);

  const RobotSensorData* data = engine.get_sensor_data(name);
  ASSERT_THAT(data, NotNull());
  // Without a map the laser scan slots exist (pre-sized) but contain default
  // (zero) scans — the sensor never fires.  The slot count still matches config.
  ASSERT_THAT(data->laser_scans, SizeIs(1));
  // The scan was never populated, so ranges will be empty.
  EXPECT_THAT(data->laser_scans[0].ranges, SizeIs(0));
}

// ---- Center-of-rotation integration tests -----------------------------------

// A robot configured with center_of_rotation={0,0} and a pure rotation command
// (v=0, w≠0) must not translate its body origin — the engine path mirrors the
// unit-level motion model behaviour across the full spawn+step pipeline.
TEST(SimulationEngineTest, DefaultCenterOfRotationPureRotationStaysInPlace)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const Pose2D start{ 1.0, 2.0, 0.0 };
  const std::string name = engine.spawn_robot(minimal_robot(), start);

  // Pure spin with zero linear velocity — body origin must not translate.
  engine.set_cmd_vel(name, Twist2D{ 0.0, 0.0, 1.0 });
  engine.step(0.1);

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_NEAR(state->pose.x, start.x, 1e-9);
  EXPECT_NEAR(state->pose.y, start.y, 1e-9);
  // Heading must have rotated by w*dt = 0.1 rad.
  EXPECT_NEAR(state->pose.theta, 0.1, 1e-9);
}

// A robot configured with a non-origin center_of_rotation and a pure rotation
// command must pivot its body origin about the configured world-frame pivot
// point.  The invariant tested is that the distance from the body origin to the
// fixed world-frame pivot equals the configured body-frame pivot magnitude —
// this is independent of the heading and requires no precomputed coordinates.
TEST(SimulationEngineTest, OffsetCenterOfRotationPivotsPoseAboutConfiguredPoint)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  // Pivot is 1 m ahead of the body origin in the body frame.  With the robot
  // starting at origin facing +x, the world-frame pivot starts at (1, 0).
  RobotConfig cfg = minimal_robot();
  // Direct RobotConfig construction bypasses load_robot_config footprint
  // validation; the wide pivot is intentional to produce a measurable arc.
  cfg.center_of_rotation = { 1.0, 0.0 };

  const Pose2D start{ 0.0, 0.0, 0.0 };
  const std::string name = engine.spawn_robot(cfg, start);

  // For v=0, the ideal model keeps the pivot fixed in world frame regardless of w.
  const double pivot_world_x = start.x + cfg.center_of_rotation.x;
  const double pivot_world_y = start.y + cfg.center_of_rotation.y;

  // Pure spin — ten steps to accumulate a measurable arc displacement.
  engine.set_cmd_vel(name, Twist2D{ 0.0, 0.0, 1.0 });
  for (int step = 0; step < 10; ++step)
  {
    engine.step(0.1);
  }

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());

  // Body origin must have translated — pure rotation about a non-origin pivot
  // sweeps the body origin in an arc.
  const double displacement = std::hypot(state->pose.x - start.x, state->pose.y - start.y);
  EXPECT_GT(displacement, 1e-6);

  // Distance from body origin to the fixed world-frame pivot must equal the
  // body-frame pivot magnitude (1.0 m) — the arc radius is preserved throughout.
  const double pivot_radius = std::hypot(cfg.center_of_rotation.x, cfg.center_of_rotation.y);
  const double dist_to_pivot = std::hypot(state->pose.x - pivot_world_x, state->pose.y - pivot_world_y);
  EXPECT_NEAR(dist_to_pivot, pivot_radius, 1e-9);
}

// ---- Rate-scheduler integration tests ---------------------------------------

// With frequency=0 (every tick) and a map, the laser fires on every step.
TEST(SimulationEngineTest, RateSchedulerDefaultRunsEverySensorEveryTick)
{
  world::WorldModel world;
  world.set_map(free_map());
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(robot_with_laser(), Pose2D{ 1.0, 1.0, 0.0 });

  // Drive the robot so each tick produces a distinct scan.
  engine.set_cmd_vel(name, Twist2D{ 0.1, 0.0, 0.0 });

  // Step 5 times — the laser (freq=0) should fire every tick, producing 5 distinct
  // range-set updates.  We track this via the Tf/Odom events appearing each tick.
  for (int i = 0; i < 5; ++i)
  {
    engine.step(0.1);
    const std::vector<StreamEvent>& events = engine.last_events(name);
    // TF and Odom default to 0 Hz (every tick) when not explicitly set.
    const bool laser_fired = std::any_of(events.begin(), events.end(), [](const StreamEvent& e) {
      return e.kind == StreamKind::Laser && e.index == 0;
    });
    EXPECT_TRUE(laser_fired) << "Laser should fire every tick when frequency=0";
  }
}

// With frequency=5 Hz and dt=0.1 s, period_ticks=2 — fires every 2nd tick.
TEST(SimulationEngineTest, RateSchedulerLaserAtHalfTickRate)
{
  world::WorldModel world;
  world.set_map(free_map());
  // dt=0.1, laser=5 Hz → period_ticks = round(1/(5*0.1)) = round(2) = 2.
  SimulationEngine engine{ world, 0.1 };

  const std::string name = engine.spawn_robot(robot_with_laser_5hz(), Pose2D{ 1.0, 1.0, 0.0 });
  engine.set_cmd_vel(name, Twist2D{ 0.05, 0.0, 0.0 });

  int laser_fire_count = 0;
  for (int i = 0; i < 20; ++i)
  {
    engine.step(0.1);
    const std::vector<StreamEvent>& events = engine.last_events(name);
    for (const StreamEvent& e : events)
    {
      if (e.kind == StreamKind::Laser && e.index == 0)
      {
        ++laser_fire_count;
      }
    }
  }
  // 20 ticks ÷ period 2 = 10 firings.
  EXPECT_EQ(laser_fire_count, 10);
}

// With odom_rate=10 and dt=0.1, period=1 → odom fires every tick.
TEST(SimulationEngineTest, RateSchedulerOdomFiresEveryTickAtMatchedRate)
{
  world::WorldModel world;
  SimulationEngine engine{ world, 0.1 };
  engine.set_odom_rate(10.0);

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 0.5, 0.5, 0.0 });

  for (int i = 0; i < 5; ++i)
  {
    engine.step(0.1);
    const std::vector<StreamEvent>& events = engine.last_events(name);
    const bool odom_fired = std::any_of(events.begin(), events.end(), [](const StreamEvent& e) {
      return e.kind == StreamKind::Odom && e.index == 0;
    });
    EXPECT_TRUE(odom_fired) << "Odom at 10 Hz / dt=0.1 should fire every tick";
  }
}

// With tf_rate=50 and dt=0.1, 1/(50*0.1)=0.2 → round(0.2)=0 → clamped to 1.
// TF fires every tick at the sim rate.
TEST(SimulationEngineTest, RateSchedulerTfClampedAtSimRate)
{
  world::WorldModel world;
  SimulationEngine engine{ world, 0.1 };
  engine.set_tf_rate(50.0);

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 0.5, 0.5, 0.0 });

  for (int i = 0; i < 5; ++i)
  {
    engine.step(0.1);
    const std::vector<StreamEvent>& events = engine.last_events(name);
    const bool tf_fired = std::any_of(events.begin(), events.end(),
                                      [](const StreamEvent& e) { return e.kind == StreamKind::Tf && e.index == 0; });
    EXPECT_TRUE(tf_fired) << "TF at 50 Hz with dt=0.1 is clamped to sim rate (fires every tick)";
  }
}

// Changing step_dt recomputes period_ticks in all schedulers.
TEST(SimulationEngineTest, RateSchedulerChangeStepDtRecomputes)
{
  world::WorldModel world;
  SimulationEngine engine{ world, 0.1 };
  engine.set_tf_rate(50.0);

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 0.5, 0.5, 0.0 });

  // With dt=0.1, tf_rate=50 → effective = 10 Hz (clamped to period 1).
  EXPECT_DOUBLE_EQ(engine.effective_tf_rate(name), 10.0);

  // Now use a finer dt; TF at 50 Hz with dt=0.02 → period=round(1/(50*0.02))=1.
  // Effective rate = 1/(1*0.02) = 50 Hz.
  engine.set_step_dt(0.02);
  EXPECT_DOUBLE_EQ(engine.effective_tf_rate(name), 50.0);

  // tf_rate=20, dt=0.02 → period=round(1/(20*0.02))=round(2.5)=3 (nearest integer).
  // Effective rate = 1/(3*0.02) ≈ 16.67 Hz.
  engine.set_tf_rate(20.0);
  const double effective = engine.effective_tf_rate(name);
  EXPECT_NEAR(effective, 1.0 / (3.0 * 0.02), 1e-9);
}

// When a robot is removed its scheduler should be erased.
TEST(SimulationEngineTest, RobotRemovalCleansScheduler)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 0.5, 0.5, 0.0 });
  engine.step(0.1);

  // Confirm scheduler registered (effective_tf_rate returns non-zero even for
  // rate=0 because period_ticks=1 ⇒ effective=1/dt).
  const double rate_before = engine.effective_tf_rate(name);
  EXPECT_GT(rate_before, 0.0);

  engine.delete_robot(name);
  engine.step(0.1);  // Must not crash.

  // After deletion the scheduler entry is gone — effective rate returns 0.
  EXPECT_DOUBLE_EQ(engine.effective_tf_rate(name), 0.0);
  // last_events returns empty (static sentinel).
  EXPECT_THAT(engine.last_events(name), SizeIs(0));
}

// Sensor data vector is pre-sized to match config on spawn so no out-of-bounds
// access occurs before the sensor fires for the first time.
TEST(SimulationEngineTest, SensorDataPresizedOnSpawn)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  // Spawn without running step() — sensor data should still have the right size.
  const std::string name = engine.spawn_robot(robot_with_laser(), Pose2D{ 1.0, 1.0, 0.0 });
  const RobotSensorData* data = engine.get_sensor_data(name);
  ASSERT_THAT(data, NotNull());
  EXPECT_THAT(data->laser_scans, SizeIs(1));
}

// Accessor returns correct step_dt after construction and after set_step_dt.
TEST(SimulationEngineTest, StepDtAccessorReflectsValue)
{
  world::WorldModel world;
  SimulationEngine engine{ world, 0.05 };
  EXPECT_DOUBLE_EQ(engine.step_dt(), 0.05);

  engine.set_step_dt(0.02);
  EXPECT_DOUBLE_EQ(engine.step_dt(), 0.02);
}

// Invalid step_dt throws std::invalid_argument.
TEST(SimulationEngineTest, InvalidStepDtThrows)
{
  world::WorldModel world;
  EXPECT_THROW(SimulationEngine(world, 0.0), std::invalid_argument);
  EXPECT_THROW(SimulationEngine(world, -0.1), std::invalid_argument);
}

// With Accumulator mode and a 7 Hz laser at dt=0.1 s, the effective rate must
// match the true target (7 Hz) rather than the snapped 10 Hz.
TEST(SimulationEngineTest, SchedulingModeAppliesToSpawnedRobots)
{
  world::WorldModel world;
  SimulationEngine engine{ world, 0.1 };

  engine.set_scheduling_mode(SchedulingMode::Accumulator);

  // 7 Hz laser — SnapToMultiple would give 10 Hz (period_ticks=1); Accumulator
  // should keep the true target of 7 Hz.
  RobotConfig cfg = robot_with_laser();
  cfg.laser_sensors[0].frequency = 7.0;

  const std::string name = engine.spawn_robot(cfg, Pose2D{ 1.0, 1.0, 0.0 });

  EXPECT_NEAR(engine.effective_sensor_rate(name, StreamKind::Laser, 0), 7.0, 1e-9);
}

// ---- Odometry belief pose ----------------------------------------------------

TEST(SimulationEngineTest, SpawnInitializesOdomPoseToInitialPose)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const Pose2D start{ 1.0, 2.0, 0.3 };
  const std::string name = engine.spawn_robot(minimal_robot(), start);

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_DOUBLE_EQ(state->odom_pose.x, start.x);
  EXPECT_DOUBLE_EQ(state->odom_pose.y, start.y);
  EXPECT_DOUBLE_EQ(state->odom_pose.theta, start.theta);
}

// Under the default OdometryModel::Perfect, truth and odometry integrate the
// same noise-free command, so they must match exactly after any number of steps.
TEST(SimulationEngineTest, PerfectOdometryMatchesTruePoseExactly)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  const std::string name = engine.spawn_robot(minimal_robot(), Pose2D{ 0.0, 0.0, 0.0 });
  engine.set_cmd_vel(name, Twist2D{ 0.5, 0.0, 0.3 });

  for (int i = 0; i < 20; ++i)
  {
    engine.step(0.1);
  }

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_DOUBLE_EQ(state->pose.x, state->odom_pose.x);
  EXPECT_DOUBLE_EQ(state->pose.y, state->odom_pose.y);
  EXPECT_DOUBLE_EQ(state->pose.theta, state->odom_pose.theta);
}

// Under OdometryModel::Velocity, odom_pose must track an independent
// noise-free integration of the same commands, while the true pose — which
// receives the sampled noise — diverges from it.
TEST(SimulationEngineTest, VelocityOdometryMatchesCleanIntegrationAndPoseDiverges)
{
  world::WorldModel world;
  SimulationEngine engine{ world };

  RobotConfig cfg = minimal_robot();
  cfg.kinematic_model.odometry_model = OdometryModel::Velocity;
  cfg.kinematic_model.a_ux_ux = 0.05;
  cfg.kinematic_model.a_w_w = 0.05;

  const Pose2D start{ 0.0, 0.0, 0.0 };
  const std::string name = engine.spawn_robot(cfg, start);
  const Twist2D cmd{ 1.0, 0.0, 0.3 };
  engine.set_cmd_vel(name, cmd);

  // Independent reference belief integrator — deterministic (integrate()
  // draws no RNG samples), so it must track the engine's odom_pose exactly.
  const motion::IdealMotionModel reference_integrator;
  Pose2D expected_odom = start;
  bool pose_diverged = false;

  for (int i = 0; i < 30; ++i)
  {
    engine.step(0.1);
    expected_odom = reference_integrator.integrate(expected_odom, cmd, 0.1);

    const world::RobotState* state = world.get_robot(name);
    ASSERT_THAT(state, NotNull());
    if (std::abs(state->pose.x - state->odom_pose.x) > 1e-9 || std::abs(state->pose.y - state->odom_pose.y) > 1e-9)
    {
      pose_diverged = true;
    }
  }

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_NEAR(state->odom_pose.x, expected_odom.x, 1e-9);
  EXPECT_NEAR(state->odom_pose.y, expected_odom.y, 1e-9);
  EXPECT_NEAR(state->odom_pose.theta, expected_odom.theta, 1e-9);
  EXPECT_TRUE(pose_diverged) << "Velocity-mode noise should separate the true pose from odometry over 30 steps";
}

// On collision the true pose must hold at its prior value, but odometry must
// keep advancing — the wheels are still turning and the encoders cannot see
// the wall.
TEST(SimulationEngineTest, CollisionHoldsTruePoseButOdomKeepsAdvancing)
{
  world::WorldModel world;
  world.set_map(wall_map());
  SimulationEngine engine{ world };

  // Explicit 0.03 m footprint radius (smaller than minimal_robot()'s default
  // 0.05 m) so the robot at x=0.55 spans [0.52, 0.58] — clear of the wall at
  // x=0.6 before stepping, not merely grazing its boundary. After a 1 m/s
  // step for 0.1 s (dx=0.1 m) the footprint spans [0.62, 0.68], squarely
  // inside the occupied cell at x in [0.6, 0.7), so the collision is
  // unambiguous on both sides of step().
  RobotConfig cfg = minimal_robot();
  cfg.footprint.radius = 0.03;
  const Pose2D start{ 0.55, 1.0, 0.0 };
  const std::string name = engine.spawn_robot(cfg, start);
  engine.set_cmd_vel(name, Twist2D{ 1.0, 0.0, 0.0 });

  engine.step(0.1);

  const world::RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_DOUBLE_EQ(state->pose.x, start.x);
  EXPECT_DOUBLE_EQ(state->pose.y, start.y);
  EXPECT_GT(state->odom_pose.x, start.x);
}

}  // namespace
}  // namespace stdr_simulation
