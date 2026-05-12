#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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

}  // namespace
}  // namespace stdr_simulation
