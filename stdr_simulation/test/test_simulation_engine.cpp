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
  // Without a map the laser scan list should be empty.
  EXPECT_THAT(data->laser_scans, SizeIs(0));
}

}  // namespace
}  // namespace stdr_simulation
