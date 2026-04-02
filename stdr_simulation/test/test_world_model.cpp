#include <stdr_simulation/world/world_model.hpp>
#include <stdr_simulation/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace stdr_simulation::world {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::SizeIs;

// Returns a minimal RobotConfig with no sensors.
RobotConfig minimal_robot_config()
{
  return RobotConfig{};
}

// ---- Error / degenerate cases -----------------------------------------------

TEST(WorldModelTest, NoMapReturnsNullptr)
{
  const WorldModel world;
  EXPECT_THAT(world.get_map(), IsNull());
}

// ---- Simple cases -----------------------------------------------------------

TEST(WorldModelTest, SetAndGetMap)
{
  WorldModel world;
  OccupancyGrid map;
  map.width = 5;
  map.height = 5;
  map.resolution = 0.1;
  world.set_map(map);
  ASSERT_THAT(world.get_map(), NotNull());
  EXPECT_EQ(world.get_map()->width, 5);
}

TEST(WorldModelTest, AddRobotAssignsName)
{
  WorldModel world;
  const std::string name = world.add_robot(minimal_robot_config());
  EXPECT_EQ(name, "robot0");
}

TEST(WorldModelTest, AddMultipleRobotsSequentialNaming)
{
  WorldModel world;
  const std::string first = world.add_robot(minimal_robot_config());
  const std::string second = world.add_robot(minimal_robot_config());
  EXPECT_EQ(first, "robot0");
  EXPECT_EQ(second, "robot1");
}

TEST(WorldModelTest, AddAndRemoveRfidTag)
{
  WorldModel world;
  RfidTag tag;
  tag.tag_id = "t0";
  tag.message = "hello";
  tag.pose = {1.0, 2.0, 0.0};

  world.add_rfid_tag(tag);
  ASSERT_THAT(world.get_rfid_tags(), SizeIs(1));

  world.remove_rfid_tag("t0");
  EXPECT_THAT(world.get_rfid_tags(), SizeIs(0));
}

// ---- Complex cases ----------------------------------------------------------

TEST(WorldModelTest, RemoveRobot)
{
  WorldModel world;
  const std::string name = world.add_robot(minimal_robot_config());
  ASSERT_THAT(world.get_robot(name), NotNull());

  world.remove_robot(name);
  EXPECT_THAT(world.get_robot(name), IsNull());
}

TEST(WorldModelTest, SetRobotPose)
{
  WorldModel world;
  const std::string name = world.add_robot(minimal_robot_config());

  const Pose2D new_pose{3.0, 4.0, 1.5};
  world.set_robot_pose(name, new_pose);

  const RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_DOUBLE_EQ(state->pose.x, 3.0);
  EXPECT_DOUBLE_EQ(state->pose.y, 4.0);
  EXPECT_DOUBLE_EQ(state->pose.theta, 1.5);
}

TEST(WorldModelTest, SetRobotCmdVel)
{
  WorldModel world;
  const std::string name = world.add_robot(minimal_robot_config());

  const Twist2D cmd{1.0, 0.5, 0.2};
  world.set_robot_cmd_vel(name, cmd);

  const RobotState* state = world.get_robot(name);
  ASSERT_THAT(state, NotNull());
  EXPECT_DOUBLE_EQ(state->cmd_vel.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(state->cmd_vel.linear_y, 0.5);
  EXPECT_DOUBLE_EQ(state->cmd_vel.angular_z, 0.2);
}

}  // namespace
}  // namespace stdr_simulation::world
