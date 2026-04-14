#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_parser/parser.hpp>

#include <string>

namespace stdr_parser
{
namespace
{

using ::testing::HasSubstr;
using ::testing::SizeIs;

TEST(Parser, LoadRobotMsgReturnsErrorForMissingFile)
{
  const tl::expected<stdr_msgs::msg::RobotMsg, std::string> result =
      load_robot_msg("/nonexistent/path/robot.yaml", "/nonexistent/base", FIXTURE_DIR "/specifications");
  EXPECT_FALSE(result.has_value());
}

TEST(Parser, LoadRobotMsgReturnsValidationError)
{
  // invalid_robot_typo.yaml has "max_rnage" (typo) — validator must catch it.
  const tl::expected<stdr_msgs::msg::RobotMsg, std::string> result =
      load_robot_msg(FIXTURE_DIR "/invalid_robot_typo.yaml", FIXTURE_DIR, FIXTURE_DIR "/specifications");
  EXPECT_FALSE(result.has_value());
  EXPECT_THAT(result.error(), HasSubstr("max_rnage"));
}

TEST(Parser, LoadRobotMsgFromValidFile)
{
  const tl::expected<stdr_msgs::msg::RobotMsg, std::string> result =
      load_robot_msg(FIXTURE_DIR "/simple_robot.yaml", FIXTURE_DIR, FIXTURE_DIR "/specifications");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_NEAR(result->initial_pose.x, 3.0, 1e-5);
  EXPECT_NEAR(result->initial_pose.y, 2.0, 1e-5);
  ASSERT_THAT(result->laser_sensors, SizeIs(1));
  EXPECT_NEAR(result->laser_sensors[0].max_range, 4.09f, 1e-5);
  EXPECT_NEAR(result->footprint.radius, 0.05f, 1e-5);
  EXPECT_EQ(result->kinematic_model.type, "ideal");
}

TEST(Parser, LoadMapReturnsErrorForMissingFile)
{
  const tl::expected<stdr_simulation::MapMetadata, std::string> result = load_map("/nonexistent/path/map.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST(Parser, LoadMapReturnsMetadata)
{
  const tl::expected<stdr_simulation::MapMetadata, std::string> result = load_map(FIXTURE_DIR "/test_map.yaml");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_DOUBLE_EQ(result->resolution, 0.02);
  EXPECT_THAT(result->image_path, HasSubstr("sparse_obstacles.png"));
}

}  // namespace
}  // namespace stdr_parser
