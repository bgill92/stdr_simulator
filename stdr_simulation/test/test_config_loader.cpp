#include <stdr_simulation/config_loader.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

namespace stdr_simulation {
namespace {

// Resolves a fixture path relative to this test file's source directory.
// The CMakeLists.txt sets FIXTURE_DIR at compile time.
std::string fixture(const std::string& name)
{
  return std::string(FIXTURE_DIR) + "/" + name;
}

// ---- load_map_metadata -------------------------------------------------------

TEST(LoadMapMetadata, ValidMapReturnsMetadata)
{
  const std::optional<MapMetadata> meta = load_map_metadata(fixture("test_map.yaml"));
  ASSERT_TRUE(meta.has_value());
  EXPECT_DOUBLE_EQ(meta->resolution, 0.02);
  EXPECT_DOUBLE_EQ(meta->origin.x, 0.0);
  EXPECT_DOUBLE_EQ(meta->origin.y, 0.0);
  EXPECT_DOUBLE_EQ(meta->origin.theta, 0.0);
  EXPECT_DOUBLE_EQ(meta->occupied_thresh, 0.6);
  EXPECT_DOUBLE_EQ(meta->free_thresh, 0.3);
  EXPECT_FALSE(meta->negate);
}

TEST(LoadMapMetadata, ImagePathIsResolved)
{
  const std::optional<MapMetadata> meta = load_map_metadata(fixture("test_map.yaml"));
  ASSERT_TRUE(meta.has_value());
  // The image path must be absolute and contain the fixture directory.
  EXPECT_TRUE(std::filesystem::path(meta->image_path).is_absolute());
  EXPECT_NE(meta->image_path.find("sparse_obstacles.png"), std::string::npos);
}

TEST(LoadMapMetadata, MissingFileReturnsNullopt)
{
  const std::optional<MapMetadata> meta = load_map_metadata("/nonexistent/path/map.yaml");
  EXPECT_FALSE(meta.has_value());
}

TEST(LoadMapMetadata, MissingImageKeyReturnsNullopt)
{
  // Write a temp YAML without the image key.
  const std::string tmp = "/tmp/map_no_image.yaml";
  {
    std::ofstream f(tmp);
    f << "resolution: 0.05\norigin: [0, 0, 0]\n";
  }
  const std::optional<MapMetadata> meta = load_map_metadata(tmp);
  EXPECT_FALSE(meta.has_value());
}

TEST(LoadMapMetadata, MissingResolutionKeyReturnsNullopt)
{
  const std::string tmp = "/tmp/map_no_resolution.yaml";
  {
    std::ofstream f(tmp);
    f << "image: map.png\norigin: [0, 0, 0]\n";
  }
  const std::optional<MapMetadata> meta = load_map_metadata(tmp);
  EXPECT_FALSE(meta.has_value());
}

TEST(LoadMapMetadata, NegateOneIsTrue)
{
  const std::string tmp = "/tmp/map_negate.yaml";
  {
    std::ofstream f(tmp);
    f << "image: map.png\nresolution: 0.05\nnegate: 1\n";
  }
  const std::optional<MapMetadata> meta = load_map_metadata(tmp);
  ASSERT_TRUE(meta.has_value());
  EXPECT_TRUE(meta->negate);
}

TEST(LoadMapMetadata, DefaultThresholdsWhenAbsent)
{
  const std::string tmp = "/tmp/map_defaults.yaml";
  {
    std::ofstream f(tmp);
    f << "image: map.png\nresolution: 0.05\n";
  }
  const std::optional<MapMetadata> meta = load_map_metadata(tmp);
  ASSERT_TRUE(meta.has_value());
  // Defaults from struct definition.
  EXPECT_DOUBLE_EQ(meta->occupied_thresh, 0.65);
  EXPECT_DOUBLE_EQ(meta->free_thresh, 0.196);
}

// ---- load_robot_config -------------------------------------------------------

TEST(LoadRobotConfig, SimpleRobotParsesInitialPose)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->initial_pose.x, 3.0);
  EXPECT_DOUBLE_EQ(config->initial_pose.y, 2.0);
  EXPECT_DOUBLE_EQ(config->initial_pose.theta, 1.57);
}

TEST(LoadRobotConfig, SimpleRobotParsesFootprintRadius)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->footprint.radius, 0.05);
}

TEST(LoadRobotConfig, SimpleRobotLoadsLaserFromFile)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  // These come from the external laser file.
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].max_range, 4.09);
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].min_range, 0.06);
  EXPECT_EQ(config->laser_sensors[0].num_rays, 667);
}

TEST(LoadRobotConfig, SimpleRobotInlinePoseOverridesFilepose)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  // The inline laser_specifications overrides pose.theta to -3.1415.
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].pose.theta, -3.1415);
}

TEST(LoadRobotConfig, SimpleRobotLoadsKinematic)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->kinematic_model.type, "ideal");
}

TEST(LoadRobotConfig, InlineOnlyLaserAndKinematic)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("robot_inline_only.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].max_range, 8.0);
  EXPECT_EQ(config->kinematic_model.type, "omni");
}

TEST(LoadRobotConfig, LaserNoiseLoadedFromFile)
{
  const std::optional<RobotConfig> config =
    load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  EXPECT_TRUE(config->laser_sensors[0].noise.enabled);
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].noise.mean, 0.5);
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].noise.std_dev, 0.05);
}

TEST(LoadRobotConfig, MissingFileReturnsNullopt)
{
  const std::optional<RobotConfig> config =
    load_robot_config("/nonexistent/robot.yaml", "/nonexistent");
  EXPECT_FALSE(config.has_value());
}

TEST(LoadRobotConfig, MissingRobotSpecificationsReturnsNullopt)
{
  const std::string tmp = "/tmp/robot_bad.yaml";
  {
    std::ofstream f(tmp);
    f << "not_a_robot: true\n";
  }
  const std::optional<RobotConfig> config = load_robot_config(tmp, "/tmp");
  EXPECT_FALSE(config.has_value());
}

TEST(LoadRobotConfig, BadSensorFilenameReturnsNullopt)
{
  const std::string tmp = "/tmp/robot_bad_sensor.yaml";
  {
    std::ofstream f(tmp);
    f << "robot:\n"
         "  robot_specifications:\n"
         "    - laser:\n"
         "        filename: nonexistent_laser.yaml\n";
  }
  const std::optional<RobotConfig> config = load_robot_config(tmp, "/tmp");
  EXPECT_FALSE(config.has_value());
}

}  // namespace
}  // namespace stdr_simulation
