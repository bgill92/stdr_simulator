#include <stdr_simulation/config_loader.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <tl_expected/expected.hpp>

#include <filesystem>
#include <fstream>
#include <string>

namespace stdr_simulation
{
namespace
{

// Resolves a fixture path relative to this test file's source directory.
// The CMakeLists.txt sets FIXTURE_DIR at compile time.
std::string fixture(const std::string& name)
{
  return std::string(FIXTURE_DIR) + "/" + name;
}

// ---- load_map_metadata -------------------------------------------------------

TEST(LoadMapMetadata, ValidMapReturnsMetadata)
{
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(fixture("test_map.yaml"));
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
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(fixture("test_map.yaml"));
  ASSERT_TRUE(meta.has_value());
  // The image path must be absolute and contain the fixture directory.
  EXPECT_TRUE(std::filesystem::path(meta->image_path).is_absolute());
  EXPECT_NE(meta->image_path.find("sparse_obstacles.png"), std::string::npos);
}

TEST(LoadMapMetadata, MissingFileReturnsError)
{
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata("/nonexistent/path/map.yaml");
  EXPECT_FALSE(meta.has_value());
  EXPECT_FALSE(meta.error().empty());
}

TEST(LoadMapMetadata, MissingImageKeyReturnsError)
{
  // Write a temp YAML without the image key.
  const std::string tmp = "/tmp/map_no_image.yaml";
  {
    std::ofstream f(tmp);
    f << "resolution: 0.05\norigin: [0, 0, 0]\n";
  }
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(tmp);
  EXPECT_FALSE(meta.has_value());
  EXPECT_FALSE(meta.error().empty());
}

TEST(LoadMapMetadata, MissingResolutionKeyReturnsError)
{
  const std::string tmp = "/tmp/map_no_resolution.yaml";
  {
    std::ofstream f(tmp);
    f << "image: map.png\norigin: [0, 0, 0]\n";
  }
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(tmp);
  EXPECT_FALSE(meta.has_value());
  EXPECT_FALSE(meta.error().empty());
}

TEST(LoadMapMetadata, NegateOneIsTrue)
{
  const std::string tmp = "/tmp/map_negate.yaml";
  {
    std::ofstream f(tmp);
    f << "image: map.png\nresolution: 0.05\nnegate: 1\n";
  }
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(tmp);
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
  const tl::expected<MapMetadata, std::string> meta = load_map_metadata(tmp);
  ASSERT_TRUE(meta.has_value());
  // Defaults from struct definition.
  EXPECT_DOUBLE_EQ(meta->occupied_thresh, 0.65);
  EXPECT_DOUBLE_EQ(meta->free_thresh, 0.196);
}

// ---- load_robot_config -------------------------------------------------------

TEST(LoadRobotConfig, SimpleRobotParsesInitialPose)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->initial_pose.x, 3.0);
  EXPECT_DOUBLE_EQ(config->initial_pose.y, 2.0);
  EXPECT_DOUBLE_EQ(config->initial_pose.theta, 1.57);
}

TEST(LoadRobotConfig, SimpleRobotParsesFootprintRadius)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_DOUBLE_EQ(config->footprint.radius, 0.05);
}

TEST(LoadRobotConfig, SimpleRobotLoadsLaserFromFile)
{
  const tl::expected<RobotConfig, std::string> config =
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
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  // The inline laser_specifications overrides pose.theta to -3.1415.
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].pose.theta, -3.1415);
}

TEST(LoadRobotConfig, SimpleRobotLoadsKinematic)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->kinematic_model.type, "ideal");
}

TEST(LoadRobotConfig, InlineOnlyLaserAndKinematic)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_inline_only.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].max_range, 8.0);
  EXPECT_EQ(config->kinematic_model.type, "omni");
}

TEST(LoadRobotConfig, LaserNoiseLoadedFromFile)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("simple_robot.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value());
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(1));
  EXPECT_TRUE(config->laser_sensors[0].noise.enabled);
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].noise.mean, 0.5);
  EXPECT_DOUBLE_EQ(config->laser_sensors[0].noise.std_dev, 0.05);
}

TEST(LoadRobotConfig, MissingFileReturnsError)
{
  const tl::expected<RobotConfig, std::string> config = load_robot_config("/nonexistent/robot.yaml", "/nonexistent");
  EXPECT_FALSE(config.has_value());
  EXPECT_FALSE(config.error().empty());
}

TEST(LoadRobotConfig, MissingRobotSpecificationsReturnsError)
{
  const std::string tmp = "/tmp/robot_bad.yaml";
  {
    std::ofstream f(tmp);
    f << "not_a_robot: true\n";
  }
  const tl::expected<RobotConfig, std::string> config = load_robot_config(tmp, "/tmp");
  EXPECT_FALSE(config.has_value());
  EXPECT_FALSE(config.error().empty());
}

TEST(LoadRobotConfig, BadSensorFilenameReturnsError)
{
  const std::string tmp = "/tmp/robot_bad_sensor.yaml";
  {
    std::ofstream f(tmp);
    f << "robot:\n"
         "  robot_specifications:\n"
         "    - laser:\n"
         "        filename: nonexistent_laser.yaml\n";
  }
  const tl::expected<RobotConfig, std::string> config = load_robot_config(tmp, "/tmp");
  EXPECT_FALSE(config.has_value());
  EXPECT_FALSE(config.error().empty());
}

TEST(LoadRobotConfig, FootprintPointsWrappedKeyParsed)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_polygon.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value()) << config.error();
  ASSERT_THAT(config->footprint.points, testing::SizeIs(4));
  EXPECT_DOUBLE_EQ(config->footprint.points[0].x, 0.1);
  EXPECT_DOUBLE_EQ(config->footprint.points[0].y, 0.2);
  EXPECT_DOUBLE_EQ(config->footprint.points[2].x, -0.1);
  EXPECT_DOUBLE_EQ(config->footprint.points[2].y, -0.2);
}

TEST(LoadRobotConfig, FootprintPointsBareKeyParsed)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_polygon_bare.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value()) << config.error();
  ASSERT_THAT(config->footprint.points, testing::SizeIs(3));
  EXPECT_DOUBLE_EQ(config->footprint.points[0].x, 0.3);
  EXPECT_DOUBLE_EQ(config->footprint.points[0].y, 0.4);
  EXPECT_DOUBLE_EQ(config->footprint.points[1].x, -0.3);
}

// ---- default frame_id assignment ---------------------------------------------

// A sensor without frame_id in the YAML gets a stable positional default so
// that publishing on <robot_name>/<frame_id> never produces an empty suffix.
TEST(LoadRobotConfig, SensorWithoutFrameIdGetsDefault)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_frame_id.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value()) << config.error();
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(2));
  EXPECT_EQ(config->laser_sensors[0].frame_id, "laser_0");
}

// A sensor that explicitly sets frame_id keeps its user-supplied value.
TEST(LoadRobotConfig, SensorWithExplicitFrameIdIsPreserved)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_frame_id.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value()) << config.error();
  ASSERT_THAT(config->laser_sensors, testing::SizeIs(2));
  EXPECT_EQ(config->laser_sensors[1].frame_id, "my_laser");
}

// The auto-index always increments so positional ordering is stable even when
// some sensors in the list have explicit names.
TEST(LoadRobotConfig, SonarAutoIndexIsStableAcrossMixedNames)
{
  const tl::expected<RobotConfig, std::string> config =
      load_robot_config(fixture("robot_frame_id.yaml"), std::string(FIXTURE_DIR));
  ASSERT_TRUE(config.has_value()) << config.error();
  ASSERT_THAT(config->sonar_sensors, testing::SizeIs(2));
  EXPECT_EQ(config->sonar_sensors[0].frame_id, "named_sonar");
  EXPECT_EQ(config->sonar_sensors[1].frame_id, "sonar_1");
}

}  // namespace
}  // namespace stdr_simulation
