#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <stdr_parser/parser.hpp>
#include <stdr_parser/yaml_writer.hpp>

#include <stdr_msgs/msg/co2_sensor_msg.hpp>
#include <stdr_msgs/msg/laser_sensor_msg.hpp>
#include <stdr_msgs/msg/rfid_sensor_msg.hpp>
#include <stdr_msgs/msg/robot_msg.hpp>
#include <stdr_msgs/msg/sonar_sensor_msg.hpp>
#include <stdr_msgs/msg/sound_sensor_msg.hpp>
#include <stdr_msgs/msg/thermal_sensor_msg.hpp>

#include <filesystem>
#include <string>

namespace stdr_parser
{
namespace
{

using ::testing::SizeIs;

// Build a minimal but complete RobotMsg for testing.
stdr_msgs::msg::RobotMsg make_test_robot()
{
  stdr_msgs::msg::RobotMsg msg;
  msg.initial_pose.x = 3.0;
  msg.initial_pose.y = 2.0;
  msg.initial_pose.theta = 1.57;

  msg.footprint.radius = 0.05f;

  stdr_msgs::msg::LaserSensorMsg laser;
  laser.max_angle = 1.57f;
  laser.min_angle = -1.57f;
  laser.max_range = 4.0f;
  laser.min_range = 0.1f;
  laser.num_rays = 180;
  laser.noise.noise_mean = 0.0f;
  laser.noise.noise_std = 0.01f;
  laser.frequency = 10.0f;
  laser.frame_id = "laser_0";
  msg.laser_sensors.push_back(laser);

  msg.kinematic_model.type = "ideal";

  return msg;
}

TEST(YamlWriter, WriteToInvalidPathReturnsError)
{
  const stdr_msgs::msg::RobotMsg msg = make_test_robot();
  const tl::expected<void, std::string> result = write_robot_yaml(msg, "/nonexistent/dir/robot.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST(YamlWriter, WriteRobotYamlCreatesFile)
{
  const stdr_msgs::msg::RobotMsg msg = make_test_robot();
  const std::string path = "/tmp/test_robot_out.yaml";
  const tl::expected<void, std::string> result = write_robot_yaml(msg, path);
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_GT(std::filesystem::file_size(path), 0u);
}

TEST(YamlWriter, WrittenYamlIsValidAndParseable)
{
  const stdr_msgs::msg::RobotMsg msg = make_test_robot();
  const std::string path = "/tmp/test_robot_parseable.yaml";
  std::ignore = write_robot_yaml(msg, path);

  const YAML::Node doc = YAML::LoadFile(path);
  ASSERT_TRUE(doc.IsMap());
  ASSERT_TRUE(doc["robot"].IsDefined());
  ASSERT_TRUE(doc["robot"]["robot_specifications"].IsDefined());
  EXPECT_TRUE(doc["robot"]["robot_specifications"].IsSequence());
}

TEST(YamlWriter, RoundTripPreservesValues)
{
  const stdr_msgs::msg::RobotMsg original = make_test_robot();
  const std::string path = "/tmp/test_robot_roundtrip.yaml";
  std::ignore = write_robot_yaml(original, path);

  const tl::expected<stdr_msgs::msg::RobotMsg, std::string> loaded =
      load_robot_msg(path, "/tmp", FIXTURE_DIR "/specifications");
  ASSERT_TRUE(loaded.has_value()) << loaded.error();

  EXPECT_NEAR(loaded->initial_pose.x, 3.0, 1e-4);
  EXPECT_NEAR(loaded->initial_pose.y, 2.0, 1e-4);
  EXPECT_NEAR(loaded->initial_pose.theta, 1.57, 1e-4);
  EXPECT_NEAR(loaded->footprint.radius, 0.05f, 1e-4);
  ASSERT_THAT(loaded->laser_sensors, SizeIs(1));
  EXPECT_NEAR(loaded->laser_sensors[0].max_range, 4.0f, 1e-4);
  EXPECT_NEAR(loaded->laser_sensors[0].min_range, 0.1f, 1e-4);
  EXPECT_EQ(loaded->kinematic_model.type, "ideal");
}

TEST(YamlWriter, RoundTripAllSensorTypes)
{
  stdr_msgs::msg::RobotMsg original;
  original.initial_pose.x = 1.0;
  original.initial_pose.y = 2.0;
  original.initial_pose.theta = 0.0;
  original.footprint.radius = 0.1f;
  original.kinematic_model.type = "ideal";

  stdr_msgs::msg::LaserSensorMsg laser;
  laser.frame_id = "laser_0";
  laser.max_range = 5.0f;
  original.laser_sensors.push_back(laser);

  stdr_msgs::msg::SonarSensorMsg sonar;
  sonar.frame_id = "sonar_0";
  sonar.max_range = 3.0f;
  original.sonar_sensors.push_back(sonar);

  stdr_msgs::msg::RfidSensorMsg rfid;
  rfid.frame_id = "rfid_0";
  rfid.max_range = 2.0f;
  original.rfid_sensors.push_back(rfid);

  stdr_msgs::msg::CO2SensorMsg co2;
  co2.frame_id = "co2_0";
  co2.max_range = 1.5f;
  original.co2_sensors.push_back(co2);

  stdr_msgs::msg::ThermalSensorMsg thermal;
  thermal.frame_id = "thermal_0";
  thermal.max_range = 2.5f;
  thermal.angle_span = 180.0f;
  original.thermal_sensors.push_back(thermal);

  stdr_msgs::msg::SoundSensorMsg sound;
  sound.frame_id = "sound_0";
  sound.max_range = 4.0f;
  sound.angle_span = 360.0f;
  original.sound_sensors.push_back(sound);

  const std::string path = "/tmp/test_robot_all_sensors.yaml";
  std::ignore = write_robot_yaml(original, path);

  const tl::expected<stdr_msgs::msg::RobotMsg, std::string> loaded =
      load_robot_msg(path, "/tmp", FIXTURE_DIR "/specifications");
  ASSERT_TRUE(loaded.has_value()) << loaded.error();

  ASSERT_THAT(loaded->laser_sensors, SizeIs(1));
  ASSERT_THAT(loaded->sonar_sensors, SizeIs(1));
  ASSERT_THAT(loaded->rfid_sensors, SizeIs(1));
  ASSERT_THAT(loaded->co2_sensors, SizeIs(1));
  ASSERT_THAT(loaded->thermal_sensors, SizeIs(1));
  ASSERT_THAT(loaded->sound_sensors, SizeIs(1));
}

}  // namespace
}  // namespace stdr_parser
