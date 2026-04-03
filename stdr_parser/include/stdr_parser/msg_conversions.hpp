#pragma once

#include <stdr_simulation/types.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <stdr_msgs/msg/co2_sensor_msg.hpp>
#include <stdr_msgs/msg/co2_source.hpp>
#include <stdr_msgs/msg/footprint_msg.hpp>
#include <stdr_msgs/msg/kinematic_msg.hpp>
#include <stdr_msgs/msg/laser_sensor_msg.hpp>
#include <stdr_msgs/msg/noise.hpp>
#include <stdr_msgs/msg/rfid_sensor_msg.hpp>
#include <stdr_msgs/msg/rfid_tag.hpp>
#include <stdr_msgs/msg/robot_msg.hpp>
#include <stdr_msgs/msg/sonar_sensor_msg.hpp>
#include <stdr_msgs/msg/sound_sensor_msg.hpp>
#include <stdr_msgs/msg/sound_source.hpp>
#include <stdr_msgs/msg/thermal_sensor_msg.hpp>
#include <stdr_msgs/msg/thermal_source.hpp>

namespace stdr_parser {

// --- Primitives ---
[[nodiscard]] geometry_msgs::msg::Pose2D to_ros_msg(const stdr_simulation::Pose2D& pose);
[[nodiscard]] stdr_simulation::Pose2D from_ros_msg(const geometry_msgs::msg::Pose2D& msg);

[[nodiscard]] geometry_msgs::msg::Point to_ros_point(const stdr_simulation::Point2D& pt);
[[nodiscard]] stdr_simulation::Point2D from_ros_point(const geometry_msgs::msg::Point& msg);

[[nodiscard]] stdr_msgs::msg::Noise to_ros_msg(const stdr_simulation::NoiseConfig& noise);
[[nodiscard]] stdr_simulation::NoiseConfig from_ros_msg(const stdr_msgs::msg::Noise& msg);

[[nodiscard]] stdr_msgs::msg::FootprintMsg to_ros_msg(const stdr_simulation::Footprint& fp);
[[nodiscard]] stdr_simulation::Footprint from_ros_msg(const stdr_msgs::msg::FootprintMsg& msg);

[[nodiscard]] stdr_msgs::msg::KinematicMsg to_ros_msg(const stdr_simulation::KinematicConfig& kin);
[[nodiscard]] stdr_simulation::KinematicConfig from_ros_msg(const stdr_msgs::msg::KinematicMsg& msg);

// --- Sensors ---
[[nodiscard]] stdr_msgs::msg::LaserSensorMsg to_ros_msg(const stdr_simulation::LaserConfig& cfg);
[[nodiscard]] stdr_simulation::LaserConfig from_ros_msg(const stdr_msgs::msg::LaserSensorMsg& msg);

[[nodiscard]] stdr_msgs::msg::SonarSensorMsg to_ros_msg(const stdr_simulation::SonarConfig& cfg);
[[nodiscard]] stdr_simulation::SonarConfig from_ros_msg(const stdr_msgs::msg::SonarSensorMsg& msg);

[[nodiscard]] stdr_msgs::msg::RfidSensorMsg to_ros_msg(const stdr_simulation::RfidSensorConfig& cfg);
[[nodiscard]] stdr_simulation::RfidSensorConfig from_ros_msg(const stdr_msgs::msg::RfidSensorMsg& msg);

[[nodiscard]] stdr_msgs::msg::CO2SensorMsg to_ros_msg(const stdr_simulation::CO2SensorConfig& cfg);
[[nodiscard]] stdr_simulation::CO2SensorConfig from_ros_msg(const stdr_msgs::msg::CO2SensorMsg& msg);

[[nodiscard]] stdr_msgs::msg::SoundSensorMsg to_ros_msg(const stdr_simulation::SoundSensorConfig& cfg);
[[nodiscard]] stdr_simulation::SoundSensorConfig from_ros_msg(const stdr_msgs::msg::SoundSensorMsg& msg);

[[nodiscard]] stdr_msgs::msg::ThermalSensorMsg to_ros_msg(const stdr_simulation::ThermalSensorConfig& cfg);
[[nodiscard]] stdr_simulation::ThermalSensorConfig from_ros_msg(const stdr_msgs::msg::ThermalSensorMsg& msg);

// --- Top-level ---
[[nodiscard]] stdr_msgs::msg::RobotMsg to_ros_msg(const stdr_simulation::RobotConfig& cfg);
[[nodiscard]] stdr_simulation::RobotConfig from_ros_msg(const stdr_msgs::msg::RobotMsg& msg);

// --- Environment sources ---
[[nodiscard]] stdr_msgs::msg::RfidTag to_ros_msg(const stdr_simulation::RfidTag& tag);
[[nodiscard]] stdr_simulation::RfidTag from_ros_msg(const stdr_msgs::msg::RfidTag& msg);

[[nodiscard]] stdr_msgs::msg::CO2Source to_ros_msg(const stdr_simulation::CO2Source& src);
[[nodiscard]] stdr_simulation::CO2Source from_ros_msg(const stdr_msgs::msg::CO2Source& msg);

[[nodiscard]] stdr_msgs::msg::SoundSource to_ros_msg(const stdr_simulation::SoundSource& src);
[[nodiscard]] stdr_simulation::SoundSource from_ros_msg(const stdr_msgs::msg::SoundSource& msg);

[[nodiscard]] stdr_msgs::msg::ThermalSource to_ros_msg(const stdr_simulation::ThermalSource& src);
[[nodiscard]] stdr_simulation::ThermalSource from_ros_msg(const stdr_msgs::msg::ThermalSource& msg);

}  // namespace stdr_parser
