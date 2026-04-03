#pragma once

/** @file High-level API: validate, load, and convert YAML configs. */

#include <stdr_simulation/types.hpp>

#include <stdr_msgs/msg/robot_msg.hpp>

#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_parser {

/** @brief Load a robot YAML file and convert to a ROS2 message.
 *
 *  Pipeline: validate YAML → load via config_loader → convert to ROS msg.
 *
 *  @param yaml_path  Path to the robot YAML file.
 *  @param base_dir   Directory for resolving filename references.
 *  @param specs_dir  Directory containing specification YAML files.
 *  @return RobotMsg, or error string describing what went wrong. */
[[nodiscard]] tl::expected<stdr_msgs::msg::RobotMsg, std::string> load_robot_msg(
    const std::string& yaml_path,
    const std::string& base_dir,
    const std::string& specs_dir);

/** @brief Load map metadata from a YAML file.
 *  @param yaml_path Path to the map YAML file.
 *  @return MapMetadata, or error string. */
[[nodiscard]] tl::expected<stdr_simulation::MapMetadata, std::string> load_map(
    const std::string& yaml_path);

}  // namespace stdr_parser
