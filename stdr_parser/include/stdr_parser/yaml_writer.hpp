#pragma once

/** @file Serialize ROS2 messages to YAML files. */

#include <stdr_msgs/msg/robot_msg.hpp>

#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_parser
{

/** @brief Write a RobotMsg to a YAML file.
 *  The output follows the stdr specification schema format so it can be
 *  validated and parsed back by load_robot_msg(). */
[[nodiscard]] tl::expected<void, std::string> write_robot_yaml(const stdr_msgs::msg::RobotMsg& msg,
                                                               const std::string& file_path);

}  // namespace stdr_parser
