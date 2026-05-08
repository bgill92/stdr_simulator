/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA

   Authors :
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com
******************************************************************************/
#include "stdr_samples/obstacle_avoidance/obstacle_avoidance.hpp"

#include <algorithm>
#include <cmath>

namespace stdr_samples
{

ObstacleAvoidance::ObstacleAvoidance(const std::string& robot_frame_id, const std::string& laser_frame_id)
  : rclcpp::Node("obstacle_avoidance")
{
  const std::string laser_topic = "/" + robot_frame_id + "/" + laser_frame_id;
  const std::string speeds_topic = "/" + robot_frame_id + "/cmd_vel";

  subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      laser_topic, rclcpp::QoS(1), [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) { on_scan(msg); });

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(speeds_topic, rclcpp::QoS(1));
}

void ObstacleAvoidance::on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  const std::size_t n = msg->ranges.size();
  if (n == 0)
  {
    RCLCPP_ERROR(get_logger(), "Received empty laser scan — skipping.");
    return;
  }

  float linear = 0.0F;
  float rotational = 0.0F;

  // Closer obstacles contribute higher weight — inverse-square in distance.
  for (std::size_t i = 0; i < n; ++i)
  {
    const float real_dist = msg->ranges[i];
    const float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
    const float weight = 1.0F / (1.0F + real_dist * real_dist);
    linear -= std::cos(angle) * weight;
    rotational -= std::sin(angle) * weight;
  }

  linear /= static_cast<float>(n);
  rotational /= static_cast<float>(n);

  // Clamp the linear component so the robot cannot reverse too aggressively.
  linear = std::clamp(linear, -0.3F, 0.3F);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.3 + static_cast<double>(linear);
  cmd.angular.z = static_cast<double>(rotational);
  cmd_vel_pub_->publish(cmd);
}

}  // namespace stdr_samples
