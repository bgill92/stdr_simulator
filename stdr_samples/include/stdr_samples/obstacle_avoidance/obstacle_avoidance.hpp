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
#pragma once

#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
@namespace stdr_samples
@brief The main namespace for STDR Samples.
**/
namespace stdr_samples
{

/**
@class ObstacleAvoidance
@brief Performs potential-field obstacle avoidance for a single robot.

Subscribes to a laser scan and publishes Twist velocity commands that steer
the robot away from obstacles using a sum-of-cosines/sines repulsion field.
**/
class ObstacleAvoidance : public rclcpp::Node
{
public:
  /**
  @brief Constructor.
  @param robot_frame_id The robot frame id used to build the topic names.
  @param laser_frame_id The laser frame id appended to form the scan topic.
  **/
  ObstacleAvoidance(const std::string& robot_frame_id, const std::string& laser_frame_id);

private:
  /**
  @brief Callback for incoming laser scan messages.

  Computes the potential-field repulsion vector from the scan, clamps the
  linear component to ±0.3, and publishes the resulting Twist command.
  @param msg The incoming laser scan message.
  **/
  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace stdr_samples
