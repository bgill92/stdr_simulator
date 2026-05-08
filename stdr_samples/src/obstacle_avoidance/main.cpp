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
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "stdr_samples/obstacle_avoidance/obstacle_avoidance.hpp"

/**
@brief Entry point. Parses robot and laser frame IDs then spins the node.
@param argc Number of command-line arguments.
@param argv Command-line argument values.
@return EXIT_SUCCESS on clean shutdown, EXIT_FAILURE on bad usage.
**/
int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: obstacle_avoidance <robot_frame_id> <laser_frame_id>\n";
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);

  const std::string robot_frame_id{ argv[1] };
  const std::string laser_frame_id{ argv[2] };

  rclcpp::spin(std::make_shared<stdr_samples::ObstacleAvoidance>(robot_frame_id, laser_frame_id));

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
