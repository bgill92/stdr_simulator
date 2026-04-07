#pragma once

/** @file Map loading utilities for STDR server. */

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_server
{

/** @brief Load a map YAML file and its associated image into an OccupancyGrid.
 *  @param yaml_path Path to the map YAML file.
 *  @return OccupancyGrid message, or error string. */
[[nodiscard]] tl::expected<nav_msgs::msg::OccupancyGrid, std::string> load_map(const std::string& yaml_path);

}  // namespace stdr_server
