#pragma once

/** @file ROS-free map loading: YAML metadata + PNG/PGM image to OccupancyGrid. */

#include <stdr_simulation/types.hpp>
#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_standalone
{

/** @brief Load a map YAML file and its associated image into an OccupancyGrid.
 *  @param yaml_path Path to the map YAML file.
 *  @return Populated OccupancyGrid, or error string. */
[[nodiscard]] tl::expected<stdr_simulation::OccupancyGrid, std::string> load_map(const std::string& yaml_path);

}  // namespace stdr_standalone
