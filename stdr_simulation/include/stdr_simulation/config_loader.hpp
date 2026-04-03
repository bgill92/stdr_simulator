#pragma once

/** @file ROS-free YAML loaders for robot and map configuration files. */

#include <stdr_simulation/types.hpp>

#include <string>
#include <tl_expected/expected.hpp>

namespace stdr_simulation
{

/**
 * @brief Load a robot configuration from a YAML file.
 *
 * Parses the robot specification hierarchy: footprint, initial_pose,
 * laser sensors, sonar sensors, kinematic model, and other sensor types.
 * Sensors and kinematics that reference an external file via `filename:`
 * are loaded from `base_dir/filename` and then merged with any inline
 * `*_specifications` overrides.
 *
 * @param yaml_path  Absolute or relative path to the robot YAML file.
 * @param base_dir   Directory used to resolve `filename:` references
 *                   (typically the resources directory for the robot pack).
 * @return The assembled RobotConfig, or an error string describing the failure.
 */
[[nodiscard]] tl::expected<RobotConfig, std::string> load_robot_config(const std::string& yaml_path,
                                                                       const std::string& base_dir);

/**
 * @brief Load map metadata from a ROS-style map YAML file.
 *
 * Reads the map descriptor (image filename, resolution, origin, thresholds).
 * Does NOT load the image pixels — that is the caller's responsibility since
 * it requires an image library. The returned `image_path` is resolved to an
 * absolute path relative to the YAML file's directory.
 *
 * @param yaml_path  Path to the map YAML file.
 * @return MapMetadata with a resolved image path, or an error string describing the failure.
 */
[[nodiscard]] tl::expected<MapMetadata, std::string> load_map_metadata(const std::string& yaml_path);

}  // namespace stdr_simulation
