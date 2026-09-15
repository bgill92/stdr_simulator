#pragma once

/** @file String conversions for OdometryModel, used by the YAML config loader
 *  and the ROS msg conversion layer (stdr_parser). Kept separate from
 *  types.hpp so that the core data types stay free of parsing concerns. */

#include <stdr_simulation/types.hpp>

#include <string>
#include <string_view>
#include <tl_expected/expected.hpp>

namespace stdr_simulation
{

/**
 * @brief Parse an `odometry_model` YAML value into an OdometryModel.
 * @param value  Expected to be "perfect" or "velocity".
 * @return The parsed OdometryModel, or an error string listing allowed values.
 */
[[nodiscard]] tl::expected<OdometryModel, std::string> parse_odometry_model(std::string_view value);

/**
 * @brief Render an OdometryModel back to its YAML string form.
 * @return "perfect" or "velocity".
 */
[[nodiscard]] std::string_view to_string(OdometryModel model);

}  // namespace stdr_simulation
