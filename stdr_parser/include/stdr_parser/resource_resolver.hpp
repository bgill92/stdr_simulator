#pragma once

/** @file Resolve resource file paths via ament_index_cpp. */

#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_parser
{

/** @brief Get the share directory for the stdr_resources package.
 *  @return Absolute path to the installed stdr_resources share directory,
 *          or an error string if the package is not found. */
[[nodiscard]] tl::expected<std::string, std::string> get_resources_dir();

/** @brief Resolve a resource file path relative to stdr_resources/resources/.
 *  @param relative_path Path relative to the resources/ subdirectory.
 *  @return Absolute path, or error if package not found or file doesn't exist. */
[[nodiscard]] tl::expected<std::string, std::string> resolve_resource_path(const std::string& relative_path);

/** @brief Get the path to the specifications directory.
 *  @return Absolute path to resources/specifications/. */
[[nodiscard]] tl::expected<std::string, std::string> get_specifications_dir();

}  // namespace stdr_parser
