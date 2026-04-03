#pragma once

/** @file Validate YAML configuration against the stdr specification schema. */

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace stdr_parser {

/** @brief Accumulated validation errors. */
struct ValidationResult {
  bool valid{true};
  std::vector<std::string> errors;
};

/** @brief Validate a YAML node against the stdr specification schema.
 *
 *  Loads stdr_specifications.yaml and stdr_multiple_allowed.yaml from
 *  specs_dir, then recursively checks the given YAML node for:
 *  - Disallowed keys (typos, unknown fields)
 *  - Missing required keys
 *
 *  @param yaml_node  The parsed YAML::Node to validate.
 *  @param root_type  The top-level entity type ("robot", "laser", etc.).
 *  @param specs_dir  Directory containing stdr_specifications.yaml.
 *  @return ValidationResult with all errors accumulated. */
[[nodiscard]] ValidationResult validate_yaml(
    const YAML::Node& yaml_node,
    const std::string& root_type,
    const std::string& specs_dir);

}  // namespace stdr_parser
