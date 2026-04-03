#include <stdr_parser/yaml_validator.hpp>

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace stdr_parser {
namespace {

/** @brief Per-type field constraints loaded from stdr_specifications.yaml. */
struct FieldSpec {
  std::set<std::string> allowed;
  std::set<std::string> required;
};

using Specs = std::unordered_map<std::string, FieldSpec>;

/** @brief Join a set of strings with ", " for human-readable error messages. */
[[nodiscard]] std::string join_set(const std::set<std::string>& items) {
  std::string result;
  for (const std::string& item : items) {
    if (!result.empty()) {
      result += ", ";
    }
    result += item;
  }
  return result;
}

/** @brief Load FieldSpec entries from stdr_specifications.yaml.
 *
 *  Only entries that declare `allowed` or `required` become specs.
 *  Leaf entries (e.g. x: {default: 0}) have no allowed/required and are
 *  intentionally skipped — they are scalar fields, not composite types. */
[[nodiscard]] Specs load_specs(const std::string& specs_dir) {
  const YAML::Node root =
      YAML::LoadFile(specs_dir + "/stdr_specifications.yaml");
  const YAML::Node specs_node = root["specifications"];

  Specs specs;
  for (const auto& entry : specs_node) {
    const std::string type_name = entry.first.as<std::string>();
    const YAML::Node& body = entry.second;

    if (!body.IsMap()) {
      continue;
    }

    const bool has_allowed = static_cast<bool>(body["allowed"]);
    const bool has_required = static_cast<bool>(body["required"]);

    if (!has_allowed && !has_required) {
      // Leaf field with only a default value — no structural rules to enforce.
      continue;
    }

    FieldSpec spec;
    if (has_allowed) {
      for (const YAML::Node& key_node : body["allowed"]) {
        spec.allowed.insert(key_node.as<std::string>());
      }
    }
    if (has_required) {
      for (const YAML::Node& key_node : body["required"]) {
        spec.required.insert(key_node.as<std::string>());
      }
    }
    specs[type_name] = std::move(spec);
  }
  return specs;
}

/** @brief Load the set of type names that may appear multiple times in a
 *  sequence (e.g. laser, sonar). */
[[nodiscard]] std::set<std::string> load_multiple_allowed(
    const std::string& specs_dir) {
  const YAML::Node root =
      YAML::LoadFile(specs_dir + "/stdr_multiple_allowed.yaml");
  const YAML::Node list = root["specifications"];

  std::set<std::string> result;
  for (const YAML::Node& item : list) {
    result.insert(item.as<std::string>());
  }
  return result;
}

// Forward declaration so validate_map and validate_sequence can call each other.
void validate_node(const YAML::Node& node, const std::string& context_type,
                   const std::string& path, const Specs& specs,
                   const std::set<std::string>& multiple_allowed,
                   ValidationResult& result);

/** @brief Validate a map node: check allowed keys, required keys, and recurse. */
void validate_map(const YAML::Node& node, const std::string& context_type,
                  const std::string& path, const Specs& specs,
                  const std::set<std::string>& multiple_allowed,
                  ValidationResult& result) {
  const Specs::const_iterator it = specs.find(context_type);
  if (it != specs.end()) {
    const FieldSpec& spec = it->second;

    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      const YAML::Node& val_node = it->second;
      if (!spec.allowed.empty() && !spec.allowed.contains(key)) {
        result.valid = false;
        result.errors.push_back("Unknown key '" + key + "' at " + path +
                                " (allowed: " + join_set(spec.allowed) + ")");
      }
      // Recurse using the key as the new context type.
      validate_node(val_node, key, path + "." + key, specs, multiple_allowed,
                    result);
    }

    // Skip required-field enforcement when a filename reference is present —
    // the referenced file is expected to supply the missing fields.
    if (!node["filename"]) {
      for (const std::string& req : spec.required) {
        if (!node[req]) {
          result.valid = false;
          result.errors.push_back("Missing required key '" + req + "' at " +
                                  path);
        }
      }
    }
  }
  // If context_type is not in specs it is an unknown/leaf — no rules to apply.
}

/** @brief Validate a sequence node by validating each element in turn. */
void validate_sequence(const YAML::Node& node, const std::string& context_type,
                       const std::string& path, const Specs& specs,
                       const std::set<std::string>& multiple_allowed,
                       ValidationResult& result) {
  for (std::size_t i = 0; i < node.size(); ++i) {
    validate_node(node[i], context_type,
                  path + "[" + std::to_string(i) + "]", specs,
                  multiple_allowed, result);
  }
}

void validate_node(const YAML::Node& node, const std::string& context_type,
                   const std::string& path, const Specs& specs,
                   const std::set<std::string>& multiple_allowed,
                   ValidationResult& result) {
  if (node.IsMap()) {
    validate_map(node, context_type, path, specs, multiple_allowed, result);
  } else if (node.IsSequence()) {
    validate_sequence(node, context_type, path, specs, multiple_allowed,
                      result);
  }
  // Scalar nodes carry no structural constraints.
}

}  // namespace

ValidationResult validate_yaml(const YAML::Node& yaml_node,
                                const std::string& root_type,
                                const std::string& specs_dir) {
  ValidationResult result;

  if (!yaml_node.IsDefined() || yaml_node.IsNull()) {
    // An undefined or null node has no keys to validate.
    return result;
  }

  const Specs specs = load_specs(specs_dir);
  const std::set<std::string> multiple_allowed =
      load_multiple_allowed(specs_dir);

  validate_node(yaml_node, root_type, root_type, specs, multiple_allowed,
                result);
  return result;
}

}  // namespace stdr_parser
