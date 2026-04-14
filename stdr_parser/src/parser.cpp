#include <stdr_parser/msg_conversions.hpp>
#include <stdr_parser/parser.hpp>
#include <stdr_parser/yaml_validator.hpp>
#include <stdr_simulation/config_loader.hpp>

#include <yaml-cpp/yaml.h>

#include <numeric>
#include <string>

namespace stdr_parser
{

tl::expected<stdr_msgs::msg::RobotMsg, std::string>
load_robot_msg(const std::string& yaml_path, const std::string& base_dir, const std::string& specs_dir)
{
  // Step 1: Parse YAML — return an error if the file cannot be opened.
  YAML::Node node;
  try
  {
    node = YAML::LoadFile(yaml_path);
  }
  catch (const YAML::Exception& e)
  {
    return tl::unexpected<std::string>(e.what());
  }

  // Step 2: Validate the robot body (not the full document wrapper) against spec.
  // Accumulate all errors for diagnostics.
  const ValidationResult validation = validate_yaml(node["robot"], "robot", specs_dir);
  if (!validation.valid)
  {
    const std::string joined =
        std::accumulate(std::next(validation.errors.begin()), validation.errors.end(), validation.errors.front(),
                        [](const std::string& a, const std::string& b) { return a + "\n" + b; });
    return tl::unexpected(joined);
  }

  // Step 3: Load the structured config from YAML.
  tl::expected<stdr_simulation::RobotConfig, std::string> config =
      stdr_simulation::load_robot_config(yaml_path, base_dir);
  if (!config)
  {
    return tl::unexpected(config.error());
  }

  // Step 4: Convert to ROS message and return.
  return to_ros_msg(*config);
}

tl::expected<stdr_simulation::MapMetadata, std::string> load_map(const std::string& yaml_path)
{
  return stdr_simulation::load_map_metadata(yaml_path);
}

}  // namespace stdr_parser
