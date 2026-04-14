#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <stdr_parser/yaml_validator.hpp>

namespace stdr_parser
{
namespace
{

using ::testing::IsEmpty;
using ::testing::Not;

// FIXTURE_DIR is injected by CMakeLists at compile time.
constexpr std::string_view kFixtureDir{ FIXTURE_DIR };
constexpr std::string_view kSpecsDir{ FIXTURE_DIR "/specifications" };

TEST(YamlValidator, ValidRobotPasses)
{
  const YAML::Node doc = YAML::LoadFile(std::string{ kFixtureDir } + "/valid_robot.yaml");
  const ValidationResult result = validate_yaml(doc["robot"], "robot", std::string{ kSpecsDir });

  EXPECT_TRUE(result.valid);
  EXPECT_THAT(result.errors, IsEmpty());
}

TEST(YamlValidator, UnknownKeyDetected)
{
  const YAML::Node doc = YAML::LoadFile(std::string{ kFixtureDir } + "/invalid_robot_typo.yaml");
  const ValidationResult result = validate_yaml(doc["robot"], "robot", std::string{ kSpecsDir });

  EXPECT_FALSE(result.valid);
  ASSERT_THAT(result.errors, Not(IsEmpty()));
  // The error must mention the misspelled key.
  bool found = false;
  for (const std::string& err : result.errors)
  {
    if (err.find("max_rnage") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected an error mentioning 'max_rnage'";
}

TEST(YamlValidator, MissingRequiredKeyDetected)
{
  const YAML::Node doc = YAML::LoadFile(std::string{ kFixtureDir } + "/invalid_robot_missing_required.yaml");
  const ValidationResult result = validate_yaml(doc["robot"], "robot", std::string{ kSpecsDir });

  EXPECT_FALSE(result.valid);
  ASSERT_THAT(result.errors, Not(IsEmpty()));
  // The error must mention the missing required field.
  bool found = false;
  for (const std::string& err : result.errors)
  {
    if (err.find("laser_specifications") != std::string::npos)
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected an error mentioning 'laser_specifications'";
}

TEST(YamlValidator, MultipleLasersAllowed)
{
  // laser is listed in stdr_multiple_allowed.yaml, so two lasers in one robot
  // should pass validation without errors.
  const YAML::Node doc = YAML::LoadFile(std::string{ kFixtureDir } + "/robot_multiple_lasers.yaml");
  const ValidationResult result = validate_yaml(doc["robot"], "robot", std::string{ kSpecsDir });

  EXPECT_TRUE(result.valid);
  EXPECT_THAT(result.errors, IsEmpty());
}

TEST(YamlValidator, ErrorsIncludePathContext)
{
  const YAML::Node doc = YAML::LoadFile(std::string{ kFixtureDir } + "/invalid_robot_typo.yaml");
  const ValidationResult result = validate_yaml(doc["robot"], "robot", std::string{ kSpecsDir });

  ASSERT_FALSE(result.valid);
  ASSERT_THAT(result.errors, Not(IsEmpty()));
  // Every error string should contain path separators indicating nesting depth.
  bool has_path = false;
  for (const std::string& err : result.errors)
  {
    if (err.find('.') != std::string::npos)
    {
      has_path = true;
      break;
    }
  }
  EXPECT_TRUE(has_path) << "Expected at least one error to include a dotted path";
}

TEST(YamlValidator, EmptyNodeReturnsValid)
{
  // An undefined node for an unknown type has no spec, so validation passes.
  const YAML::Node empty_node;
  const ValidationResult result = validate_yaml(empty_node, "unknown_type", std::string{ kSpecsDir });

  EXPECT_TRUE(result.valid);
  EXPECT_THAT(result.errors, IsEmpty());
}

}  // namespace
}  // namespace stdr_parser
