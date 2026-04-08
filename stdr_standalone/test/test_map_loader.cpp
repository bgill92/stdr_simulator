#include "stdr_standalone/map_loader.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_simulation/types.hpp>
#include <tl_expected/expected.hpp>

#include <string>

namespace stdr_standalone
{
namespace
{

using ::testing::Each;
using ::testing::Ge;
using ::testing::Le;
using ::testing::SizeIs;

constexpr auto kResourcesDir = STDR_RESOURCES_DIR;

std::string map_path()
{
  return std::string(kResourcesDir) + "/maps/maze1.yaml";
}

TEST(MapLoader, FailsOnMissingFile)
{
  const auto result = load_map("/nonexistent/map.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST(MapLoader, LoadsMaze1Successfully)
{
  const auto result = load_map(map_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto& grid = result.value();
  EXPECT_GT(grid.width, 0);
  EXPECT_GT(grid.height, 0);
  EXPECT_DOUBLE_EQ(grid.resolution, 0.003);
  ASSERT_THAT(grid.data, SizeIs(static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height)));
}

TEST(MapLoader, OccupancyValuesAreInRange)
{
  // Store the expected before accessing the value to avoid a dangling reference.
  const tl::expected<stdr_simulation::OccupancyGrid, std::string> result = load_map(map_path());
  ASSERT_TRUE(result.has_value());
  const stdr_simulation::OccupancyGrid& grid = result.value();

  for (const int8_t cell : grid.data)
  {
    EXPECT_TRUE(cell == 0 || cell == 100 || cell == -1) << "Unexpected cell value: " << static_cast<int>(cell);
  }
}

TEST(MapLoader, OriginMatchesYaml)
{
  const tl::expected<stdr_simulation::OccupancyGrid, std::string> result = load_map(map_path());
  ASSERT_TRUE(result.has_value());
  const stdr_simulation::OccupancyGrid& grid = result.value();

  EXPECT_DOUBLE_EQ(grid.origin.x, 0.0);
  EXPECT_DOUBLE_EQ(grid.origin.y, 0.0);
  EXPECT_DOUBLE_EQ(grid.origin.theta, 0.0);
}

}  // namespace
}  // namespace stdr_standalone
