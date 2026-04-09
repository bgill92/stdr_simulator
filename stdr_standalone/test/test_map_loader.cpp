#include "stdr_standalone/map_loader.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_simulation/types.hpp>
#include <tl_expected/expected.hpp>

#include <filesystem>
#include <fstream>
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

// Helper: write a minimal P5 (binary) PGM file.
// Row 0: white (255), Row 1: black (0), Row 2: gray (128), Row 3: white (255).
void write_test_pgm(const std::string& path)
{
  std::ofstream file(path, std::ios::binary);
  file << "P5\n4 4\n255\n";
  const unsigned char rows[4][4] = {
    { 255, 255, 255, 255 },
    { 0, 0, 0, 0 },
    { 128, 128, 128, 128 },
    { 255, 255, 255, 255 },
  };
  for (const auto& row : rows)
  {
    file.write(reinterpret_cast<const char*>(row), 4);
  }
}

void write_test_yaml(const std::string& yaml_path, const std::string& image_filename, double resolution, bool negate)
{
  std::ofstream file(yaml_path);
  file << "image: " << image_filename << "\n"
       << "resolution: " << resolution << "\n"
       << "origin: [0.0, 0.0, 0.0]\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.196\n"
       << "negate: " << (negate ? 1 : 0) << "\n";
}

TEST(MapLoader, LoadsPgmMap)
{
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_standalone_test_pgm";
  std::filesystem::create_directories(tmp_dir);

  write_test_pgm((tmp_dir / "test.pgm").string());
  write_test_yaml((tmp_dir / "test.yaml").string(), "test.pgm", 0.05, false);

  const auto result = load_map((tmp_dir / "test.yaml").string());
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto& grid = result.value();
  EXPECT_EQ(grid.width, 4);
  EXPECT_EQ(grid.height, 4);
  ASSERT_THAT(grid.data, SizeIs(16));

  std::filesystem::remove_all(tmp_dir);
}

TEST(MapLoader, PixelConversionWhiteToFreeBlackToOccupied)
{
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_standalone_test_pixels";
  std::filesystem::create_directories(tmp_dir);

  write_test_pgm((tmp_dir / "test.pgm").string());
  write_test_yaml((tmp_dir / "test.yaml").string(), "test.pgm", 0.05, false);

  const auto result = load_map((tmp_dir / "test.yaml").string());
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_THAT(result.value().data, SizeIs(16));

  const auto& data = result.value().data;
  // PGM rows: white(255), black(0), gray(128), white(255).
  // After vertical flip: OG row 0 = PGM row 3 (white), row 2 = PGM row 1 (black).
  EXPECT_EQ(data[0 * 4 + 0], 0);    // Row 0 (bottom) = white = free.
  EXPECT_EQ(data[1 * 4 + 0], -1);   // Row 1 = gray = unknown.
  EXPECT_EQ(data[2 * 4 + 0], 100);  // Row 2 = black = occupied.
  EXPECT_EQ(data[3 * 4 + 0], 0);    // Row 3 (top) = white = free.

  std::filesystem::remove_all(tmp_dir);
}

TEST(MapLoader, NegateFlipsMapping)
{
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_standalone_test_negate";
  std::filesystem::create_directories(tmp_dir);

  write_test_pgm((tmp_dir / "test.pgm").string());
  write_test_yaml((tmp_dir / "test.yaml").string(), "test.pgm", 0.05, true);

  const auto result = load_map((tmp_dir / "test.yaml").string());
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_THAT(result.value().data, SizeIs(16));

  const auto& data = result.value().data;
  // With negate, white (255) -> occ = pixel/255 = 1.0 -> occupied (100).
  // Black (0) -> occ = 0.0 -> free (0).
  EXPECT_EQ(data[0 * 4 + 0], 100);  // Row 0 (bottom) = white negated = occupied.
  EXPECT_EQ(data[2 * 4 + 0], 0);    // Row 2 = black negated = free.

  std::filesystem::remove_all(tmp_dir);
}

}  // namespace
}  // namespace stdr_standalone
