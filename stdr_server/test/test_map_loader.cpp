#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_server/map_loader.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace stdr_server
{
namespace
{

using ::testing::HasSubstr;
using ::testing::SizeIs;

// Write a minimal P5 (binary) PGM file to the given path.
// Layout:
//   Row 0: all white (255) → free space
//   Row 1: all black (0)   → occupied
//   Row 2: mid-gray (128)  → unknown
//   Row 3: all white (255) → free space
void write_test_pgm(const std::string& path)
{
  std::ofstream file(path, std::ios::binary);
  // P5 header: magic, width, height, maxval, then binary pixel data.
  file << "P5\n4 4\n255\n";
  const unsigned char rows[4][4] = {
    { 255, 255, 255, 255 },  // white → free
    { 0, 0, 0, 0 },          // black → occupied
    { 128, 128, 128, 128 },  // gray  → unknown
    { 255, 255, 255, 255 },  // white → free
  };
  for (const auto& row : rows)
  {
    file.write(reinterpret_cast<const char*>(row), 4);
  }
}

// Write a map YAML that references the given image filename (basename only).
// The caller is responsible for placing the image in the same directory as the YAML.
void write_test_yaml(const std::string& yaml_path, const std::string& image_filename, double resolution,
                     double origin_x, double origin_y, double origin_theta)
{
  std::ofstream file(yaml_path);
  file << "image: " << image_filename << "\n"
       << "resolution: " << resolution << "\n"
       << "origin: [" << origin_x << ", " << origin_y << ", " << origin_theta << "]\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.196\n"
       << "negate: 0\n";
}

// ─── Error path ──────────────────────────────────────────────────────────────

TEST(MapLoader, InvalidPathReturnsError)
{
  const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> result = load_map("/nonexistent/path/map.yaml");
  EXPECT_FALSE(result.has_value());
}

// ─── Valid PGM map ───────────────────────────────────────────────────────────

TEST(MapLoader, LoadsPgmMap)
{
  // Write temporary fixtures into the system temp directory to avoid
  // polluting the source tree and to ensure the test is self-contained.
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_test_map_loader";
  std::filesystem::create_directories(tmp_dir);

  const std::string pgm_path = (tmp_dir / "test_map.pgm").string();
  const std::string yaml_path = (tmp_dir / "test_map.yaml").string();

  write_test_pgm(pgm_path);
  write_test_yaml(yaml_path, "test_map.pgm", 0.05, 0.0, 0.0, 0.0);

  const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> result = load_map(yaml_path);

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_EQ(result->info.width, 4u);
  EXPECT_EQ(result->info.height, 4u);
  EXPECT_FLOAT_EQ(result->info.resolution, 0.05f);
  ASSERT_THAT(result->data, SizeIs(16));

  std::filesystem::remove_all(tmp_dir);
}

TEST(MapLoader, LoadsMapWithCorrectOrigin)
{
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_test_map_origin";
  std::filesystem::create_directories(tmp_dir);

  const std::string pgm_path = (tmp_dir / "test_map.pgm").string();
  const std::string yaml_path = (tmp_dir / "test_map.yaml").string();

  write_test_pgm(pgm_path);
  // Origin: x=1.0, y=2.0, theta=0.0 → quaternion (0, 0, 0, 1).
  write_test_yaml(yaml_path, "test_map.pgm", 0.05, 1.0, 2.0, 0.0);

  const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> result = load_map(yaml_path);

  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_DOUBLE_EQ(result->info.origin.position.x, 1.0);
  EXPECT_DOUBLE_EQ(result->info.origin.position.y, 2.0);
  EXPECT_DOUBLE_EQ(result->info.origin.position.z, 0.0);
  // theta = 0 → no rotation, quaternion is identity.
  EXPECT_NEAR(result->info.origin.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(result->info.origin.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(result->info.origin.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(result->info.origin.orientation.w, 1.0, 1e-9);

  std::filesystem::remove_all(tmp_dir);
}

TEST(MapLoader, PixelConversion)
{
  // The PGM has rows: white(255), black(0), gray(128), white(255).
  // map_loader flips vertically, so OccupancyGrid row 0 is PGM row 3 (white).
  // OccupancyGrid indexing: data[row * width + col], row 0 is bottom of map.
  //
  // After vertical flip:
  //   OccupancyGrid row 0 (y=0, bottom) ← PGM row 3 (white) → free (0)
  //   OccupancyGrid row 1               ← PGM row 2 (gray)  → unknown (-1)
  //   OccupancyGrid row 2               ← PGM row 1 (black) → occupied (100)
  //   OccupancyGrid row 3 (y=3, top)    ← PGM row 0 (white) → free (0)
  const std::filesystem::path tmp_dir = std::filesystem::temp_directory_path() / "stdr_test_map_pixels";
  std::filesystem::create_directories(tmp_dir);

  const std::string pgm_path = (tmp_dir / "test_map.pgm").string();
  const std::string yaml_path = (tmp_dir / "test_map.yaml").string();

  write_test_pgm(pgm_path);
  write_test_yaml(yaml_path, "test_map.pgm", 0.05, 0.0, 0.0, 0.0);

  const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> result = load_map(yaml_path);
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_THAT(result->data, SizeIs(16));

  // Row 0 (bottom after flip) ← white pixels → free (0)
  EXPECT_EQ(result->data[0 * 4 + 0], 0);

  // Row 1 ← gray (128) → unknown (-1)
  EXPECT_EQ(result->data[1 * 4 + 0], -1);

  // Row 2 ← black (0) → occupied (100)
  EXPECT_EQ(result->data[2 * 4 + 0], 100);

  // Row 3 (top after flip) ← white → free (0)
  EXPECT_EQ(result->data[3 * 4 + 0], 0);

  std::filesystem::remove_all(tmp_dir);
}

}  // namespace
}  // namespace stdr_server
