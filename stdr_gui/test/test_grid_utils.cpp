#include <stdr_gui/grid_utils.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace stdr_gui
{
namespace
{

// ---- Boundary values --------------------------------------------------------

TEST(OccupancyToRgbaTest, UnknownCellIsGrey)
{
  const RgbaPixel pixel = occupancy_to_rgba(-1);
  EXPECT_EQ(pixel[0], 128);
  EXPECT_EQ(pixel[1], 128);
  EXPECT_EQ(pixel[2], 128);
  EXPECT_EQ(pixel[3], 255);
}

TEST(OccupancyToRgbaTest, FreeCellIsWhite)
{
  const RgbaPixel pixel = occupancy_to_rgba(0);
  EXPECT_EQ(pixel[0], 255);
  EXPECT_EQ(pixel[1], 255);
  EXPECT_EQ(pixel[2], 255);
  EXPECT_EQ(pixel[3], 255);
}

TEST(OccupancyToRgbaTest, OccupiedCellIsBlack)
{
  const RgbaPixel pixel = occupancy_to_rgba(100);
  EXPECT_EQ(pixel[0], 0);
  EXPECT_EQ(pixel[1], 0);
  EXPECT_EQ(pixel[2], 0);
  EXPECT_EQ(pixel[3], 255);
}

// ---- Interpolation ----------------------------------------------------------

TEST(OccupancyToRgbaTest, HalfOccupiedIsMidGrey)
{
  // value=50: 255 - (50*255/100) = 255 - 127 = 128.
  const RgbaPixel pixel = occupancy_to_rgba(50);
  EXPECT_EQ(pixel[0], 128);
  EXPECT_EQ(pixel[1], 128);
  EXPECT_EQ(pixel[2], 128);
  EXPECT_EQ(pixel[3], 255);
}

TEST(OccupancyToRgbaTest, AllPixelsFullyOpaque)
{
  for (const std::int8_t value : { static_cast<std::int8_t>(-1), static_cast<std::int8_t>(0),
                                   static_cast<std::int8_t>(50), static_cast<std::int8_t>(100) })
  {
    const RgbaPixel pixel = occupancy_to_rgba(value);
    EXPECT_EQ(pixel[3], 255) << "Alpha not 255 for value " << static_cast<int>(value);
  }
}

TEST(OccupancyToRgbaTest, InterpolationIsMonotonic)
{
  // As occupancy increases from 1 to 99, brightness (R channel) should be
  // non-increasing — darker for higher occupancy.
  std::uint8_t prev = occupancy_to_rgba(1)[0];
  for (std::int8_t v = 2; v <= 99; ++v)
  {
    const std::uint8_t current = occupancy_to_rgba(v)[0];
    EXPECT_LE(current, prev) << "Brightness increased from value " << static_cast<int>(v - 1) << " to "
                             << static_cast<int>(v);
    prev = current;
  }
}

}  // namespace
}  // namespace stdr_gui
