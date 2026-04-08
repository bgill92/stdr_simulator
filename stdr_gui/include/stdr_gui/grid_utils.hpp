#pragma once

#include <array>
#include <cstdint>

namespace stdr_gui
{

/** @brief RGBA pixel value (red, green, blue, alpha). */
using RgbaPixel = std::array<std::uint8_t, 4>;

/** @brief Convert an occupancy grid cell value to an RGBA pixel.
 *
 *  Mapping: -1 = unknown (mid-grey), 0 = free (white), 100 = occupied (black),
 *  1-99 = linear interpolation from white to black. All pixels are fully opaque.
 *
 *  @pre value is in the range [-1, 100] (asserts in debug builds). */
[[nodiscard]] RgbaPixel occupancy_to_rgba(std::int8_t value);

}  // namespace stdr_gui
