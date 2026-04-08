#include <stdr_gui/grid_utils.hpp>

#include <cassert>

namespace stdr_gui
{

RgbaPixel occupancy_to_rgba(const std::int8_t value)
{
  assert(value >= -1 && value <= 100);
  if (value == -1)
  {
    return { 128, 128, 128, 255 };  // Unknown: mid-grey.
  }
  if (value == 0)
  {
    return { 255, 255, 255, 255 };  // Free: white.
  }
  if (value == 100)
  {
    return { 0, 0, 0, 255 };  // Occupied: black.
  }
  // Partially occupied: linear interpolation from white (0) to black (100).
  const std::uint8_t v = static_cast<std::uint8_t>(255 - (value * 255 / 100));
  return { v, v, v, 255 };
}

}  // namespace stdr_gui
