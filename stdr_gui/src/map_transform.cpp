#include <stdr_gui/map_transform.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace stdr_gui
{

void MapTransform::set_map_info(const double origin_x, const double origin_y, const double resolution,
                                const std::int32_t width, const std::int32_t height)
{
  // Callers must provide a positive resolution; zero or negative would
  // produce division-by-zero in every subsequent transform call.
  assert(resolution > 0.0);
  map_origin_x_ = origin_x;
  map_origin_y_ = origin_y;
  map_resolution_ = resolution;
  map_width_ = width;
  map_height_ = height;
}

ScreenPoint MapTransform::world_to_screen(const double wx, const double wy) const
{
  // Convert from world to map-pixel coordinates.
  const double px = (wx - map_origin_x_) / map_resolution_;
  const double py = (wy - map_origin_y_) / map_resolution_;

  // Flip Y: ROS +Y is up, screen +Y is down.
  const double flipped_py = static_cast<double>(map_height_ - 1) - py;

  return ScreenPoint{
    .x = static_cast<float>(px * zoom_ + offset_x_),
    .y = static_cast<float>(flipped_py * zoom_ + offset_y_),
  };
}

std::pair<double, double> MapTransform::screen_to_world(const float sx, const float sy) const
{
  // Invert zoom and pan to get map-pixel coordinates.
  const double px = (static_cast<double>(sx) - offset_x_) / zoom_;
  const double flipped_py = (static_cast<double>(sy) - offset_y_) / zoom_;

  // Invert the Y flip.
  const double py = static_cast<double>(map_height_ - 1) - flipped_py;

  // Convert from map-pixel to world coordinates.
  const double wx = px * map_resolution_ + map_origin_x_;
  const double wy = py * map_resolution_ + map_origin_y_;

  return { wx, wy };
}

void MapTransform::pan(const float dx, const float dy)
{
  offset_x_ += dx;
  offset_y_ += dy;
}

void MapTransform::zoom(const float screen_x, const float screen_y, const float factor)
{
  const float new_zoom = std::clamp(zoom_ * factor, kMinZoom, kMaxZoom);
  const float actual_factor = new_zoom / zoom_;

  // Adjust the offset so the point under the cursor stays fixed after the
  // zoom change: offset_new = screen_pivot - actual_factor * (screen_pivot - offset_old).
  offset_x_ = screen_x - actual_factor * (screen_x - offset_x_);
  offset_y_ = screen_y - actual_factor * (screen_y - offset_y_);

  zoom_ = new_zoom;
}

void MapTransform::fit_to_view(const float viewport_width, const float viewport_height)
{
  if (map_width_ <= 0 || map_height_ <= 0)
  {
    return;
  }

  // Choose the zoom that fits the entire map within the viewport.
  const float zoom_x = viewport_width / static_cast<float>(map_width_);
  const float zoom_y = viewport_height / static_cast<float>(map_height_);
  zoom_ = std::clamp(std::min(zoom_x, zoom_y), kMinZoom, kMaxZoom);

  // Center the map in the viewport.
  const float map_screen_w = static_cast<float>(map_width_) * zoom_;
  const float map_screen_h = static_cast<float>(map_height_) * zoom_;
  offset_x_ = (viewport_width - map_screen_w) / 2.0f;
  offset_y_ = (viewport_height - map_screen_h) / 2.0f;
}

float MapTransform::get_zoom() const
{
  return zoom_;
}

ScreenPoint MapTransform::get_offset() const
{
  return ScreenPoint{ .x = offset_x_, .y = offset_y_ };
}

double MapTransform::get_resolution() const
{
  return map_resolution_;
}

}  // namespace stdr_gui
