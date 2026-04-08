#pragma once

#include <cstdint>
#include <utility>

namespace stdr_gui
{

/** @brief 2D screen coordinate. */
struct ScreenPoint
{
  float x{ 0.0f };
  float y{ 0.0f };
};

/** @brief Manages world-to-screen coordinate transforms for the 2D map view.
 *
 *  The map uses the ROS convention: +X right, +Y up, origin at map_origin.
 *  Screen convention: +X right, +Y down, origin at top-left of viewport.
 *  The transform applies the map's resolution, origin offset, zoom, and pan.
 *
 *  @warning Not thread-safe. All calls must occur on the same thread
 *           (typically the render thread). */
class MapTransform
{
public:
  static constexpr float kMinZoom = 0.01f;
  static constexpr float kMaxZoom = 100.0f;
  /** @brief Set the map metadata from an OccupancyGrid.
   *  @pre resolution > 0. Calling with resolution <= 0 is undefined behaviour
   *       (asserts in debug builds). */
  void set_map_info(double origin_x, double origin_y, double resolution, std::int32_t width, std::int32_t height);

  /** @brief Convert world coordinates to screen pixel coordinates. */
  [[nodiscard]] ScreenPoint world_to_screen(double wx, double wy) const;

  /** @brief Convert screen pixel coordinates to world coordinates. */
  [[nodiscard]] std::pair<double, double> screen_to_world(float sx, float sy) const;

  /** @brief Apply a pan offset in screen pixels. */
  void pan(float dx, float dy);

  /** @brief Zoom centered on a screen point.
   *  @param screen_x Screen x-coordinate of the zoom pivot.
   *  @param screen_y Screen y-coordinate of the zoom pivot.
   *  @param factor Multiplicative zoom factor (>1 = zoom in). */
  void zoom(float screen_x, float screen_y, float factor);

  /** @brief Reset view to fit the entire map in the viewport. */
  void fit_to_view(float viewport_width, float viewport_height);

  /** @brief Get the current zoom level. */
  [[nodiscard]] float get_zoom() const;

  /** @brief Get the current pan offset in screen pixels. */
  [[nodiscard]] ScreenPoint get_offset() const;

private:
  double map_origin_x_{ 0.0 };
  double map_origin_y_{ 0.0 };
  // Default to 1.0 so dimensions and resolution form a coherent zero-size map
  // before set_map_info is called.
  double map_resolution_{ 1.0 };
  std::int32_t map_width_{ 0 };
  std::int32_t map_height_{ 0 };
  float zoom_{ 1.0f };
  float offset_x_{ 0.0f };
  float offset_y_{ 0.0f };
};

}  // namespace stdr_gui
