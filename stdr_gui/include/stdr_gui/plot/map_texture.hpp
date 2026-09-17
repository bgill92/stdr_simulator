#pragma once

/** @file Shared GL-texture helper for plotters that draw the occupancy grid
 *  as a map background (Map Trace, Odometry Trace).
 *
 *  Extracted from map_trace_plotter.cpp so both plotters upload and render
 *  the map identically instead of duplicating ~60 lines of GL/ImPlot code. */

#include <stdr_simulation/types.hpp>

namespace stdr::plot
{

/** @brief RAII wrapper around a single GL_RGBA texture holding the occupancy
 *  grid, plus the ImPlot::PlotImage call to draw it at its world-frame
 *  bounds.
 *
 *  `update()` must be called from a context where a GL context is current
 *  (i.e. from `on_sample`/`on_render` on the GUI thread) — the same
 *  requirement the original MapTracePlotter upload code had.
 *
 *  @warning NOT thread-safe, and not copyable or movable: the texture handle
 *           is unique GL state owned by this instance. */
class MapTexture
{
public:
  MapTexture() = default;
  ~MapTexture();

  MapTexture(const MapTexture&) = delete;
  MapTexture& operator=(const MapTexture&) = delete;
  MapTexture(MapTexture&&) = delete;
  MapTexture& operator=(MapTexture&&) = delete;

  /** Upload or re-upload the occupancy grid as a GL_RGBA texture.
   *
   *  Re-uploads only when the grid dimensions change or the data pointer
   *  changes, which is a cheap proxy for detecting map content updates since
   *  stdr_simulation replaces the data vector on each map load. No-ops if
   *  the grid is empty (zero width or height). */
  void update(const stdr_simulation::OccupancyGrid& grid);

  /** Draw the uploaded texture over its world-frame bounds via
   *  `ImPlot::PlotImage`. Must be called between `ImPlot::BeginPlot` and
   *  `ImPlot::EndPlot`. No-op if nothing has been uploaded yet.
   *
   *  @param label_id  ImPlot item label/ID passed to PlotImage (e.g. "##map"). */
  void plot(const char* label_id) const;

private:
  unsigned int texture_id_{ 0 };
  int width_cached_{ 0 };
  int height_cached_{ 0 };
  const void* data_cached_{ nullptr };
  double origin_x_{ 0.0 };
  double origin_y_{ 0.0 };
  double resolution_{ 0.05 };
};

}  // namespace stdr::plot
