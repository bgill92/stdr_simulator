#include <stdr_gui/grid_utils.hpp>
#include <stdr_gui/plot/map_texture.hpp>

#include <implot.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace stdr::plot
{

MapTexture::~MapTexture()
{
  if (texture_id_ != 0)
  {
    glDeleteTextures(1, &texture_id_);
  }
}

void MapTexture::update(const stdr_simulation::OccupancyGrid& grid)
{
  if (grid.width <= 0 || grid.height <= 0)
  {
    return;
  }

  const bool dimensions_changed = (grid.width != width_cached_ || grid.height != height_cached_);
  const bool data_changed = (static_cast<const void*>(grid.data.data()) != data_cached_);

  if (!dimensions_changed && !data_changed)
  {
    return;
  }

  width_cached_ = grid.width;
  height_cached_ = grid.height;
  data_cached_ = static_cast<const void*>(grid.data.data());
  origin_x_ = grid.origin.x;
  origin_y_ = grid.origin.y;
  resolution_ = grid.resolution;

  const std::size_t pixel_count = static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  // Build an RGBA pixel buffer from occupancy values. Conversion matches
  // stdr_gui/src/grid_utils.cpp:occupancy_to_rgba exactly: -1 -> mid-grey
  // (128,128,128), 0 -> white (255,255,255), 100 -> black (0,0,0), 1-99 ->
  // linear interpolation.
  std::vector<std::uint8_t> rgba(pixel_count * 4);
  for (std::size_t i = 0; i < pixel_count; ++i)
  {
    const std::int8_t val = grid.data[i];
    const stdr_gui::RgbaPixel pixel = stdr_gui::occupancy_to_rgba(val);
    std::memcpy(rgba.data() + i * 4, pixel.data(), 4);
  }

  if (texture_id_ == 0)
  {
    glGenTextures(1, &texture_id_);
  }

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  if (dimensions_changed)
  {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, grid.width, grid.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  }
  else
  {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, grid.width, grid.height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  }

  glBindTexture(GL_TEXTURE_2D, 0);
}

void MapTexture::plot(const char* label_id) const
{
  if (texture_id_ == 0)
  {
    return;
  }

  const ImPlotPoint bmin{ origin_x_, origin_y_ };
  const ImPlotPoint bmax{ origin_x_ + static_cast<double>(width_cached_) * resolution_,
                          origin_y_ + static_cast<double>(height_cached_) * resolution_ };
  // Cast the GL texture handle to ImTextureRef using the same pattern as
  // map_panel.cpp (stdr_gui/src/panels/map_panel.cpp:214). The explicit
  // ImTextureRef wrap is required because IMGUI_HAS_TEXTURES is defined,
  // making PlotImage's second argument ImTextureRef rather than ImTextureID.
  ImPlot::PlotImage(label_id, ImTextureRef(static_cast<ImTextureID>(texture_id_)), bmin, bmax,
                    ImVec2(0.0f, 1.0f),   // UV top-left: bottom of GL texture. UV coordinates flip the
                                          // texture vertically so ROS convention (y up) matches ImPlot's
                                          // y-up plot space.
                    ImVec2(1.0f, 0.0f));  // UV bottom-right: top of GL texture.
}

}  // namespace stdr::plot
