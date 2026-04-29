/** @file Map trace plotter: renders the occupancy grid as a background texture,
 *  draws the robot's footprint and heading arrow at its current pose, and
 *  overlays a breadcrumb trail colored by age (blue = oldest, red = latest).
 *
 *  Texture upload is performed in on_sample where SimView is available, so
 *  on_render only calls ImPlot::PlotImage with the pre-uploaded GL texture. */

#include <stdr_gui/grid_utils.hpp>
#include <stdr_gui/plot/helpers.hpp>
#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>

#include <imgui.h>
#include <implot.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <string_view>
#include <vector>

namespace
{

// Minimum Euclidean distance the robot must travel before a new breadcrumb is
// dropped.  Below this spacing, slow or stationary motion produces a dense,
// indistinguishable blob.
constexpr double kMarkerSpacingMeters = 0.1;

// Hard cap on stored breadcrumb markers so memory and per-frame render cost
// stay bounded regardless of how long the simulation runs.
constexpr std::size_t kMaxMarkers = 500;

// Length of the heading arrow drawn from the robot's origin, in metres.
constexpr double kHeadingArrowLen = 0.3;

// Length of the per-marker heading tick drawn in the breadcrumb loop, in metres.
// Kept shorter than the main heading arrow so individual ticks stay readable at
// typical map zoom levels without overlapping adjacent markers.
constexpr double kHeadingTickMeters = 0.08;

// Path line color: light grey at 70% opacity so the trail reads clearly
// against the map without drawing attention away from the per-marker gradient.
constexpr ImVec4 kPathColor(0.6f, 0.6f, 0.6f, 0.7f);

class MapTracePlotter : public stdr::plot::Plotter
{
public:
  MapTracePlotter() = default;

  ~MapTracePlotter() override
  {
    if (map_texture_id_ != 0)
    {
      glDeleteTextures(1, &map_texture_id_);
    }
  }

  [[nodiscard]] std::string_view name() const override
  {
    return "Map Trace";
  }

  [[nodiscard]] std::string_view description() const override
  {
    return "Map with robot pose and breadcrumb trail (latest = red, oldest = blue).";
  }

  void on_init(const stdr::plot::SimIntrospection& sim) override
  {
    // Cache the first robot ID at init time so on_sample does not re-query the
    // roster every frame.  If no robots exist yet, on_sample handles the retry.
    if (sim.num_robots() > 0)
    {
      robot_ = sim.robot_ids()[0];
    }
  }

  void on_sample(stdr::plot::SimView& sim, stdr::plot::PlotSink& /*out*/) override
  {
    // on_init may fire before any robot is spawned (PlotPanel is constructed
    // lazily on the first GUI frame), so retry here on the first call where a
    // robot is present.
    if (robot_.empty())
    {
      if (sim.num_robots() > 0)
      {
        robot_ = sim.robot_ids()[0];
      }
      else
      {
        return;
      }
    }

    // Cache the footprint once; the vertex list does not change at runtime in
    // the standalone simulator.  A single attempt is sufficient — if the
    // backend returns empty (circle-only robot or unimplemented footprint()),
    // the fallback dot rendering in on_render handles it without retrying
    // every frame.
    if (!footprint_fetched_)
    {
      footprint_robot_frame_ = sim.footprint(robot_);
      footprint_fetched_ = true;
    }

    // Upload the map texture here where SimView is available.  on_render only
    // calls PlotImage with the resulting GL texture handle.
    const stdr_simulation::OccupancyGrid& grid = sim.map();
    if (grid.width > 0 && grid.height > 0)
    {
      upload_map_texture(grid);
    }

    // Track the current pose for footprint and heading rendering.
    latest_pose_ = sim.pose(robot_);

    // Append to the breadcrumb trail if the robot has moved far enough.
    if (trail_.empty())
    {
      trail_.push_back(latest_pose_);
    }
    else
    {
      const stdr_simulation::Pose2D& last = trail_.back();
      const double dx = latest_pose_.x - last.x;
      const double dy = latest_pose_.y - last.y;
      if (std::hypot(dx, dy) >= kMarkerSpacingMeters)
      {
        // pop_front is O(1) on std::deque, avoiding the O(n) erase(begin())
        // that a std::vector-based trail would require.
        if (trail_.size() >= kMaxMarkers)
        {
          trail_.pop_front();
        }
        trail_.push_back(latest_pose_);
      }
    }
  }

  void on_render(const stdr::plot::PlotView& /*data*/) override
  {
    // When locked, disable all user interaction so the view cannot drift away
    // from the map.  ImPlotFlags_Equal preserves the 1:1 aspect ratio on both
    // axes regardless of the panel's aspect ratio.
    const ImPlotFlags plot_flags = ImPlotFlags_Equal | (lock_view_ ? ImPlotFlags_NoInputs : ImPlotFlags_None);

    if (ImPlot::BeginPlot("##map_trace", ImVec2(-1.0f, -1.0f), plot_flags))
    {
      // When locked, AutoFit on both axes lets ImPlot expand the smaller axis
      // to preserve the equal-aspect constraint rather than stretching the map.
      const ImPlotAxisFlags axis_flags = lock_view_ ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None;
      ImPlot::SetupAxes("x (m)", "y (m)", axis_flags, axis_flags);

      // Map texture as background.  UV coordinates flip the texture vertically
      // so that the ROS convention (y up) matches ImPlot's y-up plot space.
      if (map_texture_id_ != 0)
      {
        const ImPlotPoint bmin{ map_origin_x_, map_origin_y_ };
        const ImPlotPoint bmax{ map_origin_x_ + static_cast<double>(map_w_cached_) * map_resolution_,
                                map_origin_y_ + static_cast<double>(map_h_cached_) * map_resolution_ };
        // Cast the GL texture handle to ImTextureRef using the same pattern as
        // map_panel.cpp (stdr_gui/src/panels/map_panel.cpp:214).  The explicit
        // ImTextureRef wrap is required because IMGUI_HAS_TEXTURES is defined,
        // making PlotImage's second argument ImTextureRef rather than ImTextureID.
        ImPlot::PlotImage("##map", ImTextureRef(static_cast<ImTextureID>(map_texture_id_)), bmin, bmax,
                          ImVec2(0.0f, 1.0f),   // UV top-left: bottom of GL texture.
                          ImVec2(1.0f, 0.0f));  // UV bottom-right: top of GL texture.
      }

      // Rendered before the per-marker scatter loop so dots appear on top.
      if (trail_.size() > 1)
      {
        std::vector<double> xs;
        std::vector<double> ys;
        xs.reserve(trail_.size());
        ys.reserve(trail_.size());
        for (const stdr_simulation::Pose2D& p : trail_)
        {
          xs.push_back(p.x);
          ys.push_back(p.y);
        }
        ImPlot::PushStyleColor(ImPlotCol_Line, kPathColor);
        ImPlot::PlotLine("##path", xs.data(), ys.data(), static_cast<int>(xs.size()));
        ImPlot::PopStyleColor();
      }

      // One PlotScatter per point: ImPlot has no per-point color API, so the age
      // gradient (blue → red) requires per-call PushStyleColor.
      for (std::size_t i = 0; i < trail_.size(); ++i)
      {
        const float t = trail_.size() > 1 ? static_cast<float>(i) / static_cast<float>(trail_.size() - 1) : 1.0f;
        const ImVec4 color(t, 0.0f, 1.0f - t, 1.0f);
        ImPlot::PushStyleColor(ImPlotCol_MarkerFill, color);
        ImPlot::PushStyleColor(ImPlotCol_MarkerOutline, color);
        const double x = trail_[i].x;
        const double y = trail_[i].y;
        ImPlot::PlotScatter("##trail", &x, &y, 1);
        ImPlot::PopStyleColor(2);

        // Heading tick: short line from the marker center in the direction of
        // theta, using the same age-gradient color so the tick visually belongs
        // to its dot.
        const double tip_x = trail_[i].x + std::cos(trail_[i].theta) * kHeadingTickMeters;
        const double tip_y = trail_[i].y + std::sin(trail_[i].theta) * kHeadingTickMeters;
        const double tx[2] = { trail_[i].x, tip_x };
        const double ty[2] = { trail_[i].y, tip_y };
        ImPlot::PushStyleColor(ImPlotCol_Line, color);
        ImPlot::PlotLine("##tick", tx, ty, 2);
        ImPlot::PopStyleColor();
      }

      // Robot footprint outline at the current pose.
      if (!footprint_robot_frame_.empty())
      {
        // Inline 2D rigid transform from robot frame to map frame.  Using the
        // inline form rather than helpers::robot_to_map avoids constructing a
        // temporary Pose2D for each vertex.
        const double cx = std::cos(latest_pose_.theta);
        const double sx = std::sin(latest_pose_.theta);

        // Close the polygon by repeating the first vertex at the end.
        const std::size_t n = footprint_robot_frame_.size();
        std::vector<double> xs(n + 1);
        std::vector<double> ys(n + 1);
        for (std::size_t i = 0; i < n; ++i)
        {
          const double vx = footprint_robot_frame_[i].x;
          const double vy = footprint_robot_frame_[i].y;
          xs[i] = latest_pose_.x + cx * vx - sx * vy;
          ys[i] = latest_pose_.y + sx * vx + cx * vy;
        }
        xs[n] = xs[0];
        ys[n] = ys[0];

        ImPlot::SetNextLineStyle(ImVec4(0.0f, 0.8f, 0.0f, 1.0f), 2.0f);
        ImPlot::PlotLine("##footprint", xs.data(), ys.data(), static_cast<int>(n + 1));
      }
      else
      {
        // Circular or unknown footprint: render a single marker dot at the pose.
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 8.0f, ImVec4(0.0f, 0.8f, 0.0f, 1.0f), 2.0f,
                                   ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
        ImPlot::PlotScatter("##robot_dot", &latest_pose_.x, &latest_pose_.y, 1);
      }

      // Heading arrow: a 2-point line from the robot origin in the direction of
      // the current heading angle.
      {
        const double arr_x[2] = { latest_pose_.x, latest_pose_.x + std::cos(latest_pose_.theta) * kHeadingArrowLen };
        const double arr_y[2] = { latest_pose_.y, latest_pose_.y + std::sin(latest_pose_.theta) * kHeadingArrowLen };
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), 2.0f);
        ImPlot::PlotLine("##heading", arr_x, arr_y, 2);
      }

      ImPlot::EndPlot();
    }

    ImGui::Checkbox("Lock view to map", &lock_view_);
  }

private:
  /** Upload or re-upload the occupancy grid as a GL_RGBA texture.
   *
   *  Re-uploads only when the grid dimensions change or the data pointer
   *  changes, which is a cheap proxy for detecting map content updates since
   *  stdr_simulation replaces the data vector on each map load.
   *
   *  Conversion logic mirrors stdr_gui::occupancy_to_rgba (grid_utils.cpp). */
  void upload_map_texture(const stdr_simulation::OccupancyGrid& grid)
  {
    const bool dimensions_changed = (grid.width != map_w_cached_ || grid.height != map_h_cached_);
    const bool data_changed = (static_cast<const void*>(grid.data.data()) != map_data_cached_);

    if (!dimensions_changed && !data_changed)
    {
      return;
    }

    map_w_cached_ = grid.width;
    map_h_cached_ = grid.height;
    map_data_cached_ = static_cast<const void*>(grid.data.data());
    map_origin_x_ = grid.origin.x;
    map_origin_y_ = grid.origin.y;
    map_resolution_ = grid.resolution;

    const std::size_t pixel_count = static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    // Build an RGBA pixel buffer from occupancy values.  Conversion matches
    // stdr_gui/src/grid_utils.cpp:occupancy_to_rgba exactly: -1 → mid-grey
    // (128,128,128), 0 → white (255,255,255), 100 → black (0,0,0), 1-99 →
    // linear interpolation.
    std::vector<std::uint8_t> rgba(pixel_count * 4);
    for (std::size_t i = 0; i < pixel_count; ++i)
    {
      const std::int8_t val = grid.data[i];
      const stdr_gui::RgbaPixel pixel = stdr_gui::occupancy_to_rgba(val);
      std::memcpy(rgba.data() + i * 4, pixel.data(), 4);
    }

    if (map_texture_id_ == 0)
    {
      glGenTextures(1, &map_texture_id_);
    }

    glBindTexture(GL_TEXTURE_2D, map_texture_id_);
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

  std::string robot_;
  std::deque<stdr_simulation::Pose2D> trail_;
  std::vector<stdr_simulation::Point2D> footprint_robot_frame_;
  bool footprint_fetched_{ false };

  GLuint map_texture_id_{ 0 };
  int map_w_cached_{ 0 };
  int map_h_cached_{ 0 };
  const void* map_data_cached_{ nullptr };
  double map_origin_x_{ 0.0 };
  double map_origin_y_{ 0.0 };
  double map_resolution_{ 0.05 };

  stdr_simulation::Pose2D latest_pose_{};

  bool lock_view_{ true };
};

}  // namespace

REGISTER_PLOTTER(MapTracePlotter);
