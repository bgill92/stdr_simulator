/** @file Scan Trace plotter: Odometry Trace's map + breadcrumb trails, plus
 *  the ability to click any breadcrumb and see the laser scan captured there,
 *  placed either from the ground-truth pose or the odometry belief pose at
 *  that same breadcrumb.
 *
 *  Truth and odometry breadcrumbs are stored together in a single sample
 *  history (see `Sample` below) so that clicking either trail's marker at
 *  index i can replay the same scan from either pose. */

#include <stdr_gui/plot/helpers.hpp>
#include <stdr_gui/plot/map_texture.hpp>
#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>
#include <stdr_gui/plot/trail.hpp>

#include <imgui.h>
#include <implot.h>

#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{

// A same-sample truth-pose jump larger than this is a teleport or a drag in
// the GUI, not motion — drawing a line across it would be misleading, so the
// sample history is cleared instead of connecting the old and new positions.
constexpr double kTeleportThresholdMeters = 1.0;

// Maximum pixel distance from a click to a breadcrumb marker for that marker
// to be considered "hit" — beyond this the click is treated as a miss and
// clears the current selection.
constexpr float kPickRadiusPixels = 8.0f;

constexpr ImVec4 kTruthColor(0.0f, 0.8f, 0.0f, 1.0f);
constexpr ImVec4 kOdomColor(1.0f, 0.55f, 0.0f, 1.0f);
constexpr ImVec4 kScanColor(0.2f, 0.8f, 1.0f, 1.0f);

class ScanTracePlotter : public stdr::plot::Plotter
{
public:
  ScanTracePlotter() = default;

  [[nodiscard]] std::string_view name() const override
  {
    return "Scan Trace";
  }

  [[nodiscard]] std::string_view description() const override
  {
    return "Odometry Trace plus the laser scan at any clicked breadcrumb, placed from the truth or odometry pose.";
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

  void on_reset(stdr::plot::SimView& /*sim*/) override
  {
    // Drop the sample history, the selection, and the teleport-detector's last
    // pose so the next on_sample does not draw a line from the pre-reset pose
    // to spawn, nor mistake the reset teleport itself for a drag needing
    // another clear.
    samples_.clear();
    selected_.reset();
    last_truth_.reset();
  }

  void on_sample(stdr::plot::SimView& sim, stdr::plot::PlotSink& out) override
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

    // Cache the footprint once; it does not change at runtime.  See
    // map_trace_plotter.cpp for the rationale behind a single attempt.
    if (!footprint_fetched_)
    {
      footprint_robot_frame_ = sim.footprint(robot_);
      footprint_fetched_ = true;
    }

    // Cache the first laser sensor and its mount pose once, the same way as
    // the footprint above.  A robot with no laser sensor keeps laser_ empty
    // forever — on_render shows a disabled hint instead of retrying.
    if (!laser_fetched_)
    {
      const std::vector<stdr::plot::SensorId> lasers = sim.laser_sensors(robot_);
      if (!lasers.empty())
      {
        laser_ = lasers[0];
        laser_in_robot_ = sim.laser_pose(robot_, laser_).value_or(stdr_simulation::Pose2D{});
      }
      laser_fetched_ = true;
    }

    map_texture_.update(sim.map());

    const stdr_simulation::Pose2D truth = sim.pose(robot_);
    const stdr_simulation::Pose2D odom = sim.odom_pose(robot_);

    // Detect a teleport/drag between consecutive samples (not the spacing-
    // gated last stored sample, which may lag several calls behind) and clear
    // the sample history so it does not draw a line across the map.
    if (last_truth_.has_value() &&
        std::hypot(truth.x - last_truth_->x, truth.y - last_truth_->y) > kTeleportThresholdMeters)
    {
      samples_.clear();
      selected_.reset();
    }
    last_truth_ = truth;
    latest_odom_ = odom;

    // Store a sample when the trail is empty or the robot has moved far
    // enough from the last stored sample's truth pose — mirrors
    // Trail::push_if_moved's spacing gate.
    const bool should_push =
        samples_.empty() || std::hypot(truth.x - samples_.back().truth.x, truth.y - samples_.back().truth.y) >=
                                stdr::plot::kMarkerSpacingMeters;
    if (should_push)
    {
      Sample sample;
      sample.truth = truth;
      sample.odom = odom;
      if (!laser_.empty())
      {
        sample.scan = sim.latest_laser(robot_, laser_).value_or(stdr_simulation::LaserScan{});
      }

      // pop_front is O(1) on std::deque, avoiding the O(n) erase(begin()) a
      // std::vector-based history would require.  See Trail::push_if_moved.
      if (samples_.size() >= stdr::plot::kMaxMarkers)
      {
        samples_.pop_front();
        if (selected_.has_value())
        {
          if (selected_->index == 0)
          {
            selected_.reset();
          }
          else
          {
            --selected_->index;
          }
        }
      }
      samples_.push_back(std::move(sample));
    }

    const double t = sim.sim_time();
    out.scalar("err_xy", t, std::hypot(truth.x - odom.x, truth.y - odom.y));
    out.scalar("err_theta", t, stdr::plot::helpers::wrapped_angle_diff(truth.theta, odom.theta));
  }

  void on_render(const stdr::plot::PlotView& data) override
  {
    const ImPlotFlags plot_flags = ImPlotFlags_Equal | (lock_view_ ? ImPlotFlags_NoInputs : ImPlotFlags_None);

    if (ImPlot::BeginPlot("##scan_trace", ImVec2(-1.0f, -1.0f), plot_flags))
    {
      const ImPlotAxisFlags axis_flags = lock_view_ ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None;
      ImPlot::SetupAxes("x (m)", "y (m)", axis_flags, axis_flags);

      map_texture_.plot("##map");

      plot_trail(/*from_odom=*/false, kTruthColor, "truth");
      plot_trail(/*from_odom=*/true, kOdomColor, "odometry");

      // truth and odom are set together at the end of on_sample, so a single
      // has_value() check on last_truth_ guards both — drawing latest_odom_
      // unconditionally would place an orange footprint at the map origin
      // (its default-constructed Pose2D) before the first sample runs.
      if (last_truth_.has_value())
      {
        draw_pose(*last_truth_, kTruthColor, "##truth_pose");
        draw_pose(latest_odom_, kOdomColor, "##odom_pose");
      }

      if (selected_.has_value() && selected_->index < samples_.size())
      {
        const Sample& sample = samples_[selected_->index];
        const stdr_simulation::Pose2D& pose = selected_->from_odom ? sample.odom : sample.truth;
        const ImVec4& color = selected_->from_odom ? kOdomColor : kTruthColor;

        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 12.0f, color, 2.0f, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        ImPlot::PlotScatter("##selected_marker", &pose.x, &pose.y, 1);

        draw_pose(pose, color, "##selected_pose");

        const std::vector<stdr::plot::Point2> scan_points =
            stdr::plot::helpers::scan_to_map_points(sample.scan, laser_in_robot_, pose);
        if (!scan_points.empty())
        {
          std::vector<double> xs;
          std::vector<double> ys;
          xs.reserve(scan_points.size());
          ys.reserve(scan_points.size());
          for (const stdr::plot::Point2& p : scan_points)
          {
            xs.push_back(p.x);
            ys.push_back(p.y);
          }
          ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 2.0f, kScanColor, 1.0f, kScanColor);
          ImPlot::PlotScatter("scan", xs.data(), ys.data(), static_cast<int>(xs.size()));
        }
      }

      // Only a click (not a drag) selects a breadcrumb, so panning/box-zoom
      // with the mouse held down never fights the picker.
      if (ImPlot::IsPlotHovered())
      {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
          pick_breadcrumb_at_mouse();
        }
        else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
        {
          selected_.reset();
        }
      }

      ImPlot::EndPlot();
    }

    ImGui::Checkbox("Lock view to map", &lock_view_);

    if (laser_.empty())
    {
      ImGui::TextDisabled("Robot has no laser sensor.");
    }

    const std::span<const stdr::plot::TimedScalar> xy = data.scalar("err_xy");
    const std::span<const stdr::plot::TimedScalar> th = data.scalar("err_theta");
    if (!xy.empty() && !th.empty())
    {
      ImGui::Text("Position error: %.3f m   Yaw error: %.3f rad", xy.back().v, th.back().v);
    }

    if (selected_.has_value() && selected_->index < samples_.size())
    {
      const std::size_t n_rays = samples_[selected_->index].scan.ranges.size();
      ImGui::Text("Selected: %s breadcrumb #%zu  (%zu rays)", selected_->from_odom ? "odometry" : "truth",
                  selected_->index, n_rays);
      if (ImGui::Button("Clear selection"))
      {
        selected_.reset();
      }
    }
    else
    {
      ImGui::TextDisabled("Click a breadcrumb to show its scan.");
    }
  }

private:
  /** One breadcrumb: the truth and odometry poses at the moment the sample
   *  was taken, plus the laser scan captured at that same moment.  Storing
   *  all three together (rather than two separate Trails) means breadcrumb i
   *  always has a truth pose, an odom pose and a scan that are mutually
   *  consistent, so the same stored scan can be replayed from either pose. */
  struct Sample
  {
    stdr_simulation::Pose2D truth;
    stdr_simulation::Pose2D odom;
    stdr_simulation::LaserScan scan;
  };

  /** A user-picked breadcrumb: which sample, and whether it was picked from
   *  the truth trail or the odometry trail. */
  struct Selection
  {
    std::size_t index{ 0 };
    bool from_odom{ false };
  };

  /** Draw one trail's line + per-point markers under a single legend label
   *  (@p legend_label), so ImPlot's legend shows one entry per trail instead
   *  of one per marker.  @p from_odom selects which pose field of `Sample`
   *  the trail is built from. */
  void plot_trail(bool from_odom, const ImVec4& color, const char* legend_label) const
  {
    if (samples_.empty())
    {
      return;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(samples_.size());
    ys.reserve(samples_.size());
    for (const Sample& sample : samples_)
    {
      const stdr_simulation::Pose2D& p = from_odom ? sample.odom : sample.truth;
      xs.push_back(p.x);
      ys.push_back(p.y);
    }

    ImPlot::SetNextLineStyle(color, 2.0f);
    ImPlot::PlotLine(legend_label, xs.data(), ys.data(), static_cast<int>(xs.size()));

    if (xs.size() > 1)
    {
      // Hidden from the legend ("##") so only the line above contributes the
      // "truth"/"odometry" entry ImPlot shows.  The id is built from
      // legend_label (unique per trail) rather than a shared literal — see
      // odometry_trace_plotter.cpp's markers_id comment for why a shared id
      // is a bug.
      const std::string markers_id = std::string("##") + legend_label + "_markers";
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, color, 1.0f, color);
      ImPlot::PlotScatter(markers_id.c_str(), xs.data(), ys.data(), static_cast<int>(xs.size()));
    }
  }

  /** Draw the footprint outline (or a dot when the footprint is empty) and a
   *  heading arrow at @p pose in @p color.  Shared by the truth, odometry and
   *  selected-breadcrumb poses so the block is not duplicated. */
  void draw_pose(const stdr_simulation::Pose2D& pose, const ImVec4& color, const char* label_prefix) const
  {
    const double cx = std::cos(pose.theta);
    const double sx = std::sin(pose.theta);

    if (!footprint_robot_frame_.empty())
    {
      const std::size_t n = footprint_robot_frame_.size();
      std::vector<double> xs(n + 1);
      std::vector<double> ys(n + 1);
      for (std::size_t i = 0; i < n; ++i)
      {
        const double vx = footprint_robot_frame_[i].x;
        const double vy = footprint_robot_frame_[i].y;
        xs[i] = pose.x + cx * vx - sx * vy;
        ys[i] = pose.y + sx * vx + cx * vy;
      }
      xs[n] = xs[0];
      ys[n] = ys[0];

      ImPlot::SetNextLineStyle(color, 2.0f);
      ImPlot::PlotLine(label_prefix, xs.data(), ys.data(), static_cast<int>(n + 1));
    }
    else
    {
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 8.0f, color, 2.0f, color);
      ImPlot::PlotScatter(label_prefix, &pose.x, &pose.y, 1);
    }

    const double arr_x[2] = { pose.x, pose.x + cx * stdr::plot::kHeadingArrowLen };
    const double arr_y[2] = { pose.y, pose.y + sx * stdr::plot::kHeadingArrowLen };
    ImPlot::SetNextLineStyle(color, 2.0f);
    // Id built from label_prefix (unique per call) rather than a shared
    // "##heading" literal — see the markers_id comment in plot_trail for why
    // a shared id is a bug.
    const std::string heading_id = std::string(label_prefix) + "_heading";
    ImPlot::PlotLine(heading_id.c_str(), arr_x, arr_y, 2);
  }

  /** Find the sample breadcrumb (truth or odometry marker) nearest the mouse
   *  cursor, in pixel space, and select it if within kPickRadiusPixels.
   *  Clears the selection if no breadcrumb is within range.  Must be called
   *  between ImPlot::BeginPlot and ImPlot::EndPlot so PlotToPixels resolves
   *  against the current plot's axes. */
  void pick_breadcrumb_at_mouse()
  {
    const ImVec2 mouse = ImGui::GetMousePos();
    float best_dist_sq = kPickRadiusPixels * kPickRadiusPixels;
    std::optional<Selection> best;

    for (std::size_t i = 0; i < samples_.size(); ++i)
    {
      for (const bool from_odom : { false, true })
      {
        const stdr_simulation::Pose2D& p = from_odom ? samples_[i].odom : samples_[i].truth;
        const ImVec2 px = ImPlot::PlotToPixels(p.x, p.y);
        const float dx = px.x - mouse.x;
        const float dy = px.y - mouse.y;
        const float dist_sq = (dx * dx) + (dy * dy);
        if (dist_sq <= best_dist_sq)
        {
          best_dist_sq = dist_sq;
          best = Selection{ i, from_odom };
        }
      }
    }

    selected_ = best;
  }

  std::string robot_;
  std::string laser_;
  bool laser_fetched_{ false };
  stdr_simulation::Pose2D laser_in_robot_{};

  stdr::plot::MapTexture map_texture_;

  std::deque<Sample> samples_;
  std::optional<stdr_simulation::Pose2D> last_truth_;
  stdr_simulation::Pose2D latest_odom_{};

  std::vector<stdr_simulation::Point2D> footprint_robot_frame_;
  bool footprint_fetched_{ false };

  std::optional<Selection> selected_;

  bool lock_view_{ true };
};

}  // namespace

REGISTER_PLOTTER(ScanTracePlotter);
