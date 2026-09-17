/** @file Odometry trace plotter: renders the occupancy grid as a background
 *  texture and overlays two breadcrumb trails for the first robot — the
 *  ground-truth pose (green) and the simulation engine's odometry belief
 *  (orange). Under `odometry_model: perfect` the two trails coincide; under
 *  `velocity` they diverge as odometry drifts.
 *
 *  This plotter does not command the robot — drive it with teleop or with
 *  the Pose Error plotter's circle-driving behaviour while this one is open. */

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
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace
{

// A same-sample truth-pose jump larger than this is a teleport or a drag in
// the GUI, not motion — drawing a line across it would be misleading, so both
// trails are cleared instead of connecting the old and new positions.
constexpr double kTeleportThresholdMeters = 1.0;

constexpr ImVec4 kTruthColor(0.0f, 0.8f, 0.0f, 1.0f);
constexpr ImVec4 kOdomColor(1.0f, 0.55f, 0.0f, 1.0f);

class OdometryTracePlotter : public stdr::plot::Plotter
{
public:
  OdometryTracePlotter() = default;

  [[nodiscard]] std::string_view name() const override
  {
    return "Odometry Trace";
  }

  [[nodiscard]] std::string_view description() const override
  {
    return "Map with the true pose (green) and the odometry belief (orange) — trails diverge as odometry drifts.";
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
    // Drop both trails and the teleport-detector's last pose so the next
    // on_sample does not draw a line from the pre-reset pose to spawn, nor
    // mistake the reset teleport itself for a drag needing another clear.
    truth_trail_.clear();
    odom_trail_.clear();
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

    map_texture_.update(sim.map());

    const stdr_simulation::Pose2D truth = sim.pose(robot_);
    const stdr_simulation::Pose2D odom = sim.odom_pose(robot_);

    // Detect a teleport/drag between consecutive samples (not the trails'
    // spacing-gated last point, which may lag several samples behind) and
    // clear both trails so they do not draw a line across the map.
    if (last_truth_.has_value() &&
        std::hypot(truth.x - last_truth_->x, truth.y - last_truth_->y) > kTeleportThresholdMeters)
    {
      truth_trail_.clear();
      odom_trail_.clear();
    }
    last_truth_ = truth;

    // Each trail is pushed independently based on its own last stored point,
    // so the truth and odometry trails can have different densities as
    // odometry drift changes how far the belief pose moves per sample.
    truth_trail_.push_if_moved(truth);
    odom_trail_.push_if_moved(odom);

    const double t = sim.sim_time();
    out.scalar("err_xy", t, std::hypot(truth.x - odom.x, truth.y - odom.y));
    out.scalar("err_theta", t, stdr::plot::helpers::wrapped_angle_diff(truth.theta, odom.theta));

    latest_odom_ = odom;
  }

  void on_render(const stdr::plot::PlotView& data) override
  {
    const ImPlotFlags plot_flags = ImPlotFlags_Equal | (lock_view_ ? ImPlotFlags_NoInputs : ImPlotFlags_None);

    if (ImPlot::BeginPlot("##odometry_trace", ImVec2(-1.0f, -1.0f), plot_flags))
    {
      const ImPlotAxisFlags axis_flags = lock_view_ ? ImPlotAxisFlags_AutoFit : ImPlotAxisFlags_None;
      ImPlot::SetupAxes("x (m)", "y (m)", axis_flags, axis_flags);

      map_texture_.plot("##map");

      plot_trail(truth_trail_, kTruthColor, "truth");
      plot_trail(odom_trail_, kOdomColor, "odometry");

      // truth and odom are set together at the end of on_sample, so a single
      // has_value() check on last_truth_ guards both — drawing latest_odom_
      // unconditionally would place an orange footprint at the map origin
      // (its default-constructed Pose2D) before the first sample runs.
      if (last_truth_.has_value())
      {
        draw_pose(*last_truth_, kTruthColor, "##truth_pose");
        draw_pose(latest_odom_, kOdomColor, "##odom_pose");
      }

      ImPlot::EndPlot();
    }

    ImGui::Checkbox("Lock view to map", &lock_view_);

    const std::span<const stdr::plot::TimedScalar> xy = data.scalar("err_xy");
    const std::span<const stdr::plot::TimedScalar> th = data.scalar("err_theta");
    if (!xy.empty() && !th.empty())
    {
      ImGui::Text("Position error: %.3f m   Yaw error: %.3f rad", xy.back().v, th.back().v);
    }
  }

private:
  /** Draw one trail's line + per-point markers under a single legend label
   *  (@p legend_label), so ImPlot's legend shows one entry per trail instead
   *  of one per marker. */
  static void plot_trail(const stdr::plot::Trail& trail, const ImVec4& color, const char* legend_label)
  {
    if (trail.points.empty())
    {
      return;
    }

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(trail.points.size());
    ys.reserve(trail.points.size());
    for (const stdr_simulation::Pose2D& p : trail.points)
    {
      xs.push_back(p.x);
      ys.push_back(p.y);
    }

    ImPlot::SetNextLineStyle(color, 2.0f);
    ImPlot::PlotLine(legend_label, xs.data(), ys.data(), static_cast<int>(xs.size()));

    if (trail.points.size() > 1)
    {
      // Hidden from the legend ("##") so only the line above contributes the
      // "truth"/"odometry" entry ImPlot shows. The id is built from
      // legend_label (unique per trail) rather than a shared literal — ImPlot
      // treats same-id calls within one plot as a single item, and a shared
      // "##trail_markers" id across both trails let one trail's markers (and
      // the truth heading arrow, see draw_pose) silently drop out of
      // axis auto-fit.
      const std::string markers_id = std::string("##") + legend_label + "_markers";
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 3.0f, color, 1.0f, color);
      ImPlot::PlotScatter(markers_id.c_str(), xs.data(), ys.data(), static_cast<int>(xs.size()));
    }
  }

  /** Draw the footprint outline (or a dot when the footprint is empty) and a
   *  heading arrow at @p pose in @p color.  Shared by the truth and odometry
   *  poses so the block is not duplicated. */
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
    // Id built from label_prefix (unique per call: "##truth_pose_heading" /
    // "##odom_pose_heading") rather than a shared "##heading" literal — see
    // the markers_id comment in plot_trail for why a shared id is a bug.
    const std::string heading_id = std::string(label_prefix) + "_heading";
    ImPlot::PlotLine(heading_id.c_str(), arr_x, arr_y, 2);
  }

  std::string robot_;

  stdr::plot::MapTexture map_texture_;

  stdr::plot::Trail truth_trail_;
  stdr::plot::Trail odom_trail_;
  std::optional<stdr_simulation::Pose2D> last_truth_;

  std::vector<stdr_simulation::Point2D> footprint_robot_frame_;
  bool footprint_fetched_{ false };

  stdr_simulation::Pose2D latest_odom_{};

  bool lock_view_{ true };
};

}  // namespace

REGISTER_PLOTTER(OdometryTracePlotter);
