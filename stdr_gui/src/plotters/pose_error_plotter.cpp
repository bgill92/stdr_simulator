/** @file Example plotter: drives robot0 in a circle and plots the deviation
 *  between commanded-integrated (dead-reckoning) pose and actual odometry.
 *
 *  This file serves as living documentation of how to author a plotter plugin.
 *  It is also the primary integration test of the REGISTER_PLOTTER mechanism:
 *  if the WHOLE_ARCHIVE link is missing, this TU is dead-stripped and the
 *  registry stays empty, which the test_plotter_registry test would catch. */

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>

#include <implot.h>

#include <cmath>
#include <string_view>

namespace
{

class PoseErrorPlotter : public stdr::plot::Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "Pose Error";
  }

  [[nodiscard]] std::string_view description() const override
  {
    return "Drives robot0 in a circle and plots ground-truth vs dead-reckoning.";
  }

  void on_init(const stdr::plot::SimIntrospection& sim) override
  {
    // Cache the first robot ID at init time so on_sample does not re-query
    // the roster on every frame.  If no robots exist yet, on_sample performs
    // the same lookup on its first call once one is present.  robot_ids()
    // returns owning strings, so storing it as a member is safe across frames.
    if (sim.num_robots() > 0)
    {
      robot_ = sim.robot_ids()[0];
    }
  }

  void on_pause(stdr::plot::SimView& sim) override
  {
    // Stop the robot so the simulator does not keep applying the last latched
    // velocity command while the plotter is paused.
    if (!robot_.empty())
    {
      sim.cmd_velocity(robot_, 0.0, 0.0);
    }
  }

  void on_sample(stdr::plot::SimView& sim, stdr::plot::PlotSink& out) override
  {
    // on_init may fire before any robot is spawned (PlotPanel is constructed
    // lazily on the first GUI frame in gui_app.cpp), so capture the ID here on
    // the first sample where one is present.
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

    // Command the robot to drive in a circle.
    sim.cmd_velocity(robot_, 0.3, 0.5);

    const stdr_simulation::Pose2D gt = sim.pose(robot_);
    const double t = sim.sim_time();
    const double dt = t - last_t_;
    last_t_ = t;

    // Integrate dead-reckoning from the commanded velocity.
    dead_.x += std::cos(dead_.theta) * 0.3 * dt;
    dead_.y += std::sin(dead_.theta) * 0.3 * dt;
    dead_.theta += 0.5 * dt;

    out.scalar("err_xy", t, std::hypot(gt.x - dead_.x, gt.y - dead_.y));
    out.scalar("err_theta", t, gt.theta - dead_.theta);
  }

  void on_render(const stdr::plot::PlotView& data) override
  {
    const std::span<const stdr::plot::TimedScalar> xy = data.scalar("err_xy");
    const std::span<const stdr::plot::TimedScalar> th = data.scalar("err_theta");

    if (ImPlot::BeginPlot("##pose_error", ImVec2(-1.0f, -1.0f)))
    {
      ImPlot::SetupAxes("sim time (s)", "error");
      if (!xy.empty())
      {
        ImPlot::PlotLine("|gt - dead| (m)", &xy[0].t, &xy[0].v, static_cast<int>(xy.size()), 0, 0,
                         static_cast<int>(sizeof(stdr::plot::TimedScalar)));
      }
      if (!th.empty())
      {
        ImPlot::PlotLine("err theta (rad)", &th[0].t, &th[0].v, static_cast<int>(th.size()), 0, 0,
                         static_cast<int>(sizeof(stdr::plot::TimedScalar)));
      }
      ImPlot::EndPlot();
    }
  }

private:
  std::string robot_;
  stdr_simulation::Pose2D dead_{};
  double last_t_{ 0.0 };
};

}  // namespace

REGISTER_PLOTTER(PoseErrorPlotter);
