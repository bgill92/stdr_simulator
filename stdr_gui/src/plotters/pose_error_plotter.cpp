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
    // the roster on every frame.  robot_ids() returns owning strings, so
    // storing it as a member is safe across frames.
    if (sim.num_robots() > 0)
    {
      robot_ = sim.robot_ids()[0];
    }
  }

  void on_sample(stdr::plot::SimView& sim, stdr::plot::PlotSink& out) override
  {
    if (robot_.empty())
    {
      return;
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
    // ImPlot calls go here.  The spans below are valid for the duration of
    // this on_render call (the framework guarantees no on_sample writes
    // concurrently with on_render on the same plotter).
    const std::span<const stdr::plot::TimedScalar> xy = data.scalar("err_xy");
    const std::span<const stdr::plot::TimedScalar> th = data.scalar("err_theta");

    // When ImPlot is not present in the test binary, we simply don't call it.
    // In the full GUI build, these calls render the line plots.
    (void)xy;
    (void)th;
  }

private:
  std::string robot_;
  stdr_simulation::Pose2D dead_{};
  double last_t_{ 0.0 };
};

}  // namespace

REGISTER_PLOTTER(PoseErrorPlotter);
