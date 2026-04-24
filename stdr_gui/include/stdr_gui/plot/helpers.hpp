#pragma once

/** @file Small free-function utilities for plotter authors.
 *
 *  This header is the single place where plotter plugins can import:
 *  - Iteration helpers (for_each_robot, for_each_laser, for_each_sonar)
 *  - Unit conversions (rad_to_deg, deg_to_rad, m_to_mm, mm_to_m)
 *  - Coordinate frame transforms (map_to_robot, robot_to_map)
 *  - Time-aligned sampling gating (should_sample)
 *
 *  All functions are pure and side-effect free unless documented otherwise. */

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_simulation/types.hpp>

#include <cmath>
#include <numbers>

namespace stdr::plot::helpers
{

// ---------------------------------------------------------------------------
// Unit conversions — constexpr, inline
// ---------------------------------------------------------------------------

/** Convert radians to degrees. */
[[nodiscard]] constexpr double rad_to_deg(double r) noexcept
{
  return r * (180.0 / std::numbers::pi);
}

/** Convert degrees to radians. */
[[nodiscard]] constexpr double deg_to_rad(double d) noexcept
{
  return d * (std::numbers::pi / 180.0);
}

/** Convert metres to millimetres. */
[[nodiscard]] constexpr double m_to_mm(double m) noexcept
{
  return m * 1000.0;
}

/** Convert millimetres to metres. */
[[nodiscard]] constexpr double mm_to_m(double mm) noexcept
{
  return mm * 0.001;
}

// ---------------------------------------------------------------------------
// Coordinate frame transforms — non-constexpr (require std::cos/sin)
// ---------------------------------------------------------------------------

/** @brief Transform a point from the map frame into the robot's local frame.
 *
 *  Applies the inverse rigid-body transform:
 *    p_robot.x =  cos(θ) * (p_map.x - t.x) + sin(θ) * (p_map.y - t.y)
 *    p_robot.y = -sin(θ) * (p_map.x - t.x) + cos(θ) * (p_map.y - t.y)
 *  where θ = robot_in_map.theta and t = (robot_in_map.x, robot_in_map.y).
 *
 *  The output theta is the input point's orientation minus the robot's
 *  heading, reflecting the same rotation applied to the position. */
[[nodiscard]] stdr_simulation::Pose2D map_to_robot(const stdr_simulation::Pose2D& p_map,
                                                   const stdr_simulation::Pose2D& robot_in_map) noexcept;

/** @brief Transform a point from the robot's local frame to the map frame.
 *
 *  Inverse of map_to_robot.  Applies:
 *    p_map.x = t.x + cos(θ) * p_robot.x - sin(θ) * p_robot.y
 *    p_map.y = t.y + sin(θ) * p_robot.x + cos(θ) * p_robot.y
 *  where θ = robot_in_map.theta and t = (robot_in_map.x, robot_in_map.y).
 *
 *  The output theta is the input point's orientation plus the robot's heading. */
[[nodiscard]] stdr_simulation::Pose2D robot_to_map(const stdr_simulation::Pose2D& p_robot,
                                                   const stdr_simulation::Pose2D& robot_in_map) noexcept;

// ---------------------------------------------------------------------------
// Time-aligned sampling gate
// ---------------------------------------------------------------------------

/** @brief Decide whether to sample now given the last sample time and period.
 *
 *  Returns true when `now_sim_time - last_sample_sim_time >= period_seconds`,
 *  and updates `last_sample_sim_time` to `now_sim_time` in that case.
 *
 *  Typical use inside `on_sample`:
 *  @code
 *    if (!helpers::should_sample(sim.sim_time(), last_t_, 0.1)) { return; }
 *  @endcode
 *
 *  This gates work at the declared sample frequency even when the framework
 *  drives `on_sample` faster than the plotter's `sample_period()` — for
 *  example during the first frame or after a resume.
 *
 *  The last-sample time is set to `now_sim_time` on each fire (simple reset,
 *  not drift-free); for drift-free sampling, callers can update
 *  `last + period_seconds` instead. */
[[nodiscard]] bool should_sample(double now_sim_time, double& last_sample_sim_time, double period_seconds) noexcept;

// ---------------------------------------------------------------------------
// Iteration helpers — template, header-only
// ---------------------------------------------------------------------------

/** @brief Invoke @p fn(robot_id) for every robot currently in the simulation.
 *
 *  Queries `sim.robot_ids()` once and iterates the returned vector.
 *  @p fn receives a `const RobotId&` (owning string). */
template <typename Fn>
void for_each_robot(const SimView& sim, Fn&& fn)
{
  const std::vector<RobotId> ids = sim.robot_ids();
  for (const RobotId& id : ids)
  {
    fn(id);
  }
}

/** @brief Invoke @p fn(sensor_id) for every laser sensor on @p robot.
 *
 *  Queries `sim.laser_sensors(robot)` once and iterates the returned vector.
 *  @p fn receives a `const SensorId&` (owning string). */
template <typename Fn>
void for_each_laser(const SimView& sim, const RobotId& robot, Fn&& fn)
{
  const std::vector<SensorId> ids = sim.laser_sensors(robot);
  for (const SensorId& id : ids)
  {
    fn(id);
  }
}

/** @brief Invoke @p fn(sensor_id) for every sonar sensor on @p robot.
 *
 *  Queries `sim.sonar_sensors(robot)` once and iterates the returned vector.
 *  @p fn receives a `const SensorId&` (owning string). */
template <typename Fn>
void for_each_sonar(const SimView& sim, const RobotId& robot, Fn&& fn)
{
  const std::vector<SensorId> ids = sim.sonar_sensors(robot);
  for (const SensorId& id : ids)
  {
    fn(id);
  }
}

}  // namespace stdr::plot::helpers
