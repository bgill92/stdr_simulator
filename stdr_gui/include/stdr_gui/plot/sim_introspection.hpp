#pragma once

/** @file Thin read-only facade over the backend passed to `Plotter::on_init`.
 *
 *  `SimIntrospection` lets a plotter cache robot/sensor IDs at init time
 *  without exposing the full `SimulatorBackend` API or requiring the plotter
 *  header to pull in the backend header.  The backend pointer is borrowed for
 *  the duration of `on_init` only — callers must not store it. */

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace stdr::plot
{

/** @brief Read-only introspection facade for use inside `Plotter::on_init`.
 *
 *  Wraps the backend so that `on_init` can query the robot roster without
 *  being able to issue commands or sample sensors — those capabilities belong
 *  to `SimView`, which is provided during `on_sample`.
 *
 *  The backend reference is borrowed; it remains valid only for the duration
 *  of the `on_init` call.  Do not store a copy of this object. */
class SimIntrospection
{
public:
  /** @param backend  Non-owning pointer to the backend.  Must outlive this object. */
  explicit SimIntrospection(const stdr_gui::SimulatorBackend& backend) : backend_(backend)
  {
  }

  // Non-copyable — the backend reference must not be sliced out of context.
  SimIntrospection(const SimIntrospection&) = delete;
  SimIntrospection& operator=(const SimIntrospection&) = delete;
  SimIntrospection(SimIntrospection&&) = delete;
  SimIntrospection& operator=(SimIntrospection&&) = delete;

  /** Return the number of robots currently in the simulation. */
  [[nodiscard]] std::size_t num_robots() const
  {
    return backend_.num_robots();
  }

  /** Return owning IDs of all robots currently in the simulation. */
  [[nodiscard]] std::vector<RobotId> robot_ids() const
  {
    return backend_.robot_ids();
  }

  /** Return sensor frame IDs of all laser sensors on @p robot_id. */
  [[nodiscard]] std::vector<SensorId> laser_sensors(const RobotId& robot_id) const
  {
    return backend_.laser_sensors(robot_id);
  }

  /** Return sensor frame IDs of all sonar sensors on @p robot_id. */
  [[nodiscard]] std::vector<SensorId> sonar_sensors(const RobotId& robot_id) const
  {
    return backend_.sonar_sensors(robot_id);
  }

private:
  const stdr_gui::SimulatorBackend& backend_;
};

}  // namespace stdr::plot
