#pragma once

/**
 * @file thermal_simulator.hpp
 * @brief Pure C++ thermal sensor simulator; no ROS dependencies.
 */

#include <stdr_simulation/types.hpp>
#include <vector>

namespace stdr_simulation::sensors
{

/**
 * @brief Simulates a thermal sensor detecting heat sources in the environment.
 *
 * Returns the maximum temperature reading from all sources within the sensor's
 * maximum range and angular span.  The result carries a single-element vector
 * so it integrates with the ROS message format without post-processing.
 * Returns 0.0 degrees when no source is detected.
 */
class ThermalSimulator
{
public:
  ThermalSimulator() = default;
  ~ThermalSimulator() = default;

  /**
   * @brief Simulate a thermal reading from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            Thermal sensor parameters (range, angular span).
   * @param sources           Thermal emission sources in the environment.
   * @return ThermalMeasurement Single-element vector with the peak temperature.
   */
  [[nodiscard]] ThermalMeasurement simulate(const Pose2D& sensor_pose_world, const ThermalSensorConfig& config,
                                            const std::vector<ThermalSource>& sources) const;
};

}  // namespace stdr_simulation::sensors
