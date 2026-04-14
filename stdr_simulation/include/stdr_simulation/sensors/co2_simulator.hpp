#pragma once

/**
 * @file co2_simulator.hpp
 * @brief Pure C++ CO2 concentration sensor simulator; no ROS dependencies.
 */

#include <stdr_simulation/types.hpp>
#include <vector>

namespace stdr_simulation::sensors
{

/**
 * @brief Simulates a CO2 concentration sensor in an environment with sources.
 *
 * Accumulates ppm contributions from all sources using an inverse-square
 * dispersion model.  Sources closer than 0.5 m contribute their full
 * rated ppm (clamping prevents division-by-zero and models sensor saturation
 * at very short distances).  No noise or angular filtering is applied.
 */
class Co2Simulator
{
public:
  Co2Simulator() = default;
  ~Co2Simulator() = default;

  /**
   * @brief Simulate a CO2 reading from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            CO2 sensor parameters (max range).
   * @param sources           CO2 emission sources in the environment.
   * @return CO2Measurement   Total ppm accumulated from all in-range sources.
   */
  [[nodiscard]] CO2Measurement simulate(const Pose2D& sensor_pose_world, const CO2SensorConfig& config,
                                        const std::vector<CO2Source>& sources) const;
};

}  // namespace stdr_simulation::sensors
