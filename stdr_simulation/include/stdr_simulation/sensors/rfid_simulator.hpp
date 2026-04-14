#pragma once

/**
 * @file rfid_simulator.hpp
 * @brief Pure C++ RFID reader simulator; no ROS dependencies.
 */

#include <stdr_simulation/types.hpp>
#include <vector>

namespace stdr_simulation::sensors
{

/**
 * @brief Simulates an RFID reader detecting tags placed in the environment.
 *
 * Reports all tags that fall within the sensor's maximum range and angular span.
 * Signal strength is a fixed 1.0 dB for every detected tag, matching the
 * original ROS1 behaviour (no noise model is applied).
 */
class RfidSimulator
{
public:
  RfidSimulator() = default;
  ~RfidSimulator() = default;

  /**
   * @brief Simulate an RFID read from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            RFID sensor parameters (range, angular span).
   * @param tags              Tags placed in the environment.
   * @return RfidMeasurement  All tags detected in this sweep.
   */
  [[nodiscard]] RfidMeasurement simulate(const Pose2D& sensor_pose_world, const RfidSensorConfig& config,
                                         const std::vector<RfidTag>& tags) const;
};

}  // namespace stdr_simulation::sensors
