#pragma once

/**
 * @file sound_simulator.hpp
 * @brief Pure C++ sound level sensor simulator; no ROS dependencies.
 */

#include <stdr_simulation/types.hpp>
#include <vector>

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a microphone detecting sound sources in the environment.
 *
 * Accumulates dB contributions from all sources within the sensor's maximum
 * range and angular span using an inverse-square attenuation model.  Sources
 * closer than 0.5 m contribute their full rated dB (clamping prevents
 * division-by-zero and models saturation at very short distances).
 */
class SoundSimulator {
public:
  SoundSimulator() = default;
  ~SoundSimulator() = default;

  /**
   * @brief Simulate a sound level reading from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            Sound sensor parameters (range, angular span).
   * @param sources           Sound emission sources in the environment.
   * @return SoundMeasurement Total dB accumulated from all in-range sources.
   */
  [[nodiscard]] SoundMeasurement simulate(
      const Pose2D& sensor_pose_world,
      const SoundSensorConfig& config,
      const std::vector<SoundSource>& sources) const;
};

}  // namespace stdr_simulation::sensors
