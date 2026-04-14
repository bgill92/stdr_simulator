#pragma once

/**
 * @file sonar_simulator.hpp
 * @brief Pure C++ 2D ultrasonic sonar simulator; no ROS dependencies.
 */

#include <random>
#include <stdr_simulation/types.hpp>

namespace stdr_simulation::sensors
{

/**
 * @brief Simulates an ultrasonic sonar sensor against the world occupancy grid.
 *
 * Models a cone-shaped beam and returns the minimum range to any obstacle
 * within the cone, with optional Gaussian noise applied to the measurement.
 *
 * @warning Not thread-safe. The internal RNG state is mutated on each call to
 *          simulate(). Use separate instances per thread.
 */
class SonarSimulator
{
public:
  SonarSimulator();
  ~SonarSimulator() = default;

  /**
   * @brief Simulate a sonar reading from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            Sonar sensor parameters (cone angle, range, noise, etc.).
   * @param map               Occupancy grid to ray-march against.
   * @return SonarScan        Populated scan with the minimum range within the cone.
   */
  [[nodiscard]] SonarScan simulate(const Pose2D& sensor_pose_world, const SonarConfig& config,
                                   const OccupancyGrid& map) const;

private:
  // Occupancy threshold above which a cell is treated as an obstacle.
  static constexpr int kOccupancyThreshold = 70;

  // Seeded once at construction; mutable so simulate() can remain const.
  mutable std::mt19937 rng_;
};

}  // namespace stdr_simulation::sensors
