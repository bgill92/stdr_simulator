#pragma once

/**
 * @file laser_simulator.hpp
 * @brief Pure C++ 2D laser range-finder simulator; no ROS dependencies.
 */

#include <random>
#include <stdr_simulation/types.hpp>

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a 2D laser range scanner against the world occupancy grid.
 *
 * Casts rays at evenly-spaced angular increments within the configured field
 * of view and returns per-beam range measurements with optional Gaussian noise.
 *
 * @warning Not thread-safe. The internal RNG state is mutated on each call to
 *          simulate(). Use separate instances per thread.
 */
class LaserSimulator {
public:
  LaserSimulator();
  ~LaserSimulator() = default;

  /**
   * @brief Simulate a laser scan from the given world-frame sensor pose.
   *
   * @param sensor_pose_world Sensor origin and heading in the world frame.
   * @param config            Laser sensor parameters (FOV, range, noise, etc.).
   * @param map               Occupancy grid to ray-march against.
   * @return LaserScan        Populated scan with one range per ray.
   */
  [[nodiscard]] LaserScan simulate(const Pose2D& sensor_pose_world,
                                   const LaserConfig& config,
                                   const OccupancyGrid& map) const;

private:
  // Occupancy threshold above which a cell is treated as an obstacle.
  static constexpr int kOccupancyThreshold = 70;

  // Seeded once at construction; mutable so simulate() can remain const.
  mutable std::mt19937 rng_;
};

}  // namespace stdr_simulation::sensors
