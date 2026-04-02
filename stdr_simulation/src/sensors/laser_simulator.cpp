#include <stdr_simulation/sensors/laser_simulator.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace stdr_simulation::sensors {

LaserSimulator::LaserSimulator() : rng_(std::random_device{}()) {}

LaserScan LaserSimulator::simulate(const Pose2D& sensor_pose_world,
                                   const LaserConfig& config,
                                   const OccupancyGrid& map) const {
  LaserScan scan;

  // A non-positive resolution would produce nonsensical grid coordinates.
  if (map.resolution <= 0.0) {
    return scan;
  }

  const double angle_increment =
      (config.num_rays > 1)
          ? (config.max_angle - config.min_angle) /
                static_cast<double>(config.num_rays - 1)
          : 0.0;

  scan.angle_min = config.min_angle;
  scan.angle_max = config.max_angle;
  scan.angle_increment = angle_increment;
  scan.range_min = config.min_range;
  scan.range_max = config.max_range;

  const int max_steps =
      static_cast<int>(config.max_range / map.resolution);

  // Build the noise distribution only when it will actually be sampled.
  // std::normal_distribution requires std_dev > 0; the guard below enforces
  // this at the call site, but we must also avoid constructing with std_dev=0.
  const bool apply_noise =
      config.noise.enabled && config.noise.std_dev > 0.0;
  std::normal_distribution<double> noise_dist(
      config.noise.mean,
      apply_noise ? config.noise.std_dev : 1.0);

  scan.ranges.reserve(static_cast<std::size_t>(config.num_rays));

  for (std::int32_t ray_idx = 0; ray_idx < config.num_rays; ++ray_idx) {
    const double ray_angle = sensor_pose_world.theta + config.min_angle +
                             static_cast<double>(ray_idx) * angle_increment;
    const double cos_angle = std::cos(ray_angle);
    const double sin_angle = std::sin(ray_angle);

    // Starting grid cell of the sensor origin.
    const double origin_grid_x =
        (sensor_pose_world.x - map.origin.x) / map.resolution;
    const double origin_grid_y =
        (sensor_pose_world.y - map.origin.y) / map.resolution;

    double range = config.max_range;

    for (int step = 1; step <= max_steps; ++step) {
      const int cell_x =
          static_cast<int>(origin_grid_x + cos_angle * static_cast<double>(step));
      const int cell_y =
          static_cast<int>(origin_grid_y + sin_angle * static_cast<double>(step));

      if (cell_x < 0 || cell_x >= map.width || cell_y < 0 ||
          cell_y >= map.height) {
        // Ray exited the map — treat as max range.
        break;
      }

      const std::size_t idx = static_cast<std::size_t>(cell_y) *
                                  static_cast<std::size_t>(map.width) +
                              static_cast<std::size_t>(cell_x);
      if (map.data[idx] > kOccupancyThreshold) {
        range = static_cast<double>(step) * map.resolution;
        break;
      }
    }

    if (apply_noise) {
      range += noise_dist(rng_);
    }

    // Clamp to sensor limits; use signed infinities to signal out-of-bounds.
    float final_range{};
    if (range > config.max_range) {
      final_range = std::numeric_limits<float>::infinity();
    } else if (range < config.min_range) {
      final_range = -std::numeric_limits<float>::infinity();
    } else {
      final_range = static_cast<float>(range);
    }

    scan.ranges.push_back(final_range);
  }

  return scan;
}

}  // namespace stdr_simulation::sensors
