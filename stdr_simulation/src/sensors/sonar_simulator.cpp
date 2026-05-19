#include <stdr_simulation/sensors/sonar_simulator.hpp>

#include <cmath>
#include <limits>
#include <random>

namespace stdr_simulation::sensors
{

SonarSimulator::SonarSimulator() : rng_(std::random_device{}())
{
}

SonarScan SonarSimulator::simulate(const Pose2D& sensor_pose_world, const SonarConfig& config,
                                   const OccupancyGrid& map) const
{
  // A non-positive resolution would produce nonsensical grid coordinates.
  if (map.resolution <= 0.0)
  {
    return SonarScan{ config.max_range };
  }

  // Build the noise distribution only when it will actually be sampled.
  // std::normal_distribution requires std_dev > 0; the guard below enforces
  // this at the call site, but we must also avoid constructing with std_dev=0.
  const bool apply_noise = config.noise.enabled && config.noise.std_dev > 0.0;
  // Mean is always 0 — config.noise.mean is intentionally ignored here.
  // Treating it as a bias shifts the range by a constant offset, which
  // introduces systematic error rather than realistic sensor noise.
  std::normal_distribution<double> noise_dist(0.0, apply_noise ? config.noise.std_dev : 1.0);

  // One-degree steps give enough angular resolution without excessive ray count.
  // A finer step would rarely change the minimum-range result because sonar
  // typically has a wide cone (15°–30°) and coarse map resolution.
  static constexpr double kAngleStepRad = M_PI / 180.0;
  const int max_steps = static_cast<int>(config.max_range / map.resolution);

  // Precompute the sensor origin in grid coordinates — shared across all rays.
  const double origin_grid_x = (sensor_pose_world.x - map.origin.x) / map.resolution;
  const double origin_grid_y = (sensor_pose_world.y - map.origin.y) / map.resolution;

  // `hit` is the real no-hit sentinel.  The old code used `min_steps = max_steps + 1`
  // as a numeric sentinel, but that caused a systematic one-cell bias
  // (max_range + resolution) when noise was applied to a no-hit reading.
  // The `hit` flag lets us return +infinity immediately on no-hit, bypassing
  // both noise and the clamp entirely.
  bool hit = false;
  // INT_MAX makes the "unset" state self-evident: min_steps is only read after
  // `hit` becomes true, so any finite step count will always be smaller.
  int min_steps = std::numeric_limits<int>::max();

  for (double angle = -config.cone_angle / 2.0; angle <= config.cone_angle / 2.0; angle += kAngleStepRad)
  {
    const double ray_angle = sensor_pose_world.theta + angle;
    const double cos_angle = std::cos(ray_angle);
    const double sin_angle = std::sin(ray_angle);

    for (int step = 1; step <= max_steps; ++step)
    {
      const int cell_x = static_cast<int>(origin_grid_x + cos_angle * static_cast<double>(step));
      const int cell_y = static_cast<int>(origin_grid_y + sin_angle * static_cast<double>(step));

      if (cell_x < 0 || cell_x >= map.width || cell_y < 0 || cell_y >= map.height)
      {
        // Ray exited the map — treat as max range.
        break;
      }

      const std::size_t idx =
          static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(map.width) + static_cast<std::size_t>(cell_x);
      if (map.data[idx] > kOccupancyThreshold)
      {
        hit = true;
        if (step < min_steps)
        {
          min_steps = step;
        }
        break;
      }
    }
  }

  // No ray hit any obstacle: return +infinity directly without noise or clamping.
  // Applying noise to a no-hit reading would introduce a systematic bias of one
  // map cell; the caller should interpret +infinity as "beyond max range".
  if (!hit)
  {
    return SonarScan{ std::numeric_limits<double>::infinity() };
  }

  // Convert the winning step count back to metres.
  double range = static_cast<double>(min_steps) * map.resolution;

  if (apply_noise)
  {
    range += noise_dist(rng_);
  }

  // Clamp to sensor limits; use signed infinities to signal out-of-bounds.
  if (range < config.min_range)
  {
    range = -std::numeric_limits<double>::infinity();
  }
  else if (range >= config.max_range)
  {
    range = std::numeric_limits<double>::infinity();
  }

  return SonarScan{ range };
}

}  // namespace stdr_simulation::sensors
