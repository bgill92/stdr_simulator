#include <stdr_simulation/sensors/co2_simulator.hpp>

#include <cmath>
#include <vector>

namespace stdr_simulation::sensors
{

// Sources closer than this threshold contribute their full rated ppm to avoid
// division-by-zero and to model sensor saturation at very short distances.
static constexpr double kNearDistanceThreshold = 0.5;

// Dispersion constant from the ROS1 co2.cpp implementation.  The factor 0.25
// is a calibration constant that scales the inverse-square falloff.
static constexpr double kDispersionFactor = 0.25;

CO2Measurement Co2Simulator::simulate(const Pose2D& sensor_pose_world, const CO2SensorConfig& config,
                                      const std::vector<CO2Source>& sources) const
{
  double total_ppm = 0.0;

  for (const CO2Source& source : sources)
  {
    const double dx = source.pose.x - sensor_pose_world.x;
    const double dy = source.pose.y - sensor_pose_world.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > config.max_range)
    {
      continue;
    }

    if (distance > kNearDistanceThreshold)
    {
      total_ppm += source.ppm * (kDispersionFactor / (distance * distance));
    }
    else
    {
      total_ppm += source.ppm;
    }
  }

  return CO2Measurement{ total_ppm };
}

}  // namespace stdr_simulation::sensors
