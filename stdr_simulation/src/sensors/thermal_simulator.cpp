#include <stdr_simulation/sensors/thermal_simulator.hpp>

#include <stdr_simulation/sensors/angle_utils.hpp>

#include <cmath>
#include <vector>

namespace stdr_simulation::sensors {

ThermalMeasurement ThermalSimulator::simulate(
    const Pose2D& sensor_pose_world,
    const ThermalSensorConfig& config,
    const std::vector<ThermalSource>& sources) const {
  double max_degrees = 0.0;

  for (const ThermalSource& source : sources) {
    const double dx = source.pose.x - sensor_pose_world.x;
    const double dy = source.pose.y - sensor_pose_world.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > config.max_range) {
      continue;
    }

    const double angle_to_source = std::atan2(dy, dx);
    if (!angle_in_range(angle_to_source, sensor_pose_world.theta, config.angle_span)) {
      continue;
    }

    if (source.degrees > max_degrees) {
      max_degrees = source.degrees;
    }
  }

  return ThermalMeasurement{{max_degrees}};
}

}  // namespace stdr_simulation::sensors
