#include <stdr_simulation/sensors/sound_simulator.hpp>

#include <stdr_simulation/sensors/angle_utils.hpp>

#include <cmath>
#include <vector>

namespace stdr_simulation::sensors {

// Sources closer than this threshold contribute their full rated dB to avoid
// division-by-zero and to model sensor saturation at very short distances.
static constexpr double kNearDistanceThreshold = 0.5;

// Attenuation constant from the ROS1 microphone.cpp implementation.  The
// factor 0.25 is a calibration constant that scales the inverse-square falloff.
static constexpr double kAttenuationFactor = 0.25;

SoundMeasurement SoundSimulator::simulate(
    const Pose2D& sensor_pose_world,
    const SoundSensorConfig& config,
    const std::vector<SoundSource>& sources) const {
  double total_dbs = 0.0;

  for (const SoundSource& source : sources) {
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

    if (distance > kNearDistanceThreshold) {
      total_dbs += source.dbs * (kAttenuationFactor / (distance * distance));
    } else {
      total_dbs += source.dbs;
    }
  }

  return SoundMeasurement{total_dbs};
}

}  // namespace stdr_simulation::sensors
