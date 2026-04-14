#include <stdr_simulation/sensors/rfid_simulator.hpp>

#include <stdr_simulation/sensors/angle_utils.hpp>

#include <cmath>
#include <vector>

namespace stdr_simulation::sensors
{

RfidMeasurement RfidSimulator::simulate(const Pose2D& sensor_pose_world, const RfidSensorConfig& config,
                                        const std::vector<RfidTag>& tags) const
{
  RfidMeasurement measurement;

  for (const RfidTag& tag : tags)
  {
    const double dx = tag.pose.x - sensor_pose_world.x;
    const double dy = tag.pose.y - sensor_pose_world.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance > config.max_range)
    {
      continue;
    }

    const double angle_to_tag = std::atan2(dy, dx);
    if (!angle_in_range(angle_to_tag, sensor_pose_world.theta, config.angle_span))
    {
      continue;
    }

    measurement.tag_ids.push_back(tag.tag_id);
    measurement.tag_messages.push_back(tag.message);
    // Signal strength is hardcoded to 1.0 dB — the ROS1 implementation did not
    // model any distance-dependent attenuation or noise for RFID.
    measurement.tag_dbs.push_back(1.0);
  }

  return measurement;
}

}  // namespace stdr_simulation::sensors
