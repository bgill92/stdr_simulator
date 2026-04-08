#include <stdr_simulation/geometry_utils.hpp>

#include <cmath>

namespace stdr_simulation
{

Pose2D compute_sensor_world_pose(const Pose2D& robot_pose, const Pose2D& sensor_local)
{
  const double cos_theta = std::cos(robot_pose.theta);
  const double sin_theta = std::sin(robot_pose.theta);
  Pose2D result;
  result.x = robot_pose.x + sensor_local.x * cos_theta - sensor_local.y * sin_theta;
  result.y = robot_pose.y + sensor_local.x * sin_theta + sensor_local.y * cos_theta;
  result.theta = robot_pose.theta + sensor_local.theta;
  return result;
}

}  // namespace stdr_simulation
