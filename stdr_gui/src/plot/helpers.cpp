#include <stdr_gui/plot/helpers.hpp>

#include <cmath>

namespace stdr::plot::helpers
{

stdr_simulation::Pose2D map_to_robot(const stdr_simulation::Pose2D& p_map,
                                     const stdr_simulation::Pose2D& robot_in_map) noexcept
{
  const double dx = p_map.x - robot_in_map.x;
  const double dy = p_map.y - robot_in_map.y;
  const double cos_theta = std::cos(robot_in_map.theta);
  const double sin_theta = std::sin(robot_in_map.theta);
  return {
    .x = cos_theta * dx + sin_theta * dy,
    .y = -sin_theta * dx + cos_theta * dy,
    // Subtract robot heading: the point's orientation measured in robot frame
    // equals its map-frame orientation minus the robot's heading.
    .theta = p_map.theta - robot_in_map.theta,
  };
}

stdr_simulation::Pose2D robot_to_map(const stdr_simulation::Pose2D& p_robot,
                                     const stdr_simulation::Pose2D& robot_in_map) noexcept
{
  const double cos_theta = std::cos(robot_in_map.theta);
  const double sin_theta = std::sin(robot_in_map.theta);
  return {
    .x = robot_in_map.x + cos_theta * p_robot.x - sin_theta * p_robot.y,
    .y = robot_in_map.y + sin_theta * p_robot.x + cos_theta * p_robot.y,
    // Add robot heading: the point's orientation in the map frame equals its
    // robot-frame orientation plus the robot's own heading.
    .theta = p_robot.theta + robot_in_map.theta,
  };
}

bool should_sample(double now_sim_time, double& last_sample_sim_time, double period_seconds) noexcept
{
  if ((now_sim_time - last_sample_sim_time) >= period_seconds)
  {
    last_sample_sim_time = now_sim_time;
    return true;
  }
  return false;
}

}  // namespace stdr::plot::helpers
