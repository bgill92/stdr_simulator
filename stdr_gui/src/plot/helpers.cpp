#include <stdr_gui/plot/helpers.hpp>

#include <cmath>
#include <cstddef>

namespace stdr::plot::helpers
{

double wrapped_angle_diff(double a, double b) noexcept
{
  const double diff = a - b;
  return std::atan2(std::sin(diff), std::cos(diff));
}

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

std::vector<Point2> scan_to_map_points(const stdr_simulation::LaserScan& scan,
                                       const stdr_simulation::Pose2D& laser_in_robot,
                                       const stdr_simulation::Pose2D& robot_in_map)
{
  std::vector<Point2> points;
  points.reserve(scan.ranges.size());

  const stdr_simulation::Pose2D sensor_in_map = robot_to_map(laser_in_robot, robot_in_map);

  for (std::size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const double range = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max)
    {
      continue;
    }
    const double angle = sensor_in_map.theta + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    points.push_back({
        .x = sensor_in_map.x + range * std::cos(angle),
        .y = sensor_in_map.y + range * std::sin(angle),
    });
  }

  return points;
}

bool should_sample(double now_sim_time, double& last_sample_sim_time, double period_seconds) noexcept
{
  if (now_sim_time < last_sample_sim_time || (now_sim_time - last_sample_sim_time) >= period_seconds)
  {
    last_sample_sim_time = now_sim_time;
    return true;
  }
  return false;
}

}  // namespace stdr::plot::helpers
