#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_gui/simulator_backend.hpp>

#include <tl_expected/expected.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace stdr_gui_ros
{

Ros2Backend::Ros2Backend(std::shared_ptr<rclcpp::Node> node)
  : node_(std::move(node)), snapshot_(std::make_shared<stdr_gui::SimulationSnapshot>())
{
}

Ros2Backend::~Ros2Backend() = default;

tl::expected<void, std::string> Ros2Backend::load_map(const std::string& /*yaml_path*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed for map loading.
  return tl::make_unexpected("Ros2Backend::load_map: not yet implemented.");
}

tl::expected<std::string, std::string> Ros2Backend::spawn_robot(const std::string& /*yaml_path*/,
                                                                const stdr_simulation::Pose2D& /*pose*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed for spawning a robot.
  return tl::make_unexpected("Ros2Backend::spawn_robot: not yet implemented.");
}

void Ros2Backend::delete_robot(const std::string& /*name*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed for deleting a robot.
  push_message("Ros2Backend::delete_robot: not yet implemented.");
}

void Ros2Backend::start()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed to start simulation.
  push_message("Ros2Backend::start: not yet implemented.");
}

void Ros2Backend::pause()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed to pause simulation.
  push_message("Ros2Backend::pause: not yet implemented.");
}

void Ros2Backend::reset()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed to reset simulation.
  push_message("Ros2Backend::reset: not yet implemented.");
}

void Ros2Backend::set_speed(double /*multiplier*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed to set simulation speed.
  push_message("Ros2Backend::set_speed: not yet implemented.");
}

void Ros2Backend::set_step_dt(double seconds)
{
  // Stored locally; ROS2 propagation to robot/server lands later.
  // FIXME-CLAUDE: NOT IMPLEMENTED - propagate set_step_dt to ROS2 robot/server parameters.
  const double clamped = std::clamp(seconds, stdr_gui::kMinStepDt, stdr_gui::kMaxStepDt);
  step_dt_.store(clamped, std::memory_order_relaxed);
}

double Ros2Backend::get_step_dt() const
{
  return step_dt_.load(std::memory_order_relaxed);
}

void Ros2Backend::set_robot_pose(const std::string& /*name*/, const stdr_simulation::Pose2D& /*pose*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 service call needed to set robot pose.
  push_message("Ros2Backend::set_robot_pose: not yet implemented.");
}

void Ros2Backend::set_cmd_vel(const std::string& /*robot_name*/, const stdr_simulation::Twist2D& /*cmd*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - real ROS2 topic publisher needed to send cmd_vel.
  push_message("Ros2Backend::set_cmd_vel: not yet implemented.");
}

std::shared_ptr<const stdr_gui::SimulationSnapshot> Ros2Backend::get_snapshot() const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

std::vector<std::string> Ros2Backend::poll_messages()
{
  std::vector<std::string> out;
  const std::lock_guard<std::mutex> lock(msg_mutex_);
  out.swap(messages_);
  return out;
}

void Ros2Backend::push_message(std::string msg)
{
  const std::lock_guard<std::mutex> lock(msg_mutex_);
  messages_.push_back(std::move(msg));
}

}  // namespace stdr_gui_ros
