#pragma once

#include <stdr_gui/simulator_backend.hpp>
#include <stdr_simulation/types.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace stdr_gui_ros
{

/** @brief ROS2-backed implementation of SimulatorBackend.
 *
 *  Subscribes to stdr_server and stdr_robot topics and forwards state to the
 *  GUI via thread-safe snapshot exchange. Also calls stdr_server services and
 *  publishes velocity commands to stdr_robot.
 *
 *  Real subscription and service wiring is implemented in duy.2. This skeleton
 *  provides the class structure and stub overrides so the package compiles and
 *  links before the wiring work begins. */
class Ros2Backend : public stdr_gui::SimulatorBackend
{
public:
  explicit Ros2Backend(std::shared_ptr<rclcpp::Node> node);
  ~Ros2Backend() override;

  [[nodiscard]] tl::expected<void, std::string> load_map(const std::string& yaml_path) override;
  [[nodiscard]] tl::expected<std::string, std::string> spawn_robot(const std::string& yaml_path,
                                                                   const stdr_simulation::Pose2D& pose) override;
  void delete_robot(const std::string& name) override;
  void start() override;
  void pause() override;
  void reset() override;
  void set_speed(double multiplier) override;
  void set_step_dt(double seconds) override;
  [[nodiscard]] double get_step_dt() const override;
  void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) override;
  void set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd) override;
  [[nodiscard]] std::shared_ptr<const stdr_gui::SimulationSnapshot> get_snapshot() const override;
  [[nodiscard]] std::vector<std::string> poll_messages() override;

private:
  void push_message(std::string msg);

  std::shared_ptr<rclcpp::Node> node_;

  mutable std::mutex snapshot_mutex_;
  std::shared_ptr<const stdr_gui::SimulationSnapshot> snapshot_;

  std::mutex msg_mutex_;
  std::vector<std::string> messages_;

  std::atomic<double> step_dt_{ stdr_gui::kDefaultStepDt };
};

}  // namespace stdr_gui_ros
