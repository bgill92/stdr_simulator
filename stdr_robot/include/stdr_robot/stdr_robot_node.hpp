#pragma once

/** @file ROS2 composable robot node for STDR simulator. */

#include <stdr_simulation/collision/collision_checker.hpp>
#include <stdr_simulation/geometry_utils.hpp>
#include <stdr_simulation/motion/ideal_motion_model.hpp>
#include <stdr_simulation/motion/omni_motion_model.hpp>
#include <stdr_simulation/rate_scheduler.hpp>
#include <stdr_simulation/sensors/co2_simulator.hpp>
#include <stdr_simulation/sensors/laser_simulator.hpp>
#include <stdr_simulation/sensors/rfid_simulator.hpp>
#include <stdr_simulation/sensors/sonar_simulator.hpp>
#include <stdr_simulation/sensors/sound_simulator.hpp>
#include <stdr_simulation/sensors/thermal_simulator.hpp>
#include <stdr_simulation/types.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <stdr_msgs/action/register_robot.hpp>
#include <stdr_msgs/msg/co2_sensor_measurement_msg.hpp>
#include <stdr_msgs/msg/co2_source_vector.hpp>
#include <stdr_msgs/msg/rfid_sensor_measurement_msg.hpp>
#include <stdr_msgs/msg/rfid_tag_vector.hpp>
#include <stdr_msgs/msg/sound_sensor_measurement_msg.hpp>
#include <stdr_msgs/msg/sound_source_vector.hpp>
#include <stdr_msgs/msg/thermal_sensor_measurement_msg.hpp>
#include <stdr_msgs/msg/thermal_source_vector.hpp>
#include <stdr_msgs/srv/move_robot.hpp>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace stdr_robot
{

class StdrRobotNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct and configure the robot node from ROS parameters.
   *
   * @throws std::invalid_argument if `robot_name` is set to a non-empty but
   *   invalid ROS2 node name, or if `scheduling_mode` is set to a value other
   *   than "snap_to_multiple" or "accumulator".
   */
  explicit StdrRobotNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Configure the robot directly (bypasses RegisterRobot action).
   *
   * Intended for testing — sets up publishers, TF, and enables the simulation
   * timer without needing a running stdr_server.
   */
  void configure(const stdr_simulation::RobotConfig& config);

  // --- Rate accessors (for testing) ---

  /** @return Effective TF broadcast rate in Hz (post-snap). */
  [[nodiscard]] double effective_tf_rate() const;

  /** @return Effective odometry publish rate in Hz (post-snap). */
  [[nodiscard]] double effective_odom_rate() const;

  /** @return Effective laser sensor rate in Hz for the given index (post-snap). */
  [[nodiscard]] double effective_laser_rate(std::size_t index) const;

  /** @return Effective sonar sensor rate in Hz for the given index (post-snap). */
  [[nodiscard]] double effective_sonar_rate(std::size_t index) const;

private:
  using RegisterRobot = stdr_msgs::action::RegisterRobot;

  // --- Simulation timer ---
  void simulation_step();

  // --- Motion integration ---
  void integrate_motion(double dt);

  // --- Per-sensor simulation helpers ---
  void simulate_laser(std::size_t index, const rclcpp::Time& stamp);
  void simulate_sonar(std::size_t index, const rclcpp::Time& stamp);

  // --- Callbacks ---
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_rfid_list(const stdr_msgs::msg::RfidTagVector::SharedPtr msg);
  void on_co2_list(const stdr_msgs::msg::CO2SourceVector::SharedPtr msg);
  void on_thermal_list(const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg);
  void on_sound_list(const stdr_msgs::msg::SoundSourceVector::SharedPtr msg);

  // --- Service callbacks ---
  void handle_move_robot(const stdr_msgs::srv::MoveRobot::Request::SharedPtr request,
                         stdr_msgs::srv::MoveRobot::Response::SharedPtr response);

  // --- Registration ---
  void on_register_result(const rclcpp_action::ClientGoalHandle<RegisterRobot>::WrappedResult& result);

  // --- Setup (called after config is available) ---
  void setup_publishers_and_tf();

  // --- Register sensor streams with the scheduler ---
  void register_scheduler_streams();

  // --- Publishing ---
  void publish_odometry(const rclcpp::Time& stamp);
  void broadcast_robot_tf(const rclcpp::Time& stamp);

  // --- Rate parameters: read in the constructor and kept in sync by the param callback ---
  double sim_step_dt_{ stdr_simulation::kDefaultStepDt };
  double tf_rate_{ stdr_simulation::kDefaultTfRateHz };
  double odom_rate_{ stdr_simulation::kDefaultOdomRateHz };
  stdr_simulation::SchedulingMode scheduling_mode_{ stdr_simulation::SchedulingMode::SnapToMultiple };

  // --- Rate scheduler ---
  // Initialized with sim_step_dt_ after the param is read in the constructor.
  // Streams are registered after setup_publishers_and_tf() populates config_.
  stdr_simulation::RateScheduler scheduler_;

  // Guards sim_timer_ recreation from the parameter callback.  Does NOT need to
  // be held inside simulation_step() because the default MutuallyExclusiveCallbackGroup
  // ensures the param callback and simulation_step() are never concurrent.
  std::mutex timer_mutex_;

  // Keeps the parameter callback active for the lifetime of the node.  rclcpp
  // weak-references the callback handle; if the handle is destroyed, the callback
  // is silently unregistered.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // --- State ---
  std::string robot_name_;
  stdr_simulation::Pose2D pose_{};
  stdr_simulation::Pose2D previous_pose_{};
  stdr_simulation::Twist2D cmd_vel_{};
  stdr_simulation::RobotConfig config_;
  std::optional<stdr_simulation::OccupancyGrid> map_;
  bool registered_{ false };
  int register_attempts_{ 0 };

  // Cached environment sources.
  std::vector<stdr_simulation::RfidTag> rfid_tags_;
  std::vector<stdr_simulation::CO2Source> co2_sources_;
  std::vector<stdr_simulation::ThermalSource> thermal_sources_;
  std::vector<stdr_simulation::SoundSource> sound_sources_;

  // --- Simulators ---
  stdr_simulation::motion::IdealMotionModel ideal_motion_;
  stdr_simulation::motion::OmniMotionModel omni_motion_;
  stdr_simulation::collision::CollisionChecker collision_checker_;
  stdr_simulation::sensors::LaserSimulator laser_sim_;
  stdr_simulation::sensors::SonarSimulator sonar_sim_;
  stdr_simulation::sensors::RfidSimulator rfid_sim_;
  stdr_simulation::sensors::Co2Simulator co2_sim_;
  stdr_simulation::sensors::ThermalSimulator thermal_sim_;
  stdr_simulation::sensors::SoundSimulator sound_sim_;

  // --- TF ---
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // --- Publishers ---
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> laser_pubs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> sonar_pubs_;
  std::vector<rclcpp::Publisher<stdr_msgs::msg::RfidSensorMeasurementMsg>::SharedPtr> rfid_pubs_;
  std::vector<rclcpp::Publisher<stdr_msgs::msg::CO2SensorMeasurementMsg>::SharedPtr> co2_pubs_;
  std::vector<rclcpp::Publisher<stdr_msgs::msg::SoundSensorMeasurementMsg>::SharedPtr> sound_pubs_;
  std::vector<rclcpp::Publisher<stdr_msgs::msg::ThermalSensorMeasurementMsg>::SharedPtr> thermal_pubs_;

  // --- Subscriptions ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<stdr_msgs::msg::RfidTagVector>::SharedPtr rfid_sub_;
  rclcpp::Subscription<stdr_msgs::msg::CO2SourceVector>::SharedPtr co2_sub_;
  rclcpp::Subscription<stdr_msgs::msg::ThermalSourceVector>::SharedPtr thermal_sub_;
  rclcpp::Subscription<stdr_msgs::msg::SoundSourceVector>::SharedPtr sound_sub_;

  // --- Services ---
  rclcpp::Service<stdr_msgs::srv::MoveRobot>::SharedPtr replace_srv_;

  // --- Timers ---
  rclcpp::TimerBase::SharedPtr sim_timer_;
  rclcpp::TimerBase::SharedPtr register_kickoff_timer_;

  // --- Action client ---
  rclcpp_action::Client<RegisterRobot>::SharedPtr register_client_;
};

}  // namespace stdr_robot
