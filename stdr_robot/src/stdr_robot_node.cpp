#include "stdr_robot/stdr_robot_node.hpp"

#include <stdr_parser/msg_conversions.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <string>

namespace stdr_robot
{

namespace
{

constexpr std::chrono::milliseconds kSimulationPeriod{ 100 };
// Derive dt from the timer period so they stay in sync.
constexpr double kSimulationDt = static_cast<double>(kSimulationPeriod.count()) / 1000.0;

/** Convert a yaw angle to a ROS quaternion message. */
[[nodiscard]] geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

}  // namespace

// ─── Constructor ────────────────────────────────────────────────────────────

StdrRobotNode::StdrRobotNode(const rclcpp::NodeOptions& options) : rclcpp::Node("stdr_robot", options)
{
  declare_parameter<std::string>("robot_name", "");
  robot_name_ = get_parameter("robot_name").as_string();
  if (robot_name_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Parameter 'robot_name' must be set");
    return;
  }

  // TF broadcasters.
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Transient local QoS (latched) for map and environment sources.
  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();

  // Map subscription.
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched_qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(msg); });

  // Velocity command subscription.
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      robot_name_ + "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_cmd_vel(msg); });

  // Environment source subscriptions.
  rfid_sub_ = create_subscription<stdr_msgs::msg::RfidTagVector>(
      "stdr_server/rfid_list", latched_qos,
      [this](const stdr_msgs::msg::RfidTagVector::SharedPtr msg) { on_rfid_list(msg); });

  co2_sub_ = create_subscription<stdr_msgs::msg::CO2SourceVector>(
      "stdr_server/co2_sources_list", latched_qos,
      [this](const stdr_msgs::msg::CO2SourceVector::SharedPtr msg) { on_co2_list(msg); });

  thermal_sub_ = create_subscription<stdr_msgs::msg::ThermalSourceVector>(
      "stdr_server/thermal_sources_list", latched_qos,
      [this](const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg) { on_thermal_list(msg); });

  sound_sub_ = create_subscription<stdr_msgs::msg::SoundSourceVector>(
      "stdr_server/sound_sources_list", latched_qos,
      [this](const stdr_msgs::msg::SoundSourceVector::SharedPtr msg) { on_sound_list(msg); });

  // MoveRobot service.
  replace_srv_ = create_service<stdr_msgs::srv::MoveRobot>(
      robot_name_ + "/replace",
      [this](const stdr_msgs::srv::MoveRobot::Request::SharedPtr request,
             stdr_msgs::srv::MoveRobot::Response::SharedPtr response) { handle_move_robot(request, response); });

  // Simulation timer (10 Hz).
  sim_timer_ = create_wall_timer(kSimulationPeriod, [this]() { simulation_step(); });

  // RegisterRobot action client.
  register_client_ = rclcpp_action::create_client<RegisterRobot>(this, "stdr_server/register_robot");

  // Initiate registration asynchronously.
  RegisterRobot::Goal goal;
  goal.name = robot_name_;

  rclcpp_action::Client<RegisterRobot>::SendGoalOptions send_goal_options;
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<RegisterRobot>::WrappedResult& result) {
        on_register_result(result);
      };

  register_client_->async_send_goal(goal, send_goal_options);

  RCLCPP_INFO(get_logger(), "STDR Robot node '%s' created — awaiting registration", robot_name_.c_str());
}

// ─── Configure (for testing) ────────────────────────────────────────────────

void StdrRobotNode::configure(const stdr_simulation::RobotConfig& config)
{
  if (robot_name_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Cannot configure: robot_name is not set");
    return;
  }
  config_ = config;
  pose_ = config_.initial_pose;
  previous_pose_ = pose_;
  setup_publishers_and_tf();
  registered_ = true;
  RCLCPP_INFO(get_logger(), "Robot '%s' configured directly (test mode)", robot_name_.c_str());
}

// ─── Registration result ────────────────────────────────────────────────────

void StdrRobotNode::on_register_result(const rclcpp_action::ClientGoalHandle<RegisterRobot>::WrappedResult& result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(get_logger(), "RegisterRobot failed for '%s'", robot_name_.c_str());
    return;
  }

  config_ = stdr_parser::from_ros_msg(result.result->description);
  pose_ = config_.initial_pose;
  previous_pose_ = pose_;
  setup_publishers_and_tf();
  registered_ = true;
  RCLCPP_INFO(get_logger(), "Robot '%s' registered successfully", robot_name_.c_str());
}

// ─── Setup publishers and TF ────────────────────────────────────────────────

void StdrRobotNode::setup_publishers_and_tf()
{
  // Clear vectors to prevent duplicate publishers on re-configuration.
  laser_pubs_.clear();
  sonar_pubs_.clear();
  rfid_pubs_.clear();
  co2_pubs_.clear();
  sound_pubs_.clear();
  thermal_pubs_.clear();

  // Odometry publisher.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(robot_name_ + "/odom", 10);

  // Per-sensor publishers.
  for (const stdr_simulation::LaserConfig& cfg : config_.laser_sensors)
  {
    laser_pubs_.push_back(create_publisher<sensor_msgs::msg::LaserScan>(robot_name_ + "/" + cfg.frame_id, 10));
  }
  for (const stdr_simulation::SonarConfig& cfg : config_.sonar_sensors)
  {
    sonar_pubs_.push_back(create_publisher<sensor_msgs::msg::Range>(robot_name_ + "/" + cfg.frame_id, 10));
  }
  for (const stdr_simulation::RfidSensorConfig& cfg : config_.rfid_sensors)
  {
    rfid_pubs_.push_back(
        create_publisher<stdr_msgs::msg::RfidSensorMeasurementMsg>(robot_name_ + "/" + cfg.frame_id, 10));
  }
  for (const stdr_simulation::CO2SensorConfig& cfg : config_.co2_sensors)
  {
    co2_pubs_.push_back(create_publisher<stdr_msgs::msg::CO2SensorMeasurementMsg>(robot_name_ + "/" + cfg.frame_id, 10));
  }
  for (const stdr_simulation::SoundSensorConfig& cfg : config_.sound_sensors)
  {
    sound_pubs_.push_back(
        create_publisher<stdr_msgs::msg::SoundSensorMeasurementMsg>(robot_name_ + "/" + cfg.frame_id, 10));
  }
  for (const stdr_simulation::ThermalSensorConfig& cfg : config_.thermal_sensors)
  {
    thermal_pubs_.push_back(
        create_publisher<stdr_msgs::msg::ThermalSensorMeasurementMsg>(robot_name_ + "/" + cfg.frame_id, 10));
  }

  // Static TF for sensor frames (robot → sensor).
  const rclcpp::Time stamp = now();
  auto broadcast_sensor_tf = [&](const std::string& frame_id, const stdr_simulation::Pose2D& sensor_pose) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = robot_name_;
    tf_msg.child_frame_id = robot_name_ + "/" + frame_id;
    tf_msg.transform.translation.x = sensor_pose.x;
    tf_msg.transform.translation.y = sensor_pose.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = yaw_to_quaternion(sensor_pose.theta);
    static_tf_broadcaster_->sendTransform(tf_msg);
  };

  for (const stdr_simulation::LaserConfig& cfg : config_.laser_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
  for (const stdr_simulation::SonarConfig& cfg : config_.sonar_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
  for (const stdr_simulation::RfidSensorConfig& cfg : config_.rfid_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
  for (const stdr_simulation::CO2SensorConfig& cfg : config_.co2_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
  for (const stdr_simulation::SoundSensorConfig& cfg : config_.sound_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
  for (const stdr_simulation::ThermalSensorConfig& cfg : config_.thermal_sensors)
  {
    broadcast_sensor_tf(cfg.frame_id, cfg.pose);
  }
}

// ─── Subscription callbacks ─────────────────────────────────────────────────

void StdrRobotNode::on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  stdr_simulation::OccupancyGrid grid;
  grid.width = static_cast<std::int32_t>(msg->info.width);
  grid.height = static_cast<std::int32_t>(msg->info.height);
  grid.resolution = msg->info.resolution;
  grid.origin.x = msg->info.origin.position.x;
  grid.origin.y = msg->info.origin.position.y;
  grid.data = msg->data;
  map_ = std::move(grid);
}

void StdrRobotNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_.linear_x = msg->linear.x;
  cmd_vel_.linear_y = msg->linear.y;
  cmd_vel_.angular_z = msg->angular.z;
}

void StdrRobotNode::on_rfid_list(const stdr_msgs::msg::RfidTagVector::SharedPtr msg)
{
  rfid_tags_.clear();
  rfid_tags_.reserve(msg->rfid_tags.size());
  for (const stdr_msgs::msg::RfidTag& tag : msg->rfid_tags)
  {
    rfid_tags_.push_back(stdr_parser::from_ros_msg(tag));
  }
}

void StdrRobotNode::on_co2_list(const stdr_msgs::msg::CO2SourceVector::SharedPtr msg)
{
  co2_sources_.clear();
  co2_sources_.reserve(msg->co2_sources.size());
  for (const stdr_msgs::msg::CO2Source& src : msg->co2_sources)
  {
    co2_sources_.push_back(stdr_parser::from_ros_msg(src));
  }
}

void StdrRobotNode::on_thermal_list(const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg)
{
  thermal_sources_.clear();
  thermal_sources_.reserve(msg->thermal_sources.size());
  for (const stdr_msgs::msg::ThermalSource& src : msg->thermal_sources)
  {
    thermal_sources_.push_back(stdr_parser::from_ros_msg(src));
  }
}

void StdrRobotNode::on_sound_list(const stdr_msgs::msg::SoundSourceVector::SharedPtr msg)
{
  sound_sources_.clear();
  sound_sources_.reserve(msg->sound_sources.size());
  for (const stdr_msgs::msg::SoundSource& src : msg->sound_sources)
  {
    sound_sources_.push_back(stdr_parser::from_ros_msg(src));
  }
}

// ─── MoveRobot service ──────────────────────────────────────────────────────

void StdrRobotNode::handle_move_robot(const stdr_msgs::srv::MoveRobot::Request::SharedPtr request,
                                      stdr_msgs::srv::MoveRobot::Response::SharedPtr /*response*/)
{
  if (!registered_)
  {
    RCLCPP_WARN(get_logger(), "MoveRobot rejected: robot not yet registered");
    return;
  }

  const stdr_simulation::Pose2D new_pose = stdr_parser::from_ros_msg(request->new_pose);

  if (map_.has_value())
  {
    if (collision_checker_.check_collision(new_pose, config_.footprint, *map_))
    {
      RCLCPP_WARN(get_logger(), "MoveRobot rejected: collision at (%.2f, %.2f)", new_pose.x, new_pose.y);
      return;
    }
  }

  pose_ = new_pose;
  previous_pose_ = new_pose;
  RCLCPP_INFO(get_logger(), "Robot '%s' moved to (%.2f, %.2f, %.2f)", robot_name_.c_str(), new_pose.x, new_pose.y,
              new_pose.theta);
}

// ─── Simulation timer ───────────────────────────────────────────────────────

void StdrRobotNode::simulation_step()
{
  if (!registered_)
  {
    return;
  }

  integrate_motion(kSimulationDt);
  const rclcpp::Time stamp = now();
  simulate_sensors(stamp);
  publish_odometry(stamp);
  broadcast_robot_tf(stamp);
}

// ─── Motion integration ─────────────────────────────────────────────────────

void StdrRobotNode::integrate_motion(double dt)
{
  // Select motion model based on kinematic type.
  stdr_simulation::Pose2D new_pose;
  if (config_.kinematic_model.type == "omni")
  {
    new_pose = omni_motion_.update(pose_, cmd_vel_, dt, config_.kinematic_model);
  }
  else
  {
    new_pose = ideal_motion_.update(pose_, cmd_vel_, dt, config_.kinematic_model);
  }

  // Collision check against map.
  if (map_.has_value())
  {
    if (!collision_checker_.check_path_collision(new_pose, previous_pose_, config_.footprint, *map_))
    {
      previous_pose_ = pose_;
      pose_ = new_pose;
    }
    // On collision, keep current pose.
  }
  else
  {
    // No map — accept any pose.
    previous_pose_ = pose_;
    pose_ = new_pose;
  }
}

// ─── Sensor simulation ──────────────────────────────────────────────────────

void StdrRobotNode::simulate_sensors(const rclcpp::Time& stamp)
{
  // Laser sensors (require map).
  if (map_.has_value())
  {
    for (std::size_t i = 0; i < config_.laser_sensors.size(); ++i)
    {
      const stdr_simulation::LaserConfig& cfg = config_.laser_sensors[i];
      const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
      const stdr_simulation::LaserScan scan = laser_sim_.simulate(world_pose, cfg, *map_);
      sensor_msgs::msg::LaserScan msg = stdr_parser::to_ros_msg(scan);
      msg.header.stamp = stamp;
      msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
      laser_pubs_[i]->publish(msg);
    }

    // Sonar sensors (require map).
    for (std::size_t i = 0; i < config_.sonar_sensors.size(); ++i)
    {
      const stdr_simulation::SonarConfig& cfg = config_.sonar_sensors[i];
      const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
      const stdr_simulation::SonarScan scan = sonar_sim_.simulate(world_pose, cfg, *map_);
      sensor_msgs::msg::Range msg = stdr_parser::to_ros_sonar_msg(scan, cfg);
      msg.header.stamp = stamp;
      msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
      sonar_pubs_[i]->publish(msg);
    }
  }

  // RFID sensors.
  for (std::size_t i = 0; i < config_.rfid_sensors.size(); ++i)
  {
    const stdr_simulation::RfidSensorConfig& cfg = config_.rfid_sensors[i];
    const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
    const stdr_simulation::RfidMeasurement meas = rfid_sim_.simulate(world_pose, cfg, rfid_tags_);
    stdr_msgs::msg::RfidSensorMeasurementMsg msg = stdr_parser::to_ros_msg(meas);
    msg.header.stamp = stamp;
    msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
    rfid_pubs_[i]->publish(msg);
  }

  // CO2 sensors.
  for (std::size_t i = 0; i < config_.co2_sensors.size(); ++i)
  {
    const stdr_simulation::CO2SensorConfig& cfg = config_.co2_sensors[i];
    const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
    const stdr_simulation::CO2Measurement meas = co2_sim_.simulate(world_pose, cfg, co2_sources_);
    stdr_msgs::msg::CO2SensorMeasurementMsg msg = stdr_parser::to_ros_msg(meas);
    msg.header.stamp = stamp;
    msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
    co2_pubs_[i]->publish(msg);
  }

  // Thermal sensors.
  for (std::size_t i = 0; i < config_.thermal_sensors.size(); ++i)
  {
    const stdr_simulation::ThermalSensorConfig& cfg = config_.thermal_sensors[i];
    const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
    const stdr_simulation::ThermalMeasurement meas = thermal_sim_.simulate(world_pose, cfg, thermal_sources_);
    stdr_msgs::msg::ThermalSensorMeasurementMsg msg = stdr_parser::to_ros_msg(meas);
    msg.header.stamp = stamp;
    msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
    thermal_pubs_[i]->publish(msg);
  }

  // Sound sensors.
  for (std::size_t i = 0; i < config_.sound_sensors.size(); ++i)
  {
    const stdr_simulation::SoundSensorConfig& cfg = config_.sound_sensors[i];
    const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
    const stdr_simulation::SoundMeasurement meas = sound_sim_.simulate(world_pose, cfg, sound_sources_);
    stdr_msgs::msg::SoundSensorMeasurementMsg msg = stdr_parser::to_ros_msg(meas);
    msg.header.stamp = stamp;
    msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
    sound_pubs_[i]->publish(msg);
  }
}

// ─── Publishing ─────────────────────────────────────────────────────────────

void StdrRobotNode::publish_odometry(const rclcpp::Time& stamp)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map_static";
  odom.child_frame_id = robot_name_;
  odom.pose.pose.position.x = pose_.x;
  odom.pose.pose.position.y = pose_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = yaw_to_quaternion(pose_.theta);
  odom.twist.twist.linear.x = cmd_vel_.linear_x;
  odom.twist.twist.linear.y = cmd_vel_.linear_y;
  odom.twist.twist.angular.z = cmd_vel_.angular_z;
  odom_pub_->publish(odom);
}

void StdrRobotNode::broadcast_robot_tf(const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map_static";
  tf_msg.child_frame_id = robot_name_;
  tf_msg.transform.translation.x = pose_.x;
  tf_msg.transform.translation.y = pose_.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = yaw_to_quaternion(pose_.theta);
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace stdr_robot

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(stdr_robot::StdrRobotNode)
