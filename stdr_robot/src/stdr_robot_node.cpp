#include "stdr_robot/stdr_robot_node.hpp"

#include <stdr_parser/msg_conversions.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <chrono>
#include <cmath>
#include <string>

namespace stdr_robot
{

namespace
{

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

/**
 * @brief Build a ParameterDescriptor with a floating-point range for declared params.
 *
 * Using descriptors lets external tools (rqt_reconfigure, ros2 param set) know
 * the valid range without relying on the param callback to reject values at
 * runtime.
 */
[[nodiscard]] rcl_interfaces::msg::ParameterDescriptor
make_float_descriptor(const std::string& description, double from_value, double to_value, double step = 0.0)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from_value;
  range.to_value = to_value;
  range.step = step;
  desc.floating_point_range.push_back(range);
  return desc;
}

/**
 * @brief Simulate sensors and publish results.
 *
 * All sensor config types share the same simulate → convert → publish
 * pattern. The compiler selects the correct `stdr_parser::to_ros_msg`
 * overload for each measurement type.
 */
template <typename Config, typename PubPtr, typename Simulator, typename... ExtraArgs>
void simulate_and_publish(const std::vector<Config>& configs, const std::vector<PubPtr>& pubs, Simulator& sim,
                          const stdr_simulation::Pose2D& robot_pose, const std::string& robot_name,
                          const rclcpp::Time& stamp, const ExtraArgs&... extra_args)
{
  for (std::size_t i = 0; i < configs.size(); ++i)
  {
    const Config& cfg = configs[i];
    const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(robot_pose, cfg.pose);
    // Types are unknowable at the template definition site; decltype makes the
    // deduced types explicit without duplicating concrete type names.
    using Measurement = decltype(sim.simulate(world_pose, cfg, extra_args...));
    using RosMsg = decltype(stdr_parser::to_ros_msg(std::declval<Measurement>()));
    const Measurement result = sim.simulate(world_pose, cfg, extra_args...);
    RosMsg msg = stdr_parser::to_ros_msg(result);
    msg.header.stamp = stamp;
    msg.header.frame_id = robot_name + "/" + cfg.frame_id;
    pubs[i]->publish(msg);
  }
}

/** Create publishers and broadcast static TF for a set of sensor configs. */
template <typename MsgT, typename Config>
void create_sensor_publishers_and_tf(rclcpp::Node& node, const std::vector<Config>& configs,
                                     std::vector<typename rclcpp::Publisher<MsgT>::SharedPtr>& pubs,
                                     const std::string& robot_name, tf2_ros::StaticTransformBroadcaster& static_tf,
                                     const rclcpp::Time& stamp)
{
  pubs.clear();
  for (const Config& cfg : configs)
  {
    pubs.push_back(node.create_publisher<MsgT>(robot_name + "/" + cfg.frame_id, 10));

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = robot_name;
    tf_msg.child_frame_id = robot_name + "/" + cfg.frame_id;
    tf_msg.transform.translation.x = cfg.pose.x;
    tf_msg.transform.translation.y = cfg.pose.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = yaw_to_quaternion(cfg.pose.theta);
    static_tf.sendTransform(tf_msg);
  }
}

}  // namespace

// ─── Constructor ────────────────────────────────────────────────────────────

StdrRobotNode::StdrRobotNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("stdr_robot", options)
  , scheduler_(stdr_simulation::kDefaultStepDt)  // placeholder; overwritten below once param is read
{
  declare_parameter<std::string>("robot_name", "");
  robot_name_ = get_parameter("robot_name").as_string();
  if (robot_name_.empty())
  {
    RCLCPP_ERROR(get_logger(), "Parameter 'robot_name' must be set");
    return;
  }

  // Declare rate parameters with advisory floating-point range descriptors.
  // The param callback below performs the actual validation and rejection.
  declare_parameter<double>("sim_step_dt", stdr_simulation::kDefaultStepDt,
                            make_float_descriptor("Simulation physics step duration in seconds.",
                                                  stdr_simulation::kMinStepDt, stdr_simulation::kMaxStepDt));

  declare_parameter<double>("tf_rate", stdr_simulation::kDefaultTfRateHz,
                            make_float_descriptor("TF broadcast rate in Hz. 0 means every sim tick.",
                                                  stdr_simulation::kMinRateHz, stdr_simulation::kMaxRateHz));

  declare_parameter<double>("odom_rate", stdr_simulation::kDefaultOdomRateHz,
                            make_float_descriptor("Odometry publish rate in Hz. 0 means every sim tick.",
                                                  stdr_simulation::kMinRateHz, stdr_simulation::kMaxRateHz));

  sim_step_dt_ = get_parameter("sim_step_dt").as_double();
  tf_rate_ = get_parameter("tf_rate").as_double();
  odom_rate_ = get_parameter("odom_rate").as_double();

  // Re-initialize the scheduler with the validated step dt.
  scheduler_ = stdr_simulation::RateScheduler(sim_step_dt_);

  // Register the param callback to handle dynamic reconfiguration.  The handle
  // must be stored to keep the callback alive; rclcpp weak-references the callback
  // via the handle.
  param_callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Collect proposed updates so we apply all-or-nothing on success.
    double new_sim_step_dt = sim_step_dt_;
    double new_tf_rate = tf_rate_;
    double new_odom_rate = odom_rate_;
    bool sim_step_dt_changed = false;

    for (const rclcpp::Parameter& param : parameters)
    {
      if (param.get_name() == "sim_step_dt")
      {
        const double v = param.as_double();
        if (v < stdr_simulation::kMinStepDt || v > stdr_simulation::kMaxStepDt)
        {
          result.successful = false;
          result.reason = "sim_step_dt must be in [0.001, 1.0]";
          return result;
        }
        new_sim_step_dt = v;
        sim_step_dt_changed = true;
      }
      else if (param.get_name() == "tf_rate")
      {
        const double v = param.as_double();
        if (v < stdr_simulation::kMinRateHz || v > stdr_simulation::kMaxRateHz)
        {
          result.successful = false;
          result.reason = "tf_rate must be in [0.0, 1000.0]";
          return result;
        }
        new_tf_rate = v;
      }
      else if (param.get_name() == "odom_rate")
      {
        const double v = param.as_double();
        if (v < stdr_simulation::kMinRateHz || v > stdr_simulation::kMaxRateHz)
        {
          result.successful = false;
          result.reason = "odom_rate must be in [0.0, 1000.0]";
          return result;
        }
        new_odom_rate = v;
      }
    }

    // All parameters validated — apply updates.
    sim_step_dt_ = new_sim_step_dt;
    tf_rate_ = new_tf_rate;
    odom_rate_ = new_odom_rate;

    if (sim_step_dt_changed)
    {
      scheduler_.set_step_dt(sim_step_dt_);
    }

    scheduler_.set_rate(stdr_simulation::StreamKind::Tf, 0, tf_rate_);
    scheduler_.set_rate(stdr_simulation::StreamKind::Odom, 0, odom_rate_);

    if (sim_step_dt_changed)
    {
      // Serialize sim_timer_ recreation against concurrent parameter callbacks.
      // simulation_step() runs on the same callback group (MutuallyExclusiveCallbackGroup
      // by default), so reentrancy with the timer body is already prevented.
      const std::chrono::nanoseconds new_period(static_cast<std::int64_t>(sim_step_dt_ * 1e9));
      const std::lock_guard<std::mutex> lock(timer_mutex_);
      if (sim_timer_)
      {
        sim_timer_->cancel();
      }
      sim_timer_ = create_wall_timer(new_period, [this]() { simulation_step(); });
    }

    return result;
  });

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

  // Simulation timer driven by the declared sim_step_dt parameter.
  const std::chrono::nanoseconds sim_period(static_cast<std::int64_t>(sim_step_dt_ * 1e9));
  sim_timer_ = create_wall_timer(sim_period, [this]() { simulation_step(); });

  // RegisterRobot action client.
  register_client_ = rclcpp_action::create_client<RegisterRobot>(this, "stdr_server/register_robot");

  // Poll for the action server without blocking the executor thread. DDS
  // discovery is async, but other callbacks (sim timer, subscriptions) need
  // the thread — a blocking wait_for_action_server would starve them.
  constexpr int kMaxRegisterAttempts = 100;  // 100 × 100 ms = 10 s total budget
  register_kickoff_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
    ++register_attempts_;
    if (!register_client_->wait_for_action_server(std::chrono::seconds(0)))
    {
      if (register_attempts_ >= kMaxRegisterAttempts)
      {
        RCLCPP_ERROR(get_logger(), "register_robot action server unavailable after 10 s");
        register_kickoff_timer_->cancel();
      }
      return;
    }
    register_kickoff_timer_->cancel();
    RegisterRobot::Goal goal;
    goal.name = robot_name_;
    rclcpp_action::Client<RegisterRobot>::SendGoalOptions opts;
    opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<RegisterRobot>::WrappedResult& result) {
      on_register_result(result);
    };
    register_client_->async_send_goal(goal, opts);
  });

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
  register_scheduler_streams();
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
  register_scheduler_streams();
  registered_ = true;
  RCLCPP_INFO(get_logger(), "Robot '%s' registered successfully", robot_name_.c_str());
}

// ─── Setup publishers and TF ────────────────────────────────────────────────

void StdrRobotNode::setup_publishers_and_tf()
{
  // Odometry publisher.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(robot_name_ + "/odom", 10);

  const rclcpp::Time stamp = now();

  create_sensor_publishers_and_tf<sensor_msgs::msg::LaserScan>(*this, config_.laser_sensors, laser_pubs_, robot_name_,
                                                               *static_tf_broadcaster_, stamp);
  create_sensor_publishers_and_tf<sensor_msgs::msg::Range>(*this, config_.sonar_sensors, sonar_pubs_, robot_name_,
                                                           *static_tf_broadcaster_, stamp);
  create_sensor_publishers_and_tf<stdr_msgs::msg::RfidSensorMeasurementMsg>(
      *this, config_.rfid_sensors, rfid_pubs_, robot_name_, *static_tf_broadcaster_, stamp);
  create_sensor_publishers_and_tf<stdr_msgs::msg::CO2SensorMeasurementMsg>(*this, config_.co2_sensors, co2_pubs_,
                                                                           robot_name_, *static_tf_broadcaster_, stamp);
  create_sensor_publishers_and_tf<stdr_msgs::msg::SoundSensorMeasurementMsg>(
      *this, config_.sound_sensors, sound_pubs_, robot_name_, *static_tf_broadcaster_, stamp);
  create_sensor_publishers_and_tf<stdr_msgs::msg::ThermalSensorMeasurementMsg>(
      *this, config_.thermal_sensors, thermal_pubs_, robot_name_, *static_tf_broadcaster_, stamp);
}

// ─── Register scheduler streams ────────────────────────────────────────────

void StdrRobotNode::register_scheduler_streams()
{
  scheduler_.clear();

  scheduler_.set_rate(stdr_simulation::StreamKind::Tf, 0, tf_rate_);
  scheduler_.set_rate(stdr_simulation::StreamKind::Odom, 0, odom_rate_);

  for (std::size_t i = 0; i < config_.laser_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::Laser, i, config_.laser_sensors[i].frequency);
  }
  for (std::size_t i = 0; i < config_.sonar_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::Sonar, i, config_.sonar_sensors[i].frequency);
  }
  for (std::size_t i = 0; i < config_.rfid_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::Rfid, i, config_.rfid_sensors[i].frequency);
  }
  for (std::size_t i = 0; i < config_.co2_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::CO2, i, config_.co2_sensors[i].frequency);
  }
  for (std::size_t i = 0; i < config_.thermal_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::Thermal, i, config_.thermal_sensors[i].frequency);
  }
  for (std::size_t i = 0; i < config_.sound_sensors.size(); ++i)
  {
    scheduler_.set_rate(stdr_simulation::StreamKind::Sound, i, config_.sound_sensors[i].frequency);
  }
}

// ─── Rate accessors ─────────────────────────────────────────────────────────

double StdrRobotNode::effective_tf_rate() const
{
  return scheduler_.effective_rate(stdr_simulation::StreamKind::Tf, 0);
}

double StdrRobotNode::effective_odom_rate() const
{
  return scheduler_.effective_rate(stdr_simulation::StreamKind::Odom, 0);
}

double StdrRobotNode::effective_laser_rate(std::size_t index) const
{
  return scheduler_.effective_rate(stdr_simulation::StreamKind::Laser, index);
}

double StdrRobotNode::effective_sonar_rate(std::size_t index) const
{
  return scheduler_.effective_rate(stdr_simulation::StreamKind::Sonar, index);
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

  integrate_motion(sim_step_dt_);
  const rclcpp::Time stamp = now();

  const std::vector<stdr_simulation::StreamEvent> events = scheduler_.tick();

  for (const stdr_simulation::StreamEvent& event : events)
  {
    switch (event.kind)
    {
      case stdr_simulation::StreamKind::Tf:
        broadcast_robot_tf(stamp);
        break;
      case stdr_simulation::StreamKind::Odom:
        publish_odometry(stamp);
        break;
      case stdr_simulation::StreamKind::Laser:
        simulate_laser(event.index, stamp);
        break;
      case stdr_simulation::StreamKind::Sonar:
        simulate_sonar(event.index, stamp);
        break;
      case stdr_simulation::StreamKind::Rfid:
        if (event.index < config_.rfid_sensors.size() && event.index < rfid_pubs_.size())
        {
          simulate_and_publish(std::vector<stdr_simulation::RfidSensorConfig>{ config_.rfid_sensors[event.index] },
                               std::vector<rclcpp::Publisher<stdr_msgs::msg::RfidSensorMeasurementMsg>::SharedPtr>{
                                   rfid_pubs_[event.index] },
                               rfid_sim_, pose_, robot_name_, stamp, rfid_tags_);
        }
        break;
      case stdr_simulation::StreamKind::CO2:
        if (event.index < config_.co2_sensors.size() && event.index < co2_pubs_.size())
        {
          simulate_and_publish(std::vector<stdr_simulation::CO2SensorConfig>{ config_.co2_sensors[event.index] },
                               std::vector<rclcpp::Publisher<stdr_msgs::msg::CO2SensorMeasurementMsg>::SharedPtr>{
                                   co2_pubs_[event.index] },
                               co2_sim_, pose_, robot_name_, stamp, co2_sources_);
        }
        break;
      case stdr_simulation::StreamKind::Thermal:
        if (event.index < config_.thermal_sensors.size() && event.index < thermal_pubs_.size())
        {
          simulate_and_publish(std::vector<stdr_simulation::ThermalSensorConfig>{ config_.thermal_sensors[event.index] },
                               std::vector<rclcpp::Publisher<stdr_msgs::msg::ThermalSensorMeasurementMsg>::SharedPtr>{
                                   thermal_pubs_[event.index] },
                               thermal_sim_, pose_, robot_name_, stamp, thermal_sources_);
        }
        break;
      case stdr_simulation::StreamKind::Sound:
        if (event.index < config_.sound_sensors.size() && event.index < sound_pubs_.size())
        {
          simulate_and_publish(std::vector<stdr_simulation::SoundSensorConfig>{ config_.sound_sensors[event.index] },
                               std::vector<rclcpp::Publisher<stdr_msgs::msg::SoundSensorMeasurementMsg>::SharedPtr>{
                                   sound_pubs_[event.index] },
                               sound_sim_, pose_, robot_name_, stamp, sound_sources_);
        }
        break;
    }
  }
}

// ─── Per-sensor simulation helpers ─────────────────────────────────────────

void StdrRobotNode::simulate_laser(std::size_t index, const rclcpp::Time& stamp)
{
  if (!map_.has_value())
  {
    return;
  }
  if (index >= config_.laser_sensors.size() || index >= laser_pubs_.size())
  {
    return;
  }
  const stdr_simulation::LaserConfig& cfg = config_.laser_sensors[index];
  const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
  const stdr_simulation::LaserScan scan = laser_sim_.simulate(world_pose, cfg, *map_);
  sensor_msgs::msg::LaserScan msg = stdr_parser::to_ros_msg(scan);
  msg.header.stamp = stamp;
  msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
  laser_pubs_[index]->publish(msg);
}

void StdrRobotNode::simulate_sonar(std::size_t index, const rclcpp::Time& stamp)
{
  if (!map_.has_value())
  {
    return;
  }
  if (index >= config_.sonar_sensors.size() || index >= sonar_pubs_.size())
  {
    return;
  }
  const stdr_simulation::SonarConfig& cfg = config_.sonar_sensors[index];
  const stdr_simulation::Pose2D world_pose = stdr_simulation::compute_sensor_world_pose(pose_, cfg.pose);
  const stdr_simulation::SonarScan scan_result = sonar_sim_.simulate(world_pose, cfg, *map_);
  sensor_msgs::msg::Range msg = stdr_parser::to_ros_sonar_msg(scan_result, cfg);
  msg.header.stamp = stamp;
  msg.header.frame_id = robot_name_ + "/" + cfg.frame_id;
  sonar_pubs_[index]->publish(msg);
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
