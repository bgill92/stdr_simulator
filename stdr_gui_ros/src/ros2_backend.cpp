#include "stdr_gui_ros/ros2_backend.hpp"

#include <rclcpp/parameter_event_handler.hpp>

#include <stdr_gui/simulator_backend.hpp>
#include <stdr_parser/msg_conversions.hpp>
#include <stdr_simulation/plot_data/spsc_ring.hpp>
#include <stdr_simulation/rate_scheduler.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tl_expected/expected.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace stdr_gui_ros
{

Ros2Backend::Ros2Backend(std::shared_ptr<rclcpp::Node> node)
  : node_(std::move(node))
  , start_time_(node_->now())
  , alive_(std::make_shared<std::atomic<bool>>(true))
  , param_event_handler_(std::make_shared<rclcpp::ParameterEventHandler>(node_))
{
  // Transient local (latched) QoS to match stdr_server publishers, so a
  // late-joining GUI still receives the current map and robot list immediately.
  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched_qos, [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { on_map(msg); });

  active_robots_sub_ = node_->create_subscription<stdr_msgs::msg::RobotIndexedVectorMsg>(
      "stdr_server/active_robots", latched_qos,
      [this](const stdr_msgs::msg::RobotIndexedVectorMsg::SharedPtr msg) { on_active_robots(msg); });

  rfid_sub_ = node_->create_subscription<stdr_msgs::msg::RfidTagVector>(
      "stdr_server/rfid_list", latched_qos,
      [this](const stdr_msgs::msg::RfidTagVector::SharedPtr msg) { on_rfid_list(msg); });

  co2_sub_ = node_->create_subscription<stdr_msgs::msg::CO2SourceVector>(
      "stdr_server/co2_sources_list", latched_qos,
      [this](const stdr_msgs::msg::CO2SourceVector::SharedPtr msg) { on_co2_list(msg); });

  thermal_sub_ = node_->create_subscription<stdr_msgs::msg::ThermalSourceVector>(
      "stdr_server/thermal_sources_list", latched_qos,
      [this](const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg) { on_thermal_list(msg); });

  sound_sub_ = node_->create_subscription<stdr_msgs::msg::SoundSourceVector>(
      "stdr_server/sound_sources_list", latched_qos,
      [this](const stdr_msgs::msg::SoundSourceVector::SharedPtr msg) { on_sound_list(msg); });
}

Ros2Backend::~Ros2Backend()
{
  alive_->store(false);
}

// ─── Map loading ─────────────────────────────────────────────────────────────

tl::expected<void, std::string> Ros2Backend::load_map(const std::string& yaml_path)
{
  // Capture a local copy of the shared_ptr while holding the lock so a second
  // concurrent call sees the same client object rather than creating a duplicate.
  rclcpp::Client<stdr_msgs::srv::LoadMap>::SharedPtr client;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!load_map_client_)
    {
      load_map_client_ = node_->create_client<stdr_msgs::srv::LoadMap>("stdr_server/load_static_map");
    }
    client = load_map_client_;
  }

  if (!client->wait_for_service(std::chrono::seconds(1)))
  {
    return tl::make_unexpected("load_map: stdr_server/load_static_map service not available.");
  }

  stdr_msgs::srv::LoadMap::Request::SharedPtr request = std::make_shared<stdr_msgs::srv::LoadMap::Request>();
  request->map_file = yaml_path;

  std::shared_future<stdr_msgs::srv::LoadMap::Response::SharedPtr> future =
      client->async_send_request(request).future.share();

  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
  {
    return tl::make_unexpected("load_map: request timed out after 5 seconds.");
  }

  // The map subscription will update the cached map when stdr_server re-publishes.
  // Record the filename here so get_snapshot() can return it immediately.
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    map_name_ = std::filesystem::path(yaml_path).filename().string();
  }

  push_message("Loaded map: " + yaml_path);
  return {};
}

// ─── Stubs (no ROS2 service exists for these yet) ────────────────────────────

tl::expected<std::string, std::string> Ros2Backend::spawn_robot(const std::string& /*yaml_path*/,
                                                                const stdr_simulation::Pose2D& /*pose*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - no ROS2 spawn service in GUI path yet; robot
  // lifecycle is managed by launch files and the stdr_server spawn action.
  return tl::make_unexpected("Ros2Backend::spawn_robot: no ROS2 spawn service exists yet.");
}

void Ros2Backend::delete_robot(const std::string& /*name*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - no ROS2 delete service in GUI path yet;
  // robot removal is managed via the stdr_server delete action.
  push_message("Ros2Backend::delete_robot: no ROS2 delete service exists yet.");
}

void Ros2Backend::start()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - central pause/resume not implemented in ROS2 mode.
  push_message("Ros2Backend::start: central start not implemented in ROS2 mode.");
}

void Ros2Backend::pause()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - central pause/resume not implemented in ROS2 mode.
  push_message("Ros2Backend::pause: central pause not implemented in ROS2 mode.");
}

void Ros2Backend::reset()
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - central reset not implemented in ROS2 mode.
  push_message("Ros2Backend::reset: central reset not implemented in ROS2 mode.");
}

void Ros2Backend::set_speed(double /*multiplier*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - central speed multiplier not implemented in ROS2 mode.
  push_message("Ros2Backend::set_speed: central speed control not implemented in ROS2 mode.");
}

void Ros2Backend::set_step_dt(double seconds)
{
  const double clamped = std::clamp(seconds, stdr_gui::kMinStepDt, stdr_gui::kMaxStepDt);
  step_dt_.store(clamped, std::memory_order_relaxed);
  propagate_parameter("sim_step_dt", clamped);
}

double Ros2Backend::get_step_dt() const
{
  return step_dt_.load(std::memory_order_relaxed);
}

void Ros2Backend::set_tf_rate(double hz)
{
  const double clamped = std::clamp(hz, stdr_gui::kMinRateHz, stdr_gui::kMaxRateHz);
  tf_rate_.store(clamped, std::memory_order_relaxed);
  propagate_parameter("tf_rate", clamped);
}

double Ros2Backend::get_tf_rate() const
{
  return tf_rate_.load(std::memory_order_relaxed);
}

void Ros2Backend::set_odom_rate(double hz)
{
  const double clamped = std::clamp(hz, stdr_gui::kMinRateHz, stdr_gui::kMaxRateHz);
  odom_rate_.store(clamped, std::memory_order_relaxed);
  propagate_parameter("odom_rate", clamped);
}

double Ros2Backend::get_odom_rate() const
{
  return odom_rate_.load(std::memory_order_relaxed);
}

stdr_simulation::SchedulingMode Ros2Backend::get_scheduling_mode() const
{
  return scheduling_mode_.load(std::memory_order_relaxed);
}

// ─── Parameter propagation ────────────────────────────────────────────────────

void Ros2Backend::propagate_parameter(const std::string& param_name, double value)
{
  // Snapshot the client list under the lock so we don't hold it during the
  // async calls, which may spin the executor.
  std::vector<rclcpp::AsyncParametersClient::SharedPtr> clients;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    clients.reserve(param_clients_.size());
    for (const auto& [name, client] : param_clients_)
    {
      clients.push_back(client);
    }
  }

  for (const rclcpp::AsyncParametersClient::SharedPtr& client : clients)
  {
    if (!client->service_is_ready())
    {
      RCLCPP_WARN(node_->get_logger(), "Parameter service not ready for a robot node; skipping '%s' update.",
                  param_name.c_str());
      continue;
    }
    // Fire-and-forget: let the future destruct without awaiting. The request
    // has already been sent before the future destructs.
    std::ignore = client->set_parameters({ rclcpp::Parameter(param_name, value) });
  }
}

// ─── Robot pose (per-robot service call) ─────────────────────────────────────

void Ros2Backend::set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose)
{
  rclcpp::Client<stdr_msgs::srv::MoveRobot>::SharedPtr client;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);
    auto it = move_robot_clients_.find(name);
    if (it == move_robot_clients_.end())
    {
      rclcpp::Client<stdr_msgs::srv::MoveRobot>::SharedPtr new_client =
          node_->create_client<stdr_msgs::srv::MoveRobot>(name + "/replace");
      move_robot_clients_[name] = new_client;
      client = new_client;
    }
    else
    {
      client = it->second;
    }
  }

  stdr_msgs::srv::MoveRobot::Request::SharedPtr request = std::make_shared<stdr_msgs::srv::MoveRobot::Request>();
  request->new_pose = stdr_parser::to_ros_msg(pose);

  // Send asynchronously to avoid blocking the GUI thread. The lambda captures
  // the alive sentinel rather than raw `this` so it remains safe if the backend
  // is destroyed before the service call completes.
  std::weak_ptr<std::atomic<bool>> weak_alive = alive_;
  client->async_send_request(request, [this, name,
                                       weak_alive](rclcpp::Client<stdr_msgs::srv::MoveRobot>::SharedFuture /*future*/) {
    const std::shared_ptr<std::atomic<bool>> alive = weak_alive.lock();
    if (!alive || !alive->load())
    {
      return;
    }
    push_message("set_robot_pose: move request completed for robot '" + name + "'.");
  });
}

// ─── Velocity commands ────────────────────────────────────────────────────────

void Ros2Backend::set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd)
{
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
  std::string error_msg;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);

    // Validate that the robot is known before publishing, to catch stale names.
    const bool robot_known =
        std::any_of(robot_states_.begin(), robot_states_.end(),
                    [&robot_name](const stdr_simulation::world::RobotState& s) { return s.name == robot_name; });
    if (!robot_known)
    {
      // Build the message under the lock, push it after to avoid holding two locks.
      error_msg = "set_cmd_vel: robot '" + robot_name + "' not in active-robots list.";
    }
    else
    {
      // Cache the commanded velocity so twist() can return it.
      latest_cmd_vel_[robot_name] = cmd;

      auto it = cmd_vel_pubs_.find(robot_name);
      if (it == cmd_vel_pubs_.end())
      {
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr new_pub =
            node_->create_publisher<geometry_msgs::msg::Twist>(robot_name + "/cmd_vel", 10);
        cmd_vel_pubs_[robot_name] = new_pub;
        pub = new_pub;
      }
      else
      {
        pub = it->second;
      }
    }
  }

  if (!error_msg.empty())
  {
    push_message(std::move(error_msg));
    return;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x = cmd.linear_x;
  msg.linear.y = cmd.linear_y;
  msg.angular.z = cmd.angular_z;
  pub->publish(msg);
}

// ─── Snapshot ─────────────────────────────────────────────────────────────────

std::shared_ptr<const stdr_gui::SimulationSnapshot> Ros2Backend::get_snapshot() const
{
  stdr_gui::SimulationSnapshot snapshot;
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);

    snapshot.map = map_;
    snapshot.map_name = map_name_;
    snapshot.rfid_tags = rfid_tags_;
    snapshot.co2_sources = co2_sources_;
    snapshot.thermal_sources = thermal_sources_;
    snapshot.sound_sources = sound_sources_;

    // Build the robot list with the most recent odom-derived poses. Fall back
    // to config.initial_pose if no odom has been received yet for a robot.
    snapshot.robots = robot_states_;
    for (stdr_simulation::world::RobotState& state : snapshot.robots)
    {
      const auto pose_it = latest_pose_.find(state.name);
      if (pose_it != latest_pose_.end())
      {
        state.pose = pose_it->second;
      }
    }

    // Sensor measurement reverse conversions (LaserScan → simulation type) are
    // not yet implemented, so sensor_data is left empty.
  }

  // The GUI uses `running` only for the play/pause indicator. In ROS2 mode
  // there is no central lifecycle, so we approximate: if any robots are active
  // the simulation is considered "running".
  snapshot.elapsed_time = sim_time();
  snapshot.running = !snapshot.robots.empty();

  return std::make_shared<stdr_gui::SimulationSnapshot>(std::move(snapshot));
}

// ─── Message queue ─────────────────────────────────────────────────────────────

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

// ─── Introspection ────────────────────────────────────────────────────────────
//
// Each method acquires snapshot_mutex_ for the duration of the call so the
// returned values are internally consistent. This mirrors the StandaloneBackend
// pattern which holds sim_mutex_ for the same purpose.

std::size_t Ros2Backend::num_robots() const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return robot_states_.size();
}

std::vector<std::string> Ros2Backend::robot_ids() const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  std::vector<std::string> ids;
  ids.reserve(robot_states_.size());
  for (const stdr_simulation::world::RobotState& state : robot_states_)
  {
    ids.push_back(state.name);
  }
  return ids;
}

std::vector<std::string> Ros2Backend::laser_sensors(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto it = std::find_if(robot_states_.begin(), robot_states_.end(),
                               [&robot_id](const stdr_simulation::world::RobotState& s) { return s.name == robot_id; });
  if (it == robot_states_.end())
  {
    return {};
  }
  std::vector<std::string> ids;
  ids.reserve(it->config.laser_sensors.size());
  for (const stdr_simulation::LaserConfig& cfg : it->config.laser_sensors)
  {
    ids.push_back(cfg.frame_id);
  }
  return ids;
}

std::vector<std::string> Ros2Backend::sonar_sensors(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto it = std::find_if(robot_states_.begin(), robot_states_.end(),
                               [&robot_id](const stdr_simulation::world::RobotState& s) { return s.name == robot_id; });
  if (it == robot_states_.end())
  {
    return {};
  }
  std::vector<std::string> ids;
  ids.reserve(it->config.sonar_sensors.size());
  for (const stdr_simulation::SonarConfig& cfg : it->config.sonar_sensors)
  {
    ids.push_back(cfg.frame_id);
  }
  return ids;
}

std::optional<stdr_simulation::Pose2D> Ros2Backend::pose(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto it = latest_pose_.find(robot_id);
  if (it == latest_pose_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<stdr_simulation::Twist2D> Ros2Backend::twist(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  // Return the most-recently commanded velocity, matching StandaloneBackend's
  // cmd_vel semantics: the value last passed to set_cmd_vel(), not measured odom.
  const auto it = latest_cmd_vel_.find(robot_id);
  if (it == latest_cmd_vel_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

std::optional<bool> Ros2Backend::collided(const std::string& robot_id) const
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - stdr_robot does not publish collision state.
  // Tracked separately.
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const bool robot_known =
      std::any_of(robot_states_.begin(), robot_states_.end(),
                  [&robot_id](const stdr_simulation::world::RobotState& s) { return s.name == robot_id; });
  if (!robot_known)
  {
    return std::nullopt;
  }
  return false;
}

std::vector<stdr_simulation::Point2D> Ros2Backend::footprint(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  const auto it = std::find_if(robot_states_.begin(), robot_states_.end(),
                               [&robot_id](const stdr_simulation::world::RobotState& s) { return s.name == robot_id; });
  if (it == robot_states_.end())
  {
    return {};
  }
  return it->config.footprint.points;
}

double Ros2Backend::sim_time() const
{
  // Wall-clock since construction; no discrete simulation steps exist in ROS2 mode.
  return (node_->now() - start_time_).seconds();
}

std::optional<stdr_simulation::LaserScan> Ros2Backend::latest_laser(const std::string& robot_id,
                                                                    const std::string& sensor_id) const
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  const auto it = latest_laser_.find(key);
  if (it == latest_laser_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

stdr_gui::DrainedLaserResult Ros2Backend::poll_laser_events(std::uint64_t& cursor, const std::string& robot_id,
                                                            const std::string& sensor_id)
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  const auto it = laser_rings_.find(key);
  if (it == laser_rings_.end())
  {
    return {};
  }
  const stdr::plot_data::DrainResult<stdr::plot::TimedLaserScan> result = it->second->drain(cursor);
  // Copy the span into an owning vector while still holding ring_lock.  The
  // ROS callback thread can call push() as soon as the lock is released, so
  // any access to result.entries after this scope would be a data race.
  return stdr_gui::DrainedLaserResult{
    std::vector<stdr::plot::TimedLaserScan>(result.entries.begin(), result.entries.end()), result.dropped
  };
}

std::optional<stdr_simulation::SonarScan> Ros2Backend::latest_sonar(const std::string& robot_id,
                                                                    const std::string& sensor_id) const
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  const auto it = latest_sonar_.find(key);
  if (it == latest_sonar_.end())
  {
    return std::nullopt;
  }
  return it->second;
}

stdr_gui::DrainedSonarResult Ros2Backend::poll_sonar_events(std::uint64_t& cursor, const std::string& robot_id,
                                                            const std::string& sensor_id)
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  const auto it = sonar_rings_.find(key);
  if (it == sonar_rings_.end())
  {
    return {};
  }
  const stdr::plot_data::DrainResult<stdr::plot::TimedSonarReading> result = it->second->drain(cursor);
  // Copy the span into an owning vector while still holding ring_lock.  The
  // ROS callback thread can call push() as soon as the lock is released, so
  // any access to result.entries after this scope would be a data race.
  return stdr_gui::DrainedSonarResult{
    std::vector<stdr::plot::TimedSonarReading>(result.entries.begin(), result.entries.end()), result.dropped
  };
}

// ─── Subscription callbacks ──────────────────────────────────────────────────

void Ros2Backend::on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  stdr_simulation::OccupancyGrid grid;
  grid.width = static_cast<std::int32_t>(msg->info.width);
  grid.height = static_cast<std::int32_t>(msg->info.height);
  grid.resolution = msg->info.resolution;
  grid.origin.x = msg->info.origin.position.x;
  grid.origin.y = msg->info.origin.position.y;
  grid.data = msg->data;

  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  map_ = std::move(grid);
}

void Ros2Backend::on_active_robots(const stdr_msgs::msg::RobotIndexedVectorMsg::SharedPtr msg)
{
  // Pre-parse all robot configs once before any lock is acquired.  This avoids
  // calling from_ros_msg() a second time in phase 4 when we enumerate laser
  // sensors for subscription creation.
  std::vector<stdr_simulation::RobotConfig> parsed_configs;
  parsed_configs.reserve(msg->robots.size());
  for (const stdr_msgs::msg::RobotIndexedMsg& entry : msg->robots)
  {
    parsed_configs.push_back(stdr_parser::from_ros_msg(entry.robot));
  }

  // Build the set of names in the new message so we can diff against the
  // current set efficiently.
  std::unordered_set<std::string> new_names;
  new_names.reserve(msg->robots.size());
  for (const stdr_msgs::msg::RobotIndexedMsg& entry : msg->robots)
  {
    new_names.insert(entry.name);
  }

  // Collect the laser and sonar subscription keys belonging to robots that
  // disappeared.  laser_subs_ and sonar_subs_ are only touched on the executor
  // thread so no lock is required here.
  std::vector<std::string> laser_keys_to_remove;
  for (const auto& [key, unused_sub] : laser_subs_)
  {
    const std::string::size_type slash = key.find('/');
    if (slash == std::string::npos)
    {
      continue;
    }
    if (new_names.find(key.substr(0, slash)) == new_names.end())
    {
      laser_keys_to_remove.push_back(key);
    }
  }

  std::vector<std::string> sonar_keys_to_remove;
  for (const auto& [key, unused_sub] : sonar_subs_)
  {
    const std::string::size_type slash = key.find('/');
    if (slash == std::string::npos)
    {
      continue;
    }
    if (new_names.find(key.substr(0, slash)) == new_names.end())
    {
      sonar_keys_to_remove.push_back(key);
    }
  }

  // Destroy subscriptions on the executor thread before acquiring any lock.
  for (const std::string& key : laser_keys_to_remove)
  {
    laser_subs_.erase(key);
  }
  for (const std::string& key : sonar_keys_to_remove)
  {
    sonar_subs_.erase(key);
  }

  // Erase ring data for removed keys under sensor_ring_mutex_.
  // This lock is disjoint from snapshot_mutex_ — never hold both simultaneously.
  {
    const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
    for (const std::string& key : laser_keys_to_remove)
    {
      latest_laser_.erase(key);
      laser_rings_.erase(key);
    }
    for (const std::string& key : sonar_keys_to_remove)
    {
      latest_sonar_.erase(key);
      sonar_rings_.erase(key);
    }
  }

  // Now update snapshot state under snapshot_mutex_.
  {
    const std::lock_guard<std::mutex> lock(snapshot_mutex_);

    // Drop odom subscriptions and cached state for robots that disappeared.
    // Iterating the current map and erasing by name is safe here because we hold
    // the lock and no other thread modifies these containers.
    std::vector<std::string> to_remove;
    for (const auto& [name, unused_sub] : odom_subs_)
    {
      if (new_names.find(name) == new_names.end())
      {
        to_remove.push_back(name);
      }
    }
    for (const std::string& name : to_remove)
    {
      odom_subs_.erase(name);
      latest_pose_.erase(name);
      latest_cmd_vel_.erase(name);
      cmd_vel_pubs_.erase(name);
      move_robot_clients_.erase(name);
      param_clients_.erase(name);
      // Erasing the handles de-registers the parameter event callbacks for
      // this robot.  The ParameterEventHandler weak-references each handle,
      // so destruction here is sufficient.
      param_event_subs_.erase(name);
      // Cancel any in-progress seed timer before erasing it.  If the timer
      // callback is currently executing, cancel() is idempotent and safe; the
      // map entry is then destroyed here on the same executor thread.
      if (const auto it = seed_timers_.find(name); it != seed_timers_.end())
      {
        it->second->cancel();
        seed_timers_.erase(it);
      }
    }

    // Build the new robot_states_ vector and create odom subs for new robots.
    // We do NOT reset existing odom subs when a robot is still present: dropping
    // and recreating a subscription would lose any in-flight messages and cause a
    // brief pose gap visible to the GUI.
    std::vector<stdr_simulation::world::RobotState> new_states;
    new_states.reserve(msg->robots.size());

    for (std::size_t i = 0; i < msg->robots.size(); ++i)
    {
      const stdr_msgs::msg::RobotIndexedMsg& entry = msg->robots[i];
      stdr_simulation::world::RobotState state;
      state.name = entry.name;
      state.config = parsed_configs[i];
      state.pose = state.config.initial_pose;

      // The active-robots topic carries identity and config, not runtime command
      // state. Preserve any in-flight cmd_vel for robots that are already known
      // so that a server republish (e.g. after an env-source change) does not
      // silently zero out velocity commands mid-move.
      const auto existing_it =
          std::find_if(robot_states_.begin(), robot_states_.end(),
                       [&entry](const stdr_simulation::world::RobotState& s) { return s.name == entry.name; });
      if (existing_it != robot_states_.end())
      {
        state.cmd_vel = existing_it->cmd_vel;
      }
      // New robots default-construct cmd_vel (zero).

      if (odom_subs_.find(entry.name) == odom_subs_.end())
      {
        // New robot — create its odom subscription and seed the pose cache.
        const std::string robot_name = entry.name;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub =
            node_->create_subscription<nav_msgs::msg::Odometry>(
                robot_name + "/odom", 10, [this, robot_name](const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
                  on_odom(robot_name, odom_msg);
                });
        odom_subs_[robot_name] = sub;
        latest_pose_[robot_name] = state.config.initial_pose;

        // Create an AsyncParametersClient targeting this robot's own node.
        // StdrRobotNode derives its node name from the robot_name parameter
        // override (see node_name_from_options in stdr_robot_node.cpp), so
        // /robot0's parameter service lives on /robot0, /robot1 on /robot1,
        // etc.  Targeting by robot_name makes multi-robot propagation correct:
        // set_step_dt on the GUI reaches exactly the intended robot node.
        rclcpp::AsyncParametersClient::SharedPtr new_client =
            std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_name);
        param_clients_[robot_name] = new_client;

        // Register parameter event callbacks so external changes (e.g. via
        // `ros2 param set`) are reflected in the backend's atomic caches.
        // This keeps the GUI info panel accurate even when parameters are
        // changed outside the GUI.
        //
        // The weak_alive guard is needed because the ParameterCallbackHandle
        // destructor calls into the middleware, which may deliver a final
        // callback after the backend's atomic members have been destroyed.
        // Checking alive_ before touching any member prevents use-after-free
        // during backend teardown.
        std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> handles;
        const std::weak_ptr<std::atomic<bool>> weak_alive_param = alive_;

        handles.push_back(param_event_handler_->add_parameter_callback(
            "sim_step_dt",
            [weak_alive_param, this](const rclcpp::Parameter& p) {
              const std::shared_ptr<std::atomic<bool>> live = weak_alive_param.lock();
              if (!live || !live->load(std::memory_order_relaxed))
              {
                return;
              }
              step_dt_.store(p.as_double(), std::memory_order_relaxed);
            },
            robot_name));

        handles.push_back(param_event_handler_->add_parameter_callback(
            "tf_rate",
            [weak_alive_param, this](const rclcpp::Parameter& p) {
              const std::shared_ptr<std::atomic<bool>> live = weak_alive_param.lock();
              if (!live || !live->load(std::memory_order_relaxed))
              {
                return;
              }
              tf_rate_.store(p.as_double(), std::memory_order_relaxed);
            },
            robot_name));

        handles.push_back(param_event_handler_->add_parameter_callback(
            "odom_rate",
            [weak_alive_param, this](const rclcpp::Parameter& p) {
              const std::shared_ptr<std::atomic<bool>> live = weak_alive_param.lock();
              if (!live || !live->load(std::memory_order_relaxed))
              {
                return;
              }
              odom_rate_.store(p.as_double(), std::memory_order_relaxed);
            },
            robot_name));

        handles.push_back(param_event_handler_->add_parameter_callback(
            "scheduling_mode",
            [weak_alive_param, this, robot_name](const rclcpp::Parameter& p) {
              const std::shared_ptr<std::atomic<bool>> live = weak_alive_param.lock();
              if (!live || !live->load(std::memory_order_relaxed))
              {
                return;
              }
              const tl::expected<stdr_simulation::SchedulingMode, std::string> mode =
                  stdr_simulation::scheduling_mode_from_string(p.as_string());
              if (mode.has_value())
              {
                scheduling_mode_.store(mode.value(), std::memory_order_relaxed);
              }
              else
              {
                RCLCPP_WARN(node_->get_logger(), "Ignoring invalid scheduling_mode '%s' from robot '%s': %s",
                            p.as_string().c_str(), robot_name.c_str(), mode.error().c_str());
              }
            },
            robot_name));

        param_event_subs_[robot_name] = std::move(handles);

        // Retry seed: read the robot's current parameter values so the GUI
        // reflects actuals rather than defaults on first render.
        //
        // service_is_ready() alone is racy: the param service may not be
        // discoverable yet immediately after the robot node starts.  We install
        // a 50 ms retry timer that checks once per tick.  If the service is
        // ready and no request is outstanding, we fire get_parameters().  The
        // result callback cancels the timer on success, or clears the in-flight
        // flag on failure so the next tick can retry.  The timer gives up only
        // after kMaxSeedAttempts as a backstop against a robot that never
        // responds (up to ~10 s of wall time; ticks while a request is in
        // flight are no-ops and do not count as useful round trips).
        constexpr int kMaxSeedAttempts = 200;
        // Shared attempt counter so the lambda owns its own lifetime without
        // needing to reach into a container.
        const std::shared_ptr<int> attempt_count = std::make_shared<int>(0);
        // Guards against issuing a second get_parameters while one is already
        // in flight.  Both the timer callback and the result callback run on the
        // same executor thread, so a plain bool is safe — no data race.
        const std::shared_ptr<bool> request_in_flight = std::make_shared<bool>(false);
        const std::weak_ptr<std::atomic<bool>> weak_alive_seed = alive_;

        // Store the timer in seed_timers_ so cancellation is always possible:
        // the timer fires after on_active_robots() returns, at which point any
        // local shared_ptr to the timer would be gone.  A member map keeps the
        // strong reference alive and lets the callbacks cancel reliably.
        seed_timers_[robot_name] =
            node_->create_wall_timer(std::chrono::milliseconds(50), [this, robot_name, new_client, attempt_count,
                                                                     request_in_flight, weak_alive_seed]() {
              // Cancel if the backend has been destroyed.
              const std::shared_ptr<std::atomic<bool>> live = weak_alive_seed.lock();
              if (!live || !live->load(std::memory_order_relaxed))
              {
                if (const auto it = seed_timers_.find(robot_name); it != seed_timers_.end())
                {
                  it->second->cancel();
                }
                return;
              }

              ++(*attempt_count);
              if (*attempt_count > kMaxSeedAttempts)
              {
                if (const auto it = seed_timers_.find(robot_name); it != seed_timers_.end())
                {
                  it->second->cancel();
                }
                RCLCPP_WARN(node_->get_logger(),
                            "Parameter seed for robot '%s' gave up after %d attempts — GUI may show stale defaults.",
                            robot_name.c_str(), kMaxSeedAttempts);
                return;
              }

              // A request is already outstanding; wait for its result before sending another.
              if (*request_in_flight)
              {
                return;
              }

              if (!new_client->service_is_ready())
              {
                return;  // Not ready yet — try again next tick.
              }

              // Service is ready and no request is in flight: issue get_parameters.
              // The timer stays running; the result callback cancels it on success.
              *request_in_flight = true;

              std::ignore = new_client->get_parameters(
                  { "sim_step_dt", "tf_rate", "odom_rate", "scheduling_mode" },
                  [this, weak_alive_seed, request_in_flight,
                   robot_name](std::shared_future<std::vector<rclcpp::Parameter>> future) {
                    const std::shared_ptr<std::atomic<bool>> alive = weak_alive_seed.lock();
                    if (!alive || !alive->load())
                    {
                      *request_in_flight = false;
                      return;
                    }
                    try
                    {
                      for (const rclcpp::Parameter& p : future.get())
                      {
                        if (p.get_name() == "sim_step_dt")
                        {
                          step_dt_.store(p.as_double(), std::memory_order_relaxed);
                        }
                        else if (p.get_name() == "tf_rate")
                        {
                          tf_rate_.store(p.as_double(), std::memory_order_relaxed);
                        }
                        else if (p.get_name() == "odom_rate")
                        {
                          odom_rate_.store(p.as_double(), std::memory_order_relaxed);
                        }
                        else if (p.get_name() == "scheduling_mode")
                        {
                          const tl::expected<stdr_simulation::SchedulingMode, std::string> mode =
                              stdr_simulation::scheduling_mode_from_string(p.as_string());
                          if (mode.has_value())
                          {
                            scheduling_mode_.store(mode.value(), std::memory_order_relaxed);
                          }
                          else
                          {
                            RCLCPP_WARN(node_->get_logger(),
                                        "Ignoring invalid scheduling_mode '%s' from robot '%s': %s",
                                        p.as_string().c_str(), robot_name.c_str(), mode.error().c_str());
                          }
                        }
                      }
                      // All parameters stored successfully — stop retrying.
                      *request_in_flight = false;
                      if (const auto it = seed_timers_.find(robot_name); it != seed_timers_.end())
                      {
                        it->second->cancel();
                      }
                    }
                    catch (const std::exception& ex)
                    {
                      RCLCPP_DEBUG(node_->get_logger(), "Parameter seed for '%s' failed: %s", robot_name.c_str(),
                                   ex.what());
                      // Clear the flag so the next timer tick can issue a fresh request.
                      *request_in_flight = false;
                    }
                  });
            });
      }

      new_states.push_back(std::move(state));
    }

    robot_states_ = std::move(new_states);
  }

  // Create laser and sonar subscriptions for new (robot, sensor) pairs.
  // laser_subs_ and sonar_subs_ are executor-thread-only; no lock needed.
  for (std::size_t i = 0; i < msg->robots.size(); ++i)
  {
    const stdr_msgs::msg::RobotIndexedMsg& entry = msg->robots[i];

    for (const stdr_simulation::LaserConfig& laser_cfg : parsed_configs[i].laser_sensors)
    {
      const std::string key = entry.name + "/" + laser_cfg.frame_id;
      if (laser_subs_.find(key) == laser_subs_.end())
      {
        const std::string robot_name = entry.name;
        const std::string frame_id = laser_cfg.frame_id;
        // The topic stdr_robot publishes is "robot_name/frame_id" (see
        // stdr_robot_node.cpp create_sensor_publishers_and_tf).
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub =
            node_->create_subscription<sensor_msgs::msg::LaserScan>(
                key, 10, [this, robot_name, frame_id](const sensor_msgs::msg::LaserScan::SharedPtr laser_msg) {
                  on_laser(robot_name, frame_id, laser_msg);
                });
        laser_subs_[key] = sub;
      }
    }

    for (const stdr_simulation::SonarConfig& sonar_cfg : parsed_configs[i].sonar_sensors)
    {
      const std::string key = entry.name + "/" + sonar_cfg.frame_id;
      if (sonar_subs_.find(key) == sonar_subs_.end())
      {
        const std::string robot_name = entry.name;
        const std::string frame_id = sonar_cfg.frame_id;
        // The topic stdr_robot publishes is "robot_name/frame_id" using the
        // same template as laser sensors (see stdr_robot_node.cpp
        // create_sensor_publishers_and_tf).
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub =
            node_->create_subscription<sensor_msgs::msg::Range>(
                key, 10, [this, robot_name, frame_id](const sensor_msgs::msg::Range::SharedPtr range_msg) {
                  on_sonar(robot_name, frame_id, range_msg);
                });
        sonar_subs_[key] = sub;
      }
    }
  }
}

void Ros2Backend::on_odom(const std::string& robot_name, const nav_msgs::msg::Odometry::SharedPtr msg)
{
  stdr_simulation::Pose2D pose;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  // Extract yaw from the quaternion. tf2::getYaw handles gimbal lock gracefully
  // and is consistent with the yaw convention used by StdrRobotNode.
  pose.theta = tf2::getYaw(msg->pose.pose.orientation);

  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  latest_pose_[robot_name] = pose;
}

void Ros2Backend::on_laser(const std::string& robot_name, const std::string& frame_id,
                           const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const stdr_simulation::LaserScan scan = stdr_parser::from_ros_msg(*msg);
  const std::string key = robot_name + "/" + frame_id;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  latest_laser_[key] = scan;

  auto ring_it = laser_rings_.find(key);
  if (ring_it == laser_rings_.end())
  {
    ring_it = laser_rings_
                  .emplace(key, std::make_unique<stdr::plot_data::SpscRing<stdr::plot::TimedLaserScan>>(
                                    stdr::plot_data::kDefaultSensorLogCapacity))
                  .first;
  }
  ring_it->second->push(stdr::plot::TimedLaserScan{ timestamp, scan });
}

void Ros2Backend::on_sonar(const std::string& robot_name, const std::string& frame_id,
                           const sensor_msgs::msg::Range::SharedPtr msg)
{
  const stdr_simulation::SonarScan scan = stdr_parser::from_ros_msg(*msg);
  const std::string key = robot_name + "/" + frame_id;
  const double timestamp = rclcpp::Time(msg->header.stamp).seconds();

  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  latest_sonar_[key] = scan;

  auto ring_it = sonar_rings_.find(key);
  if (ring_it == sonar_rings_.end())
  {
    ring_it = sonar_rings_
                  .emplace(key, std::make_unique<stdr::plot_data::SpscRing<stdr::plot::TimedSonarReading>>(
                                    stdr::plot_data::kDefaultSensorLogCapacity))
                  .first;
  }
  ring_it->second->push(stdr::plot::TimedSonarReading{ timestamp, scan });
}

void Ros2Backend::on_rfid_list(const stdr_msgs::msg::RfidTagVector::SharedPtr msg)
{
  std::vector<stdr_simulation::RfidTag> tags;
  tags.reserve(msg->rfid_tags.size());
  for (const stdr_msgs::msg::RfidTag& tag : msg->rfid_tags)
  {
    tags.push_back(stdr_parser::from_ros_msg(tag));
  }
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  rfid_tags_ = std::move(tags);
}

void Ros2Backend::on_co2_list(const stdr_msgs::msg::CO2SourceVector::SharedPtr msg)
{
  std::vector<stdr_simulation::CO2Source> sources;
  sources.reserve(msg->co2_sources.size());
  for (const stdr_msgs::msg::CO2Source& src : msg->co2_sources)
  {
    sources.push_back(stdr_parser::from_ros_msg(src));
  }
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  co2_sources_ = std::move(sources);
}

void Ros2Backend::on_thermal_list(const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg)
{
  std::vector<stdr_simulation::ThermalSource> sources;
  sources.reserve(msg->thermal_sources.size());
  for (const stdr_msgs::msg::ThermalSource& src : msg->thermal_sources)
  {
    sources.push_back(stdr_parser::from_ros_msg(src));
  }
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  thermal_sources_ = std::move(sources);
}

void Ros2Backend::on_sound_list(const stdr_msgs::msg::SoundSourceVector::SharedPtr msg)
{
  std::vector<stdr_simulation::SoundSource> sources;
  sources.reserve(msg->sound_sources.size());
  for (const stdr_msgs::msg::SoundSource& src : msg->sound_sources)
  {
    sources.push_back(stdr_parser::from_ros_msg(src));
  }
  const std::lock_guard<std::mutex> lock(snapshot_mutex_);
  sound_sources_ = std::move(sources);
}

}  // namespace stdr_gui_ros
