#include "stdr_server/stdr_server_node.hpp"

#include "stdr_server/map_loader.hpp"

#include <stdr_parser/msg_conversions.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <stdr_msgs/msg/robot_indexed_msg.hpp>

#include <set>
#include <string>
#include <vector>

namespace stdr_server
{

// ─── Constructor ────────────────────────────────────────────────────────────

StdrServerNode::StdrServerNode(const rclcpp::NodeOptions& options) : rclcpp::Node("stdr_server", options)
{
  // Transient local (latched) QoS so late-joining subscribers receive the last
  // published value immediately.
  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", latched_qos);
  robots_pub_ = create_publisher<stdr_msgs::msg::RobotIndexedVectorMsg>("stdr_server/active_robots", latched_qos);
  rfid_pub_ = create_publisher<stdr_msgs::msg::RfidTagVector>("stdr_server/rfid_list", latched_qos);
  co2_pub_ = create_publisher<stdr_msgs::msg::CO2SourceVector>("stdr_server/co2_sources_list", latched_qos);
  thermal_pub_ = create_publisher<stdr_msgs::msg::ThermalSourceVector>("stdr_server/thermal_sources_list", latched_qos);
  sound_pub_ = create_publisher<stdr_msgs::msg::SoundSourceVector>("stdr_server/sound_sources_list", latched_qos);
  markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("stdr_server/sources_visualization", latched_qos);

  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // ── Services ──────────────────────────────────────────────────────────────
  load_map_srv_ = create_service<stdr_msgs::srv::LoadMap>(
      "stdr_server/load_static_map",
      [this](const stdr_msgs::srv::LoadMap::Request::SharedPtr request,
             stdr_msgs::srv::LoadMap::Response::SharedPtr response) { handle_load_map(request, response); });

  load_external_map_srv_ = create_service<stdr_msgs::srv::LoadExternalMap>(
      "stdr_server/load_external_map", [this](const stdr_msgs::srv::LoadExternalMap::Request::SharedPtr request,
                                              stdr_msgs::srv::LoadExternalMap::Response::SharedPtr response) {
        handle_load_external_map(request, response);
      });

  move_robot_srv_ = create_service<stdr_msgs::srv::MoveRobot>(
      "stdr_server/move_robot",
      [this](const stdr_msgs::srv::MoveRobot::Request::SharedPtr request,
             stdr_msgs::srv::MoveRobot::Response::SharedPtr response) { handle_move_robot(request, response); });

  add_rfid_srv_ = create_service<stdr_msgs::srv::AddRfidTag>(
      "stdr_server/add_rfid_tag",
      [this](const stdr_msgs::srv::AddRfidTag::Request::SharedPtr request,
             stdr_msgs::srv::AddRfidTag::Response::SharedPtr response) { handle_add_rfid_tag(request, response); });

  delete_rfid_srv_ = create_service<stdr_msgs::srv::DeleteRfidTag>(
      "stdr_server/delete_rfid_tag", [this](const stdr_msgs::srv::DeleteRfidTag::Request::SharedPtr request,
                                            stdr_msgs::srv::DeleteRfidTag::Response::SharedPtr response) {
        handle_delete_rfid_tag(request, response);
      });

  add_co2_srv_ = create_service<stdr_msgs::srv::AddCO2Source>(
      "stdr_server/add_co2_source",
      [this](const stdr_msgs::srv::AddCO2Source::Request::SharedPtr request,
             stdr_msgs::srv::AddCO2Source::Response::SharedPtr response) { handle_add_co2_source(request, response); });

  delete_co2_srv_ = create_service<stdr_msgs::srv::DeleteCO2Source>(
      "stdr_server/delete_co2_source", [this](const stdr_msgs::srv::DeleteCO2Source::Request::SharedPtr request,
                                              stdr_msgs::srv::DeleteCO2Source::Response::SharedPtr response) {
        handle_delete_co2_source(request, response);
      });

  add_thermal_srv_ = create_service<stdr_msgs::srv::AddThermalSource>(
      "stdr_server/add_thermal_source", [this](const stdr_msgs::srv::AddThermalSource::Request::SharedPtr request,
                                               stdr_msgs::srv::AddThermalSource::Response::SharedPtr response) {
        handle_add_thermal_source(request, response);
      });

  delete_thermal_srv_ = create_service<stdr_msgs::srv::DeleteThermalSource>(
      "stdr_server/delete_thermal_source", [this](const stdr_msgs::srv::DeleteThermalSource::Request::SharedPtr request,
                                                  stdr_msgs::srv::DeleteThermalSource::Response::SharedPtr response) {
        handle_delete_thermal_source(request, response);
      });

  add_sound_srv_ = create_service<stdr_msgs::srv::AddSoundSource>(
      "stdr_server/add_sound_source", [this](const stdr_msgs::srv::AddSoundSource::Request::SharedPtr request,
                                             stdr_msgs::srv::AddSoundSource::Response::SharedPtr response) {
        handle_add_sound_source(request, response);
      });

  delete_sound_srv_ = create_service<stdr_msgs::srv::DeleteSoundSource>(
      "stdr_server/delete_sound_source", [this](const stdr_msgs::srv::DeleteSoundSource::Request::SharedPtr request,
                                                stdr_msgs::srv::DeleteSoundSource::Response::SharedPtr response) {
        handle_delete_sound_source(request, response);
      });

  // ── Actions ───────────────────────────────────────────────────────────────
  spawn_action_ = rclcpp_action::create_server<SpawnRobot>(
      this, "stdr_server/spawn_robot",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SpawnRobot::Goal> goal) {
        return handle_spawn_goal(uuid, goal);
      },
      [this](const std::shared_ptr<SpawnGoalHandle> goal_handle) { return handle_spawn_cancel(goal_handle); },
      [this](const std::shared_ptr<SpawnGoalHandle> goal_handle) { handle_spawn_accepted(goal_handle); });

  delete_action_ = rclcpp_action::create_server<DeleteRobot>(
      this, "stdr_server/delete_robot",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DeleteRobot::Goal> goal) {
        return handle_delete_goal(uuid, goal);
      },
      [this](const std::shared_ptr<DeleteGoalHandle> goal_handle) { return handle_delete_cancel(goal_handle); },
      [this](const std::shared_ptr<DeleteGoalHandle> goal_handle) { handle_delete_accepted(goal_handle); });

  register_action_ = rclcpp_action::create_server<RegisterRobot>(
      this, "stdr_server/register_robot",
      [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RegisterRobot::Goal> goal) {
        return handle_register_goal(uuid, goal);
      },
      [this](const std::shared_ptr<RegisterGoalHandle> goal_handle) { return handle_register_cancel(goal_handle); },
      [this](const std::shared_ptr<RegisterGoalHandle> goal_handle) { handle_register_accepted(goal_handle); });

  // ── Optional startup map load ──────────────────────────────────────────────
  // Safe without mutex: ROS2 callbacks cannot fire until construction finishes
  // and the node is added to an executor.
  declare_parameter<std::string>("map_file", "");
  const std::string map_file = get_parameter("map_file").as_string();
  if (!map_file.empty())
  {
    load_map_from_file(map_file);
  }

  RCLCPP_INFO(get_logger(), "STDR Server node created");
}

// ─── Map loading ────────────────────────────────────────────────────────────

void StdrServerNode::load_map_from_file(const std::string& yaml_path)
{
  const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> result = stdr_server::load_map(yaml_path);
  if (!result)
  {
    RCLCPP_ERROR(get_logger(), "Failed to load map from '%s': %s", yaml_path.c_str(), result.error().c_str());
    return;
  }

  map_msg_ = *result;

  // Convert to simulation OccupancyGrid and store in the world model.
  stdr_simulation::OccupancyGrid sim_map;
  sim_map.width = map_msg_.info.width;
  sim_map.height = map_msg_.info.height;
  sim_map.resolution = map_msg_.info.resolution;
  sim_map.origin.x = map_msg_.info.origin.position.x;
  sim_map.origin.y = map_msg_.info.origin.position.y;
  sim_map.data = map_msg_.data;
  world_model_.set_map(std::move(sim_map));

  map_loaded_ = true;
  publish_map();
  broadcast_map_transform();
}

void StdrServerNode::publish_map()
{
  map_pub_->publish(map_msg_);
}

void StdrServerNode::broadcast_map_transform()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "map_static";

  // The map origin encodes the real-world location of the map's (0,0) cell.
  tf_msg.transform.translation.x = map_msg_.info.origin.position.x;
  tf_msg.transform.translation.y = map_msg_.info.origin.position.y;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = map_msg_.info.origin.orientation;

  static_tf_broadcaster_->sendTransform(tf_msg);
}

// ─── Service callbacks ──────────────────────────────────────────────────────

void StdrServerNode::handle_load_map(const stdr_msgs::srv::LoadMap::Request::SharedPtr request,
                                     stdr_msgs::srv::LoadMap::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (map_loaded_)
    {
      RCLCPP_WARN(get_logger(), "Map already loaded — ignoring load_static_map request");
      return;
    }
  }
  load_map_from_file(request->map_file);
}

void StdrServerNode::handle_load_external_map(const stdr_msgs::srv::LoadExternalMap::Request::SharedPtr request,
                                              stdr_msgs::srv::LoadExternalMap::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (map_loaded_)
    {
      RCLCPP_WARN(get_logger(), "Map already loaded — ignoring load_external_map request");
      return;
    }

    map_msg_ = request->map;

    stdr_simulation::OccupancyGrid sim_map;
    sim_map.width = map_msg_.info.width;
    sim_map.height = map_msg_.info.height;
    sim_map.resolution = map_msg_.info.resolution;
    sim_map.origin.x = map_msg_.info.origin.position.x;
    sim_map.origin.y = map_msg_.info.origin.position.y;
    sim_map.data = map_msg_.data;
    world_model_.set_map(std::move(sim_map));

    map_loaded_ = true;
  }
  publish_map();
  broadcast_map_transform();
}

void StdrServerNode::handle_move_robot(const stdr_msgs::srv::MoveRobot::Request::SharedPtr /*request*/,
                                       stdr_msgs::srv::MoveRobot::Response::SharedPtr /*response*/)
{
  // FIXME-CLAUDE: NOT IMPLEMENTED - per-robot MoveRobot routing deferred to Phase 6
  // MoveRobot.srv only carries new_pose, not a robot name. In ROS1 the caller
  // invoked a per-robot service endpoint. Per-robot routing will be wired in
  // Phase 6 when stdr_robot nodes are available.
  RCLCPP_WARN(get_logger(), "handle_move_robot not yet implemented — deferred to Phase 6");
}

// ─── Environment source callbacks ───────────────────────────────────────────

void StdrServerNode::handle_add_rfid_tag(const stdr_msgs::srv::AddRfidTag::Request::SharedPtr request,
                                         stdr_msgs::srv::AddRfidTag::Response::SharedPtr response)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const stdr_simulation::RfidTag& existing : world_model_.get_rfid_tags())
    {
      if (existing.tag_id == request->new_tag.tag_id)
      {
        response->success = false;
        response->message = "RFID tag with id '" + request->new_tag.tag_id + "' already exists";
        return;
      }
    }
    world_model_.add_rfid_tag(stdr_parser::from_ros_msg(request->new_tag));
    response->success = true;
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_delete_rfid_tag(const stdr_msgs::srv::DeleteRfidTag::Request::SharedPtr request,
                                            stdr_msgs::srv::DeleteRfidTag::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    // DeleteRfidTag.srv field `name` corresponds to RfidTag::tag_id.
    world_model_.remove_rfid_tag(request->name);
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_add_co2_source(const stdr_msgs::srv::AddCO2Source::Request::SharedPtr request,
                                           stdr_msgs::srv::AddCO2Source::Response::SharedPtr response)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const stdr_simulation::CO2Source& existing : world_model_.get_co2_sources())
    {
      if (existing.id == request->new_source.id)
      {
        response->success = false;
        response->message = "CO2 source with id '" + request->new_source.id + "' already exists";
        return;
      }
    }
    world_model_.add_co2_source(stdr_parser::from_ros_msg(request->new_source));
    response->success = true;
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_delete_co2_source(const stdr_msgs::srv::DeleteCO2Source::Request::SharedPtr request,
                                              stdr_msgs::srv::DeleteCO2Source::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    world_model_.remove_co2_source(request->name);
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_add_thermal_source(const stdr_msgs::srv::AddThermalSource::Request::SharedPtr request,
                                               stdr_msgs::srv::AddThermalSource::Response::SharedPtr response)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const stdr_simulation::ThermalSource& existing : world_model_.get_thermal_sources())
    {
      if (existing.id == request->new_source.id)
      {
        response->success = false;
        response->message = "Thermal source with id '" + request->new_source.id + "' already exists";
        return;
      }
    }
    world_model_.add_thermal_source(stdr_parser::from_ros_msg(request->new_source));
    response->success = true;
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_delete_thermal_source(const stdr_msgs::srv::DeleteThermalSource::Request::SharedPtr request,
                                                  stdr_msgs::srv::DeleteThermalSource::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    world_model_.remove_thermal_source(request->name);
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_add_sound_source(const stdr_msgs::srv::AddSoundSource::Request::SharedPtr request,
                                             stdr_msgs::srv::AddSoundSource::Response::SharedPtr response)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const stdr_simulation::SoundSource& existing : world_model_.get_sound_sources())
    {
      if (existing.id == request->new_source.id)
      {
        response->success = false;
        response->message = "Sound source with id '" + request->new_source.id + "' already exists";
        return;
      }
    }
    world_model_.add_sound_source(stdr_parser::from_ros_msg(request->new_source));
    response->success = true;
  }
  publish_source_lists();
  publish_source_markers();
}

void StdrServerNode::handle_delete_sound_source(const stdr_msgs::srv::DeleteSoundSource::Request::SharedPtr request,
                                                stdr_msgs::srv::DeleteSoundSource::Response::SharedPtr /*response*/)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    world_model_.remove_sound_source(request->name);
  }
  publish_source_lists();
  publish_source_markers();
}

// ─── Action callbacks ────────────────────────────────────────────────────────

rclcpp_action::GoalResponse StdrServerNode::handle_spawn_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                              std::shared_ptr<const SpawnRobot::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse StdrServerNode::handle_spawn_cancel(const std::shared_ptr<SpawnGoalHandle> /*goal_handle*/)
{
  // Spawning is synchronous and cannot be cancelled mid-flight.
  return rclcpp_action::CancelResponse::REJECT;
}

void StdrServerNode::handle_spawn_accepted(const std::shared_ptr<SpawnGoalHandle> goal_handle)
{
  const stdr_msgs::msg::RobotMsg& robot_msg = goal_handle->get_goal()->description;

  {
    const std::lock_guard<std::mutex> lock(mutex_);

    // Reject if any sensor uses a frame_id that is already in use.
    std::string duplicate_id;
    if (has_duplicate_frame_ids(robot_msg, duplicate_id))
    {
      auto result = std::make_shared<SpawnRobot::Result>();
      result->message = "Duplicate frame_id detected: " + duplicate_id;
      goal_handle->abort(result);
      return;
    }

    const stdr_simulation::RobotConfig config = stdr_parser::from_ros_msg(robot_msg);
    const std::string name = world_model_.add_robot(config);

    // FIXME-CLAUDE: NOT IMPLEMENTED - component container loading deferred to Phase 6

    auto result = std::make_shared<SpawnRobot::Result>();
    result->indexed_description.name = name;
    result->indexed_description.robot = robot_msg;
    result->message = "Robot '" + name + "' spawned successfully";
    goal_handle->succeed(result);
  }

  publish_robot_list();
}

rclcpp_action::GoalResponse StdrServerNode::handle_delete_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                               std::shared_ptr<const DeleteRobot::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
StdrServerNode::handle_delete_cancel(const std::shared_ptr<DeleteGoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void StdrServerNode::handle_delete_accepted(const std::shared_ptr<DeleteGoalHandle> goal_handle)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const std::string& name = goal_handle->get_goal()->name;

    // FIXME-CLAUDE: NOT IMPLEMENTED - component container teardown deferred to Phase 6

    world_model_.remove_robot(name);

    auto result = std::make_shared<DeleteRobot::Result>();
    result->success = true;
    goal_handle->succeed(result);
  }

  publish_robot_list();
}

rclcpp_action::GoalResponse StdrServerNode::handle_register_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                                 std::shared_ptr<const RegisterRobot::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
StdrServerNode::handle_register_cancel(const std::shared_ptr<RegisterGoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void StdrServerNode::handle_register_accepted(const std::shared_ptr<RegisterGoalHandle> goal_handle)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::string& name = goal_handle->get_goal()->name;

  auto result = std::make_shared<RegisterRobot::Result>();
  const stdr_simulation::world::RobotState* state = world_model_.get_robot(name);
  if (state == nullptr)
  {
    goal_handle->abort(result);
    return;
  }

  result->description = stdr_parser::to_ros_msg(state->config);
  goal_handle->succeed(result);
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

void StdrServerNode::publish_robot_list()
{
  stdr_msgs::msg::RobotIndexedVectorMsg msg;
  for (const stdr_simulation::world::RobotState& state : world_model_.get_all_robots())
  {
    stdr_msgs::msg::RobotIndexedMsg indexed;
    indexed.name = state.name;
    indexed.robot = stdr_parser::to_ros_msg(state.config);
    msg.robots.push_back(std::move(indexed));
  }
  robots_pub_->publish(msg);
}

void StdrServerNode::publish_source_lists()
{
  {
    stdr_msgs::msg::RfidTagVector rfid_msg;
    for (const stdr_simulation::RfidTag& tag : world_model_.get_rfid_tags())
    {
      rfid_msg.rfid_tags.push_back(stdr_parser::to_ros_msg(tag));
    }
    rfid_pub_->publish(rfid_msg);
  }
  {
    stdr_msgs::msg::CO2SourceVector co2_msg;
    for (const stdr_simulation::CO2Source& src : world_model_.get_co2_sources())
    {
      co2_msg.co2_sources.push_back(stdr_parser::to_ros_msg(src));
    }
    co2_pub_->publish(co2_msg);
  }
  {
    stdr_msgs::msg::ThermalSourceVector thermal_msg;
    for (const stdr_simulation::ThermalSource& src : world_model_.get_thermal_sources())
    {
      thermal_msg.thermal_sources.push_back(stdr_parser::to_ros_msg(src));
    }
    thermal_pub_->publish(thermal_msg);
  }
  {
    stdr_msgs::msg::SoundSourceVector sound_msg;
    for (const stdr_simulation::SoundSource& src : world_model_.get_sound_sources())
    {
      sound_msg.sound_sources.push_back(stdr_parser::to_ros_msg(src));
    }
    sound_pub_->publish(sound_msg);
  }
}

void StdrServerNode::publish_source_markers()
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Clear all previous markers before republishing.
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  const rclcpp::Time stamp = now();

  // Helper lambda to build a sphere marker from a 2D position.
  auto make_marker = [&stamp](const std::string& ns, int id, double x, double y, float r, float g,
                              float b) -> visualization_msgs::msg::Marker {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.7f;
    return marker;
  };

  // CO2 sources — green
  {
    int id = 0;
    for (const stdr_simulation::CO2Source& src : world_model_.get_co2_sources())
    {
      marker_array.markers.push_back(make_marker("co2_sources", id++, src.pose.x, src.pose.y, 0.0f, 1.0f, 0.0f));
    }
  }

  // Thermal sources — red
  {
    int id = 0;
    for (const stdr_simulation::ThermalSource& src : world_model_.get_thermal_sources())
    {
      marker_array.markers.push_back(make_marker("thermal_sources", id++, src.pose.x, src.pose.y, 1.0f, 0.0f, 0.0f));
    }
  }

  // Sound sources — blue
  {
    int id = 0;
    for (const stdr_simulation::SoundSource& src : world_model_.get_sound_sources())
    {
      marker_array.markers.push_back(make_marker("sound_sources", id++, src.pose.x, src.pose.y, 0.0f, 0.0f, 1.0f));
    }
  }

  // RFID tags — black
  {
    int id = 0;
    for (const stdr_simulation::RfidTag& tag : world_model_.get_rfid_tags())
    {
      marker_array.markers.push_back(make_marker("rfid_tags", id++, tag.pose.x, tag.pose.y, 0.0f, 0.0f, 0.0f));
    }
  }

  markers_pub_->publish(marker_array);
}

bool StdrServerNode::has_duplicate_frame_ids(const stdr_msgs::msg::RobotMsg& robot, std::string& duplicate_id) const
{
  std::set<std::string> seen;

  auto check = [&](const std::string& frame_id) -> bool {
    if (frame_id.empty())
    {
      return false;
    }
    if (!seen.insert(frame_id).second)
    {
      duplicate_id = frame_id;
      return true;
    }
    return false;
  };

  for (const stdr_msgs::msg::LaserSensorMsg& s : robot.laser_sensors)
  {
    if (check(s.frame_id))
      return true;
  }
  for (const stdr_msgs::msg::SonarSensorMsg& s : robot.sonar_sensors)
  {
    if (check(s.frame_id))
      return true;
  }
  for (const stdr_msgs::msg::RfidSensorMsg& s : robot.rfid_sensors)
  {
    if (check(s.frame_id))
      return true;
  }
  for (const stdr_msgs::msg::CO2SensorMsg& s : robot.co2_sensors)
  {
    if (check(s.frame_id))
      return true;
  }
  for (const stdr_msgs::msg::SoundSensorMsg& s : robot.sound_sensors)
  {
    if (check(s.frame_id))
      return true;
  }
  for (const stdr_msgs::msg::ThermalSensorMsg& s : robot.thermal_sensors)
  {
    if (check(s.frame_id))
      return true;
  }

  return false;
}

}  // namespace stdr_server
