#pragma once

/** @file Central STDR server ROS2 node. */

#include <stdr_simulation/world/world_model.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <stdr_msgs/action/delete_robot.hpp>
#include <stdr_msgs/action/register_robot.hpp>
#include <stdr_msgs/action/spawn_robot.hpp>
#include <stdr_msgs/msg/co2_source_vector.hpp>
#include <stdr_msgs/msg/rfid_tag_vector.hpp>
#include <stdr_msgs/msg/robot_indexed_vector_msg.hpp>
#include <stdr_msgs/msg/sound_source_vector.hpp>
#include <stdr_msgs/msg/thermal_source_vector.hpp>
#include <stdr_msgs/srv/add_co2_source.hpp>
#include <stdr_msgs/srv/add_rfid_tag.hpp>
#include <stdr_msgs/srv/add_sound_source.hpp>
#include <stdr_msgs/srv/add_thermal_source.hpp>
#include <stdr_msgs/srv/delete_co2_source.hpp>
#include <stdr_msgs/srv/delete_rfid_tag.hpp>
#include <stdr_msgs/srv/delete_sound_source.hpp>
#include <stdr_msgs/srv/delete_thermal_source.hpp>
#include <stdr_msgs/srv/load_external_map.hpp>
#include <stdr_msgs/srv/load_map.hpp>
#include <stdr_msgs/srv/move_robot.hpp>

#include <mutex>
#include <string>

namespace stdr_server
{

class StdrServerNode : public rclcpp::Node
{
public:
  explicit StdrServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Use type aliases for readability
  using SpawnRobot = stdr_msgs::action::SpawnRobot;
  using DeleteRobot = stdr_msgs::action::DeleteRobot;
  using RegisterRobot = stdr_msgs::action::RegisterRobot;
  using SpawnGoalHandle = rclcpp_action::ServerGoalHandle<SpawnRobot>;
  using DeleteGoalHandle = rclcpp_action::ServerGoalHandle<DeleteRobot>;
  using RegisterGoalHandle = rclcpp_action::ServerGoalHandle<RegisterRobot>;

  // --- Map loading ---
  void load_map_from_file(const std::string& yaml_path);
  void publish_map();
  void broadcast_map_transform();

  // --- Service callbacks ---
  void handle_load_map(const stdr_msgs::srv::LoadMap::Request::SharedPtr request,
                       stdr_msgs::srv::LoadMap::Response::SharedPtr response);

  void handle_load_external_map(const stdr_msgs::srv::LoadExternalMap::Request::SharedPtr request,
                                stdr_msgs::srv::LoadExternalMap::Response::SharedPtr response);

  void handle_move_robot(const stdr_msgs::srv::MoveRobot::Request::SharedPtr request,
                         stdr_msgs::srv::MoveRobot::Response::SharedPtr response);

  // Environment source service callbacks (add/delete for rfid, co2, thermal, sound)
  void handle_add_rfid_tag(const stdr_msgs::srv::AddRfidTag::Request::SharedPtr request,
                           stdr_msgs::srv::AddRfidTag::Response::SharedPtr response);
  void handle_delete_rfid_tag(const stdr_msgs::srv::DeleteRfidTag::Request::SharedPtr request,
                              stdr_msgs::srv::DeleteRfidTag::Response::SharedPtr response);
  void handle_add_co2_source(const stdr_msgs::srv::AddCO2Source::Request::SharedPtr request,
                             stdr_msgs::srv::AddCO2Source::Response::SharedPtr response);
  void handle_delete_co2_source(const stdr_msgs::srv::DeleteCO2Source::Request::SharedPtr request,
                                stdr_msgs::srv::DeleteCO2Source::Response::SharedPtr response);
  void handle_add_thermal_source(const stdr_msgs::srv::AddThermalSource::Request::SharedPtr request,
                                 stdr_msgs::srv::AddThermalSource::Response::SharedPtr response);
  void handle_delete_thermal_source(const stdr_msgs::srv::DeleteThermalSource::Request::SharedPtr request,
                                    stdr_msgs::srv::DeleteThermalSource::Response::SharedPtr response);
  void handle_add_sound_source(const stdr_msgs::srv::AddSoundSource::Request::SharedPtr request,
                               stdr_msgs::srv::AddSoundSource::Response::SharedPtr response);
  void handle_delete_sound_source(const stdr_msgs::srv::DeleteSoundSource::Request::SharedPtr request,
                                  stdr_msgs::srv::DeleteSoundSource::Response::SharedPtr response);

  // --- Action callbacks ---
  // SpawnRobot
  [[nodiscard]] rclcpp_action::GoalResponse handle_spawn_goal(const rclcpp_action::GoalUUID& uuid,
                                                              std::shared_ptr<const SpawnRobot::Goal> goal);
  [[nodiscard]] rclcpp_action::CancelResponse handle_spawn_cancel(const std::shared_ptr<SpawnGoalHandle> goal_handle);
  void handle_spawn_accepted(const std::shared_ptr<SpawnGoalHandle> goal_handle);

  // DeleteRobot
  [[nodiscard]] rclcpp_action::GoalResponse handle_delete_goal(const rclcpp_action::GoalUUID& uuid,
                                                               std::shared_ptr<const DeleteRobot::Goal> goal);
  [[nodiscard]] rclcpp_action::CancelResponse handle_delete_cancel(const std::shared_ptr<DeleteGoalHandle> goal_handle);
  void handle_delete_accepted(const std::shared_ptr<DeleteGoalHandle> goal_handle);

  // RegisterRobot
  [[nodiscard]] rclcpp_action::GoalResponse handle_register_goal(const rclcpp_action::GoalUUID& uuid,
                                                                 std::shared_ptr<const RegisterRobot::Goal> goal);
  [[nodiscard]] rclcpp_action::CancelResponse
  handle_register_cancel(const std::shared_ptr<RegisterGoalHandle> goal_handle);
  void handle_register_accepted(const std::shared_ptr<RegisterGoalHandle> goal_handle);

  // --- Helpers ---
  void publish_robot_list();
  void publish_source_lists();
  void publish_source_markers();

  /** Check for duplicate frame_ids across all sensor types in a robot description. */
  [[nodiscard]] bool has_duplicate_frame_ids(const stdr_msgs::msg::RobotMsg& robot, std::string& duplicate_id) const;

  // --- State ---
  stdr_simulation::world::WorldModel world_model_;
  std::mutex mutex_;
  bool map_loaded_{ false };

  // --- Map ---
  nav_msgs::msg::OccupancyGrid map_msg_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // --- Publishers ---
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<stdr_msgs::msg::RobotIndexedVectorMsg>::SharedPtr robots_pub_;
  rclcpp::Publisher<stdr_msgs::msg::RfidTagVector>::SharedPtr rfid_pub_;
  rclcpp::Publisher<stdr_msgs::msg::CO2SourceVector>::SharedPtr co2_pub_;
  rclcpp::Publisher<stdr_msgs::msg::ThermalSourceVector>::SharedPtr thermal_pub_;
  rclcpp::Publisher<stdr_msgs::msg::SoundSourceVector>::SharedPtr sound_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // --- Services ---
  rclcpp::Service<stdr_msgs::srv::LoadMap>::SharedPtr load_map_srv_;
  rclcpp::Service<stdr_msgs::srv::LoadExternalMap>::SharedPtr load_external_map_srv_;
  rclcpp::Service<stdr_msgs::srv::MoveRobot>::SharedPtr move_robot_srv_;
  rclcpp::Service<stdr_msgs::srv::AddRfidTag>::SharedPtr add_rfid_srv_;
  rclcpp::Service<stdr_msgs::srv::DeleteRfidTag>::SharedPtr delete_rfid_srv_;
  rclcpp::Service<stdr_msgs::srv::AddCO2Source>::SharedPtr add_co2_srv_;
  rclcpp::Service<stdr_msgs::srv::DeleteCO2Source>::SharedPtr delete_co2_srv_;
  rclcpp::Service<stdr_msgs::srv::AddThermalSource>::SharedPtr add_thermal_srv_;
  rclcpp::Service<stdr_msgs::srv::DeleteThermalSource>::SharedPtr delete_thermal_srv_;
  rclcpp::Service<stdr_msgs::srv::AddSoundSource>::SharedPtr add_sound_srv_;
  rclcpp::Service<stdr_msgs::srv::DeleteSoundSource>::SharedPtr delete_sound_srv_;

  // --- Actions ---
  rclcpp_action::Server<SpawnRobot>::SharedPtr spawn_action_;
  rclcpp_action::Server<DeleteRobot>::SharedPtr delete_action_;
  rclcpp_action::Server<RegisterRobot>::SharedPtr register_action_;
};

}  // namespace stdr_server
