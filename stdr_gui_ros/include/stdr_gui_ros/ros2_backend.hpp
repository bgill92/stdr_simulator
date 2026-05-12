#pragma once

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/simulator_backend.hpp>
#include <stdr_simulation/plot_data/spsc_ring.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stdr_msgs/msg/co2_source_vector.hpp>
#include <stdr_msgs/msg/rfid_tag_vector.hpp>
#include <stdr_msgs/msg/robot_indexed_vector_msg.hpp>
#include <stdr_msgs/msg/sound_source_vector.hpp>
#include <stdr_msgs/msg/thermal_source_vector.hpp>
#include <stdr_msgs/srv/load_map.hpp>
#include <stdr_msgs/srv/move_robot.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace stdr_gui_ros
{

/** @brief ROS2-backed implementation of SimulatorBackend.
 *
 *  Subscribes to stdr_server and stdr_robot topics and forwards state to the
 *  GUI via thread-safe snapshot exchange. Also calls stdr_server services and
 *  publishes velocity commands to stdr_robot.
 *
 *  All ROS2 callbacks fire on the executor thread passed in via the node.
 *  The GUI thread may call any public method concurrently. One mutex
 *  (snapshot_mutex_) guards shared state; a separate msg_mutex_ guards the
 *  message queue so log writes never block the main state lock. */
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
  void set_tf_rate(double hz) override;
  [[nodiscard]] double get_tf_rate() const override;
  void set_odom_rate(double hz) override;
  [[nodiscard]] double get_odom_rate() const override;
  void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) override;
  void set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd) override;
  [[nodiscard]] std::shared_ptr<const stdr_gui::SimulationSnapshot> get_snapshot() const override;
  [[nodiscard]] std::vector<std::string> poll_messages() override;

  // ── Introspection methods ───────────────────────────────────────────────────

  [[nodiscard]] std::size_t num_robots() const override;
  [[nodiscard]] std::vector<std::string> robot_ids() const override;
  [[nodiscard]] std::vector<std::string> laser_sensors(const std::string& robot_id) const override;
  [[nodiscard]] std::vector<std::string> sonar_sensors(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::Pose2D> pose(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::Twist2D> twist(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<bool> collided(const std::string& robot_id) const override;
  [[nodiscard]] std::vector<stdr_simulation::Point2D> footprint(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::LaserScan> latest_laser(const std::string& robot_id,
                                                                       const std::string& sensor_id) const override;
  [[nodiscard]] stdr_gui::DrainedLaserResult poll_laser_events(std::uint64_t& cursor, const std::string& robot_id,
                                                               const std::string& sensor_id) override;
  [[nodiscard]] std::optional<stdr_simulation::SonarScan> latest_sonar(const std::string& robot_id,
                                                                       const std::string& sensor_id) const override;
  [[nodiscard]] stdr_gui::DrainedSonarResult poll_sonar_events(std::uint64_t& cursor, const std::string& robot_id,
                                                               const std::string& sensor_id) override;

  /** @brief Return elapsed wall-clock seconds since construction.
   *
   *  This backend has no discrete simulation steps so there is no
   *  authoritative simulation clock.  We approximate with wall-clock time
   *  since construction, which gives a monotonically increasing value that
   *  the Plot Panel can use as a time axis — matching the intent of
   *  StandaloneBackend's monotonic elapsed_time_ accumulator. */
  [[nodiscard]] double sim_time() const override;

private:
  void push_message(std::string msg);

  // Subscription callbacks — all called on the executor thread.
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_active_robots(const stdr_msgs::msg::RobotIndexedVectorMsg::SharedPtr msg);
  void on_odom(const std::string& robot_name, const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_laser(const std::string& robot_name, const std::string& frame_id,
                const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_sonar(const std::string& robot_name, const std::string& frame_id,
                const sensor_msgs::msg::Range::SharedPtr msg);
  void on_rfid_list(const stdr_msgs::msg::RfidTagVector::SharedPtr msg);
  void on_co2_list(const stdr_msgs::msg::CO2SourceVector::SharedPtr msg);
  void on_thermal_list(const stdr_msgs::msg::ThermalSourceVector::SharedPtr msg);
  void on_sound_list(const stdr_msgs::msg::SoundSourceVector::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;

  // ── Persistent subscriptions ────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<stdr_msgs::msg::RobotIndexedVectorMsg>::SharedPtr active_robots_sub_;
  rclcpp::Subscription<stdr_msgs::msg::RfidTagVector>::SharedPtr rfid_sub_;
  rclcpp::Subscription<stdr_msgs::msg::CO2SourceVector>::SharedPtr co2_sub_;
  rclcpp::Subscription<stdr_msgs::msg::ThermalSourceVector>::SharedPtr thermal_sub_;
  rclcpp::Subscription<stdr_msgs::msg::SoundSourceVector>::SharedPtr sound_sub_;

  // ── Guarded state (snapshot_mutex_) ────────────────────────────────────────
  mutable std::mutex snapshot_mutex_;

  // Cached map received from the server.
  stdr_simulation::OccupancyGrid map_;
  std::string map_name_;

  // Active robot states (config + name). Pose is kept separately in
  // latest_pose_ so odom callbacks can update it without rebuilding the whole
  // robot_states_ vector on every tick.
  std::vector<stdr_simulation::world::RobotState> robot_states_;

  // Most-recently received odom pose per robot, keyed by robot name.
  std::unordered_map<std::string, stdr_simulation::Pose2D> latest_pose_;

  // Most-recently commanded velocity per robot, keyed by robot name.
  // Updated by set_cmd_vel(); cleaned up when a robot disappears in on_active_robots().
  std::unordered_map<std::string, stdr_simulation::Twist2D> latest_cmd_vel_;

  // Per-robot odom subscriptions, created/destroyed as robots appear/vanish.
  std::unordered_map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;

  // Per-(robot, sensor) laser subscriptions, keyed as "robot_name/frame_id".
  // Created when a new robot appears in on_active_robots(); destroyed when the
  // robot is removed.
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;

  // Per-(robot, sensor) sonar subscriptions, keyed as "robot_name/frame_id".
  // Mirror of laser_subs_ for Range topics.
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> sonar_subs_;

  // Lazily-created cmd_vel publishers, keyed by robot name.
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_pubs_;

  // Lazily-created MoveRobot service clients, keyed by robot name.
  std::unordered_map<std::string, rclcpp::Client<stdr_msgs::srv::MoveRobot>::SharedPtr> move_robot_clients_;

  // Environment sources.
  std::vector<stdr_simulation::RfidTag> rfid_tags_;
  std::vector<stdr_simulation::CO2Source> co2_sources_;
  std::vector<stdr_simulation::ThermalSource> thermal_sources_;
  std::vector<stdr_simulation::SoundSource> sound_sources_;

  // Lazily-created LoadMap client.
  rclcpp::Client<stdr_msgs::srv::LoadMap>::SharedPtr load_map_client_;

  // ── Sensor rings and latest snapshots (sensor_ring_mutex_) ────────────────
  //
  // Lock discipline in on_active_robots() — four phases, never two locks at once:
  //   Phase 1 (no lock): build laser_keys_to_remove and sonar_keys_to_remove
  //     by reading laser_subs_ and sonar_subs_, which are executor-thread-only
  //     state; no mutex is needed.
  //   Phase 2 (sensor_ring_mutex_ only): erase latest_laser_, laser_rings_,
  //     latest_sonar_, and sonar_rings_ entries for robots that disappeared.
  //   Phase 3 (snapshot_mutex_ only): update robot_states_, odom_subs_, and
  //     other snapshot state for the new robot set.
  //   Phase 4 (no lock): create new laser and sonar subscriptions on the
  //     executor thread.
  //
  // sensor_ring_mutex_ and snapshot_mutex_ are never held simultaneously.
  // laser_subs_ and sonar_subs_ are touched only from the executor thread
  // (no mutex required).
  mutable std::mutex sensor_ring_mutex_;

  // Most-recently received laser scan per (robot, sensor), keyed as
  // "robot_name/frame_id".  Updated by on_laser() under sensor_ring_mutex_.
  std::unordered_map<std::string, stdr_simulation::LaserScan> latest_laser_;

  // Per-(robot, sensor) SPSC rings for the plot panel to drain.  Keys mirror
  // latest_laser_.  SpscRing is non-movable, so we store it behind unique_ptr
  // so the map can rehash without moving the ring objects.
  std::unordered_map<std::string, std::unique_ptr<stdr::plot_data::SpscRing<stdr::plot::TimedLaserScan>>> laser_rings_;

  // Most-recently received sonar reading per (robot, sensor), keyed as
  // "robot_name/frame_id".  Updated by on_sonar() under sensor_ring_mutex_.
  std::unordered_map<std::string, stdr_simulation::SonarScan> latest_sonar_;

  // Per-(robot, sensor) SPSC rings for sonar data.  Keys mirror latest_sonar_.
  std::unordered_map<std::string, std::unique_ptr<stdr::plot_data::SpscRing<stdr::plot::TimedSonarReading>>> sonar_rings_;

  // ── Per-message log queue (msg_mutex_) ─────────────────────────────────────
  std::mutex msg_mutex_;
  std::vector<std::string> messages_;

  // Wall-clock time at construction, used to compute sim_time().
  rclcpp::Time start_time_;

  // ── Thread-safe atomic fields ───────────────────────────────────────────────
  std::atomic<double> step_dt_{ stdr_gui::kDefaultStepDt };
  // TF/odom rates stored locally; full propagation to robot nodes is Phase E.
  std::atomic<double> tf_rate_{ stdr_gui::kDefaultTfRate };
  std::atomic<double> odom_rate_{ stdr_gui::kDefaultOdomRate };

  // Shared flag cleared in the destructor so async callbacks can detect
  // backend teardown before touching member state.
  std::shared_ptr<std::atomic<bool>> alive_;
};

}  // namespace stdr_gui_ros
