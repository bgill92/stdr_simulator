#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <stdr_simulation/types.hpp>

namespace stdr_simulation::world
{

/** @brief Snapshot of a single robot's runtime state: config, pose, and
 *  current velocity command. */
struct RobotState
{
  std::string name;
  stdr_simulation::RobotConfig config;
  stdr_simulation::Pose2D pose;
  stdr_simulation::Twist2D cmd_vel;
};

/**
 * @brief Central state container for one simulation run.
 *
 * Owns the occupancy-grid map, all robot instances (with their configs, poses,
 * and velocity commands), and all environment sources (RFID tags, CO2, thermal,
 * sound). Provides the single source of truth queried by sensor simulators and
 * the collision checker.
 *
 * @warning Not thread-safe. Intended for use in a single-threaded simulation
 * step loop.
 */
class WorldModel
{
public:
  WorldModel() = default;
  ~WorldModel() = default;

  // --- Map ---

  /**
   * @brief Replace the current occupancy grid.
   * @param map New map to store; the caller may std::move into this call.
   */
  void set_map(stdr_simulation::OccupancyGrid map);

  /**
   * @brief Return a pointer to the stored map, or nullptr if none has been set.
   */
  [[nodiscard]] const stdr_simulation::OccupancyGrid* get_map() const;

  // --- Robots ---

  /**
   * @brief Add a robot to the world and return its assigned name.
   *
   * Names are assigned sequentially: robot0, robot1, …
   * The robot's initial pose is taken from config.initial_pose.
   *
   * @param config Full sensor and kinematic configuration for the robot.
   * @return The name assigned to this robot instance.
   */
  [[nodiscard]] std::string add_robot(const stdr_simulation::RobotConfig& config);

  /**
   * @brief Remove the robot with the given name. No-op if not found.
   * @param name Name previously returned by add_robot.
   */
  void remove_robot(const std::string& name);

  /**
   * @brief Update the pose of a robot. No-op if the robot is not found.
   * @param name Robot name.
   * @param pose New pose in world frame.
   */
  void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose);

  /**
   * @brief Update the velocity command for a robot. No-op if not found.
   * @param name Robot name.
   * @param cmd  Velocity command to store.
   */
  void set_robot_cmd_vel(const std::string& name, const stdr_simulation::Twist2D& cmd);

  /**
   * @brief Return a pointer to a robot's state, or nullptr if not found.
   * @param name Robot name.
   */
  [[nodiscard]] const RobotState* get_robot(const std::string& name) const;

  /** @brief Return a snapshot of every robot currently in the world. */
  [[nodiscard]] std::vector<RobotState> get_all_robots() const;

  // --- Environment sources ---

  /** @brief Add an RFID tag to the environment. */
  void add_rfid_tag(const stdr_simulation::RfidTag& tag);

  /**
   * @brief Remove the RFID tag with the given tag_id. No-op if not found.
   * @param id Value of RfidTag::tag_id to remove.
   */
  void remove_rfid_tag(const std::string& id);

  /** @brief Return all RFID tags currently in the environment. */
  [[nodiscard]] const std::vector<stdr_simulation::RfidTag>& get_rfid_tags() const;

  /** @brief Add a CO2 emission source to the environment. */
  void add_co2_source(const stdr_simulation::CO2Source& source);

  /**
   * @brief Remove the CO2 source with the given id. No-op if not found.
   * @param id Value of CO2Source::id to remove.
   */
  void remove_co2_source(const std::string& id);

  /** @brief Return all CO2 sources currently in the environment. */
  [[nodiscard]] const std::vector<stdr_simulation::CO2Source>& get_co2_sources() const;

  /** @brief Add a thermal emission source to the environment. */
  void add_thermal_source(const stdr_simulation::ThermalSource& source);

  /**
   * @brief Remove the thermal source with the given id. No-op if not found.
   * @param id Value of ThermalSource::id to remove.
   */
  void remove_thermal_source(const std::string& id);

  /** @brief Return all thermal sources currently in the environment. */
  [[nodiscard]] const std::vector<stdr_simulation::ThermalSource>& get_thermal_sources() const;

  /** @brief Add a sound emission source to the environment. */
  void add_sound_source(const stdr_simulation::SoundSource& source);

  /**
   * @brief Remove the sound source with the given id. No-op if not found.
   * @param id Value of SoundSource::id to remove.
   */
  void remove_sound_source(const std::string& id);

  /** @brief Return all sound sources currently in the environment. */
  [[nodiscard]] const std::vector<stdr_simulation::SoundSource>& get_sound_sources() const;

private:
  std::optional<stdr_simulation::OccupancyGrid> map_;
  std::unordered_map<std::string, RobotState> robots_;
  int next_robot_id_{ 0 };

  std::vector<stdr_simulation::RfidTag> rfid_tags_;
  std::vector<stdr_simulation::CO2Source> co2_sources_;
  std::vector<stdr_simulation::ThermalSource> thermal_sources_;
  std::vector<stdr_simulation::SoundSource> sound_sources_;
};

}  // namespace stdr_simulation::world
