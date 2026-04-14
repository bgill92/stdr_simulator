#pragma once

#include <stdr_simulation/collision/collision_checker.hpp>
#include <stdr_simulation/motion/ideal_motion_model.hpp>
#include <stdr_simulation/motion/omni_motion_model.hpp>
#include <stdr_simulation/sensors/co2_simulator.hpp>
#include <stdr_simulation/sensors/laser_simulator.hpp>
#include <stdr_simulation/sensors/rfid_simulator.hpp>
#include <stdr_simulation/sensors/sonar_simulator.hpp>
#include <stdr_simulation/sensors/sound_simulator.hpp>
#include <stdr_simulation/sensors/thermal_simulator.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace stdr_simulation
{

/** Sensor data for a single robot, populated by step(). */
struct RobotSensorData
{
  std::vector<LaserScan> laser_scans;
  std::vector<SonarScan> sonar_scans;
  std::vector<RfidMeasurement> rfid_measurements;
  std::vector<CO2Measurement> co2_measurements;
  std::vector<ThermalMeasurement> thermal_measurements;
  std::vector<SoundMeasurement> sound_measurements;
};

/**
 * @brief Core simulation tick loop: updates robot motion, resolves collisions,
 * and runs all sensor simulators each step.
 *
 * @warning Not thread-safe. Intended for use in a single-threaded simulation
 * step loop.
 */
class SimulationEngine
{
public:
  /** @brief Construct the engine, binding it to the given world model. */
  explicit SimulationEngine(world::WorldModel& world);
  ~SimulationEngine() = default;

  /**
   * @brief Spawn a robot into the world at the given pose.
   *
   * The provided pose overrides any initial_pose baked into the config so the
   * caller controls placement without mutating the config they pass in.
   *
   * @param config Full sensor and kinematic configuration.
   * @param pose   Initial world-frame pose for the robot.
   * @return The name assigned to this robot by the world model.
   */
  [[nodiscard]] std::string spawn_robot(const RobotConfig& config, const Pose2D& pose);

  /**
   * @brief Remove a robot from the world by name.
   * @param name Name previously returned by spawn_robot.
   */
  void delete_robot(const std::string& name);

  /**
   * @brief Set the velocity command for a robot.
   * @param robot_name Target robot name.
   * @param cmd        Velocity command to apply on the next step.
   */
  void set_cmd_vel(const std::string& robot_name, const Twist2D& cmd);

  /**
   * @brief Advance the simulation by dt seconds.
   *
   * For each robot: integrates motion using the configured kinematic model,
   * rejects the new pose if it collides with the map, then runs all sensors.
   * Sensor results are stored and retrievable via get_sensor_data().
   *
   * @param dt Timestep in seconds.
   */
  void step(double dt);

  /**
   * @brief Return the latest sensor data for a robot, or nullptr if not found.
   * @param robot_name Target robot name.
   */
  [[nodiscard]] const RobotSensorData* get_sensor_data(const std::string& robot_name) const;

private:
  world::WorldModel& world_;
  collision::CollisionChecker collision_checker_;
  motion::IdealMotionModel ideal_motion_;
  motion::OmniMotionModel omni_motion_;
  sensors::LaserSimulator laser_sim_;
  sensors::SonarSimulator sonar_sim_;
  sensors::RfidSimulator rfid_sim_;
  sensors::Co2Simulator co2_sim_;
  sensors::ThermalSimulator thermal_sim_;
  sensors::SoundSimulator sound_sim_;

  std::unordered_map<std::string, RobotSensorData> sensor_data_;
};

}  // namespace stdr_simulation
