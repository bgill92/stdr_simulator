#pragma once

#include <stdr_simulation/collision/collision_checker.hpp>
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
#include <stdr_simulation/world/world_model.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace stdr_simulation
{

/** Default sim step duration in seconds — matches the nav-stack convention. */
inline constexpr double kDefaultStepDt = 0.1;

/** Sensor data for a single robot, populated by step(). */
struct RobotSensorData
{
  std::vector<LaserScan> laser_scans;
  std::vector<SonarScan> sonar_scans;
  std::vector<RfidMeasurement> rfid_measurements;
  std::vector<CO2Measurement> co2_measurements;
  std::vector<ThermalMeasurement> thermal_measurements;
  std::vector<SoundMeasurement> sound_measurements;
  /// True if the most recent step detected a collision for this robot.
  bool collided{ false };
};

/**
 * @brief Core simulation tick loop: updates robot motion, resolves collisions,
 * and runs sensor simulators at their configured rates each step.
 *
 * Each robot has its own RateScheduler initialized with the engine step_dt.
 * Sensors fire at the frequency specified in their config; a frequency of 0
 * means "every tick" (backward-compatible default).  Between firings, the most
 * recent sensor measurement is preserved in sensor_data_ so consumers always
 * read a valid (if potentially stale) measurement.
 *
 * TF and Odom streams are tracked per-robot in the scheduler so that
 * StandaloneBackend and ROS2 robot nodes can query last_events() to decide
 * whether to publish on a given tick.
 *
 * @warning Not thread-safe. Intended for use in a single-threaded simulation
 * step loop.
 */
class SimulationEngine
{
public:
  /**
   * @brief Construct the engine, binding it to the given world model.
   *
   * @param world    World model that owns robot state and map.
   * @param step_dt  Sim step duration in seconds.  Must be > 0.
   * @throws std::invalid_argument if step_dt <= 0.
   */
  explicit SimulationEngine(world::WorldModel& world, double step_dt = kDefaultStepDt);
  ~SimulationEngine() = default;

  // --- Step dt ---

  /**
   * @brief Update the sim step duration forwarded to all per-robot schedulers.
   *
   * @param step_dt New step duration in seconds.  Must be > 0.
   * @throws std::invalid_argument if step_dt <= 0 (delegated from RateScheduler).
   */
  void set_step_dt(double step_dt);

  /** @return Current sim step duration in seconds. */
  [[nodiscard]] double step_dt() const noexcept;

  // --- TF / Odom rates ---

  /**
   * @brief Set the target TF publish frequency for all robots.
   *
   * The rate is stored and forwarded to every current and future per-robot
   * scheduler.  The effective rate may be lower than the target if the target
   * exceeds the sim rate (clamped to one fire per tick).
   *
   * @param freq_hz Target frequency in Hz.  <= 0 means every tick.
   */
  void set_tf_rate(double freq_hz);

  /** @return Target TF publish frequency in Hz as last set by set_tf_rate(). */
  [[nodiscard]] double tf_rate() const noexcept;

  /**
   * @brief Set the target odometry publish frequency for all robots.
   *
   * Same semantics as set_tf_rate().
   */
  void set_odom_rate(double freq_hz);

  /** @return Target odom publish frequency in Hz as last set by set_odom_rate(). */
  [[nodiscard]] double odom_rate() const noexcept;

  /**
   * @brief Effective TF rate for a specific robot after snap-to-multiple.
   *
   * @return 0.0 if the robot has no scheduler (not yet registered).
   */
  [[nodiscard]] double effective_tf_rate(const std::string& robot_name) const;

  /**
   * @brief Effective odom rate for a specific robot after snap-to-multiple.
   *
   * @return 0.0 if the robot has no scheduler.
   */
  [[nodiscard]] double effective_odom_rate(const std::string& robot_name) const;

  /**
   * @brief Effective rate for an arbitrary sensor stream on a robot.
   *
   * @param robot_name  Robot name.
   * @param kind        Stream type (Laser, Sonar, …).
   * @param index       Sensor instance index.
   * @return 0.0 if the robot or stream is not registered.
   */
  [[nodiscard]] double effective_sensor_rate(const std::string& robot_name, StreamKind kind, std::size_t index) const;

  // --- Robot lifecycle ---

  /**
   * @brief Spawn a robot into the world at the given pose.
   *
   * Also creates a RateScheduler for the robot and registers TF, Odom, and all
   * per-sensor streams using the frequencies from the config.  Sensors that
   * omit a frequency (freq == 0) default to firing every tick.
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

  // --- Simulation step ---

  /**
   * @brief Advance the simulation by dt seconds.
   *
   * For each robot: integrates motion using the configured kinematic model,
   * rejects the new pose if it collides with the map, then fires only the
   * sensors whose RateScheduler entry is due this tick.  Sensor results for
   * sensors that did not fire are unchanged from the previous tick.
   *
   * @param dt Timestep in seconds.
   */
  void step(double dt);

  // --- Data access ---

  /**
   * @brief Return the latest sensor data for a robot, or nullptr if not found.
   * @param robot_name Target robot name.
   */
  [[nodiscard]] const RobotSensorData* get_sensor_data(const std::string& robot_name) const;

  /**
   * @brief Return the stream events that fired on the most recent tick for a robot.
   *
   * Consumers (e.g. StandaloneBackend) use this to decide whether to publish
   * TF/odom for a given robot on this tick without re-querying the scheduler.
   *
   * @param robot_name  Target robot name.
   * @return Reference to the last tick's event vector.  Empty if not found.
   */
  [[nodiscard]] const std::vector<StreamEvent>& last_events(const std::string& robot_name) const;

private:
  /** Register all sensor streams for a robot into its scheduler. */
  void register_sensor_streams(const std::string& robot_name, const RobotConfig& config);

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

  double step_dt_{ kDefaultStepDt };
  double tf_rate_{ 0.0 };
  double odom_rate_{ 0.0 };

  std::unordered_map<std::string, RobotSensorData> sensor_data_;
  std::unordered_map<std::string, RateScheduler> schedulers_;
  std::unordered_map<std::string, std::vector<StreamEvent>> last_events_;

  // Returned by last_events() when the robot name is not found.
  static const std::vector<StreamEvent> kEmptyEvents;
};

}  // namespace stdr_simulation
