#pragma once

// RobotSensorData is defined here; there is no lighter header for it yet.
#include <stdr_gui/plot/plotter.hpp>
#include <stdr_simulation/plot_data/command_queue.hpp>
#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>
#include <tl_expected/expected.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace stdr_gui
{

inline constexpr double kMinStepDt = 0.001;
inline constexpr double kMaxStepDt = 1.0;
inline constexpr double kDefaultStepDt = stdr_simulation::kDefaultStepDt;
inline constexpr double kDefaultTfRate = 50.0;
inline constexpr double kDefaultOdomRate = 10.0;
inline constexpr double kMinRateHz = 0.0;
inline constexpr double kMaxRateHz = 1000.0;

/** @brief Thread-safe snapshot of simulation state for the GUI to render.
 *
 *  The backend populates this on its thread and the GUI reads it on the
 *  render thread. Exchanged atomically via shared_ptr swap. */
struct SimulationSnapshot
{
  stdr_simulation::OccupancyGrid map;
  std::string map_name;  // Filename of the loaded map (e.g., "maze1.yaml").
  std::vector<stdr_simulation::world::RobotState> robots;
  std::unordered_map<std::string, stdr_simulation::RobotSensorData> sensor_data;
  std::vector<stdr_simulation::RfidTag> rfid_tags;
  std::vector<stdr_simulation::CO2Source> co2_sources;
  std::vector<stdr_simulation::ThermalSource> thermal_sources;
  std::vector<stdr_simulation::SoundSource> sound_sources;
  double elapsed_time{ 0.0 };
  bool running{ false };

  /** @brief Look up a robot by name in the snapshot.
   *
   *  Avoids forcing every plotter to write an identical std::find_if loop.
   *  Returns a non-owning pointer into `robots` — valid as long as the caller
   *  holds a reference to this `SimulationSnapshot` instance (typically via
   *  `shared_ptr<const SimulationSnapshot>`).  Returns nullptr if no robot
   *  with the given name is present. */
  [[nodiscard]] const stdr_simulation::world::RobotState* robot(std::string_view name) const noexcept
  {
    const auto it = std::find_if(robots.begin(), robots.end(),
                                 [name](const stdr_simulation::world::RobotState& r) { return r.name == name; });
    return it != robots.end() ? &(*it) : nullptr;
  }
};

/** @brief Result of a poll_laser_events() call.
 *
 *  Owns the copied scan data — the copy is performed under the sensor ring
 *  lock so callers never access the ring's internal buffer after the lock is
 *  released.  Eliminates the data race that existed when callers inserted from
 *  a span returned after the lock had already been dropped. */
struct DrainedLaserResult
{
  std::vector<stdr::plot::TimedLaserScan> scans;
  std::size_t dropped{ 0 };
};

/** @brief Result of a poll_sonar_events() call.
 *
 *  Same ownership model as `DrainedLaserResult` — copy is done under lock. */
struct DrainedSonarResult
{
  std::vector<stdr::plot::TimedSonarReading> scans;
  std::size_t dropped{ 0 };
};

/** @brief Abstract interface between the GUI and the simulation.
 *
 *  Implementations must be safe to call from the GUI (render) thread.
 *  The backend is responsible for any internal synchronization needed. */
class SimulatorBackend
{
public:
  virtual ~SimulatorBackend() = default;

  // Non-copyable.
  SimulatorBackend(const SimulatorBackend&) = delete;
  SimulatorBackend& operator=(const SimulatorBackend&) = delete;

  /** @brief Load a map from a YAML metadata file. */
  [[nodiscard]] virtual tl::expected<void, std::string> load_map(const std::string& yaml_path) = 0;

  /** @brief Spawn a robot from a YAML config file at the given pose.
   *  @return The assigned robot name on success. */
  [[nodiscard]] virtual tl::expected<std::string, std::string> spawn_robot(const std::string& yaml_path,
                                                                           const stdr_simulation::Pose2D& pose) = 0;

  /** @brief Delete a robot by name. */
  virtual void delete_robot(const std::string& name) = 0;

  /** @brief Start or resume the simulation loop. */
  virtual void start() = 0;

  /** @brief Pause the simulation loop. */
  virtual void pause() = 0;

  /** @brief Reset the simulation: remove all robots, keep map. */
  virtual void reset() = 0;

  /** @brief Set simulation speed multiplier (1.0 = realtime). */
  virtual void set_speed(double multiplier) = 0;

  /** @brief Set the simulation step duration in seconds.
   *
   *  Controls both the real-time tick interval and the dt passed to the
   *  physics engine. Smaller values increase CPU cost and sensor sampling
   *  rate; larger values coarsen integration. Orthogonal to set_speed().
   *
   *  Implementations must clamp to [kMinStepDt, kMaxStepDt]. */
  virtual void set_step_dt(double seconds) = 0;

  /** @brief Current simulation step duration in seconds. */
  [[nodiscard]] virtual double get_step_dt() const = 0;

  /**
   * @brief Set the TF broadcast rate for all robots.
   *
   * Implementations must clamp to [kMinRateHz, kMaxRateHz].
   * A value of 0 means "every sim tick".
   */
  virtual void set_tf_rate(double hz) = 0;

  /** @brief Current TF broadcast rate in Hz. */
  [[nodiscard]] virtual double get_tf_rate() const = 0;

  /**
   * @brief Set the odometry publish rate for all robots.
   *
   * Implementations must clamp to [kMinRateHz, kMaxRateHz].
   * A value of 0 means "every sim tick".
   */
  virtual void set_odom_rate(double hz) = 0;

  /** @brief Current odometry publish rate in Hz. */
  [[nodiscard]] virtual double get_odom_rate() const = 0;

  /**
   * @brief Effective laser sensor rate for a specific instance on a robot.
   *
   * Returns 0.0 if the robot or sensor index is unknown.  Default
   * implementation returns 0.0 so backends that have not yet wired this
   * (e.g. Ros2Backend) compile without overriding.
   */
  [[nodiscard]] virtual double get_laser_rate(const std::string& /*robot_id*/, std::size_t /*sensor_index*/) const
  {
    return 0.0;
  }

  /**
   * @brief Effective sonar sensor rate for a specific instance on a robot.
   *
   * Returns 0.0 if the robot or sensor index is unknown.  Default
   * implementation returns 0.0 so backends that have not yet wired this
   * compile without overriding.
   */
  [[nodiscard]] virtual double get_sonar_rate(const std::string& /*robot_id*/, std::size_t /*sensor_index*/) const
  {
    return 0.0;
  }

  /** @brief Teleport a robot to a new pose (e.g. drag-and-drop). */
  virtual void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) = 0;

  /** @brief Send a velocity command to a robot.
   *
   *  Replaces any previously outstanding command for that robot. If
   *  @p robot_name is not present, the call is a no-op.
   *
   *  @param robot_name  Name of the target robot.
   *  @param cmd         Desired linear and angular velocity. */
  virtual void set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd) = 0;

  /** @brief Get a thread-safe snapshot of the current simulation state.
   *
   *  The returned shared_ptr is immutable. The GUI holds it for one frame
   *  while the backend may atomically swap in a new snapshot. */
  [[nodiscard]] virtual std::shared_ptr<const SimulationSnapshot> get_snapshot() const = 0;

  /** @brief Poll for backend-produced log/status messages (non-blocking). */
  [[nodiscard]] virtual std::vector<std::string> poll_messages() = 0;

  // --- Introspection methods ---
  //
  // Each call acquires the simulation lock for the duration of the call, so
  // returned values are internally consistent. Callers making multiple
  // successive calls may observe updates between calls; batching via a single
  // get_snapshot() call is the caller's responsibility when cross-call
  // consistency is required.
  //
  // Default implementations return empty / nullopt so that backends that have
  // not yet wired these (e.g. Ros2Backend) compile and behave safely without
  // overriding them.

  /** @brief Return the number of robots currently in the simulation. */
  [[nodiscard]] virtual std::size_t num_robots() const
  {
    return 0;
  }

  /** @brief Return the names of all robots currently in the simulation.
   *
   *  Returns owning strings rather than views to prevent dangling references
   *  if the caller stores IDs across frames. */
  [[nodiscard]] virtual std::vector<std::string> robot_ids() const
  {
    return {};
  }

  /** @brief Return the sensor frame IDs of all laser sensors on @p robot_id.
   *
   *  Returns an empty vector if @p robot_id is not found. */
  [[nodiscard]] virtual std::vector<std::string> laser_sensors(const std::string& /*robot_id*/) const
  {
    return {};
  }

  /** @brief Return the sensor frame IDs of all sonar sensors on @p robot_id.
   *
   *  Returns an empty vector if @p robot_id is not found. */
  [[nodiscard]] virtual std::vector<std::string> sonar_sensors(const std::string& /*robot_id*/) const
  {
    return {};
  }

  /** @brief Return the current pose of @p robot_id, or nullopt if not found. */
  [[nodiscard]] virtual std::optional<stdr_simulation::Pose2D> pose(const std::string& /*robot_id*/) const
  {
    return std::nullopt;
  }

  /** @brief Return the current velocity command of @p robot_id, or nullopt if not found. */
  [[nodiscard]] virtual std::optional<stdr_simulation::Twist2D> twist(const std::string& /*robot_id*/) const
  {
    return std::nullopt;
  }

  /** @brief Return the latest laser scan from @p sensor_id on @p robot_id.
   *
   *  Returns nullopt if the robot or sensor is not found, or if no scan has
   *  been produced yet (e.g. no map loaded). */
  [[nodiscard]] virtual std::optional<stdr_simulation::LaserScan> latest_laser(const std::string& /*robot_id*/,
                                                                               const std::string& /*sensor_id*/) const
  {
    return std::nullopt;
  }

  /** @brief Return the latest sonar reading from @p sensor_id on @p robot_id.
   *
   *  Returns nullopt if the robot or sensor is not found, or if no scan has
   *  been produced yet (e.g. no map loaded). */
  [[nodiscard]] virtual std::optional<stdr_simulation::SonarScan> latest_sonar(const std::string& /*robot_id*/,
                                                                               const std::string& /*sensor_id*/) const
  {
    return std::nullopt;
  }

  /** @brief Return whether @p robot_id collided on the most recent step.
   *
   *  Returns nullopt if @p robot_id is not found. */
  [[nodiscard]] virtual std::optional<bool> collided(const std::string& /*robot_id*/) const
  {
    return std::nullopt;
  }

  /** @brief Return the robot's footprint polygon vertices in robot-local frame.
   *
   *  Returns an empty vector if the robot is not found or has no footprint.
   *  @note Default is a no-op fallback for backends that have not yet wired
   *        footprint introspection (e.g. Ros2Backend). */
  [[nodiscard]] virtual std::vector<stdr_simulation::Point2D> footprint(const std::string& /*robot_id*/) const
  {
    return {};
  }

  /** @brief Return the elapsed simulation time in seconds.
   *  @return 0.0 if the backend has not yet wired this method. */
  [[nodiscard]] virtual double sim_time() const
  {
    return 0.0;
  }

  // --- Sensor event log polling ---
  //
  // These methods allow the GUI thread to drain per-sensor SPSC rings that the
  // sim thread fills after each physics step.  Each plotter maintains its own
  // cursor so multiple plotters can independently consume the same ring without
  // starving each other.
  //
  // The cursor is an opaque sequence number: initialise to 0 before the first
  // call and pass the same variable on every subsequent call.  The backend
  // advances it past any returned (and any dropped) entries.
  //
  // DrainedLaserResult/DrainedSonarResult own their scan data via std::vector.
  // The copy from the ring's internal buffer is performed while the sensor ring
  // lock is held, so callers receive stable, owned data with no lifetime
  // dependency on the ring.
  //
  // Default implementations return empty results so backends that have not
  // wired event logs (e.g. Ros2Backend) compile and behave safely.

  /** @brief Drain laser scans produced since the last call with @p cursor.
   *
   *  @param[in,out] cursor  Opaque per-consumer sequence number; initialise
   *                         to 0 before the first call.
   *  @param robot_id        Robot name.
   *  @param sensor_id       Laser sensor frame ID.
   *  @return Owning vector of scans (copied under lock) and a count of dropped
   *          entries.  Empty if the robot/sensor pair is not found. */
  [[nodiscard]] virtual DrainedLaserResult poll_laser_events(std::uint64_t& /*cursor*/, const std::string& /*robot_id*/,
                                                             const std::string& /*sensor_id*/)
  {
    return {};
  }

  /** @brief Drain sonar readings produced since the last call with @p cursor.
   *
   *  See `poll_laser_events` for cursor and lifetime semantics. */
  [[nodiscard]] virtual DrainedSonarResult poll_sonar_events(std::uint64_t& /*cursor*/, const std::string& /*robot_id*/,
                                                             const std::string& /*sensor_id*/)
  {
    return {};
  }

  // --- Command injection ---

  /** @brief Enqueue a command to be dispatched by the sim thread.
   *
   *  Backends that do not yet support command injection (e.g. Ros2Backend)
   *  inherit this no-op default.  `StandaloneBackend` overrides it to push
   *  into `cmd_queue_`, which the sim thread drains at the top of each step.
   *
   *  Callers must not hold `sim_mutex_` when calling this method — the point
   *  is to avoid GUI-thread contention with the sim thread.
   *
   *  @param cmd  Command to enqueue.  Copied into the queue; the caller's
   *              copy is unaffected. */
  virtual void push_command(stdr::plot_data::Command /*cmd*/)
  {
  }

protected:
  SimulatorBackend() = default;
  SimulatorBackend(SimulatorBackend&&) = default;
  SimulatorBackend& operator=(SimulatorBackend&&) = default;
};

}  // namespace stdr_gui
