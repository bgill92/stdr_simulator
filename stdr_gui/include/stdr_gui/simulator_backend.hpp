#pragma once

// RobotSensorData is defined here; there is no lighter header for it yet.
#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>
#include <tl_expected/expected.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace stdr_gui
{

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

  /** @brief Teleport a robot to a new pose (e.g. drag-and-drop). */
  virtual void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) = 0;

  /** @brief Get a thread-safe snapshot of the current simulation state.
   *
   *  The returned shared_ptr is immutable. The GUI holds it for one frame
   *  while the backend may atomically swap in a new snapshot. */
  [[nodiscard]] virtual std::shared_ptr<const SimulationSnapshot> get_snapshot() const = 0;

  /** @brief Poll for backend-produced log/status messages (non-blocking). */
  [[nodiscard]] virtual std::vector<std::string> poll_messages() = 0;

protected:
  SimulatorBackend() = default;
  SimulatorBackend(SimulatorBackend&&) = default;
  SimulatorBackend& operator=(SimulatorBackend&&) = default;
};

}  // namespace stdr_gui
