#pragma once

#include <stdr_gui/panels/sensor_window.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace stdr_gui
{

/** @brief Dockable panel showing a list of robots and their details.
 *
 *  @warning Not thread-safe. Must be called from the render thread only. */
class RobotInfoPanel
{
public:
  /** @brief Render the panel.
   *  @param sensor_windows Mutable reference so the panel can spawn new sensor windows. */
  void render(const SimulationSnapshot& snapshot, SimulatorBackend& backend,
              std::vector<std::unique_ptr<SensorWindow>>& sensor_windows);

  /** @brief Check if sensor visualization is enabled for a robot. */
  [[nodiscard]] bool show_sensors_for(const std::string& robot_name) const;

  /** @brief Get the currently selected robot name (empty if none). */
  [[nodiscard]] const std::string& selected_robot() const;

private:
  void render_robot_entry(const stdr_simulation::world::RobotState& robot,
                          const stdr_simulation::RobotSensorData* sensor_data, SimulatorBackend& backend,
                          std::vector<std::unique_ptr<SensorWindow>>& sensor_windows);

  std::string selected_robot_;
  std::unordered_set<std::string> show_sensors_;
};

}  // namespace stdr_gui
