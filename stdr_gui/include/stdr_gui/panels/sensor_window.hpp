#pragma once

#include <stdr_gui/simulator_backend.hpp>

#include <cstddef>
#include <string>

namespace stdr_gui
{

/** @brief Type of sensor to visualize. */
enum class SensorType
{
  kLaser,
  kSonar,
  kRfid,
  kCO2,
  kThermal,
  kSound,
};

/** @brief Standalone dockable window for visualizing a single sensor's data.
 *
 *  @warning Not thread-safe. Must be called from the render thread only. */
class SensorWindow
{
public:
  SensorWindow(std::string robot_name, SensorType type, std::size_t sensor_index);

  /** @brief Render the window. Returns false if the window was closed. */
  [[nodiscard]] bool render(const SimulationSnapshot& snapshot);

  /** @brief Get the robot this window is associated with. */
  [[nodiscard]] const std::string& robot_name() const;

private:
  void render_laser(const stdr_simulation::LaserScan& scan);
  void render_sonar(const stdr_simulation::SonarScan& scan);
  void render_rfid(const stdr_simulation::RfidMeasurement& meas);
  void render_co2(const stdr_simulation::CO2Measurement& meas);
  void render_thermal(const stdr_simulation::ThermalMeasurement& meas);
  void render_sound(const stdr_simulation::SoundMeasurement& meas);

  std::string robot_name_;
  SensorType type_;
  std::size_t sensor_index_;
  bool open_{ true };
  std::string window_title_;
};

}  // namespace stdr_gui
