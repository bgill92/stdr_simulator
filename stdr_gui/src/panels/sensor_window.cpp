#include <stdr_gui/panels/sensor_window.hpp>

#include <imgui.h>
#include <implot.h>

#include <cmath>
#include <cstddef>
#include <format>
#include <string>
#include <vector>

namespace stdr_gui
{

namespace
{

constexpr const char* kSensorTypeNames[] = { "Laser", "Sonar", "RFID", "CO2", "Thermal", "Sound" };

}  // namespace

SensorWindow::SensorWindow(std::string robot_name, SensorType type, std::size_t sensor_index)
  : robot_name_(std::move(robot_name)), type_(type), sensor_index_(sensor_index)
{
  const std::size_t type_idx = static_cast<std::size_t>(type_);
  const char* type_name = (type_idx < std::size(kSensorTypeNames)) ? kSensorTypeNames[type_idx] : "Unknown";
  window_title_ = std::format("{} {} [{}]##SensorWindow_{}_{}_{}", robot_name_, type_name, sensor_index_, robot_name_,
                              type_idx, sensor_index_);
}

bool SensorWindow::render(const SimulationSnapshot& snapshot)
{
  if (!open_)
  {
    return false;
  }

  ImGui::SetNextWindowSize(ImVec2(400.0f, 300.0f), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin(window_title_.c_str(), &open_))
  {
    ImGui::End();
    return open_;
  }

  const auto it = snapshot.sensor_data.find(robot_name_);
  if (it == snapshot.sensor_data.end())
  {
    ImGui::TextUnformatted("No sensor data available.");
    ImGui::End();
    return open_;
  }

  const stdr_simulation::RobotSensorData& data = it->second;

  switch (type_)
  {
    case SensorType::kLaser:
      if (sensor_index_ < data.laser_scans.size())
      {
        render_laser(data.laser_scans[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("Laser sensor index out of range.");
      }
      break;
    case SensorType::kSonar:
      if (sensor_index_ < data.sonar_scans.size())
      {
        render_sonar(data.sonar_scans[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("Sonar sensor index out of range.");
      }
      break;
    case SensorType::kRfid:
      if (sensor_index_ < data.rfid_measurements.size())
      {
        render_rfid(data.rfid_measurements[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("RFID sensor index out of range.");
      }
      break;
    case SensorType::kCO2:
      if (sensor_index_ < data.co2_measurements.size())
      {
        render_co2(data.co2_measurements[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("CO2 sensor index out of range.");
      }
      break;
    case SensorType::kThermal:
      if (sensor_index_ < data.thermal_measurements.size())
      {
        render_thermal(data.thermal_measurements[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("Thermal sensor index out of range.");
      }
      break;
    case SensorType::kSound:
      if (sensor_index_ < data.sound_measurements.size())
      {
        render_sound(data.sound_measurements[sensor_index_]);
      }
      else
      {
        ImGui::TextUnformatted("Sound sensor index out of range.");
      }
      break;
  }

  ImGui::End();
  return open_;
}

const std::string& SensorWindow::robot_name() const
{
  return robot_name_;
}

void SensorWindow::render_laser(const stdr_simulation::LaserScan& scan)
{
  const std::size_t n = scan.ranges.size();
  if (n == 0)
  {
    ImGui::TextUnformatted("No laser data.");
    return;
  }

  // Convert polar laser scan to Cartesian points for plotting.
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(n);
  ys.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double range = static_cast<double>(scan.ranges[i]);
    xs.push_back(range * std::cos(angle));
    ys.push_back(range * std::sin(angle));
  }

  if (ImPlot::BeginPlot("Laser Scan", ImVec2(-1.0f, -1.0f)))
  {
    ImPlot::SetupAxes("X (m)", "Y (m)");
    ImPlot::SetupAxisLimits(ImAxis_X1, -scan.range_max, scan.range_max, ImGuiCond_Once);
    ImPlot::SetupAxisLimits(ImAxis_Y1, -scan.range_max, scan.range_max, ImGuiCond_Once);
    ImPlot::PlotScatter("Points", xs.data(), ys.data(), static_cast<int>(n));
    ImPlot::EndPlot();
  }
}

void SensorWindow::render_sonar(const stdr_simulation::SonarScan& scan)
{
  ImGui::Text("Range: %.3f m", scan.range);

  // Sonar max range is not available in the measurement data; use a reasonable
  // default ceiling for the progress bar display.
  constexpr float kDefaultSonarMaxRange = 10.0f;
  const float fraction = (scan.range > 0.0) ? static_cast<float>(scan.range) / kDefaultSonarMaxRange : 0.0f;
  ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f));
}

void SensorWindow::render_rfid(const stdr_simulation::RfidMeasurement& meas)
{
  const std::size_t n = meas.tag_ids.size();
  ImGui::Text("Tags detected: %zu", n);
  ImGui::Separator();
  for (std::size_t i = 0; i < n; ++i)
  {
    ImGui::Text("ID: %s  Msg: %s  Strength: %.1f dB", meas.tag_ids[i].c_str(), meas.tag_messages[i].c_str(),
                (i < meas.tag_dbs.size()) ? meas.tag_dbs[i] : 0.0);
  }
}

void SensorWindow::render_co2(const stdr_simulation::CO2Measurement& meas)
{
  ImGui::Text("CO2 concentration: %.2f ppm", meas.ppm);
  const float fraction = static_cast<float>(meas.ppm) / 1000.0f;
  ImGui::ProgressBar(fraction > 1.0f ? 1.0f : fraction, ImVec2(-1.0f, 0.0f));
}

void SensorWindow::render_thermal(const stdr_simulation::ThermalMeasurement& meas)
{
  ImGui::Text("Thermal sources detected: %zu", meas.source_degrees.size());
  ImGui::Separator();
  for (std::size_t i = 0; i < meas.source_degrees.size(); ++i)
  {
    ImGui::Text("Source %zu: %.1f deg", i, meas.source_degrees[i]);
  }
}

void SensorWindow::render_sound(const stdr_simulation::SoundMeasurement& meas)
{
  ImGui::Text("Sound level: %.1f dB", meas.dbs);
  // Reference ceiling for sound level display (typical maximum for acoustic sensors).
  constexpr float kSoundMaxDb = 120.0f;
  const float fraction = static_cast<float>(meas.dbs) / kSoundMaxDb;
  ImGui::ProgressBar(fraction > 1.0f ? 1.0f : fraction, ImVec2(-1.0f, 0.0f));
}

}  // namespace stdr_gui
