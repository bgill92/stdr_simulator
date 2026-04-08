#include <stdr_gui/panels/robot_info_panel.hpp>

#include <imgui.h>

#include <format>
#include <string>

namespace stdr_gui
{

void RobotInfoPanel::render(const SimulationSnapshot& snapshot, SimulatorBackend& backend,
                            std::vector<std::unique_ptr<SensorWindow>>& sensor_windows)
{
  ImGui::Begin("Robot Info", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

  if (snapshot.robots.empty())
  {
    ImGui::TextUnformatted("No robots in simulation.");
    ImGui::End();
    return;
  }

  for (const stdr_simulation::world::RobotState& robot : snapshot.robots)
  {
    const stdr_simulation::RobotSensorData* sensor_data = nullptr;
    const auto it = snapshot.sensor_data.find(robot.name);
    if (it != snapshot.sensor_data.end())
    {
      sensor_data = &it->second;
    }
    render_robot_entry(robot, sensor_data, backend, sensor_windows);
  }

  ImGui::End();
}

void RobotInfoPanel::render_robot_entry(const stdr_simulation::world::RobotState& robot,
                                        const stdr_simulation::RobotSensorData* sensor_data, SimulatorBackend& backend,
                                        std::vector<std::unique_ptr<SensorWindow>>& sensor_windows)
{
  const bool is_selected = (robot.name == selected_robot_);
  if (is_selected)
  {
    ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.3f, 0.5f, 0.9f, 0.8f));
  }

  const bool node_open =
      ImGui::TreeNodeEx(robot.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth);

  if (is_selected)
  {
    ImGui::PopStyleColor();
  }

  if (ImGui::IsItemClicked())
  {
    selected_robot_ = robot.name;
  }

  if (!node_open)
  {
    return;
  }

  // Pose.
  ImGui::Text("Pose:  x=%.3f  y=%.3f  θ=%.3f", robot.pose.x, robot.pose.y, robot.pose.theta);

  // Velocity.
  ImGui::Text("Vel:   vx=%.3f  vy=%.3f  ω=%.3f", robot.cmd_vel.linear_x, robot.cmd_vel.linear_y,
              robot.cmd_vel.angular_z);

  // Sensor summary.
  ImGui::Separator();
  ImGui::Text("Lasers: %zu  Sonars: %zu  RFID: %zu  CO2: %zu  Thermal: %zu  Sound: %zu",
              robot.config.laser_sensors.size(), robot.config.sonar_sensors.size(), robot.config.rfid_sensors.size(),
              robot.config.co2_sensors.size(), robot.config.thermal_sensors.size(), robot.config.sound_sensors.size());

  ImGui::Separator();

  // Action buttons: Delete.
  if (ImGui::Button("Delete"))
  {
    backend.delete_robot(robot.name);
    if (selected_robot_ == robot.name)
    {
      selected_robot_.clear();
    }
    ImGui::TreePop();
    return;
  }

  // Buttons to open sensor windows.
  for (std::size_t i = 0; i < robot.config.laser_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("Laser {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kLaser, i));
    }
  }

  for (std::size_t i = 0; i < robot.config.sonar_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("Sonar {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kSonar, i));
    }
  }

  for (std::size_t i = 0; i < robot.config.rfid_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("RFID {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kRfid, i));
    }
  }

  for (std::size_t i = 0; i < robot.config.co2_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("CO2 {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kCO2, i));
    }
  }

  for (std::size_t i = 0; i < robot.config.thermal_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("Thermal {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kThermal, i));
    }
  }

  for (std::size_t i = 0; i < robot.config.sound_sensors.size(); ++i)
  {
    ImGui::SameLine();
    const std::string label = std::format("Sound {}##{}_{}", i, robot.name, i);
    if (ImGui::Button(label.c_str()))
    {
      sensor_windows.push_back(std::make_unique<SensorWindow>(robot.name, SensorType::kSound, i));
    }
  }

  // Show live sensor values if data is available.
  if (sensor_data != nullptr)
  {
    ImGui::Separator();

    if (!sensor_data->laser_scans.empty())
    {
      for (std::size_t i = 0; i < sensor_data->laser_scans.size(); ++i)
      {
        ImGui::Text("Laser[%zu]: %zu rays", i, sensor_data->laser_scans[i].ranges.size());
      }
    }

    if (!sensor_data->sonar_scans.empty())
    {
      for (std::size_t i = 0; i < sensor_data->sonar_scans.size(); ++i)
      {
        ImGui::Text("Sonar[%zu]: %.3f m", i, sensor_data->sonar_scans[i].range);
      }
    }

    if (!sensor_data->co2_measurements.empty())
    {
      for (std::size_t i = 0; i < sensor_data->co2_measurements.size(); ++i)
      {
        ImGui::Text("CO2[%zu]: %.2f ppm", i, sensor_data->co2_measurements[i].ppm);
      }
    }

    if (!sensor_data->sound_measurements.empty())
    {
      for (std::size_t i = 0; i < sensor_data->sound_measurements.size(); ++i)
      {
        ImGui::Text("Sound[%zu]: %.1f dB", i, sensor_data->sound_measurements[i].dbs);
      }
    }
  }

  ImGui::TreePop();
}

}  // namespace stdr_gui
