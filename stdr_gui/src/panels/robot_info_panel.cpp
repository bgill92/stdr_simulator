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

  // Sim / TF / odom rate status block — global, not per-robot.
  // Sim step is always shown; TF and odom rows are gated on the backend because
  // standalone mode has no ROS publishers and those rates are meaningless there.
  const double step_dt = backend.get_step_dt();
  const double sim_hz = step_dt > 0.0 ? 1.0 / step_dt : 0.0;
  ImGui::Text("Sim step: %.2f s (%.1f Hz)", step_dt, sim_hz);
  if (backend.publishes_ros_topics())
  {
    const double tf_target = backend.get_tf_rate();
    const double tf_eff = backend.get_effective_tf_rate();
    ImGui::Text("TF: %.1f Hz (eff. %.1f Hz)", tf_target, tf_eff);
    const double odom_target = backend.get_odom_rate();
    const double odom_eff = backend.get_effective_odom_rate();
    ImGui::Text("Odom: %.1f Hz (eff. %.1f Hz)", odom_target, odom_eff);
  }
  ImGui::Separator();

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

bool RobotInfoPanel::show_sensors_for(const std::string& robot_name) const
{
  return show_sensors_.contains(robot_name);
}

const std::string& RobotInfoPanel::selected_robot() const
{
  return selected_robot_;
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

  // Per-sensor rate summary — only shown when the backend provides non-zero
  // values (i.e. StandaloneBackend). Ros2Backend does not expose per-sensor
  // rate getters; their defaults return 0.
  // TF/odom effective rates are computed from the stored target rate and step_dt.
  {
    const std::string tree_label = "Sensor rates##" + robot.name;
    if (ImGui::TreeNode(tree_label.c_str()))
    {
      bool any_shown = false;
      // Per-sensor rate getters currently exist only for laser and sonar.
      // Other sensor types (rfid/co2/thermal/sound) fire at the global sim step.
      for (std::size_t i = 0; i < robot.config.laser_sensors.size(); ++i)
      {
        const double rate = backend.get_laser_rate(robot.name, i);
        if (rate > 0.0)
        {
          ImGui::Text("Laser[%zu]: %.1f Hz", i, rate);
          any_shown = true;
        }
      }
      for (std::size_t i = 0; i < robot.config.sonar_sensors.size(); ++i)
      {
        const double rate = backend.get_sonar_rate(robot.name, i);
        if (rate > 0.0)
        {
          ImGui::Text("Sonar[%zu]: %.1f Hz", i, rate);
          any_shown = true;
        }
      }
      if (!any_shown)
      {
        ImGui::TextUnformatted("No rate data available.");
      }
      ImGui::TreePop();
    }
  }

  // Toggle sensor visualization on the map.
  bool show = show_sensors_.contains(robot.name);
  const std::string checkbox_label = "Show Sensors##" + robot.name;
  if (ImGui::Checkbox(checkbox_label.c_str(), &show))
  {
    if (show)
    {
      show_sensors_.insert(robot.name);
    }
    else
    {
      show_sensors_.erase(robot.name);
    }
  }

  ImGui::Separator();

  // Action buttons: Delete.
  if (ImGui::Button("Delete"))
  {
    show_sensors_.erase(robot.name);
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

    // Guard each slot with an emptiness check: after Group 1 pre-sizing, slots
    // are always allocated at spawn time but ranges/values are zero until the
    // first map-assisted physics step produces real measurements.
    for (std::size_t i = 0; i < sensor_data->laser_scans.size(); ++i)
    {
      const stdr_simulation::LaserScan& scan = sensor_data->laser_scans[i];
      if (scan.ranges.empty())
      {
        continue;
      }
      ImGui::Text("Laser[%zu]: %zu rays", i, scan.ranges.size());
    }

    for (std::size_t i = 0; i < sensor_data->sonar_scans.size(); ++i)
    {
      const stdr_simulation::SonarScan& sonar = sensor_data->sonar_scans[i];
      // SonarScan has no "no-measurement" flag; range==0 is used as a proxy for
      // "slot pre-sized but no scan produced yet". This also suppresses
      // legitimate readings at exactly 0 m, which is acceptable since real
      // sonar physics enforce a positive min_range.
      if (sonar.range <= 0.0)
      {
        continue;
      }
      ImGui::Text("Sonar[%zu]: %.3f m", i, sonar.range);
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
