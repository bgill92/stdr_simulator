#include <stdr_simulation/simulation_engine.hpp>

#include <stdr_simulation/geometry_utils.hpp>

#include <stdexcept>

namespace stdr_simulation
{

SimulationEngine::SimulationEngine(world::WorldModel& world) : world_(world)
{
}

std::string SimulationEngine::spawn_robot(const RobotConfig& config, const Pose2D& pose)
{
  const std::string& model_type = config.kinematic_model.type;
  if (!model_type.empty() && model_type != "ideal" && model_type != "omni")
  {
    throw std::invalid_argument("SimulationEngine: unknown kinematic model type '" + model_type +
                                "'. Supported types: 'ideal', 'omni'.");
  }

  // Override the baked-in initial pose so the world model stores the correct
  // starting position regardless of what the config file contained.
  RobotConfig cfg = config;
  cfg.initial_pose = pose;

  // add_robot uses cfg.initial_pose as the starting pose, so a separate
  // set_robot_pose call is not needed.
  const std::string name = world_.add_robot(cfg);
  sensor_data_[name] = RobotSensorData{};
  return name;
}

void SimulationEngine::delete_robot(const std::string& name)
{
  world_.remove_robot(name);
  sensor_data_.erase(name);
}

void SimulationEngine::set_cmd_vel(const std::string& robot_name, const Twist2D& cmd)
{
  world_.set_robot_cmd_vel(robot_name, cmd);
}

void SimulationEngine::step(double dt)
{
  const std::vector<world::RobotState> robots = world_.get_all_robots();
  const OccupancyGrid* map = world_.get_map();

  for (const world::RobotState& robot : robots)
  {
    // --- Motion update ---
    Pose2D new_pose;
    if (robot.config.kinematic_model.type == "omni")
    {
      new_pose = omni_motion_.update(robot.pose, robot.cmd_vel, dt, robot.config.kinematic_model);
    }
    else
    {
      new_pose = ideal_motion_.update(robot.pose, robot.cmd_vel, dt, robot.config.kinematic_model);
    }

    // --- Collision check and pose commit ---
    bool robot_collided = false;
    if (map != nullptr)
    {
      const bool collides = collision_checker_.check_path_collision(new_pose, robot.pose, robot.config.footprint, *map);
      if (!collides)
      {
        world_.set_robot_pose(robot.name, new_pose);
      }
      else
      {
        // On collision, keep the old pose — do not call set_robot_pose.
        robot_collided = true;
      }
    }
    else
    {
      world_.set_robot_pose(robot.name, new_pose);
    }

    // --- Sensor simulation ---
    // Re-read the robot state after the pose update.  If the robot was
    // concurrently removed (or never existed after the snapshot), skip it
    // entirely so sensor_data_ never holds stale entries.
    const world::RobotState* updated = world_.get_robot(robot.name);
    if (updated == nullptr)
    {
      continue;
    }

    RobotSensorData data{};
    data.collided = robot_collided;

    // Laser and sonar require a map — skip if none has been loaded.
    if (map != nullptr)
    {
      for (const LaserConfig& cfg : updated->config.laser_sensors)
      {
        const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
        data.laser_scans.push_back(laser_sim_.simulate(sw, cfg, *map));
      }
      for (const SonarConfig& cfg : updated->config.sonar_sensors)
      {
        const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
        data.sonar_scans.push_back(sonar_sim_.simulate(sw, cfg, *map));
      }
    }

    // Environment sensors do not need a map.
    for (const RfidSensorConfig& cfg : updated->config.rfid_sensors)
    {
      const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
      data.rfid_measurements.push_back(rfid_sim_.simulate(sw, cfg, world_.get_rfid_tags()));
    }
    for (const CO2SensorConfig& cfg : updated->config.co2_sensors)
    {
      const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
      data.co2_measurements.push_back(co2_sim_.simulate(sw, cfg, world_.get_co2_sources()));
    }
    for (const ThermalSensorConfig& cfg : updated->config.thermal_sensors)
    {
      const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
      data.thermal_measurements.push_back(thermal_sim_.simulate(sw, cfg, world_.get_thermal_sources()));
    }
    for (const SoundSensorConfig& cfg : updated->config.sound_sensors)
    {
      const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
      data.sound_measurements.push_back(sound_sim_.simulate(sw, cfg, world_.get_sound_sources()));
    }

    sensor_data_[robot.name] = std::move(data);
  }
}

const RobotSensorData* SimulationEngine::get_sensor_data(const std::string& robot_name) const
{
  const auto it = sensor_data_.find(robot_name);
  if (it == sensor_data_.end())
  {
    return nullptr;
  }
  return &it->second;
}

}  // namespace stdr_simulation
