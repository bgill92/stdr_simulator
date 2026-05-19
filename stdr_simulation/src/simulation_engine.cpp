#include <stdr_simulation/simulation_engine.hpp>

#include <stdr_simulation/geometry_utils.hpp>

#include <algorithm>
#include <stdexcept>

namespace stdr_simulation
{

const std::vector<StreamEvent> SimulationEngine::kEmptyEvents{};

SimulationEngine::SimulationEngine(world::WorldModel& world, double step_dt) : world_(world), step_dt_(step_dt)
{
  if (step_dt <= 0.0)
  {
    throw std::invalid_argument("step_dt must be positive");
  }
}

// --- Step dt / rate setters ---

void SimulationEngine::set_step_dt(double step_dt)
{
  if (step_dt <= 0.0)
  {
    throw std::invalid_argument("step_dt must be positive");
  }
  for (auto& [name, scheduler] : schedulers_)
  {
    scheduler.set_step_dt(step_dt);
  }
  step_dt_ = step_dt;
}

double SimulationEngine::step_dt() const noexcept
{
  return step_dt_;
}

void SimulationEngine::set_scheduling_mode(SchedulingMode mode)
{
  scheduling_mode_ = mode;
}

SchedulingMode SimulationEngine::scheduling_mode() const noexcept
{
  return scheduling_mode_;
}

void SimulationEngine::set_tf_rate(double freq_hz)
{
  tf_rate_ = freq_hz;
  for (auto& [name, scheduler] : schedulers_)
  {
    scheduler.set_rate(StreamKind::Tf, 0, freq_hz, scheduling_mode_);
  }
}

double SimulationEngine::tf_rate() const noexcept
{
  return tf_rate_;
}

void SimulationEngine::set_odom_rate(double freq_hz)
{
  odom_rate_ = freq_hz;
  for (auto& [name, scheduler] : schedulers_)
  {
    scheduler.set_rate(StreamKind::Odom, 0, freq_hz, scheduling_mode_);
  }
}

double SimulationEngine::odom_rate() const noexcept
{
  return odom_rate_;
}

double SimulationEngine::effective_tf_rate(const std::string& robot_name) const
{
  const auto it = schedulers_.find(robot_name);
  if (it == schedulers_.end())
  {
    return 0.0;
  }
  return it->second.effective_rate(StreamKind::Tf, 0);
}

double SimulationEngine::effective_odom_rate(const std::string& robot_name) const
{
  const auto it = schedulers_.find(robot_name);
  if (it == schedulers_.end())
  {
    return 0.0;
  }
  return it->second.effective_rate(StreamKind::Odom, 0);
}

double SimulationEngine::effective_sensor_rate(const std::string& robot_name, StreamKind kind, std::size_t index) const
{
  const auto it = schedulers_.find(robot_name);
  if (it == schedulers_.end())
  {
    return 0.0;
  }
  return it->second.effective_rate(kind, index);
}

// --- Robot lifecycle ---

void SimulationEngine::register_sensor_streams(const std::string& robot_name, const RobotConfig& config)
{
  RateScheduler& scheduler = schedulers_.at(robot_name);

  scheduler.set_rate(StreamKind::Tf, 0, tf_rate_, scheduling_mode_);
  scheduler.set_rate(StreamKind::Odom, 0, odom_rate_, scheduling_mode_);

  for (std::size_t i = 0; i < config.laser_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::Laser, i, config.laser_sensors[i].frequency, scheduling_mode_);
  }
  for (std::size_t i = 0; i < config.sonar_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::Sonar, i, config.sonar_sensors[i].frequency, scheduling_mode_);
  }
  for (std::size_t i = 0; i < config.rfid_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::Rfid, i, config.rfid_sensors[i].frequency, scheduling_mode_);
  }
  for (std::size_t i = 0; i < config.co2_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::CO2, i, config.co2_sensors[i].frequency, scheduling_mode_);
  }
  for (std::size_t i = 0; i < config.thermal_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::Thermal, i, config.thermal_sensors[i].frequency, scheduling_mode_);
  }
  for (std::size_t i = 0; i < config.sound_sensors.size(); ++i)
  {
    scheduler.set_rate(StreamKind::Sound, i, config.sound_sensors[i].frequency, scheduling_mode_);
  }
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

  // Pre-size sensor vectors to match the config so that event.index is always
  // a valid slot, even before the first sensor fires.
  RobotSensorData data{};
  data.laser_scans.resize(cfg.laser_sensors.size());
  data.sonar_scans.resize(cfg.sonar_sensors.size());
  data.rfid_measurements.resize(cfg.rfid_sensors.size());
  data.co2_measurements.resize(cfg.co2_sensors.size());
  data.thermal_measurements.resize(cfg.thermal_sensors.size());
  data.sound_measurements.resize(cfg.sound_sensors.size());
  sensor_data_[name] = std::move(data);

  schedulers_.emplace(name, RateScheduler{ step_dt_ });
  last_events_[name] = {};

  register_sensor_streams(name, cfg);

  return name;
}

void SimulationEngine::delete_robot(const std::string& name)
{
  world_.remove_robot(name);
  sensor_data_.erase(name);
  schedulers_.erase(name);
  last_events_.erase(name);
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
      new_pose = omni_motion_.update(robot.pose, robot.cmd_vel, dt, robot.config.kinematic_model,
                                     robot.config.center_of_rotation);
    }
    else
    {
      new_pose = ideal_motion_.update(robot.pose, robot.cmd_vel, dt, robot.config.kinematic_model,
                                      robot.config.center_of_rotation);
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

    // --- Re-read the robot state after the pose update. ---
    // If the robot was concurrently removed (or never existed after the
    // snapshot), skip it entirely.
    const world::RobotState* updated = world_.get_robot(robot.name);
    if (updated == nullptr)
    {
      continue;
    }

    // --- Ensure scheduler exists (handles robots added externally). ---
    auto sched_it = schedulers_.find(robot.name);
    if (sched_it == schedulers_.end())
    {
      schedulers_.emplace(robot.name, RateScheduler{ step_dt_ });
      sched_it = schedulers_.find(robot.name);
      last_events_[robot.name] = {};
      register_sensor_streams(robot.name, updated->config);

      // Pre-size sensor data for any newly discovered robot.
      RobotSensorData data{};
      data.laser_scans.resize(updated->config.laser_sensors.size());
      data.sonar_scans.resize(updated->config.sonar_sensors.size());
      data.rfid_measurements.resize(updated->config.rfid_sensors.size());
      data.co2_measurements.resize(updated->config.co2_sensors.size());
      data.thermal_measurements.resize(updated->config.thermal_sensors.size());
      data.sound_measurements.resize(updated->config.sound_sensors.size());
      sensor_data_[robot.name] = std::move(data);
    }

    // Update the collision flag on the existing (pre-sized) data entry.
    sensor_data_[robot.name].collided = robot_collided;

    // --- Rate-scheduled sensor simulation ---
    const std::vector<StreamEvent> events = sched_it->second.tick();
    last_events_[robot.name] = events;

    for (const StreamEvent& event : events)
    {
      switch (event.kind)
      {
        case StreamKind::Laser:
        {
          if (map == nullptr || event.index >= updated->config.laser_sensors.size())
          {
            break;
          }
          const LaserConfig& cfg = updated->config.laser_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].laser_scans[event.index] = laser_sim_.simulate(sw, cfg, *map);
          break;
        }
        case StreamKind::Sonar:
        {
          if (map == nullptr || event.index >= updated->config.sonar_sensors.size())
          {
            break;
          }
          const SonarConfig& cfg = updated->config.sonar_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].sonar_scans[event.index] = sonar_sim_.simulate(sw, cfg, *map);
          break;
        }
        case StreamKind::Rfid:
        {
          if (event.index >= updated->config.rfid_sensors.size())
          {
            break;
          }
          const RfidSensorConfig& cfg = updated->config.rfid_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].rfid_measurements[event.index] = rfid_sim_.simulate(sw, cfg, world_.get_rfid_tags());
          break;
        }
        case StreamKind::CO2:
        {
          if (event.index >= updated->config.co2_sensors.size())
          {
            break;
          }
          const CO2SensorConfig& cfg = updated->config.co2_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].co2_measurements[event.index] = co2_sim_.simulate(sw, cfg, world_.get_co2_sources());
          break;
        }
        case StreamKind::Thermal:
        {
          if (event.index >= updated->config.thermal_sensors.size())
          {
            break;
          }
          const ThermalSensorConfig& cfg = updated->config.thermal_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].thermal_measurements[event.index] =
              thermal_sim_.simulate(sw, cfg, world_.get_thermal_sources());
          break;
        }
        case StreamKind::Sound:
        {
          if (event.index >= updated->config.sound_sensors.size())
          {
            break;
          }
          const SoundSensorConfig& cfg = updated->config.sound_sensors[event.index];
          const Pose2D sw = compute_sensor_world_pose(updated->pose, cfg.pose);
          sensor_data_[robot.name].sound_measurements[event.index] =
              sound_sim_.simulate(sw, cfg, world_.get_sound_sources());
          break;
        }
        case StreamKind::Tf:
        case StreamKind::Odom:
          // TF/Odom publication is the responsibility of the transport layer
          // (StandaloneBackend or ROS2 robot node).  The engine records the
          // event in last_events_ so consumers can check whether to publish.
          break;
      }
    }
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

const std::vector<StreamEvent>& SimulationEngine::last_events(const std::string& robot_name) const
{
  const auto it = last_events_.find(robot_name);
  if (it == last_events_.end())
  {
    return kEmptyEvents;
  }
  return it->second;
}

}  // namespace stdr_simulation
