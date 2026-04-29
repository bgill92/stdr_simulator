#include "stdr_standalone/standalone_backend.hpp"

#include "stdr_standalone/map_loader.hpp"

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_simulation/config_loader.hpp>
#include <stdr_simulation/plot_data/spsc_ring.hpp>

#include <tl_expected/expected.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

namespace stdr_standalone
{

StandaloneBackend::StandaloneBackend() : engine_(world_model_)
{
  sim_thread_ = std::jthread([this](std::stop_token stop_token) { simulation_loop(std::move(stop_token)); });
}

StandaloneBackend::~StandaloneBackend()
{
  sim_thread_.request_stop();
  cv_.notify_all();
  // jthread destructor joins automatically.
}

tl::expected<void, std::string> StandaloneBackend::load_map(const std::string& yaml_path)
{
  const tl::expected<stdr_simulation::OccupancyGrid, std::string> result = stdr_standalone::load_map(yaml_path);
  if (!result)
  {
    return tl::make_unexpected(result.error());
  }
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    world_model_.set_map(result.value());
    map_name_ = std::filesystem::path(yaml_path).filename().string();
  }
  push_message("Loaded map: " + yaml_path);
  return {};
}

tl::expected<std::string, std::string> StandaloneBackend::spawn_robot(const std::string& yaml_path,
                                                                      const stdr_simulation::Pose2D& pose)
{
  const char* env_resources = std::getenv("STDR_RESOURCES_DIR");
  const std::string base_dir =
      (env_resources != nullptr) ? std::string(env_resources) : std::filesystem::path(yaml_path).parent_path().string();
  const tl::expected<stdr_simulation::RobotConfig, std::string> config_result =
      stdr_simulation::load_robot_config(yaml_path, base_dir);
  if (!config_result)
  {
    return tl::make_unexpected(config_result.error());
  }

  std::string robot_name;
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    robot_name = engine_.spawn_robot(config_result.value(), pose);
  }
  push_message("Spawned robot: " + robot_name);
  return robot_name;
}

void StandaloneBackend::delete_robot(const std::string& name)
{
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    engine_.delete_robot(name);
  }
  push_message("Deleted robot: " + name);
}

void StandaloneBackend::start()
{
  running_ = true;
  cv_.notify_all();
  push_message("Simulation started");
}

void StandaloneBackend::pause()
{
  running_ = false;
  push_message("Simulation paused");
}

void StandaloneBackend::reset()
{
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);
    running_.store(false, std::memory_order_relaxed);
    const std::vector<stdr_simulation::world::RobotState> robots = world_model_.get_all_robots();
    for (const stdr_simulation::world::RobotState& robot : robots)
    {
      engine_.delete_robot(robot.name);
    }
    elapsed_time_ = 0.0;
  }
  push_message("Simulation reset");
}

void StandaloneBackend::set_speed(double multiplier)
{
  speed_.store(multiplier, std::memory_order_relaxed);
}

void StandaloneBackend::set_step_dt(double seconds)
{
  const double clamped = std::clamp(seconds, stdr_gui::kMinStepDt, stdr_gui::kMaxStepDt);
  step_dt_.store(clamped, std::memory_order_relaxed);
}

double StandaloneBackend::get_step_dt() const
{
  return step_dt_.load(std::memory_order_relaxed);
}

void StandaloneBackend::set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose)
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  world_model_.set_robot_pose(name, pose);
}

void StandaloneBackend::set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd)
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  engine_.set_cmd_vel(robot_name, cmd);
}

std::shared_ptr<const stdr_gui::SimulationSnapshot> StandaloneBackend::get_snapshot() const
{
  stdr_gui::SimulationSnapshot snapshot;
  {
    const std::lock_guard<std::mutex> lock(sim_mutex_);

    // Copy map if one has been loaded.
    const stdr_simulation::OccupancyGrid* map_ptr = world_model_.get_map();
    if (map_ptr != nullptr)
    {
      snapshot.map = *map_ptr;
    }

    snapshot.robots = world_model_.get_all_robots();

    for (const stdr_simulation::world::RobotState& robot : snapshot.robots)
    {
      const stdr_simulation::RobotSensorData* data = engine_.get_sensor_data(robot.name);
      if (data != nullptr)
      {
        snapshot.sensor_data[robot.name] = *data;
      }
    }

    snapshot.map_name = map_name_;
    snapshot.rfid_tags = world_model_.get_rfid_tags();
    snapshot.co2_sources = world_model_.get_co2_sources();
    snapshot.thermal_sources = world_model_.get_thermal_sources();
    snapshot.sound_sources = world_model_.get_sound_sources();
    snapshot.elapsed_time = elapsed_time_;
    snapshot.running = running_.load();
  }
  return std::make_shared<stdr_gui::SimulationSnapshot>(std::move(snapshot));
}

std::vector<std::string> StandaloneBackend::poll_messages()
{
  std::vector<std::string> out;
  const std::lock_guard<std::mutex> lock(msg_mutex_);
  out.swap(messages_);
  return out;
}

void StandaloneBackend::push_command(stdr::plot_data::Command cmd)
{
  cmd_queue_.push(std::move(cmd));
}

void StandaloneBackend::simulation_loop(std::stop_token stop_token)
{
  while (!stop_token.stop_requested())
  {
    // Drain queued commands before taking sim_mutex_ for the physics step.
    // The swap inside drain() holds cmd_queue_'s internal lock only briefly,
    // so the GUI thread is never blocked waiting for sim_mutex_.
    //
    // Last-write-wins: if two Velocity commands for the same robot arrived
    // in the same drain batch, set_cmd_vel() is called twice and the second
    // overwrites the first — the same semantics the keyboard teleop path uses.
    const std::vector<stdr::plot_data::Command> commands = cmd_queue_.drain();

    double step_dt = 0.0;
    // Warning messages collected during the locked dispatch loop are pushed
    // after sim_mutex_ is released to preserve the lock-ordering invariant:
    // sim_mutex_ must always be released before acquiring msg_mutex_ (as
    // load_map, spawn_robot, and delete_robot all do).
    std::vector<std::string> deferred_messages;
    {
      std::unique_lock<std::mutex> lock(sim_mutex_);
      // Returns false when stop is requested while waiting, in which case we exit.
      if (!cv_.wait(lock, stop_token, [this] { return running_.load(std::memory_order_relaxed); }))
      {
        break;
      }

      // Dispatch drained commands under sim_mutex_ so they are applied before
      // this tick's physics step runs.
      for (const stdr::plot_data::Command& cmd : commands)
      {
        switch (cmd.kind)
        {
          case stdr::plot_data::Command::Kind::Velocity:
          {
            // Guard against stale robot names from a race with delete_robot.
            // set_cmd_vel is a no-op for unknown names; log so users can debug.
            const stdr_simulation::world::RobotState* robot = world_model_.get_robot(cmd.robot);
            if (robot == nullptr)
            {
              deferred_messages.push_back("push_command(Velocity): unknown robot '" + cmd.robot + "' — command dropped");
              break;
            }
            engine_.set_cmd_vel(cmd.robot, cmd.twist);
            break;
          }
          case stdr::plot_data::Command::Kind::Teleport:
          {
            const stdr_simulation::world::RobotState* robot = world_model_.get_robot(cmd.robot);
            if (robot == nullptr)
            {
              deferred_messages.push_back("push_command(Teleport): unknown robot '" + cmd.robot + "' — command dropped");
              break;
            }
            world_model_.set_robot_pose(cmd.robot, cmd.pose);
            break;
          }
          case stdr::plot_data::Command::Kind::Pause:
            // FIXME-CLAUDE: NOT IMPLEMENTED - Pause via command queue.
            // The existing pause() method is called from the GUI thread and sets
            // running_=false, which prevents the sim loop from proceeding.
            // Wiring that from inside the sim loop requires an additional
            // mechanism to avoid a deadlock on cv_.wait().
            deferred_messages.push_back("push_command(Pause): not yet implemented — command ignored");
            break;
          case stdr::plot_data::Command::Kind::Resume:
            // FIXME-CLAUDE: NOT IMPLEMENTED - Resume via command queue.
            // See Pause case above.
            deferred_messages.push_back("push_command(Resume): not yet implemented — command ignored");
            break;
        }
      }

      // Load step_dt once per iteration for a consistent tick interval and physics dt.
      step_dt = step_dt_.load(std::memory_order_relaxed);
      // Still holding sim_mutex_ — step the simulation while state is locked.
      const double effective_dt = step_dt * speed_.load(std::memory_order_relaxed);
      engine_.step(effective_dt);
      elapsed_time_ += effective_dt;

      // Push new sensor readings into SPSC rings so plotters can drain them.
      // We do this under sim_mutex_ (already held) so the push happens
      // atomically with the step — the snapshot and rings are always consistent.
      push_sensor_events_locked(elapsed_time_);
    }

    // Push any messages collected during dispatch now that sim_mutex_ is
    // released, so we never hold sim_mutex_ while acquiring msg_mutex_.
    for (std::string& msg : deferred_messages)
    {
      push_message(std::move(msg));
    }

    // Sleep for the real-time step interval outside the lock.
    std::this_thread::sleep_for(std::chrono::duration<double>(step_dt));
  }
}

void StandaloneBackend::push_message(std::string msg)
{
  const std::lock_guard<std::mutex> lock(msg_mutex_);
  messages_.push_back(std::move(msg));
}

// --- Sensor event log ---

void StandaloneBackend::push_sensor_events_locked(double timestamp)
{
  // Called from simulation_loop() while sim_mutex_ is held.
  // Pushes the laser/sonar results from the most recent engine_.step() into
  // SPSC rings so plotters can drain them independently.
  //
  // Lock ordering: sim_mutex_ (caller) → sensor_ring_mutex_ (here).
  // poll_laser_events() / poll_sonar_events() acquire only sensor_ring_mutex_
  // and never sim_mutex_, so this ordering is deadlock-free.
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  const std::vector<stdr_simulation::world::RobotState> robots = world_model_.get_all_robots();
  for (const stdr_simulation::world::RobotState& robot : robots)
  {
    const stdr_simulation::RobotSensorData* data = engine_.get_sensor_data(robot.name);
    if (data == nullptr)
    {
      continue;
    }

    // Laser sensors — correlate by index with robot.config.laser_sensors.
    for (std::size_t i = 0; i < robot.config.laser_sensors.size() && i < data->laser_scans.size(); ++i)
    {
      const std::string key = robot.name + "/" + robot.config.laser_sensors[i].frame_id;
      auto ring_it = laser_rings_.find(key);
      if (ring_it == laser_rings_.end())
      {
        ring_it = laser_rings_
                      .emplace(key, std::make_unique<stdr::plot_data::SpscRing<stdr::plot::TimedLaserScan>>(
                                        stdr::plot_data::kDefaultSensorLogCapacity))
                      .first;
      }
      ring_it->second->push(stdr::plot::TimedLaserScan{ timestamp, data->laser_scans[i] });
    }

    // Sonar sensors — same pattern.
    for (std::size_t i = 0; i < robot.config.sonar_sensors.size() && i < data->sonar_scans.size(); ++i)
    {
      const std::string key = robot.name + "/" + robot.config.sonar_sensors[i].frame_id;
      auto ring_it = sonar_rings_.find(key);
      if (ring_it == sonar_rings_.end())
      {
        ring_it = sonar_rings_
                      .emplace(key, std::make_unique<stdr::plot_data::SpscRing<stdr::plot::TimedSonarReading>>(
                                        stdr::plot_data::kDefaultSensorLogCapacity))
                      .first;
      }
      ring_it->second->push(stdr::plot::TimedSonarReading{ timestamp, data->sonar_scans[i] });
    }
  }
}

stdr_gui::DrainedLaserResult StandaloneBackend::poll_laser_events(std::uint64_t& cursor, const std::string& robot_id,
                                                                  const std::string& sensor_id)
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  auto it = laser_rings_.find(key);
  if (it == laser_rings_.end())
  {
    return {};
  }
  const stdr::plot_data::DrainResult<stdr::plot::TimedLaserScan> result = it->second->drain(cursor);
  // Copy the span into an owning vector while still holding ring_lock.  The
  // sim thread can call push() as soon as the lock is released, so any access
  // to result.entries after this scope would be a data race.
  return stdr_gui::DrainedLaserResult{
    std::vector<stdr::plot::TimedLaserScan>(result.entries.begin(), result.entries.end()), result.dropped
  };
}

stdr_gui::DrainedSonarResult StandaloneBackend::poll_sonar_events(std::uint64_t& cursor, const std::string& robot_id,
                                                                  const std::string& sensor_id)
{
  const std::string key = robot_id + "/" + sensor_id;
  const std::lock_guard<std::mutex> ring_lock(sensor_ring_mutex_);
  auto it = sonar_rings_.find(key);
  if (it == sonar_rings_.end())
  {
    return {};
  }
  const stdr::plot_data::DrainResult<stdr::plot::TimedSonarReading> result = it->second->drain(cursor);
  // Copy under lock — same rationale as poll_laser_events above.
  return stdr_gui::DrainedSonarResult{
    std::vector<stdr::plot::TimedSonarReading>(result.entries.begin(), result.entries.end()), result.dropped
  };
}

// --- Introspection methods ---
//
// Each method takes a single snapshot so the values it returns are
// internally consistent. The lock is held only long enough to copy the
// relevant state — the same duration as get_snapshot() itself.

std::size_t StandaloneBackend::num_robots() const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  return world_model_.robot_count();
}

std::vector<std::string> StandaloneBackend::robot_ids() const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  return world_model_.robot_names();
}

std::vector<std::string> StandaloneBackend::laser_sensors(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return {};
  }
  std::vector<std::string> ids;
  ids.reserve(robot->config.laser_sensors.size());
  for (const stdr_simulation::LaserConfig& cfg : robot->config.laser_sensors)
  {
    ids.push_back(cfg.frame_id);
  }
  return ids;
}

std::vector<std::string> StandaloneBackend::sonar_sensors(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return {};
  }
  std::vector<std::string> ids;
  ids.reserve(robot->config.sonar_sensors.size());
  for (const stdr_simulation::SonarConfig& cfg : robot->config.sonar_sensors)
  {
    ids.push_back(cfg.frame_id);
  }
  return ids;
}

std::optional<stdr_simulation::Pose2D> StandaloneBackend::pose(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return std::nullopt;
  }
  return robot->pose;
}

std::optional<stdr_simulation::Twist2D> StandaloneBackend::twist(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return std::nullopt;
  }
  return robot->cmd_vel;
}

std::optional<stdr_simulation::LaserScan> StandaloneBackend::latest_laser(const std::string& robot_id,
                                                                          const std::string& sensor_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return std::nullopt;
  }
  const stdr_simulation::RobotSensorData* data = engine_.get_sensor_data(robot_id);
  if (data == nullptr)
  {
    return std::nullopt;
  }
  // Match sensor by frame_id, preserving order parity with laser_sensors().
  for (std::size_t i = 0; i < robot->config.laser_sensors.size(); ++i)
  {
    if (robot->config.laser_sensors[i].frame_id == sensor_id)
    {
      if (i < data->laser_scans.size())
      {
        return data->laser_scans[i];
      }
      // Scan slot exists in config but hasn't been produced yet (no map).
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<stdr_simulation::SonarScan> StandaloneBackend::latest_sonar(const std::string& robot_id,
                                                                          const std::string& sensor_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return std::nullopt;
  }
  const stdr_simulation::RobotSensorData* data = engine_.get_sensor_data(robot_id);
  if (data == nullptr)
  {
    return std::nullopt;
  }
  // Match sensor by frame_id, preserving order parity with sonar_sensors().
  for (std::size_t i = 0; i < robot->config.sonar_sensors.size(); ++i)
  {
    if (robot->config.sonar_sensors[i].frame_id == sensor_id)
    {
      if (i < data->sonar_scans.size())
      {
        return data->sonar_scans[i];
      }
      // Scan slot exists in config but hasn't been produced yet (no map).
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<bool> StandaloneBackend::collided(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return std::nullopt;
  }
  // Defensive: sensor_data_ is always populated after spawn_robot succeeds,
  // so this null-check should never fire in practice.
  const stdr_simulation::RobotSensorData* data = engine_.get_sensor_data(robot_id);
  if (data == nullptr)
  {
    return std::nullopt;
  }
  return data->collided;
}

double StandaloneBackend::sim_time() const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  return elapsed_time_;
}

std::vector<stdr_simulation::Point2D> StandaloneBackend::footprint(const std::string& robot_id) const
{
  const std::lock_guard<std::mutex> lock(sim_mutex_);
  const stdr_simulation::world::RobotState* robot = world_model_.get_robot(robot_id);
  if (robot == nullptr)
  {
    return {};
  }
  return robot->config.footprint.points;
}

}  // namespace stdr_standalone
