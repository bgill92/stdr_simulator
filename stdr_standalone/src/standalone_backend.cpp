#include "stdr_standalone/standalone_backend.hpp"

#include "stdr_standalone/map_loader.hpp"

#include <stdr_simulation/config_loader.hpp>

#include <tl_expected/expected.hpp>

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

void StandaloneBackend::simulation_loop(std::stop_token stop_token)
{
  // 10 Hz simulation step, matching the ROS-based mode.
  constexpr double kStepDt = 0.1;

  while (!stop_token.stop_requested())
  {
    {
      std::unique_lock<std::mutex> lock(sim_mutex_);
      // Returns false when stop is requested while waiting, in which case we exit.
      if (!cv_.wait(lock, stop_token, [this] { return running_.load(std::memory_order_relaxed); }))
      {
        break;
      }
      // Still holding sim_mutex_ — step the simulation while state is locked.
      const double effective_dt = kStepDt * speed_.load(std::memory_order_relaxed);
      engine_.step(effective_dt);
      elapsed_time_ += effective_dt;
    }

    // Sleep for the real-time step interval outside the lock.
    std::this_thread::sleep_for(std::chrono::duration<double>(kStepDt));
  }
}

void StandaloneBackend::push_message(std::string msg)
{
  const std::lock_guard<std::mutex> lock(msg_mutex_);
  messages_.push_back(std::move(msg));
}

}  // namespace stdr_standalone
