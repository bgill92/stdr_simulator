#pragma once

#include <stdr_gui/simulator_backend.hpp>
#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace stdr_standalone
{

/** @brief Standalone backend that owns a SimulationEngine and WorldModel.
 *
 *  Runs the simulation on a background thread and provides thread-safe
 *  snapshots to the GUI. No ROS dependencies. */
class StandaloneBackend : public stdr_gui::SimulatorBackend
{
public:
  StandaloneBackend();
  ~StandaloneBackend() override;

  [[nodiscard]] tl::expected<void, std::string> load_map(const std::string& yaml_path) override;
  [[nodiscard]] tl::expected<std::string, std::string> spawn_robot(const std::string& yaml_path,
                                                                   const stdr_simulation::Pose2D& pose) override;
  void delete_robot(const std::string& name) override;
  void start() override;
  void pause() override;
  void reset() override;
  void set_speed(double multiplier) override;
  void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) override;
  [[nodiscard]] std::shared_ptr<const stdr_gui::SimulationSnapshot> get_snapshot() const override;
  [[nodiscard]] std::vector<std::string> poll_messages() override;

private:
  void simulation_loop(std::stop_token stop_token);
  void push_message(std::string msg);

  stdr_simulation::world::WorldModel world_model_;
  stdr_simulation::SimulationEngine engine_;

  mutable std::mutex sim_mutex_;
  std::jthread sim_thread_;
  std::condition_variable_any cv_;
  std::atomic<bool> running_{ false };
  std::atomic<double> speed_{ 1.0 };
  double elapsed_time_{ 0.0 };

  std::mutex msg_mutex_;
  std::vector<std::string> messages_;
};

}  // namespace stdr_standalone
