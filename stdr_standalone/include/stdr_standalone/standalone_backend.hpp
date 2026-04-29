#pragma once

#include <stdr_gui/simulator_backend.hpp>
#include <stdr_simulation/plot_data/command_queue.hpp>
#include <stdr_simulation/plot_data/spsc_ring.hpp>
#include <stdr_simulation/simulation_engine.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
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
  void set_step_dt(double seconds) override;
  [[nodiscard]] double get_step_dt() const override;
  void set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose) override;
  void set_cmd_vel(const std::string& robot_name, const stdr_simulation::Twist2D& cmd) override;
  [[nodiscard]] std::shared_ptr<const stdr_gui::SimulationSnapshot> get_snapshot() const override;
  [[nodiscard]] std::vector<std::string> poll_messages() override;

  [[nodiscard]] std::size_t num_robots() const override;
  [[nodiscard]] std::vector<std::string> robot_ids() const override;
  [[nodiscard]] std::vector<std::string> laser_sensors(const std::string& robot_id) const override;
  [[nodiscard]] std::vector<std::string> sonar_sensors(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::Pose2D> pose(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::Twist2D> twist(const std::string& robot_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::LaserScan> latest_laser(const std::string& robot_id,
                                                                       const std::string& sensor_id) const override;
  [[nodiscard]] std::optional<stdr_simulation::SonarScan> latest_sonar(const std::string& robot_id,
                                                                       const std::string& sensor_id) const override;
  [[nodiscard]] std::optional<bool> collided(const std::string& robot_id) const override;
  [[nodiscard]] double sim_time() const override;
  [[nodiscard]] std::vector<stdr_simulation::Point2D> footprint(const std::string& robot_id) const override;

  [[nodiscard]] stdr_gui::DrainedLaserResult poll_laser_events(std::uint64_t& cursor, const std::string& robot_id,
                                                               const std::string& sensor_id) override;
  [[nodiscard]] stdr_gui::DrainedSonarResult poll_sonar_events(std::uint64_t& cursor, const std::string& robot_id,
                                                               const std::string& sensor_id) override;

  void push_command(stdr::plot_data::Command cmd) override;

private:
  void simulation_loop(std::stop_token stop_token);
  void push_message(std::string msg);

  /** Push laser/sonar scan results from the most recent engine_.step() into
   *  the SPSC sensor-event rings.  Must be called from the sim thread while
   *  sim_mutex_ is held, immediately after engine_.step() completes. */
  void push_sensor_events_locked(double timestamp);

  stdr_simulation::world::WorldModel world_model_;
  stdr_simulation::SimulationEngine engine_;

  mutable std::mutex sim_mutex_;
  std::jthread sim_thread_;
  std::condition_variable_any cv_;
  std::atomic<bool> running_{ false };
  std::atomic<double> speed_{ 1.0 };
  std::atomic<double> step_dt_{ stdr_gui::kDefaultStepDt };
  double elapsed_time_{ 0.0 };

  std::string map_name_;

  std::mutex msg_mutex_;
  std::vector<std::string> messages_;

  // Receives commands from the GUI/plotter threads and is drained by the sim
  // thread at the top of each step — decoupled from sim_mutex_ so callers
  // never contend with the sim thread's physics lock.
  stdr::plot_data::CommandQueue cmd_queue_;

  // Per-sensor SPSC rings for laser and sonar scan history.
  // Keyed as "robot_name/sensor_frame_id" — the "/" separator cannot appear
  // in either component because sensor frame IDs are YAML identifiers.
  // The sim thread pushes after each engine_.step(); the GUI thread drains via
  // poll_laser_events() / poll_sonar_events().
  //
  // SpscRing is non-movable (contains atomics), so we store it behind a
  // unique_ptr so the map can rehash without moving the ring objects.
  // Rings are created on the first push for a given (robot, sensor) pair using
  // kDefaultSensorLogCapacity (256 entries).
  //
  // Map insertions happen only from the sim thread (under sim_mutex_) so no
  // concurrent structural modification occurs.  Reads (drain calls) from the
  // GUI thread find an already-inserted ring by key — no lock is needed for
  // reads after insertion because unordered_map iterators are stable after
  // insert() when no rehash occurs (C++11 §23.5.4/14).  To be safe against
  // rehash, we protect the entire map with sensor_ring_mutex_.
  mutable std::mutex sensor_ring_mutex_;
  std::unordered_map<std::string, std::unique_ptr<stdr::plot_data::SpscRing<stdr::plot::TimedLaserScan>>> laser_rings_;
  std::unordered_map<std::string, std::unique_ptr<stdr::plot_data::SpscRing<stdr::plot::TimedSonarReading>>> sonar_rings_;
};

}  // namespace stdr_standalone
