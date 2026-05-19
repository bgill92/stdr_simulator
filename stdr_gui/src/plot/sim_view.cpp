#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <stdr_simulation/plot_data/command_queue.hpp>
#include <stdr_simulation/types.hpp>

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stdr::plot
{

/** @brief Concrete SimView adapter over a SimulatorBackend.
 *
 *  Created fresh for each `on_sample` invocation.  Delegates introspection
 *  calls to the backend's thread-safe introspection API; delegates command
 *  calls to `push_command()` so the GUI thread never contends on `sim_mutex_`.
 *
 *  ## Laser/sonar ownership
 *
 *  `poll_laser_events()` / `poll_sonar_events()` return owning vectors whose
 *  data was copied while the backend held its sensor ring lock.  There is no
 *  lifetime dependency on the ring's internal buffer after the call returns.
 *  This class accumulates the per-segment owned vectors into a single
 *  per-key buffer so callers see a contiguous span across ring-wrap boundaries.
 *
 *  ## SpscRing segment wrapping
 *
 *  `SpscRing::drain()` returns one contiguous physical segment.  When the
 *  available entries wrap around the ring buffer's physical end, a single call
 *  returns only up to `capacity - start_slot` elements and advances the cursor.
 *  A second call retrieves the remainder.  This class drains in a loop until
 *  no new entries arrive so callers always receive the full set of new scans. */
class BackendSimView : public SimView
{
public:
  /** @param backend       Non-owning reference to the backend.  Must outlive this view.
   *  @param laser_cursors Per-(robot/sensor) cursor map owned by the framework.
   *                       Updated in place by each drain call.
   *  @param sonar_cursors Per-(robot/sensor) cursor map for sonar. */
  BackendSimView(stdr_gui::SimulatorBackend& backend, std::unordered_map<std::string, std::uint64_t>& laser_cursors,
                 std::unordered_map<std::string, std::uint64_t>& sonar_cursors)
    : backend_(backend), laser_cursors_(laser_cursors), sonar_cursors_(sonar_cursors)
  {
  }

  // ---------------------------------------------------------------------------
  // Introspection — delegates to backend introspection API
  // ---------------------------------------------------------------------------

  [[nodiscard]] std::size_t num_robots() const override
  {
    return backend_.num_robots();
  }

  [[nodiscard]] std::vector<RobotId> robot_ids() const override
  {
    return backend_.robot_ids();
  }

  [[nodiscard]] std::vector<SensorId> laser_sensors(const RobotId& robot_id) const override
  {
    return backend_.laser_sensors(robot_id);
  }

  [[nodiscard]] std::vector<SensorId> sonar_sensors(const RobotId& robot_id) const override
  {
    return backend_.sonar_sensors(robot_id);
  }

  [[nodiscard]] stdr_simulation::Pose2D pose(const RobotId& robot_id) const override
  {
    const std::optional<stdr_simulation::Pose2D> p = backend_.pose(robot_id);
    return p.value_or(stdr_simulation::Pose2D{});
  }

  [[nodiscard]] stdr_simulation::Twist2D twist(const RobotId& robot_id) const override
  {
    const std::optional<stdr_simulation::Twist2D> t = backend_.twist(robot_id);
    return t.value_or(stdr_simulation::Twist2D{});
  }

  [[nodiscard]] bool collided(const RobotId& robot_id) const override
  {
    const std::optional<bool> c = backend_.collided(robot_id);
    return c.value_or(false);
  }

  [[nodiscard]] std::vector<stdr_simulation::Point2D> footprint(const RobotId& robot_id) const override
  {
    return backend_.footprint(robot_id);
  }

  [[nodiscard]] stdr_simulation::Point2D center_of_rotation(const RobotId& robot_id) const override
  {
    const std::optional<stdr_simulation::Point2D> cor = backend_.center_of_rotation(robot_id);
    return cor.value_or(stdr_simulation::Point2D{});
  }

  [[nodiscard]] double sim_time() const override
  {
    return backend_.sim_time();
  }

  [[nodiscard]] const stdr_simulation::OccupancyGrid& map() const override
  {
    // Fetch and cache the snapshot for this SimView's lifetime.  The snapshot
    // shared_ptr keeps the map allocation alive even if the backend swaps in a
    // newer snapshot between introspection calls within the same on_sample.
    if (!snapshot_)
    {
      snapshot_ = backend_.get_snapshot();
    }
    if (snapshot_)
    {
      return snapshot_->map;
    }
    static const stdr_simulation::OccupancyGrid kEmptyGrid{};
    return kEmptyGrid;
  }

  // ---------------------------------------------------------------------------
  // Latest sensor readings
  // ---------------------------------------------------------------------------

  [[nodiscard]] std::optional<stdr_simulation::LaserScan> latest_laser(const RobotId& robot_id,
                                                                       const SensorId& sensor_id) const override
  {
    return backend_.latest_laser(robot_id, sensor_id);
  }

  [[nodiscard]] std::optional<stdr_simulation::SonarScan> latest_sonar(const RobotId& robot_id,
                                                                       const SensorId& sensor_id) const override
  {
    return backend_.latest_sonar(robot_id, sensor_id);
  }

  // ---------------------------------------------------------------------------
  // History drain — copies ring data into local buffers for safe span lifetime
  // ---------------------------------------------------------------------------

  [[nodiscard]] DrainedLaser new_laser_scans(const RobotId& robot_id, const SensorId& sensor_id) override
  {
    const std::string key = robot_id + "/" + sensor_id;
    std::uint64_t& cursor = laser_cursors_[key];
    std::vector<TimedLaserScan>& buf = laser_drain_bufs_[key];
    buf.clear();

    // poll_laser_events() returns an owning vector copied under the backend's
    // sensor ring lock — no span lifetime dependency on the ring buffer.
    // SpscRing::drain() returns one contiguous physical segment per call; loop
    // until empty to collect entries that straddle the ring's physical end.
    std::size_t total_dropped = 0;
    for (;;)
    {
      stdr_gui::DrainedLaserResult segment = backend_.poll_laser_events(cursor, robot_id, sensor_id);
      total_dropped += segment.dropped;
      if (segment.scans.empty())
      {
        break;
      }
      buf.insert(buf.end(), std::make_move_iterator(segment.scans.begin()),
                 std::make_move_iterator(segment.scans.end()));
    }

    return DrainedLaser{ std::span<const TimedLaserScan>{ buf.data(), buf.size() }, total_dropped };
  }

  [[nodiscard]] DrainedSonar new_sonar_readings(const RobotId& robot_id, const SensorId& sensor_id) override
  {
    const std::string key = robot_id + "/" + sensor_id;
    std::uint64_t& cursor = sonar_cursors_[key];
    std::vector<TimedSonarReading>& buf = sonar_drain_bufs_[key];
    buf.clear();

    // Same ownership model as new_laser_scans — copy under lock, move into buf.
    std::size_t total_dropped = 0;
    for (;;)
    {
      stdr_gui::DrainedSonarResult segment = backend_.poll_sonar_events(cursor, robot_id, sensor_id);
      total_dropped += segment.dropped;
      if (segment.scans.empty())
      {
        break;
      }
      buf.insert(buf.end(), std::make_move_iterator(segment.scans.begin()),
                 std::make_move_iterator(segment.scans.end()));
    }

    return DrainedSonar{ std::span<const TimedSonarReading>{ buf.data(), buf.size() }, total_dropped };
  }

  // ---------------------------------------------------------------------------
  // Commands — enqueue without blocking
  // ---------------------------------------------------------------------------

  void cmd_velocity(const RobotId& robot_id, double v, double w) override
  {
    stdr::plot_data::Command cmd;
    cmd.kind = stdr::plot_data::Command::Kind::Velocity;
    cmd.robot = robot_id;
    cmd.twist = stdr_simulation::Twist2D{ .linear_x = v, .linear_y = 0.0, .angular_z = w };
    backend_.push_command(std::move(cmd));
  }

  void teleport(const RobotId& robot_id, stdr_simulation::Pose2D target_pose) override
  {
    stdr::plot_data::Command cmd;
    cmd.kind = stdr::plot_data::Command::Kind::Teleport;
    cmd.robot = robot_id;
    cmd.pose = target_pose;
    backend_.push_command(std::move(cmd));
  }

  void pause() override
  {
    stdr::plot_data::Command cmd;
    cmd.kind = stdr::plot_data::Command::Kind::Pause;
    backend_.push_command(std::move(cmd));
  }

  void resume() override
  {
    stdr::plot_data::Command cmd;
    cmd.kind = stdr::plot_data::Command::Kind::Resume;
    backend_.push_command(std::move(cmd));
  }

private:
  stdr_gui::SimulatorBackend& backend_;

  // Per-plotter cursors for the SPSC rings — one entry per "robot/sensor" key.
  // Updated in place across on_sample calls so each plotter sees only new data.
  std::unordered_map<std::string, std::uint64_t>& laser_cursors_;
  std::unordered_map<std::string, std::uint64_t>& sonar_cursors_;

  // Drain buffers — lifetime matches this SimView (one on_sample call).
  // Spans returned from new_laser_scans/new_sonar_readings point into these.
  std::unordered_map<std::string, std::vector<TimedLaserScan>> laser_drain_bufs_;
  std::unordered_map<std::string, std::vector<TimedSonarReading>> sonar_drain_bufs_;

  // Snapshot cached on first map() call so the map reference stays valid for
  // the entire on_sample call even if the backend swaps in a newer snapshot.
  mutable std::shared_ptr<const stdr_gui::SimulationSnapshot> snapshot_;
};

/** Factory: create a `SimView` backed by @p backend.
 *
 *  The returned view borrows @p backend, @p laser_cursors, and
 *  @p sonar_cursors by reference.  It must not outlive any of them. */
std::unique_ptr<SimView> make_backend_sim_view(stdr_gui::SimulatorBackend& backend,
                                               std::unordered_map<std::string, std::uint64_t>& laser_cursors,
                                               std::unordered_map<std::string, std::uint64_t>& sonar_cursors)
{
  return std::make_unique<BackendSimView>(backend, laser_cursors, sonar_cursors);
}

}  // namespace stdr::plot
