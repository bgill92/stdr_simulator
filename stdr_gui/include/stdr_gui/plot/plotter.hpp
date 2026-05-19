#pragma once

/** @file Core types and abstract base classes for the plot panel plugin system.
 *
 *  User code sees exactly four things from this header:
 *  `Plotter`, `SimView`, `PlotSink`, `PlotView`, plus the ID typedefs and
 *  timed-reading structs they need to name their data.
 *
 *  Threading contract: `on_sample` and `on_render` are both called on the GUI
 *  thread, serialized per frame — `on_sample` for all enabled plotters first,
 *  then `on_render` for all enabled plotters.  No cross-thread synchronization
 *  is needed between `PlotSink` and `PlotView`. */

#include <stdr_simulation/types.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace stdr::plot
{

// ---------------------------------------------------------------------------
// Owning ID typedefs — string values, not views.
//
// Owning strings prevent the dangling-view bug that arises when a plotter
// stores `RobotId r_ = sim.robot_ids()[0]` as a member: the vector returned
// by robot_ids() is a temporary, so a string_view into it dangles immediately.
// ---------------------------------------------------------------------------

/** Owning name of a robot in the simulation. */
using RobotId = std::string;

/** Owning sensor frame ID identifying one sensor on a robot. */
using SensorId = std::string;

// ---------------------------------------------------------------------------
// Timed reading types
// ---------------------------------------------------------------------------

/** A scalar plot sample: timestamp in seconds and the value. */
struct TimedScalar
{
  double t{ 0.0 };
  double v{ 0.0 };
};

/** A 2-D point plot sample. */
struct Point2
{
  double x{ 0.0 };
  double y{ 0.0 };
};

/** A laser scan paired with the simulation time at which it was produced.
 *  The scan fields mirror `stdr_simulation::LaserScan`; the timestamp allows
 *  SLAM-style algorithms to correlate scans across calls. */
struct TimedLaserScan
{
  double timestamp{ 0.0 };
  stdr_simulation::LaserScan scan;
};

/** A sonar reading paired with the simulation time at which it was produced. */
struct TimedSonarReading
{
  double timestamp{ 0.0 };
  stdr_simulation::SonarScan scan;
};

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------

class SimView;
class PlotSink;
class PlotView;

/** @brief Introspection-only facade passed to `Plotter::on_init`.
 *
 *  Provides read access to the backend's robot and sensor roster at
 *  initialisation time.  Declared here; defined in `sim_introspection.hpp`
 *  so callers that only need the `Plotter` base class do not pull in the full
 *  backend header. */
class SimIntrospection;

// ---------------------------------------------------------------------------
// PlotSink — write side of per-plotter named ring buffers
// ---------------------------------------------------------------------------

namespace detail
{

/** Internal ring for a single named channel, stored as a contiguous
 *  deque-like vector of elements.  New elements are pushed to the back;
 *  reads see a stable span until the next write phase. */
struct ChannelBase
{
  virtual ~ChannelBase() = default;
};

/** Type-erased channel storage. */
template <typename T>
struct TypedChannel : ChannelBase
{
  std::vector<T> data;
  explicit TypedChannel(std::size_t capacity)
  {
    data.reserve(capacity);
  }
};

}  // namespace detail

/** @brief Write-side API for a plotter's plot data.
 *
 *  All writes happen from `on_sample`.  Channels are lazily allocated on the
 *  first write with the given name and a default capacity of 4096 elements;
 *  the capacity is only a pre-allocation hint — the underlying `std::vector`
 *  grows unbounded (matching ImPlot's expectation of a full history).
 *
 *  @warning NOT thread-safe.  Must only be called from `on_sample`, which
 *           runs on the GUI thread. */
class PlotSink
{
public:
  PlotSink() = default;

  // Non-copyable — channels hold unique ownership of their data.
  PlotSink(const PlotSink&) = delete;
  PlotSink& operator=(const PlotSink&) = delete;
  PlotSink(PlotSink&&) = default;
  PlotSink& operator=(PlotSink&&) = default;

  /** Append a scalar sample.  Lazily creates the channel on first use. */
  void scalar(std::string_view name, double timestamp, double value);

  /** Append a 2-D point.  Lazily creates the channel on first use. */
  void point(std::string_view name, double x, double y);

  /** Append multiple (x, y) pairs from parallel spans.
   *
   *  Silently truncates to `min(xs.size(), ys.size())` elements if the spans
   *  differ in length — the caller is responsible for keeping them aligned. */
  void vector(std::string_view name, std::span<const double> xs, std::span<const double> ys);

  /** Access (or create) a typed channel directly for cases where the user
   *  wants full control over the element type.  The channel is allocated on
   *  the first call with the given name; `capacity` is ignored on subsequent
   *  calls. */
  template <typename T>
  std::vector<T>& channel(std::string_view name, std::size_t capacity = 4096)
  {
    const std::string key(name);
    auto it = channels_.find(key);
    if (it == channels_.end())
    {
      auto ch = std::make_unique<detail::TypedChannel<T>>(capacity);
      detail::TypedChannel<T>* raw = ch.get();
      channels_.emplace(key, std::move(ch));
      return raw->data;
    }
    // Channel already exists — caller must ensure T matches.
    return dynamic_cast<detail::TypedChannel<T>&>(*it->second).data;
  }

  /** @internal Called by PlotView to look up a typed channel by name.
   *  Returns nullptr if the channel does not exist or has a different type. */
  template <typename T>
  const detail::TypedChannel<T>* find_channel(std::string_view name) const
  {
    const auto it = channels_.find(std::string(name));
    if (it == channels_.end())
    {
      return nullptr;
    }
    return dynamic_cast<const detail::TypedChannel<T>*>(it->second.get());
  }

private:
  static constexpr std::size_t kDefaultCapacity = 4096;

  std::unordered_map<std::string, std::unique_ptr<detail::ChannelBase>> channels_;
};

// ---------------------------------------------------------------------------
// PlotView — read side, valid during on_render
// ---------------------------------------------------------------------------

/** @brief Read-side API for a plotter's plot data.
 *
 *  All reads happen from `on_render`.  Returned spans point directly into
 *  the underlying `std::vector` buffers owned by `PlotSink` and are valid
 *  until the next `on_sample` call that writes to the same channel — the
 *  framework guarantees no `on_sample` runs concurrently with `on_render`. */
class PlotView
{
public:
  /** @param sink  The `PlotSink` for the same plotter instance. */
  explicit PlotView(const PlotSink& sink) : sink_(sink)
  {
  }

  // Non-copyable — holds a const reference, so move-assignment is also deleted
  // (a const reference member cannot be reseated).
  PlotView(const PlotView&) = delete;
  PlotView& operator=(const PlotView&) = delete;
  PlotView(PlotView&&) = default;

  /** Return all scalar samples written to @p name, oldest first.
   *  Returns an empty span if the channel does not exist. */
  [[nodiscard]] std::span<const TimedScalar> scalar(std::string_view name) const;

  /** Return all 2-D points written to @p name, oldest first.
   *  Returns an empty span if the channel does not exist. */
  [[nodiscard]] std::span<const Point2> points(std::string_view name) const;

  /** Return all values of type @p T written to @p name via `channel<T>()`,
   *  oldest first.  Returns an empty span if the channel does not exist or
   *  has a different type. */
  template <typename T>
  [[nodiscard]] std::span<const T> values(std::string_view name) const
  {
    const detail::TypedChannel<T>* ch = sink_.find_channel<T>(name);
    if (ch == nullptr)
    {
      return {};
    }
    return std::span<const T>{ ch->data.data(), ch->data.size() };
  }

private:
  const PlotSink& sink_;
};

// ---------------------------------------------------------------------------
// SimView — the only simulation interface a Plotter touches
// ---------------------------------------------------------------------------

/** @brief Snapshot-consistent view of the simulation for a single on_sample call.
 *
 *  Constructed fresh for each `on_sample` invocation.  All state-reading
 *  methods delegate to the backend's introspection API and hold the backend
 *  lock only for the duration of the individual call.
 *
 *  ## Laser/sonar history
 *
 *  `new_laser_scans` and `new_sonar_readings` drain the backend's SPSC ring
 *  into an internal buffer and return a span into that buffer.  The span is
 *  valid **only for the duration of this `on_sample` call** — the buffer is
 *  owned by the `SimView` instance, which is destroyed when `on_sample`
 *  returns.  Callers that need the data to outlive the call must copy it.
 *
 *  @warning NOT thread-safe.  Must only be used from `on_sample`,
 *           `on_pause`, or `on_resume` — all of which run on the GUI thread. */
class SimView
{
public:
  // --- Introspection (snapshot-consistent for this on_sample call) ---

  /** Return the number of robots currently in the simulation. */
  [[nodiscard]] virtual std::size_t num_robots() const = 0;

  /** Return owning IDs of all robots currently in the simulation. */
  [[nodiscard]] virtual std::vector<RobotId> robot_ids() const = 0;

  /** Return sensor frame IDs of all laser sensors on @p robot_id. */
  [[nodiscard]] virtual std::vector<SensorId> laser_sensors(const RobotId& robot_id) const = 0;

  /** Return sensor frame IDs of all sonar sensors on @p robot_id. */
  [[nodiscard]] virtual std::vector<SensorId> sonar_sensors(const RobotId& robot_id) const = 0;

  /** Return the current pose of @p robot_id, or a zero-pose if not found. */
  [[nodiscard]] virtual stdr_simulation::Pose2D pose(const RobotId& robot_id) const = 0;

  /** Return the current velocity command of @p robot_id, or zero if not found. */
  [[nodiscard]] virtual stdr_simulation::Twist2D twist(const RobotId& robot_id) const = 0;

  /** Return whether @p robot_id collided on the most recent step.
   *  Returns false if @p robot_id is not found. */
  [[nodiscard]] virtual bool collided(const RobotId& robot_id) const = 0;

  /** Return the robot's footprint polygon vertices in robot-local frame.
   *  Returns an empty vector if the robot is not found or has no footprint. */
  [[nodiscard]] virtual std::vector<stdr_simulation::Point2D> footprint(const RobotId& robot_id) const = 0;

  /** Return the center-of-rotation of @p robot_id in robot body frame.
   *  Returns a zero point if @p robot_id is not found. */
  [[nodiscard]] virtual stdr_simulation::Point2D center_of_rotation(const RobotId& robot_id) const = 0;

  /** Return the elapsed simulation time in seconds. */
  [[nodiscard]] virtual double sim_time() const = 0;

  /** Return the current occupancy grid. */
  [[nodiscard]] virtual const stdr_simulation::OccupancyGrid& map() const = 0;

  // --- Latest sensor readings ---

  /** Return the most recent laser scan from @p sensor_id on @p robot_id.
   *  Returns nullopt if the robot or sensor is not found, or if no scan has
   *  been produced yet. */
  [[nodiscard]] virtual std::optional<stdr_simulation::LaserScan> latest_laser(const RobotId& robot_id,
                                                                               const SensorId& sensor_id) const = 0;

  /** Return the most recent sonar reading from @p sensor_id on @p robot_id.
   *  Returns nullopt if the robot or sensor is not found, or if no reading
   *  has been produced yet. */
  [[nodiscard]] virtual std::optional<stdr_simulation::SonarScan> latest_sonar(const RobotId& robot_id,
                                                                               const SensorId& sensor_id) const = 0;

  // --- History (all readings since this plotter's last on_sample call) ---

  /** All laser scans produced since this plotter's last on_sample call. */
  struct DrainedLaser
  {
    /** Scans in arrival order, oldest first.  Valid only until this
     *  `on_sample` call returns — copy if longer lifetime is needed. */
    std::span<const TimedLaserScan> scans;

    /** Number of scans dropped because the producer lapped this consumer.
     *  Non-zero signals that the plotter should increase `sample_period()`
     *  or reduce processing time per call. */
    std::size_t dropped{ 0 };
  };

  /** All sonar readings produced since this plotter's last on_sample call. */
  struct DrainedSonar
  {
    /** Readings in arrival order, oldest first.  Valid only until this
     *  `on_sample` call returns — copy if longer lifetime is needed. */
    std::span<const TimedSonarReading> scans;
    std::size_t dropped{ 0 };
  };

  /** Drain laser scans for @p robot_id / @p sensor_id since the last call.
   *
   *  The returned span points into an internal buffer owned by this SimView
   *  instance.  It is valid only until this `on_sample` call returns. */
  [[nodiscard]] virtual DrainedLaser new_laser_scans(const RobotId& robot_id, const SensorId& sensor_id) = 0;

  /** Drain sonar readings for @p robot_id / @p sensor_id since the last call.
   *
   *  The returned span points into an internal buffer owned by this SimView
   *  instance.  It is valid only until this `on_sample` call returns. */
  [[nodiscard]] virtual DrainedSonar new_sonar_readings(const RobotId& robot_id, const SensorId& sensor_id) = 0;

  // --- Commands (routed through backend command queue, never block) ---

  /** Enqueue a velocity command for @p robot_id.
   *
   *  The command is dispatched by the sim thread at the top of the next step.
   *  Does not block or take any mutex. */
  virtual void cmd_velocity(const RobotId& robot_id, double v, double w) = 0;

  /** Enqueue a teleport command for @p robot_id (debug use only). */
  virtual void teleport(const RobotId& robot_id, stdr_simulation::Pose2D pose) = 0;

  /** Enqueue a pause command. */
  virtual void pause() = 0;

  /** Enqueue a resume command. */
  virtual void resume() = 0;

  virtual ~SimView() = default;

protected:
  SimView() = default;
  SimView(const SimView&) = default;
  SimView& operator=(const SimView&) = default;
  SimView(SimView&&) = default;
  SimView& operator=(SimView&&) = default;
};

// ---------------------------------------------------------------------------
// Plotter — abstract base for all user-defined plotters
// ---------------------------------------------------------------------------

/** @brief Abstract base class for a user-defined plot plugin.
 *
 *  Subclass, override `on_sample`, `on_render`, and `name()`, then register
 *  with `REGISTER_PLOTTER(MyPlotter)` at file scope.  Everything else —
 *  threading, buffering, command routing — is handled by the framework.
 *
 *  ## Lifetime of name()
 *
 *  `name()` must return a `string_view` whose underlying storage outlives the
 *  registration.  The simplest correct approach: return a string literal.
 *  The underlying bytes are in the read-only data segment and have program
 *  lifetime.  Do NOT return a `string_view` into a local or member `std::string`
 *  — such a view would dangle after the Plotter is destroyed. */
class Plotter
{
public:
  virtual ~Plotter() = default;

  // Non-copyable — plotters are unique algorithm instances.
  Plotter(const Plotter&) = delete;
  Plotter& operator=(const Plotter&) = delete;
  Plotter(Plotter&&) = delete;
  Plotter& operator=(Plotter&&) = delete;

  /** Called once after the backend is ready.  Use to cache robot/sensor IDs
   *  so `on_sample` does not query the roster on every call. */
  virtual void on_init(const SimIntrospection& /*sim*/)
  {
  }

  /** Called once when the plotter is paused (false → true transition).
   *  Plotters that issue commands should send a final stop here so the
   *  simulator does not keep applying the last latched command.
   *  Default: no-op.
   *
   *  The `SimView` passed here shares the slot's live sensor cursors; calling
   *  `new_laser_scans()` or `new_sonar_readings()` inside this hook will drain
   *  events that would otherwise be visible on the next `on_sample` call. */
  virtual void on_pause(SimView& /*sim*/)
  {
  }

  /** Called once when the plotter is unpaused (true → false transition).
   *  Symmetric counterpart to on_pause; plotters can use it to re-publish
   *  any state that was cleared on pause.  Default: no-op.
   *
   *  The `SimView` passed here shares the slot's live sensor cursors; calling
   *  `new_laser_scans()` or `new_sonar_readings()` inside this hook will drain
   *  events that would otherwise be visible on the next `on_sample` call. */
  virtual void on_resume(SimView& /*sim*/)
  {
  }

  /** Called periodically on the GUI thread.  Read sim state, run algorithm,
   *  write plot data via @p out.  May also issue commands via sim.cmd_*(). */
  virtual void on_sample(SimView& sim, PlotSink& out) = 0;

  /** Called every frame on the GUI thread.  Read plot data from @p data and
   *  make ImPlot calls to visualise it. */
  virtual void on_render(const PlotView& data) = 0;

  /** Display name shown in the plot panel menu.
   *
   *  Return a string literal (e.g., `return "My Plotter";`) so the
   *  string_view never dangles. */
  [[nodiscard]] virtual std::string_view name() const = 0;

  /** Optional long-form description shown on hover in the panel menu. */
  [[nodiscard]] virtual std::string_view description() const
  {
    return "";
  }

  /** Minimum interval between `on_sample` calls.
   *
   *  Default (0 ms) means "every frame".  Override to reduce per-frame CPU
   *  cost for plotters that only need metric-per-simulation-tick frequency. */
  [[nodiscard]] virtual std::chrono::milliseconds sample_period() const
  {
    return std::chrono::milliseconds{ 0 };
  }

protected:
  Plotter() = default;
};

}  // namespace stdr::plot
