#pragma once

/**
 * @file rate_scheduler.hpp
 * @brief Maps target stream frequencies to integer tick multiples of the sim step.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <tl_expected/expected.hpp>
#include <unordered_map>
#include <vector>

namespace stdr_simulation
{

/** Default sim step duration in seconds — matches the nav-stack convention. */
inline constexpr double kDefaultStepDt = 0.1;

/** Lower bound for sim step dt in seconds. */
inline constexpr double kMinStepDt = 0.001;

/** Upper bound for sim step dt in seconds. */
inline constexpr double kMaxStepDt = 1.0;

/** Default TF broadcast rate in Hz — nav-stack convention. */
inline constexpr double kDefaultTfRateHz = 50.0;

/** Default odometry publish rate in Hz — typical mobile-base driver value. */
inline constexpr double kDefaultOdomRateHz = 10.0;

/** Lower bound for per-stream rates; 0 means "every sim tick". */
inline constexpr double kMinRateHz = 0.0;

/** Upper bound for per-stream rates. */
inline constexpr double kMaxRateHz = 1000.0;

/** Stream identifiers for rate scheduling. */
enum class StreamKind
{
  Tf,
  Odom,
  Laser,
  Sonar,
  Rfid,
  CO2,
  Thermal,
  Sound,
};

/** Scheduling strategy for converting a target freq into per-tick fire decisions. */
enum class SchedulingMode
{
  /** Round 1/(freq*dt) to nearest integer ticks; deterministic but quantized. */
  SnapToMultiple,
  /** Accumulate dt per tick; fire when accum >= 1/freq; subtract period.
   *  Average rate matches target; carries sub-tick jitter. */
  Accumulator,
};

/**
 * @brief Return a canonical lowercase string for a SchedulingMode.
 *
 * @param mode The scheduling mode to convert.
 * @return `"snap_to_multiple"` or `"accumulator"`.
 */
[[nodiscard]] std::string_view to_string(SchedulingMode mode) noexcept;

/**
 * @brief Parse a SchedulingMode from a string.
 *
 * Accepted values: `"snap_to_multiple"`, `"accumulator"` (case-sensitive).
 *
 * @param str Input string to parse.
 * @return The matching SchedulingMode, or an error message describing the
 *         accepted values and the unrecognised input.
 */
[[nodiscard]] tl::expected<SchedulingMode, std::string> scheduling_mode_from_string(std::string_view str);

/** A stream that should fire this tick. */
struct StreamEvent
{
  StreamKind kind;
  /// Sensor instance index; 0 for Tf/Odom (single stream).
  std::size_t index;
};

/**
 * @brief Maps target rates to integer tick multiples of the sim step.
 *
 * For each registered stream the scheduler caches
 * `period_ticks = max(1, round(1.0 / (freq * dt)))`.  On each `tick()` it
 * returns the streams whose `tick_count % period_ticks == 0`.
 *
 * `freq <= 0` is treated as "every tick" (period_ticks = 1), preserving
 * backward-compatible behaviour for sensors that omit a frequency field.
 *
 * Streams are returned from `tick()` in a stable order: ascending StreamKind
 * enum value, then ascending index.  This makes test assertions deterministic.
 *
 * @warning Not thread-safe.  The simulation engine is expected to drive this
 *          scheduler on a single thread.
 */
class RateScheduler
{
public:
  /**
   * @brief Construct with initial sim step duration (seconds).
   *
   * @param step_dt_seconds Simulation timestep in seconds.  Must be > 0.
   * @throws std::invalid_argument if step_dt_seconds <= 0.
   */
  explicit RateScheduler(double step_dt_seconds);

  /**
   * @brief Update sim step dt.
   *
   * Recomputes cached period_ticks for all registered entries so that the
   * effective rates track the new timestep immediately.
   *
   * @param step_dt_seconds New timestep in seconds.  Must be > 0.
   * @throws std::invalid_argument if step_dt_seconds <= 0.
   */
  void set_step_dt(double step_dt_seconds);

  /** @return Current sim step duration in seconds. */
  [[nodiscard]] double step_dt() const noexcept;

  /**
   * @brief Register or update a stream's target frequency using SnapToMultiple mode.
   *
   * If the (kind, index) pair already exists its period is recomputed from the
   * new frequency; the tick counter is preserved so it does not jump.
   *
   * @param kind    Stream type.
   * @param index   Sensor instance index (0 for single-instance streams).
   * @param freq_hz Target frequency in Hz.  <= 0 means "fire every tick".
   */
  void set_rate(StreamKind kind, std::size_t index, double freq_hz);

  /**
   * @brief Register or update a stream's target frequency with an explicit scheduling mode.
   *
   * Delegates to the 3-arg overload's logic with the specified mode.  Existing
   * tick_count and accum_seconds are preserved across rate updates so in-flight
   * cadence is not disrupted.
   *
   * @param kind    Stream type.
   * @param index   Sensor instance index (0 for single-instance streams).
   * @param freq_hz Target frequency in Hz.  <= 0 means "fire every tick".
   * @param mode    Scheduling strategy to apply.
   */
  void set_rate(StreamKind kind, std::size_t index, double freq_hz, SchedulingMode mode);

  /**
   * @brief Return the scheduling mode for a registered stream.
   *
   * @return The mode the stream was registered with, or SchedulingMode::SnapToMultiple
   *         if the stream is not registered (consistent with period_ticks() returning 0).
   */
  [[nodiscard]] SchedulingMode mode(StreamKind kind, std::size_t index) const;

  /**
   * @brief Remove all registered streams.
   *
   * Useful when re-registering after a configuration change, since it avoids
   * stale entries for sensors that no longer exist.
   */
  void clear();

  /**
   * @brief Advance one simulation tick.
   *
   * Increments each stream's tick counter and returns the subset that fired
   * this tick.  The firing condition depends on the stream's SchedulingMode:
   * - SnapToMultiple: fires when `tick_count % period_ticks == 0`.
   * - Accumulator: fires when the accumulated dt crosses `1 / freq_hz`.
   *
   * The returned vector is ordered by StreamKind (ascending enum value), then
   * by index (ascending).
   *
   * @return Streams that fire on this tick.  Empty if no streams are registered.
   */
  [[nodiscard]] std::vector<StreamEvent> tick();

  /**
   * @brief Return the effective post-snap frequency for a registered stream.
   *
   * @return `1.0 / (period_ticks * step_dt)`, or 0 if the stream is not
   *         registered.
   */
  [[nodiscard]] double effective_rate(StreamKind kind, std::size_t index) const;

  /**
   * @brief Return the cached period (in ticks) for a registered stream.
   *
   * @return Period in ticks, or 0 if the stream is not registered.
   */
  [[nodiscard]] std::size_t period_ticks(StreamKind kind, std::size_t index) const;

private:
  struct Entry
  {
    std::size_t period_ticks{ 1 };
    std::size_t tick_count{ 0 };
    double freq_hz{ 0.0 };
    SchedulingMode mode{ SchedulingMode::SnapToMultiple };
    double accum_seconds{ 0.0 };   // only meaningful in Accumulator mode
    double period_seconds{ 0.0 };  // cached 1/freq_hz; 0.0 means "every tick"
  };

  // Key encoding: upper 32 bits = StreamKind cast to uint32_t,
  // lower 32 bits = index (truncated to uint32_t).  Avoids the need for a
  // custom hash on std::pair while staying O(1) average for lookup.
  static uint64_t make_key(StreamKind kind, std::size_t index) noexcept;
  static StreamKind key_kind(uint64_t key) noexcept;
  static std::size_t key_index(uint64_t key) noexcept;

  // Recomputes period_ticks for one entry given the current step_dt_.
  void recompute_period(Entry& entry) const;

  double step_dt_;
  std::unordered_map<uint64_t, Entry> entries_;
};

}  // namespace stdr_simulation
