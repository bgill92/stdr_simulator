#include <stdr_simulation/rate_scheduler.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace stdr_simulation
{

// ---------------------------------------------------------------------------
// Key helpers
// ---------------------------------------------------------------------------

uint64_t RateScheduler::make_key(StreamKind kind, std::size_t index) noexcept
{
  const uint64_t kind_bits = static_cast<uint64_t>(static_cast<uint32_t>(kind)) << 32U;
  const uint64_t idx_bits = static_cast<uint64_t>(static_cast<uint32_t>(index));
  return kind_bits | idx_bits;
}

StreamKind RateScheduler::key_kind(uint64_t key) noexcept
{
  return static_cast<StreamKind>(static_cast<uint32_t>(key >> 32U));
}

std::size_t RateScheduler::key_index(uint64_t key) noexcept
{
  // Sensor index is assumed < 2^32; values >= 2^32 would be silently truncated.
  return static_cast<std::size_t>(static_cast<uint32_t>(key & 0xFFFF'FFFFU));
}

// ---------------------------------------------------------------------------
// Period computation
// ---------------------------------------------------------------------------

void RateScheduler::recompute_period(Entry& entry) const
{
  if (entry.freq_hz <= 0.0)
  {
    // Treat non-positive frequency as "fire every tick" — backward-compatible
    // default for sensors that omit a frequency in their config.
    entry.period_ticks = 1;
    return;
  }
  // Round to the nearest tick multiple.  Using std::round rather than floor so
  // that a target slightly above the exact multiple snaps to the closer option,
  // minimising the effective-rate error.
  const double raw_period = 1.0 / (entry.freq_hz * step_dt_);
  const std::size_t snapped = static_cast<std::size_t>(std::round(raw_period));
  // Clamp to at least 1: a target rate faster than the sim rate is silently
  // capped at the sim rate (one fire per tick).
  entry.period_ticks = std::max(std::size_t{ 1 }, snapped);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

RateScheduler::RateScheduler(double step_dt_seconds) : step_dt_(step_dt_seconds)
{
  if (step_dt_seconds <= 0.0)
  {
    throw std::invalid_argument("step_dt_seconds must be positive");
  }
}

void RateScheduler::set_step_dt(double step_dt_seconds)
{
  if (step_dt_seconds <= 0.0)
  {
    throw std::invalid_argument("step_dt_seconds must be positive");
  }
  step_dt_ = step_dt_seconds;
  for (auto& [key, entry] : entries_)
  {
    recompute_period(entry);
  }
}

double RateScheduler::step_dt() const noexcept
{
  return step_dt_;
}

void RateScheduler::set_rate(StreamKind kind, std::size_t index, double freq_hz)
{
  const uint64_t key = make_key(kind, index);
  Entry& entry = entries_[key];  // inserts default-constructed entry if absent
  entry.freq_hz = freq_hz;
  recompute_period(entry);
  // tick_count is intentionally preserved so an in-flight update does not
  // reset the fire cadence mid-run.
}

void RateScheduler::clear()
{
  entries_.clear();
}

std::vector<StreamEvent> RateScheduler::tick()
{
  std::vector<StreamEvent> fired;
  fired.reserve(entries_.size());

  for (auto& [key, entry] : entries_)
  {
    ++entry.tick_count;
    if (entry.tick_count % entry.period_ticks == 0)
    {
      fired.push_back(StreamEvent{ key_kind(key), key_index(key) });
    }
  }

  // Sort by (kind, index) so callers receive a deterministic ordering
  // regardless of unordered_map iteration order.
  std::sort(fired.begin(), fired.end(), [](const StreamEvent& a, const StreamEvent& b) {
    if (a.kind != b.kind)
    {
      return static_cast<std::underlying_type_t<StreamKind>>(a.kind) <
             static_cast<std::underlying_type_t<StreamKind>>(b.kind);
    }
    return a.index < b.index;
  });

  return fired;
}

double RateScheduler::effective_rate(StreamKind kind, std::size_t index) const
{
  const auto it = entries_.find(make_key(kind, index));
  if (it == entries_.end())
  {
    return 0.0;
  }
  return 1.0 / (static_cast<double>(it->second.period_ticks) * step_dt_);
}

std::size_t RateScheduler::period_ticks(StreamKind kind, std::size_t index) const
{
  const auto it = entries_.find(make_key(kind, index));
  if (it == entries_.end())
  {
    return 0;
  }
  return it->second.period_ticks;
}

}  // namespace stdr_simulation
