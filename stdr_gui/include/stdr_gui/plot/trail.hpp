#pragma once

/** @file Breadcrumb-trail helper shared by plotters that draw a pose history
 *  over the map (Map Trace, Odometry Trace).
 *
 *  Extracted from map_trace_plotter.cpp so Odometry Trace can keep two
 *  independent trails (truth and odometry belief) without duplicating the
 *  spacing/cap bookkeeping. */

#include <stdr_simulation/types.hpp>

#include <cstddef>
#include <deque>

namespace stdr::plot
{

// Minimum Euclidean distance the robot must travel before a new breadcrumb is
// dropped. Below this spacing, slow or stationary motion produces a dense,
// indistinguishable blob.
inline constexpr double kMarkerSpacingMeters = 0.1;

// Hard cap on stored breadcrumb markers so memory and per-frame render cost
// stay bounded regardless of how long the simulation runs.
inline constexpr std::size_t kMaxMarkers = 500;

// Length of the heading arrow drawn from a robot's origin, in metres. Shared
// by Map Trace and Odometry Trace so both plotters draw the same-sized arrow.
inline constexpr double kHeadingArrowLen = 0.3;

/** @brief A capped, spacing-gated breadcrumb trail of `Pose2D` samples. */
struct Trail
{
  std::deque<stdr_simulation::Pose2D> points;

  /** Append @p pose if it is the first point, or at least
   *  `kMarkerSpacingMeters` away from the last stored point. Drops the
   *  oldest point once `kMaxMarkers` is reached before appending. */
  void push_if_moved(const stdr_simulation::Pose2D& pose);

  /** Discard all stored points, e.g. after a teleport so the trail does not
   *  draw a line across the map. */
  void clear();
};

}  // namespace stdr::plot
