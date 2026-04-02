#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a 2D laser range scanner against the world occupancy grid.
 *
 * Casts rays at evenly-spaced angular increments within the configured field
 * of view and returns per-beam range measurements with optional Gaussian noise.
 */
class LaserSimulator {
public:
  LaserSimulator() = default;
  ~LaserSimulator() = default;
};

}  // namespace stdr_simulation::sensors
