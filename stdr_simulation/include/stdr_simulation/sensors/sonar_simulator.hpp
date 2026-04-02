#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates an ultrasonic sonar sensor against the world occupancy grid.
 *
 * Models a cone-shaped beam and returns the minimum range to any obstacle
 * within the cone, with optional Gaussian noise applied to the measurement.
 */
class SonarSimulator {
public:
  SonarSimulator() = default;
  ~SonarSimulator() = default;
};

}  // namespace stdr_simulation::sensors
