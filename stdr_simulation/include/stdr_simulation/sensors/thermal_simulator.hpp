#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a thermal sensor detecting heat sources in the environment.
 *
 * Returns the temperature reading at the sensor pose by integrating
 * contributions from all thermal sources, with optional Gaussian noise.
 */
class ThermalSimulator {
public:
  ThermalSimulator() = default;
  ~ThermalSimulator() = default;
};

}  // namespace stdr_simulation::sensors
