#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a CO2 concentration sensor in an environment with sources.
 *
 * Computes concentration at the sensor pose by summing contributions from
 * all CO2 sources using an inverse-square dispersion model, then applies
 * optional Gaussian noise.
 */
class Co2Simulator {
public:
  Co2Simulator() = default;
  ~Co2Simulator() = default;
};

}  // namespace stdr_simulation::sensors
