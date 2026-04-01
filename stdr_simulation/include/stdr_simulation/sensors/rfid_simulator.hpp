#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates an RFID reader detecting tags placed in the environment.
 *
 * Reports tags within the configured detection radius of the sensor pose,
 * with optional signal-strength variation modelled as Gaussian noise.
 */
class RfidSimulator {
public:
  RfidSimulator() = default;
  ~RfidSimulator() = default;
};

}  // namespace stdr_simulation::sensors
