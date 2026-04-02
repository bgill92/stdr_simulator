#pragma once

namespace stdr_simulation::sensors {

/**
 * @brief Simulates a microphone detecting sound sources in the environment.
 *
 * Returns the sound pressure level at the sensor pose by accumulating
 * contributions from all active sound sources, with optional Gaussian noise.
 */
class SoundSimulator {
public:
  SoundSimulator() = default;
  ~SoundSimulator() = default;
};

}  // namespace stdr_simulation::sensors
