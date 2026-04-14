#pragma once

#include <cmath>

namespace stdr_simulation::sensors
{

/**
 * @brief Check if a target angle lies within the arc [center - span/2, center + span/2].
 *
 * Normalizes the difference to [-π, π] before comparing so that the check
 * handles wraparound at ±π correctly.
 *
 * @param target Angle to test (radians).
 * @param center Center of the arc (radians).
 * @param span   Full angular width of the arc (radians).
 * @return true if target falls within the arc.
 */
[[nodiscard]] inline bool angle_in_range(double target, double center, double span)
{
  const double diff = std::atan2(std::sin(target - center), std::cos(target - center));
  return std::abs(diff) <= span / 2.0;
}

}  // namespace stdr_simulation::sensors
