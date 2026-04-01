#pragma once

namespace stdr_simulation::motion {

/**
 * @brief Ideal omnidirectional motion model for holonomic robots.
 *
 * Integrates a full Twist2D command (including lateral linear_y) over a
 * timestep using exact kinematics.  Used for robots with omnidirectional
 * wheels or similar holonomic drive systems.
 */
class OmniMotionModel {
public:
  OmniMotionModel() = default;
  ~OmniMotionModel() = default;
};

}  // namespace stdr_simulation::motion
