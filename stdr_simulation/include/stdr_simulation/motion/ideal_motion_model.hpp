#pragma once

namespace stdr_simulation::motion {

/**
 * @brief Ideal (noiseless) differential-drive motion model.
 *
 * Integrates a Twist2D command over a timestep using exact kinematics to
 * produce the next Pose2D. Intended as the reference model for testing and
 * as a base for noise-augmented variants.
 */
class IdealMotionModel {
public:
  IdealMotionModel() = default;
  ~IdealMotionModel() = default;
};

}  // namespace stdr_simulation::motion
