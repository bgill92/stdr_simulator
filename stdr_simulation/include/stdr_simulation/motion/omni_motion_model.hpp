#pragma once

/** @file Omnidirectional motion model with velocity-dependent noise. */

#include <stdr_simulation/types.hpp>

#include <random>

namespace stdr_simulation::motion {

/**
 * @brief Holonomic (omnidirectional) motion model with optional Gaussian velocity noise.
 *
 * Integrates a full Twist2D command — including lateral linear_y — over a
 * timestep using decoupled x/y/theta kinematics.  Noise follows the same
 * velocity-dependent model as IdealMotionModel; when all KinematicConfig
 * alpha coefficients are zero the model is noiseless.
 *
 * @warning Not thread-safe — each thread should own its own instance because
 *          the internal RNG is mutated on every call to update(). */
class OmniMotionModel {
public:
  OmniMotionModel();
  ~OmniMotionModel() = default;

  /**
   * @brief Propagate a pose forward by one timestep under a velocity command.
   *
   * @param current      The robot's current 2D pose.
   * @param cmd          Commanded velocity (all three components are used).
   * @param dt           Timestep in seconds.
   * @param noise_params KinematicConfig encoding the alpha noise coefficients.
   * @return             The predicted next pose.
   */
  [[nodiscard]] Pose2D update(
      const Pose2D& current,
      const Twist2D& cmd,
      double dt,
      const KinematicConfig& noise_params) const;

private:
  // Mutable because the RNG is internal implementation state, not observable
  // robot state — callers see a logically const model.
  mutable std::mt19937 rng_;
};

}  // namespace stdr_simulation::motion
