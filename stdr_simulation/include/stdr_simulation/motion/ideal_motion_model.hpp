#pragma once

/** @file Differential-drive motion model with velocity-dependent noise. */

#include <stdr_simulation/types.hpp>

#include <random>

namespace stdr_simulation::motion
{

/**
 * @brief Differential-drive motion model with optional Gaussian velocity noise.
 *
 * Integrates a Twist2D command over a timestep using exact arc kinematics
 * (Thrun et al., Probabilistic Robotics, Chapter 5).  Noise is sampled from
 * the velocity-dependent model in noise_model.hpp; when all KinematicConfig
 * alpha coefficients are zero the model is noiseless.
 *
 * @warning Not thread-safe — each thread should own its own instance because
 *          the internal RNG is mutated on every call to update(). */
class IdealMotionModel
{
public:
  IdealMotionModel();
  ~IdealMotionModel() = default;

  /**
   * @brief Propagate a pose forward by one timestep under a velocity command.
   *
   * @param current             The robot's current 2D pose.
   * @param cmd                 Commanded velocity (linear_y is ignored — non-holonomic).
   * @param dt                  Timestep in seconds.
   * @param noise_params        KinematicConfig encoding the alpha noise coefficients.
   * @param center_of_rotation  Rotation pivot expressed in the robot body frame.
   *                            Defaults to (0, 0) — the body origin — which
   *                            reproduces the previous behaviour exactly.
   * @return                    The predicted next pose.
   */
  [[nodiscard]] Pose2D update(const Pose2D& current, const Twist2D& cmd, double dt, const KinematicConfig& noise_params,
                              const Point2D& center_of_rotation = {}) const;

private:
  // Mutable because the RNG is internal implementation state, not observable
  // robot state — callers see a logically const model.
  mutable std::mt19937 rng_;
};

}  // namespace stdr_simulation::motion
