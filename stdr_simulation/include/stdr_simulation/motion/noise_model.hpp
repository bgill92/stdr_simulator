#pragma once

/** @file Shared velocity-dependent noise model for motion controllers.
 *
 *  Implements the Thrun et al. "Probabilistic Robotics" sample_motion_model_velocity
 *  noise scheme.  Both IdealMotionModel and OmniMotionModel call into these
 *  free functions so the noise logic lives in exactly one place. */

#include <stdr_simulation/types.hpp>

#include <random>

namespace stdr_simulation::motion {

/** Holds the result of apply_noise: the noisy command and the extra drift term. */
struct NoiseResult {
  Twist2D noisy_cmd;
  double drift;  ///< Extra angular drift term (g) that couples translation into heading error.
};

/** Draw a sample from N(0, sigma^2) using the standard library.
 *
 *  We use std::normal_distribution rather than the 12-sample uniform
 *  approximation from the original ROS1 code — the standard library gives
 *  exact Gaussian draws via the Box-Muller / ziggurat algorithm. */
[[nodiscard]] double sample_normal(double sigma, std::mt19937& rng);

/** Apply velocity-dependent noise to a Twist2D command.
 *
 *  Noise variances scale with the squared velocity components following the
 *  Thrun et al. motion model.  When all alpha coefficients are zero (the
 *  noiseless case), every sqrt argument is 0 and sample_normal returns 0,
 *  so no noise is added.
 *
 *  @param cmd          The commanded velocity before noise.
 *  @param params       KinematicConfig holding the alpha noise coefficients.
 *  @param rng          Random engine (caller-owned; modified in place).
 *  @return             NoiseResult containing the noisy command and drift term. */
[[nodiscard]] NoiseResult apply_noise(
    const Twist2D& cmd,
    const KinematicConfig& params,
    std::mt19937& rng);

}  // namespace stdr_simulation::motion
