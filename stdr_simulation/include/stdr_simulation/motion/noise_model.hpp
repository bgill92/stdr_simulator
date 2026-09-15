#pragma once

/** @file Shared velocity-dependent noise model for motion controllers.
 *
 *  Implements the Thrun et al. "Probabilistic Robotics" sample_motion_model_velocity
 *  noise scheme.  Both IdealMotionModel and OmniMotionModel call into these
 *  free functions so the noise logic lives in exactly one place. */

#include <stdr_simulation/types.hpp>

#include <random>

namespace stdr_simulation::motion
{

/** Holds the result of apply_noise: the noisy command and the extra drift term. */
struct NoiseResult
{
  Twist2D noisy_cmd;
  double drift;  ///< Extra angular drift term (g) that couples translation into heading error.
};

/** Holds the expected odometry error variance accumulated over a reporting interval. */
struct OdometryVariance
{
  double translational;  ///< Variance of the along-track (linear_x) error, in m^2.
  double rotational;     ///< Variance of the heading error, in rad^2.
};

/** Draw a sample from N(0, sigma^2) using the standard library.
 *
 *  We use std::normal_distribution rather than the 12-sample uniform
 *  approximation from the original ROS1 code — the standard library gives
 *  exact Gaussian draws via the Box-Muller / ziggurat algorithm. */
[[nodiscard]] double sample_normal(double sigma, std::mt19937& rng);

/** Apply velocity-dependent noise to a Twist2D command.
 *
 *  When params.odometry_model is OdometryModel::Perfect the alpha
 *  coefficients are ignored entirely: the command is returned unchanged with
 *  zero drift and no sample is drawn from rng.  This keeps Perfect mode both
 *  noiseless and free of any RNG side effects regardless of what alphas a
 *  YAML file happens to carry.
 *
 *  Otherwise, noise variances scale with the squared velocity components
 *  following the Thrun et al. motion model, and are additionally scaled by
 *  1/dt so that the *per-tick* variance shrinks as the tick rate increases:
 *  sigma = sqrt((a_ux_ux*ux^2 + ...) / dt).  This keeps total drift over a
 *  fixed time interval independent of how many ticks it is split into (see
 *  odometry-sim-highlights.md §5, "Rate-invariant noise scaling") — without
 *  it, sampling fresh noise every tick would make the simulator noisier at
 *  higher tick rates for the same alpha coefficients.  When all alpha
 *  coefficients are zero, every sqrt argument is 0 and sample_normal
 *  returns 0, so no noise is added.
 *
 *  @param cmd          The commanded velocity before noise.
 *  @param params       KinematicConfig holding the alpha noise coefficients
 *                      and the selected OdometryModel.
 *  @param dt           Timestep in seconds over which cmd is being applied.
 *  @param rng          Random engine (caller-owned; modified in place).
 *  @return             NoiseResult containing the noisy command and drift term.
 *  @throws std::invalid_argument if dt is not positive — the 1/dt variance
 *          scaling would otherwise divide by zero or a negative number,
 *          producing a NaN sigma that std::normal_distribution cannot accept. */
[[nodiscard]] NoiseResult apply_noise(const Twist2D& cmd, const KinematicConfig& params, double dt, std::mt19937& rng);

/** Compute the expected odometry error variance accumulated over `interval` seconds
 *  of velocity-model noise for command `cmd`.
 *
 *  apply_noise scales its per-tick velocity-noise variance by 1/dt so that
 *  accumulated drift over a fixed interval is independent of tick rate (see
 *  apply_noise's doc comment and odometry-sim-highlights.md §5, "Rate-invariant
 *  noise scaling"): per-tick velocity noise variance is (alpha * u^2) / dt.
 *  Integrating that noisy velocity over the tick turns it into a position
 *  increment, which scales variance by dt^2, giving a per-tick position
 *  variance of ((alpha * u^2) / dt) * dt^2 = (alpha * u^2) * dt. Summing T / dt
 *  independent ticks over an interval T gives total variance (alpha * u^2) * T,
 *  independent of dt. This function returns that closed-form sum directly
 *  rather than summing samples.
 *
 *  translational is the linear_x noise variance (the a_ux_* coefficients);
 *  rotational is the sum of the angular_z noise variance (a_w_*) and the drift
 *  term variance (a_g_*), since the two perturbations are drawn independently
 *  and their variances add.
 *
 *  When params.odometry_model is OdometryModel::Perfect both variances are zero,
 *  matching apply_noise's noise-free behaviour in that mode.
 *
 *  @param cmd      The commanded velocity the interval was driven at.
 *  @param params   KinematicConfig holding the alpha noise coefficients and the
 *                  selected OdometryModel.
 *  @param interval Reporting interval in seconds over which the variance accumulates.
 *  @return         OdometryVariance{translational, rotational}.
 *  @throws std::invalid_argument if interval is not positive, matching apply_noise's
 *          dt validation. */
[[nodiscard]] OdometryVariance odometry_variance(const Twist2D& cmd, const KinematicConfig& params, double interval);

}  // namespace stdr_simulation::motion
