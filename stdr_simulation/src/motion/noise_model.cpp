#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>
#include <stdexcept>

namespace stdr_simulation::motion
{

double sample_normal(const double sigma, std::mt19937& rng)
{
  // Constructing normal_distribution with sigma=0 is UB per the standard.
  if (sigma <= 0.0)
  {
    return 0.0;
  }
  std::normal_distribution<double> dist(0.0, sigma);
  return dist(rng);
}

NoiseResult apply_noise(const Twist2D& cmd, const KinematicConfig& params, const double dt, std::mt19937& rng)
{
  // Validate dt before the Perfect early-return so a non-positive dt is
  // always rejected, rather than silently accepted whenever the model
  // happens to be Perfect. Without the guard the 1/dt scaling below would
  // give a NaN sigma, which sample_normal's sigma <= 0 check cannot catch,
  // and std::normal_distribution with a NaN stddev is UB.
  if (dt <= 0.0)
  {
    throw std::invalid_argument("apply_noise: dt must be positive");
  }

  // Perfect odometry: alphas are ignored and truth exactly tracks the
  // command, so odom == truth by construction (no RNG draw).
  if (params.odometry_model == OdometryModel::Perfect)
  {
    return NoiseResult{ cmd, 0.0 };
  }

  const double ux = cmd.linear_x;
  const double uy = cmd.linear_y;
  const double w = cmd.angular_z;

  // Per-tick variance is scaled by 1/dt so accumulated drift over a fixed
  // interval does not depend on the simulation tick rate.
  Twist2D noisy = cmd;
  noisy.linear_x +=
      sample_normal(std::sqrt((params.a_ux_ux * ux * ux + params.a_ux_uy * uy * uy + params.a_ux_w * w * w) / dt), rng);
  noisy.linear_y +=
      sample_normal(std::sqrt((params.a_uy_ux * ux * ux + params.a_uy_uy * uy * uy + params.a_uy_w * w * w) / dt), rng);
  noisy.angular_z +=
      sample_normal(std::sqrt((params.a_w_ux * ux * ux + params.a_w_uy * uy * uy + params.a_w_w * w * w) / dt), rng);

  // Extra drift term that couples translational speed into heading error.
  const double drift =
      sample_normal(std::sqrt((params.a_g_ux * ux * ux + params.a_g_uy * uy * uy + params.a_g_w * w * w) / dt), rng);

  return NoiseResult{ noisy, drift };
}

}  // namespace stdr_simulation::motion
