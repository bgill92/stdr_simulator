#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion {

double sample_normal(const double sigma, std::mt19937& rng) {
  // Constructing normal_distribution with sigma=0 is UB per the standard.
  if (sigma <= 0.0) { return 0.0; }
  std::normal_distribution<double> dist(0.0, sigma);
  return dist(rng);
}

NoiseResult apply_noise(
    const Twist2D& cmd,
    const KinematicConfig& params,
    std::mt19937& rng) {
  const double ux = cmd.linear_x;
  const double uy = cmd.linear_y;
  const double w = cmd.angular_z;

  Twist2D noisy = cmd;
  noisy.linear_x += sample_normal(
      std::sqrt(params.a_ux_ux * ux * ux + params.a_ux_uy * uy * uy + params.a_ux_w * w * w),
      rng);
  noisy.linear_y += sample_normal(
      std::sqrt(params.a_uy_ux * ux * ux + params.a_uy_uy * uy * uy + params.a_uy_w * w * w),
      rng);
  noisy.angular_z += sample_normal(
      std::sqrt(params.a_w_ux * ux * ux + params.a_w_uy * uy * uy + params.a_w_w * w * w), rng);

  // Extra drift term that couples translational speed into heading error.
  const double drift = sample_normal(
      std::sqrt(params.a_g_ux * ux * ux + params.a_g_uy * uy * uy + params.a_g_w * w * w), rng);

  return NoiseResult{noisy, drift};
}

}  // namespace stdr_simulation::motion
