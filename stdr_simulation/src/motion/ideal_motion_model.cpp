#include <stdr_simulation/motion/ideal_motion_model.hpp>

#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion
{

IdealMotionModel::IdealMotionModel() : rng_(std::random_device{}())
{
}

Pose2D IdealMotionModel::update(const Pose2D& current, const Twist2D& cmd, const double dt,
                                const KinematicConfig& noise_params) const
{
  const auto [noisy, g] = apply_noise(cmd, noise_params, rng_);

  const double v = noisy.linear_x;
  const double w_noisy = noisy.angular_z;
  const double w_total = w_noisy + g;

  Pose2D next = current;

  // Arc radius exceeds 10^9 m at this threshold — effectively straight-line motion.
  if (std::abs(w_total) < 1e-9)
  {
    next.x += v * dt * std::cos(current.theta);
    next.y += v * dt * std::sin(current.theta);
    // Heading unchanged for pure translation.
  }
  else
  {
    const double r = v / w_total;
    next.x += r * (std::sin(current.theta + w_total * dt) - std::sin(current.theta));
    next.y -= r * (std::cos(current.theta + w_total * dt) - std::cos(current.theta));
    next.theta += w_total * dt;
  }

  // Keep theta in [-π, π] to prevent precision degradation over long runs.
  next.theta = std::atan2(std::sin(next.theta), std::cos(next.theta));

  return next;
}

}  // namespace stdr_simulation::motion
