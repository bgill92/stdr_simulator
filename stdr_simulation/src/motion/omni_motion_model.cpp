#include <stdr_simulation/motion/omni_motion_model.hpp>

#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion {

OmniMotionModel::OmniMotionModel() : rng_(std::random_device{}()) {}

Pose2D OmniMotionModel::update(
    const Pose2D& current,
    const Twist2D& cmd,
    const double dt,
    const KinematicConfig& noise_params) const {
  const auto [noisy, g] = apply_noise(cmd, noise_params, rng_);

  const double vx = noisy.linear_x;
  const double vy = noisy.linear_y;
  const double w_total = noisy.angular_z + g;

  const double cos_theta = std::cos(current.theta);
  const double sin_theta = std::sin(current.theta);

  // Holonomic kinematics: x and y are decoupled from rotation, so the full
  // lateral velocity contributes directly after a frame rotation.
  Pose2D next = current;
  next.x += vx * dt * cos_theta - vy * dt * sin_theta;
  next.y += vx * dt * sin_theta + vy * dt * cos_theta;
  next.theta += w_total * dt;

  // Keep theta in [-π, π] to prevent precision degradation over long runs.
  next.theta = std::atan2(std::sin(next.theta), std::cos(next.theta));

  return next;
}

}  // namespace stdr_simulation::motion
