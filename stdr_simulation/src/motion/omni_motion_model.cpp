#include <stdr_simulation/motion/omni_motion_model.hpp>

#include <stdr_simulation/geometry_utils.hpp>
#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion
{

OmniMotionModel::OmniMotionModel() : rng_(std::random_device{}())
{
}

Pose2D OmniMotionModel::update(const Pose2D& current, const Twist2D& cmd, const double dt,
                               const KinematicConfig& noise_params, const Point2D& center_of_rotation) const
{
  const auto [noisy, g] = apply_noise(cmd, noise_params, rng_);

  const double vx = noisy.linear_x;
  const double vy = noisy.linear_y;
  const double w_total = noisy.angular_z + g;

  // Shift integration from body origin to the pivot point so that rotation is
  // applied about center_of_rotation rather than the body origin.  When the
  // pivot is (0,0) the transform is identity and the result is unchanged.
  const Pose2D pivot_current = body_to_pivot_pose(current, center_of_rotation);

  const double cos_theta = std::cos(pivot_current.theta);
  const double sin_theta = std::sin(pivot_current.theta);

  // Holonomic kinematics: x and y are decoupled from rotation, so the full
  // lateral velocity contributes directly after a frame rotation.
  Pose2D pivot_next = pivot_current;
  pivot_next.x += vx * dt * cos_theta - vy * dt * sin_theta;
  pivot_next.y += vx * dt * sin_theta + vy * dt * cos_theta;
  pivot_next.theta += w_total * dt;

  // Keep theta in [-π, π] to prevent precision degradation over long runs.
  pivot_next.theta = std::atan2(std::sin(pivot_next.theta), std::cos(pivot_next.theta));

  // Recover body origin from the updated pivot pose.
  return pivot_to_body_pose(pivot_next, center_of_rotation);
}

}  // namespace stdr_simulation::motion
