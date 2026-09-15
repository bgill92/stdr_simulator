#include <stdr_simulation/motion/ideal_motion_model.hpp>

#include <stdr_simulation/geometry_utils.hpp>
#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion
{

IdealMotionModel::IdealMotionModel() : rng_(std::random_device{}())
{
}

IdealMotionModel::IdealMotionModel(const std::uint32_t seed) : rng_(seed)
{
}

Pose2D IdealMotionModel::update(const Pose2D& current, const Twist2D& cmd, const double dt,
                                const KinematicConfig& noise_params, const Point2D& center_of_rotation) const
{
  const auto [noisy, g] = apply_noise(cmd, noise_params, dt, rng_);
  return integrate_kinematics(current, noisy.linear_x, noisy.angular_z + g, dt, center_of_rotation);
}

Pose2D IdealMotionModel::integrate(const Pose2D& current, const Twist2D& cmd, const double dt,
                                   const Point2D& center_of_rotation) const
{
  return integrate_kinematics(current, cmd.linear_x, cmd.angular_z, dt, center_of_rotation);
}

Pose2D IdealMotionModel::integrate_kinematics(const Pose2D& current, const double v, const double w, const double dt,
                                              const Point2D& center_of_rotation)
{
  // Shift integration from body origin to the pivot point so that rotation is
  // applied about center_of_rotation rather than the body origin.  When the
  // pivot is (0,0) the transform is identity and the result is unchanged.
  const Pose2D pivot_current = body_to_pivot_pose(current, center_of_rotation);
  Pose2D pivot_next = pivot_current;

  // Arc radius exceeds 10^9 m at this threshold — effectively straight-line motion.
  if (std::abs(w) < 1e-9)
  {
    pivot_next.x += v * dt * std::cos(pivot_current.theta);
    pivot_next.y += v * dt * std::sin(pivot_current.theta);
    // Heading unchanged for pure translation.
  }
  else
  {
    const double r = v / w;
    pivot_next.x += r * (std::sin(pivot_current.theta + w * dt) - std::sin(pivot_current.theta));
    pivot_next.y -= r * (std::cos(pivot_current.theta + w * dt) - std::cos(pivot_current.theta));
    pivot_next.theta += w * dt;
  }

  // Keep theta in [-π, π] to prevent precision degradation over long runs.
  pivot_next.theta = std::atan2(std::sin(pivot_next.theta), std::cos(pivot_next.theta));

  // Recover body origin from the updated pivot pose.
  return pivot_to_body_pose(pivot_next, center_of_rotation);
}

}  // namespace stdr_simulation::motion
