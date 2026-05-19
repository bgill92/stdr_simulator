#include <stdr_simulation/motion/ideal_motion_model.hpp>

#include <stdr_simulation/geometry_utils.hpp>
#include <stdr_simulation/motion/noise_model.hpp>

#include <cmath>

namespace stdr_simulation::motion
{

IdealMotionModel::IdealMotionModel() : rng_(std::random_device{}())
{
}

Pose2D IdealMotionModel::update(const Pose2D& current, const Twist2D& cmd, const double dt,
                                const KinematicConfig& noise_params, const Point2D& center_of_rotation) const
{
  const auto [noisy, g] = apply_noise(cmd, noise_params, rng_);

  const double v = noisy.linear_x;
  const double w_noisy = noisy.angular_z;
  const double w_total = w_noisy + g;

  // Shift integration from body origin to the pivot point so that rotation is
  // applied about center_of_rotation rather than the body origin.  When the
  // pivot is (0,0) the transform is identity and the result is unchanged.
  const Pose2D pivot_current = body_to_pivot_pose(current, center_of_rotation);
  Pose2D pivot_next = pivot_current;

  // Arc radius exceeds 10^9 m at this threshold — effectively straight-line motion.
  if (std::abs(w_total) < 1e-9)
  {
    pivot_next.x += v * dt * std::cos(pivot_current.theta);
    pivot_next.y += v * dt * std::sin(pivot_current.theta);
    // Heading unchanged for pure translation.
  }
  else
  {
    const double r = v / w_total;
    pivot_next.x += r * (std::sin(pivot_current.theta + w_total * dt) - std::sin(pivot_current.theta));
    pivot_next.y -= r * (std::cos(pivot_current.theta + w_total * dt) - std::cos(pivot_current.theta));
    pivot_next.theta += w_total * dt;
  }

  // Keep theta in [-π, π] to prevent precision degradation over long runs.
  pivot_next.theta = std::atan2(std::sin(pivot_next.theta), std::cos(pivot_next.theta));

  // Recover body origin from the updated pivot pose.
  return pivot_to_body_pose(pivot_next, center_of_rotation);
}

}  // namespace stdr_simulation::motion
