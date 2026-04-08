#include <stdr_gui/pose_utils.hpp>

#include <cmath>

namespace stdr_gui
{

stdr_simulation::Pose2D transform_to_world(const stdr_simulation::Pose2D& parent,
                                           const stdr_simulation::Pose2D& child_offset)
{
  const double cos_theta = std::cos(parent.theta);
  const double sin_theta = std::sin(parent.theta);
  return {
    .x = parent.x + child_offset.x * cos_theta - child_offset.y * sin_theta,
    .y = parent.y + child_offset.x * sin_theta + child_offset.y * cos_theta,
    .theta = parent.theta + child_offset.theta,
  };
}

}  // namespace stdr_gui
