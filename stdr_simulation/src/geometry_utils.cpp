#include <stdr_simulation/geometry_utils.hpp>

#include <cmath>
#include <cstddef>

namespace stdr_simulation
{

// Tolerance used for on-edge classification in point_in_footprint.
// Chosen to be well above floating-point noise (~1e-15) but negligible
// for any physically meaningful robot geometry (sub-millimetre at 1 mm/unit).
static constexpr double kFootprintEpsilon = 1e-9;

Pose2D compute_sensor_world_pose(const Pose2D& robot_pose, const Pose2D& sensor_local)
{
  const double cos_theta = std::cos(robot_pose.theta);
  const double sin_theta = std::sin(robot_pose.theta);
  Pose2D result;
  result.x = robot_pose.x + sensor_local.x * cos_theta - sensor_local.y * sin_theta;
  result.y = robot_pose.y + sensor_local.x * sin_theta + sensor_local.y * cos_theta;
  result.theta = robot_pose.theta + sensor_local.theta;
  return result;
}

Pose2D body_to_pivot_pose(const Pose2D& body_pose, const Point2D& pivot)
{
  const double cos_theta = std::cos(body_pose.theta);
  const double sin_theta = std::sin(body_pose.theta);
  Pose2D result;
  result.x = body_pose.x + pivot.x * cos_theta - pivot.y * sin_theta;
  result.y = body_pose.y + pivot.x * sin_theta + pivot.y * cos_theta;
  result.theta = body_pose.theta;
  return result;
}

Pose2D pivot_to_body_pose(const Pose2D& pivot_pose, const Point2D& pivot)
{
  // Rotate the body-frame pivot offset by the new heading, then subtract it
  // from the pivot world position to recover the body origin.
  const double cos_theta = std::cos(pivot_pose.theta);
  const double sin_theta = std::sin(pivot_pose.theta);
  Pose2D result;
  result.x = pivot_pose.x - (pivot.x * cos_theta - pivot.y * sin_theta);
  result.y = pivot_pose.y - (pivot.x * sin_theta + pivot.y * cos_theta);
  result.theta = pivot_pose.theta;
  return result;
}

bool point_in_footprint(const Point2D& p, const Footprint& fp)
{
  if (fp.points.empty())
  {
    // Circle branch: distance from origin (robot body frame centre) to point.
    return std::hypot(p.x, p.y) <= fp.radius + kFootprintEpsilon;
  }

  const std::size_t n = fp.points.size();
  if (n < 3)
  {
    // A degenerate polygon cannot enclose any area; treat as outside rather
    // than silently passing a point through collision detection.
    return false;
  }

  // Crossing-number (ray-cast) algorithm.  A horizontal ray is cast from the
  // test point to the right (+x direction).  Each edge crossing increments the
  // counter; an odd count means inside.  On-edge points are caught by the
  // explicit edge-distance check below.
  int crossings = 0;
  for (std::size_t i = 0; i < n; ++i)
  {
    const Point2D& a = fp.points[i];
    const Point2D& b = fp.points[(i + 1) % n];

    // Check whether p lies on the segment [a, b] within epsilon.
    // We use the cross-product magnitude for perpendicular distance and the
    // dot-product check to confirm the projection falls within the segment.
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;

    if (len_sq > 0.0)
    {
      // Cross product (a→b) × (a→p) gives twice the signed area of the
      // triangle; dividing by |a→b| yields the perpendicular distance.
      const double cross = dx * (p.y - a.y) - dy * (p.x - a.x);
      if (std::abs(cross) <= kFootprintEpsilon * std::sqrt(len_sq))
      {
        // Point is on the infinite line through a and b; confirm it is between
        // them along the edge direction.
        const double dot = (p.x - a.x) * dx + (p.y - a.y) * dy;
        if (dot >= -kFootprintEpsilon && dot <= len_sq + kFootprintEpsilon)
        {
          return true;  // On edge — treated as inside.
        }
      }
    }

    // Standard ray-cast crossing test.
    if (((a.y <= p.y) && (b.y > p.y)) || ((b.y <= p.y) && (a.y > p.y)))
    {
      // Compute x-coordinate of intersection of the edge with the horizontal
      // ray at height p.y.
      const double x_intersect = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < x_intersect)
      {
        ++crossings;
      }
    }
  }

  return (crossings % 2) != 0;
}

}  // namespace stdr_simulation
