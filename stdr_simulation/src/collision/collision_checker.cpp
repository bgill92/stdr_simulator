#include <stdr_simulation/collision/collision_checker.hpp>

#include <cmath>
#include <utility>
#include <vector>

namespace stdr_simulation::collision
{

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::vector<Point2D> CollisionChecker::expand_footprint(const Footprint& footprint)
{
  if (!footprint.points.empty())
  {
    return footprint.points;
  }

  // Generate a circular approximation with 1-degree resolution so that robots
  // described only by a radius are handled identically to polygon robots.
  constexpr int kSteps = 360;
  std::vector<Point2D> points;
  points.reserve(kSteps);
  for (int i = 0; i < kSteps; ++i)
  {
    const double angle = static_cast<double>(i) * M_PI / 180.0;
    points.push_back({ footprint.radius * std::cos(angle), footprint.radius * std::sin(angle) });
  }
  return points;
}

std::vector<std::pair<int, int>> CollisionChecker::get_points_between(int x1, int y1, int x2, int y2)
{
  std::vector<std::pair<int, int>> points;

  const double angle = std::atan2(static_cast<double>(y2 - y1), static_cast<double>(x2 - x1));
  const double dist = std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));

  // Step one grid unit at a time along the segment, collecting every cell.
  for (double d = 0.0; d < dist; d += 1.0)
  {
    const int cx = x1 + static_cast<int>(std::round(d * std::cos(angle)));
    const int cy = y1 + static_cast<int>(std::round(d * std::sin(angle)));
    points.emplace_back(cx, cy);
  }
  // Always include the endpoint.
  points.emplace_back(x2, y2);

  return points;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool CollisionChecker::check_collision(const Pose2D& pose, const Footprint& footprint, const OccupancyGrid& map) const
{
  // Zero resolution would cause division by zero; treat as collision to be safe.
  if (map.resolution <= 0.0)
  {
    return true;
  }

  const std::vector<Point2D> fp_points = expand_footprint(footprint);
  const double cos_theta = std::cos(pose.theta);
  const double sin_theta = std::sin(pose.theta);

  for (const Point2D& pt : fp_points)
  {
    // Rotate footprint point into world frame.
    const double wx = pt.x * cos_theta - pt.y * sin_theta + pose.x;
    const double wy = pt.x * sin_theta + pt.y * cos_theta + pose.y;

    // Convert to grid cell indices, accounting for the map origin offset.
    const int grid_x = static_cast<int>((wx - map.origin.x) / map.resolution);
    const int grid_y = static_cast<int>((wy - map.origin.y) / map.resolution);

    // Out-of-bounds is treated as a collision — the robot must stay inside the
    // known map to maintain safe operation.
    if (grid_x < 0 || grid_x >= map.width || grid_y < 0 || grid_y >= map.height)
    {
      return true;
    }

    const std::size_t idx =
        static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(map.width) + static_cast<std::size_t>(grid_x);

    // Unknown cells (value -1 in nav_msgs convention) are treated as occupied
    // for safety — we cannot assume clear space we have no information about.
    if (map.data[idx] < 0 || map.data[idx] > kOccupancyThreshold)
    {
      return true;
    }
  }

  return false;
}

bool CollisionChecker::check_path_collision(const Pose2D& new_pose, const Pose2D& previous_pose,
                                            const Footprint& footprint, const OccupancyGrid& map) const
{
  // Zero resolution would cause division by zero; treat as collision to be safe.
  if (map.resolution <= 0.0)
  {
    return true;
  }

  const std::vector<Point2D> fp_points = expand_footprint(footprint);

  // Convert both endpoints to grid coordinates for the path walk,
  // accounting for the map origin offset.
  const int prev_gx = static_cast<int>((previous_pose.x - map.origin.x) / map.resolution);
  const int prev_gy = static_cast<int>((previous_pose.y - map.origin.y) / map.resolution);
  const int new_gx = static_cast<int>((new_pose.x - map.origin.x) / map.resolution);
  const int new_gy = static_cast<int>((new_pose.y - map.origin.y) / map.resolution);

  const double path_angle = std::atan2(static_cast<double>(new_gy - prev_gy), static_cast<double>(new_gx - prev_gx));
  const double path_dist = std::hypot(static_cast<double>(new_gx - prev_gx), static_cast<double>(new_gy - prev_gy));

  // Always check at least the new_pose position along the path, so that moves
  // shorter than one cell are not silently skipped.
  double d = 0.0;
  while (d <= path_dist)
  {
    const int step_x = prev_gx + static_cast<int>(std::round(d * std::cos(path_angle)));
    const int step_y = prev_gy + static_cast<int>(std::round(d * std::sin(path_angle)));

    // Interpolate orientation along the path so the footprint is rotated
    // correctly at each intermediate step, not just at the destination.
    const double t = (path_dist > 0.0) ? d / path_dist : 1.0;
    const double theta = previous_pose.theta + t * (new_pose.theta - previous_pose.theta);
    const double cos_theta = std::cos(theta);
    const double sin_theta = std::sin(theta);

    // Check each footprint edge (line segment between consecutive points).
    const std::size_t n = fp_points.size();
    for (std::size_t i = 0; i < n; ++i)
    {
      const Point2D& pa = fp_points[i];
      const Point2D& pb = fp_points[(i + 1) % n];

      // Rotate both edge endpoints into world-frame grid coordinates.
      const int ax = step_x + static_cast<int>((pa.x * cos_theta - pa.y * sin_theta) / map.resolution);
      const int ay = step_y + static_cast<int>((pa.x * sin_theta + pa.y * cos_theta) / map.resolution);
      const int bx = step_x + static_cast<int>((pb.x * cos_theta - pb.y * sin_theta) / map.resolution);
      const int by = step_y + static_cast<int>((pb.x * sin_theta + pb.y * cos_theta) / map.resolution);

      // Check every intermediate cell on this edge.
      const std::vector<std::pair<int, int>> edge_cells = get_points_between(ax, ay, bx, by);
      for (const auto& [cx, cy] : edge_cells)
      {
        // Check 3x3 neighbourhood around each edge cell to account for
        // discretisation error at cell boundaries.
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            const int nx = cx + dx;
            const int ny = cy + dy;

            if (nx < 0 || nx >= map.width || ny < 0 || ny >= map.height)
            {
              return true;
            }

            const std::size_t idx =
                static_cast<std::size_t>(ny) * static_cast<std::size_t>(map.width) + static_cast<std::size_t>(nx);

            // Unknown cells (value -1 in nav_msgs convention) are treated as
            // occupied for safety — we cannot assume clear space we have no
            // information about.
            if (map.data[idx] < 0 || map.data[idx] > kOccupancyThreshold)
            {
              return true;
            }
          }
        }
      }
    }

    d += 1.0;
  }

  return false;
}

}  // namespace stdr_simulation::collision
