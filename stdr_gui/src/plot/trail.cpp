#include <stdr_gui/plot/trail.hpp>

#include <cmath>

namespace stdr::plot
{

void Trail::push_if_moved(const stdr_simulation::Pose2D& pose)
{
  if (points.empty())
  {
    points.push_back(pose);
    return;
  }

  const stdr_simulation::Pose2D& last = points.back();
  const double dx = pose.x - last.x;
  const double dy = pose.y - last.y;
  if (std::hypot(dx, dy) < kMarkerSpacingMeters)
  {
    return;
  }

  // pop_front is O(1) on std::deque, avoiding the O(n) erase(begin()) that a
  // std::vector-based trail would require.
  if (points.size() >= kMaxMarkers)
  {
    points.pop_front();
  }
  points.push_back(pose);
}

void Trail::clear()
{
  points.clear();
}

}  // namespace stdr::plot
