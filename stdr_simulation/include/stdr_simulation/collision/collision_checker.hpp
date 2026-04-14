#pragma once

#include <stdr_simulation/types.hpp>

#include <utility>
#include <vector>

namespace stdr_simulation::collision
{

/**
 * @brief Checks whether a robot footprint intersects occupied cells in the map.
 *
 * Given a robot Pose2D and its footprint geometry, determines if any part of
 * the footprint overlaps an obstacle in the OccupancyGrid. Used by the
 * simulator to reject motion commands that would cause a collision.
 *
 * All methods are stateless and safe to call from multiple threads
 * concurrently.
 */
class CollisionChecker
{
public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;

  /**
   * @brief Check if a pose collides with the map (single pose, no path).
   *
   * Rotates the footprint into world coordinates at the given pose and checks
   * every footprint point against the occupancy grid. Returns true on the first
   * occupied cell found, so it is suitable as an early-exit guard.
   *
   * @param pose      Robot pose in world coordinates.
   * @param footprint Robot footprint (polygon or circular).
   * @param map       Occupancy grid to query.
   * @return true if any footprint point lands on an occupied or out-of-bounds cell.
   */
  [[nodiscard]] bool check_collision(const Pose2D& pose, const Footprint& footprint, const OccupancyGrid& map) const;

  /**
   * @brief Check if moving from previous_pose to new_pose collides along the path.
   *
   * Interpolates between the two grid positions and sweeps the footprint edges
   * through all intermediate cells, checking a 3x3 neighbourhood around each.
   * More thorough than check_collision because it detects obstacles that a
   * single-pose check would miss when the step size is large relative to the
   * obstacle.
   *
   * @param new_pose      Candidate destination pose.
   * @param previous_pose Previous (known-safe) pose.
   * @param footprint     Robot footprint (polygon or circular).
   * @param map           Occupancy grid to query.
   * @return true if any cell along the swept path is occupied or out-of-bounds.
   */
  [[nodiscard]] bool check_path_collision(const Pose2D& new_pose, const Pose2D& previous_pose,
                                          const Footprint& footprint, const OccupancyGrid& map) const;

private:
  /** Occupancy value above which a cell is treated as an obstacle. */
  static constexpr int kOccupancyThreshold = 70;

  /**
   * @brief Expand a Footprint into a flat list of local-frame points.
   *
   * If the footprint polygon is empty, generates 360 points on the circumscribed
   * circle (1-degree spacing) so that circular robots are handled uniformly.
   *
   * @param footprint Footprint to expand.
   * @return Vector of local-frame 2D points.
   */
  [[nodiscard]] static std::vector<Point2D> expand_footprint(const Footprint& footprint);

  /**
   * @brief Enumerate all grid cells on the line segment from (x1,y1) to (x2,y2).
   *
   * Uses an angle-based stepping approach: computes the direction angle and
   * advances one unit at a time, collecting every cell touched.
   *
   * @return Ordered list of (col, row) grid cell pairs along the segment.
   */
  [[nodiscard]] static std::vector<std::pair<int, int>> get_points_between(int x1, int y1, int x2, int y2);
};

}  // namespace stdr_simulation::collision
