#pragma once

namespace stdr_simulation::collision {

/**
 * @brief Checks whether a robot footprint intersects occupied cells in the world.
 *
 * Given a robot Pose2D and its footprint geometry, queries the WorldModel to
 * determine if any part of the footprint overlaps an obstacle.  Used by the
 * simulator to reject motion commands that would cause a collision.
 */
class CollisionChecker {
public:
  CollisionChecker() = default;
  ~CollisionChecker() = default;
};

}  // namespace stdr_simulation::collision
