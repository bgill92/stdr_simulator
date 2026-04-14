#include <stdr_simulation/collision/collision_checker.hpp>
#include <stdr_simulation/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace stdr_simulation::collision
{
namespace
{

using ::testing::IsFalse;
using ::testing::IsTrue;

// Builds a 10x10 grid at 0.1 m/cell with origin at (0,0).
// Cells listed in (col, row) pairs are marked occupied (value=100).
OccupancyGrid make_grid(const std::vector<std::pair<int, int>>& occupied_cells = {})
{
  constexpr int kSize = 10;
  constexpr double kResolution = 0.1;

  OccupancyGrid grid;
  grid.width = kSize;
  grid.height = kSize;
  grid.resolution = kResolution;
  grid.origin = { 0.0, 0.0, 0.0 };
  grid.data.assign(static_cast<std::size_t>(kSize * kSize), 0);

  for (const auto& [col, row] : occupied_cells)
  {
    grid.data[static_cast<std::size_t>(row * kSize + col)] = 100;
  }
  return grid;
}

// A circular footprint with 0.05 m radius — smaller than one cell so the
// robot can sit cleanly in a free cell without touching neighbours.
Footprint small_circular_footprint()
{
  Footprint fp;
  fp.radius = 0.05;
  return fp;
}

// ---- check_collision ---------------------------------------------------------

TEST(CollisionCheckerTest, FreeCellNoCollision)
{
  const CollisionChecker checker;
  const OccupancyGrid map = make_grid();
  const Pose2D pose{ 0.45, 0.45, 0.0 };  // Centre of free map.
  EXPECT_THAT(checker.check_collision(pose, small_circular_footprint(), map), IsFalse());
}

TEST(CollisionCheckerTest, OccupiedCellCollision)
{
  const CollisionChecker checker;
  // Cell (5,5) is the one the robot stands on.
  const OccupancyGrid map = make_grid({ { 5, 5 } });
  // Robot centre at (0.55, 0.55) — exactly cell (5,5) in a 0.1 m grid.
  const Pose2D pose{ 0.55, 0.55, 0.0 };
  EXPECT_THAT(checker.check_collision(pose, small_circular_footprint(), map), IsTrue());
}

TEST(CollisionCheckerTest, OutOfBoundsCollision)
{
  const CollisionChecker checker;
  const OccupancyGrid map = make_grid();
  // Well outside the 10×10 grid (grid covers [0,1) m).
  const Pose2D pose{ 5.0, 5.0, 0.0 };
  EXPECT_THAT(checker.check_collision(pose, small_circular_footprint(), map), IsTrue());
}

TEST(CollisionCheckerTest, ZeroResolutionReturnsCollision)
{
  const CollisionChecker checker;
  OccupancyGrid map = make_grid();
  map.resolution = 0.0;  // Degenerate — division by zero must not crash.
  const Pose2D pose{ 0.5, 0.5, 0.0 };
  // Safety: a degenerate map must report collision rather than silently pass.
  EXPECT_THAT(checker.check_collision(pose, small_circular_footprint(), map), IsTrue());
}

TEST(CollisionCheckerTest, EmptyMapReturnsCollision)
{
  const CollisionChecker checker;
  OccupancyGrid map;
  map.width = 0;
  map.height = 0;
  map.resolution = 0.1;
  const Pose2D pose{ 0.5, 0.5, 0.0 };
  // Safety: empty map means no known-free space.
  EXPECT_THAT(checker.check_collision(pose, small_circular_footprint(), map), IsTrue());
}

// ---- check_path_collision ----------------------------------------------------

TEST(CollisionCheckerTest, PathCollisionDetected)
{
  const CollisionChecker checker;
  // Place a wall of occupied cells across column 5 (x ≈ 0.55 m).
  std::vector<std::pair<int, int>> wall;
  for (int row = 0; row < 10; ++row)
  {
    wall.emplace_back(5, row);
  }
  const OccupancyGrid map = make_grid(wall);

  // Robot tries to move from x=0.25 to x=0.75, crossing the wall.
  const Pose2D previous{ 0.25, 0.45, 0.0 };
  const Pose2D next{ 0.75, 0.45, 0.0 };
  EXPECT_THAT(checker.check_path_collision(next, previous, small_circular_footprint(), map), IsTrue());
}

TEST(CollisionCheckerTest, PathNoCollision)
{
  const CollisionChecker checker;
  const OccupancyGrid map = make_grid();  // Entirely free.

  const Pose2D previous{ 0.15, 0.15, 0.0 };
  const Pose2D next{ 0.45, 0.45, 0.0 };
  EXPECT_THAT(checker.check_path_collision(next, previous, small_circular_footprint(), map), IsFalse());
}

}  // namespace
}  // namespace stdr_simulation::collision
