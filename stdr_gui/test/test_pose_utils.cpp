#include <stdr_gui/pose_utils.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace stdr_gui
{
namespace
{

constexpr double kTol = 1e-10;

// ---- Identity and pure translation ------------------------------------------

TEST(TransformToWorldTest, IdentityParentPreservesChild)
{
  // Parent at origin with zero orientation: child offset is returned unchanged.
  const stdr_simulation::Pose2D parent{ 0.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D child{ 1.0, 2.0, 0.5 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 1.0, kTol);
  EXPECT_NEAR(result.y, 2.0, kTol);
  EXPECT_NEAR(result.theta, 0.5, kTol);
}

TEST(TransformToWorldTest, TranslationOnly)
{
  // Parent translated, no rotation: world = parent + child offset.
  const stdr_simulation::Pose2D parent{ 3.0, 4.0, 0.0 };
  const stdr_simulation::Pose2D child{ 1.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 4.0, kTol);
  EXPECT_NEAR(result.y, 4.0, kTol);
  EXPECT_NEAR(result.theta, 0.0, kTol);
}

// ---- Rotation ---------------------------------------------------------------

TEST(TransformToWorldTest, Rotation90Degrees)
{
  // Parent at origin facing pi/2: child at (1,0) rotates to (0,1).
  const stdr_simulation::Pose2D parent{ 0.0, 0.0, std::numbers::pi / 2.0 };
  const stdr_simulation::Pose2D child{ 1.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 0.0, kTol);
  EXPECT_NEAR(result.y, 1.0, kTol);
  EXPECT_NEAR(result.theta, std::numbers::pi / 2.0, kTol);
}

TEST(TransformToWorldTest, Rotation90WithOffset)
{
  // Parent at (1,2) facing pi/2: child at (1,0) maps to (1,3).
  const stdr_simulation::Pose2D parent{ 1.0, 2.0, std::numbers::pi / 2.0 };
  const stdr_simulation::Pose2D child{ 1.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 1.0, kTol);
  EXPECT_NEAR(result.y, 3.0, kTol);
  EXPECT_NEAR(result.theta, std::numbers::pi / 2.0, kTol);
}

// ---- General transform -------------------------------------------------------

TEST(TransformToWorldTest, FullTransform)
{
  // Parent at (1,2,pi/4), child at (1,1,pi/4).
  // cos(pi/4) = sin(pi/4) = sqrt(2)/2.
  // x = 1 + 1*(sqrt(2)/2) - 1*(sqrt(2)/2) = 1
  // y = 2 + 1*(sqrt(2)/2) + 1*(sqrt(2)/2) = 2 + sqrt(2)
  // theta = pi/4 + pi/4 = pi/2
  const stdr_simulation::Pose2D parent{ 1.0, 2.0, std::numbers::pi / 4.0 };
  const stdr_simulation::Pose2D child{ 1.0, 1.0, std::numbers::pi / 4.0 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 1.0, kTol);
  EXPECT_NEAR(result.y, 2.0 + std::sqrt(2.0), kTol);
  EXPECT_NEAR(result.theta, std::numbers::pi / 2.0, kTol);
}

TEST(TransformToWorldTest, NegativeAngles)
{
  // Parent at origin facing -pi/2: child at (1,0) rotates to (0,-1).
  const stdr_simulation::Pose2D parent{ 0.0, 0.0, -std::numbers::pi / 2.0 };
  const stdr_simulation::Pose2D child{ 1.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.x, 0.0, kTol);
  EXPECT_NEAR(result.y, -1.0, kTol);
  EXPECT_NEAR(result.theta, -std::numbers::pi / 2.0, kTol);
}

// ---- Orientation addition ---------------------------------------------------

TEST(TransformToWorldTest, ChildOrientationAddsToParent)
{
  // Theta must equal parent.theta + child.theta regardless of position.
  const stdr_simulation::Pose2D parent{ 0.0, 0.0, 1.2 };
  const stdr_simulation::Pose2D child{ 0.0, 0.0, 0.8 };
  const stdr_simulation::Pose2D result = transform_to_world(parent, child);
  EXPECT_NEAR(result.theta, 2.0, kTol);
}

}  // namespace
}  // namespace stdr_gui
