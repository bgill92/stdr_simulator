#include <stdr_simulation/motion/ideal_motion_model.hpp>
#include <stdr_simulation/motion/omni_motion_model.hpp>
#include <stdr_simulation/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <numbers>

namespace stdr_simulation::motion
{
namespace
{

using ::testing::DoubleNear;

// All alpha coefficients zero → deterministic, noise-free kinematics.
KinematicConfig zero_noise_config()
{
  return KinematicConfig{};
}

constexpr double kDt = 0.1;
constexpr double kTol = 1e-9;

// ---- IdealMotionModel -------------------------------------------------------

TEST(IdealMotionModelTest, StationaryRobotStaysStill)
{
  const IdealMotionModel model;
  const Pose2D start{ 1.0, 2.0, 0.5 };
  const Twist2D cmd{ 0.0, 0.0, 0.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  EXPECT_THAT(result.x, DoubleNear(start.x, kTol));
  EXPECT_THAT(result.y, DoubleNear(start.y, kTol));
  EXPECT_THAT(result.theta, DoubleNear(start.theta, kTol));
}

TEST(IdealMotionModelTest, StraightLineMotion)
{
  const IdealMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pure forward motion along x-axis.
  const Twist2D cmd{ 1.0, 0.0, 0.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  EXPECT_THAT(result.x, DoubleNear(0.1, kTol));
  EXPECT_THAT(result.y, DoubleNear(0.0, kTol));
  EXPECT_THAT(result.theta, DoubleNear(0.0, kTol));
}

TEST(IdealMotionModelTest, PureRotation)
{
  const IdealMotionModel model;
  const Pose2D start{ 1.0, 1.0, 0.0 };
  // Pure spin — position must not change.
  const Twist2D cmd{ 0.0, 0.0, 1.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  // For a differential drive with v=0 and w≠0, exact arc kinematics produce
  // a degenerate arc (radius → ∞) so position stays fixed.
  EXPECT_THAT(result.x, DoubleNear(start.x, kTol));
  EXPECT_THAT(result.y, DoubleNear(start.y, kTol));
  EXPECT_THAT(result.theta, DoubleNear(0.1, kTol));
}

TEST(IdealMotionModelTest, ArcMotion)
{
  const IdealMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Combined linear and angular → robot traces an arc.
  const Twist2D cmd{ 1.0, 0.0, 1.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  // Both position components and heading must change.
  EXPECT_THAT(result.x, DoubleNear(0.0, 0.2));  // x moved but stays near origin.
  EXPECT_THAT(result.theta, DoubleNear(0.1, kTol));
  // y should be non-zero because the arc curves.
  EXPECT_GT(result.x, 0.0);
}

TEST(IdealMotionModelTest, ThetaNormalized)
{
  const IdealMotionModel model;
  const Pose2D start{ 0.0, 0.0, std::numbers::pi - 0.01 };
  // A large angular step that pushes theta past π.
  const Twist2D cmd{ 0.0, 0.0, 10.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  EXPECT_GE(result.theta, -std::numbers::pi);
  EXPECT_LE(result.theta, std::numbers::pi);
}

// ---- OmniMotionModel --------------------------------------------------------

TEST(OmniMotionModelTest, StationaryRobotStaysStill)
{
  const OmniMotionModel model;
  const Pose2D start{ 3.0, -1.5, 0.8 };
  const Twist2D cmd{ 0.0, 0.0, 0.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  EXPECT_THAT(result.x, DoubleNear(start.x, kTol));
  EXPECT_THAT(result.y, DoubleNear(start.y, kTol));
  EXPECT_THAT(result.theta, DoubleNear(start.theta, kTol));
}

TEST(OmniMotionModelTest, LateralMotion)
{
  const OmniMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pure lateral command (strafe).  With theta=0 the y-axis aligns with
  // the robot's lateral direction, so y must increase.
  const Twist2D cmd{ 0.0, 1.0, 0.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  EXPECT_THAT(result.x, DoubleNear(0.0, kTol));
  EXPECT_GT(result.y, 0.0);
  EXPECT_THAT(result.theta, DoubleNear(0.0, kTol));
}

TEST(OmniMotionModelTest, CombinedMotion)
{
  const OmniMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  const Twist2D cmd{ 1.0, 1.0, 1.0 };
  const Pose2D result = model.update(start, cmd, kDt, zero_noise_config());
  // All three DOF must change from zero.
  EXPECT_GT(result.x, 0.0);
  EXPECT_GT(result.y, 0.0);
  EXPECT_GT(result.theta, 0.0);
}

// ---- Center-of-rotation tests: IdealMotionModel -----------------------------

// Zero pivot reproduces pre-existing results exactly.
TEST(IdealMotionModelCenterOfRotationTest, ZeroPivotMatchesDefaultBehaviour)
{
  const IdealMotionModel model;
  const Pose2D start{ 1.0, 2.0, 0.3 };
  const Twist2D cmd{ 1.0, 0.0, 0.5 };
  const Point2D zero_pivot{ 0.0, 0.0 };
  // update() with explicit zero pivot must produce an identical result to
  // calling update() without the parameter.
  const Pose2D without_pivot = model.update(start, cmd, kDt, zero_noise_config());
  const Pose2D with_zero_pivot = model.update(start, cmd, kDt, zero_noise_config(), zero_pivot);
  EXPECT_THAT(with_zero_pivot.x, DoubleNear(without_pivot.x, kTol));
  EXPECT_THAT(with_zero_pivot.y, DoubleNear(without_pivot.y, kTol));
  EXPECT_THAT(with_zero_pivot.theta, DoubleNear(without_pivot.theta, kTol));
}

// Straight-line motion (zero angular velocity): center_of_rotation has no effect.
// Body-origin path must be identical regardless of the pivot offset.
TEST(IdealMotionModelCenterOfRotationTest, StraightLineUnaffectedByPivot)
{
  const IdealMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pure forward motion — no rotation, so pivot is irrelevant.
  const Twist2D cmd{ 1.0, 0.0, 0.0 };
  const Point2D pivot{ 0.5, 0.3 };
  const Pose2D without_pivot = model.update(start, cmd, kDt, zero_noise_config());
  const Pose2D with_pivot = model.update(start, cmd, kDt, zero_noise_config(), pivot);
  EXPECT_THAT(with_pivot.x, DoubleNear(without_pivot.x, kTol));
  EXPECT_THAT(with_pivot.y, DoubleNear(without_pivot.y, kTol));
  EXPECT_THAT(with_pivot.theta, DoubleNear(without_pivot.theta, kTol));
}

// Offset pivot, pure rotation (v=0, w≠0): the body origin sweeps an arc of
// radius |c| around the world-frame pivot point, so the origin-to-pivot
// distance must remain constant and the origin must actually move.
TEST(IdealMotionModelCenterOfRotationTest, PureRotationAboutOffsetPivotSweepsArc)
{
  const IdealMotionModel model;
  // Robot starts at origin facing along +x.
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pivot is 1 m ahead in body frame — world pivot starts at (1, 0).
  const Point2D pivot{ 1.0, 0.0 };
  // Pure spin — differential drive with v=0 keeps the pivot fixed in world frame.
  const Twist2D cmd{ 0.0, 0.0, 1.0 };

  // World-frame pivot position does not move during pure rotation (v=0).
  const double pivot_world_x = start.x + pivot.x;  // = 1.0
  const double pivot_world_y = start.y + pivot.y;  // = 0.0

  // Run several steps to get a measurable arc displacement.
  Pose2D pose = start;
  for (int step = 0; step < 10; ++step)
  {
    pose = model.update(pose, cmd, kDt, zero_noise_config(), pivot);
  }

  // Body origin must have moved.
  EXPECT_GT(std::hypot(pose.x - start.x, pose.y - start.y), 1e-6);

  // Distance from body origin to the world-frame pivot must equal |pivot| = 1.
  const double dist_to_pivot = std::hypot(pose.x - pivot_world_x, pose.y - pivot_world_y);
  EXPECT_THAT(dist_to_pivot, DoubleNear(1.0, 1e-9));
}

// ---- Center-of-rotation tests: OmniMotionModel ------------------------------

// Zero pivot reproduces pre-existing results exactly.
TEST(OmniMotionModelCenterOfRotationTest, ZeroPivotMatchesDefaultBehaviour)
{
  const OmniMotionModel model;
  const Pose2D start{ 3.0, -1.5, 0.8 };
  const Twist2D cmd{ 1.0, 0.5, 0.4 };
  const Point2D zero_pivot{ 0.0, 0.0 };
  const Pose2D without_pivot = model.update(start, cmd, kDt, zero_noise_config());
  const Pose2D with_zero_pivot = model.update(start, cmd, kDt, zero_noise_config(), zero_pivot);
  EXPECT_THAT(with_zero_pivot.x, DoubleNear(without_pivot.x, kTol));
  EXPECT_THAT(with_zero_pivot.y, DoubleNear(without_pivot.y, kTol));
  EXPECT_THAT(with_zero_pivot.theta, DoubleNear(without_pivot.theta, kTol));
}

// Straight-line motion (zero angular velocity): pivot has no effect.
TEST(OmniMotionModelCenterOfRotationTest, StraightLineUnaffectedByPivot)
{
  const OmniMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pure forward + lateral — no rotation.
  const Twist2D cmd{ 1.0, 0.5, 0.0 };
  const Point2D pivot{ 0.4, -0.2 };
  const Pose2D without_pivot = model.update(start, cmd, kDt, zero_noise_config());
  const Pose2D with_pivot = model.update(start, cmd, kDt, zero_noise_config(), pivot);
  EXPECT_THAT(with_pivot.x, DoubleNear(without_pivot.x, kTol));
  EXPECT_THAT(with_pivot.y, DoubleNear(without_pivot.y, kTol));
  EXPECT_THAT(with_pivot.theta, DoubleNear(without_pivot.theta, kTol));
}

// Offset pivot, pure rotation (vx=0, vy=0, w≠0): body origin sweeps an arc
// of radius |c| around the fixed world-frame pivot point.
TEST(OmniMotionModelCenterOfRotationTest, PureRotationAboutOffsetPivotSweepsArc)
{
  const OmniMotionModel model;
  const Pose2D start{ 0.0, 0.0, 0.0 };
  // Pivot is 1 m to the side (y direction) in body frame — world pivot = (0, 1).
  const Point2D pivot{ 0.0, 1.0 };
  const Twist2D cmd{ 0.0, 0.0, 1.0 };

  // pivot.x == 0 here, so the general body_to_pivot_pose formula simplifies:
  // world_x = body_x + pivot.x*cos(θ) - pivot.y*sin(θ) → start.x - pivot.y*sin(θ).
  // world_y = body_y + pivot.x*sin(θ) + pivot.y*cos(θ) → start.y + pivot.y*cos(θ).
  const double pivot_world_x = start.x - pivot.y * std::sin(start.theta);  // = 0.0
  const double pivot_world_y = start.y + pivot.y * std::cos(start.theta);  // = 1.0

  Pose2D pose = start;
  for (int step = 0; step < 10; ++step)
  {
    pose = model.update(pose, cmd, kDt, zero_noise_config(), pivot);
  }

  // Body origin must have moved.
  EXPECT_GT(std::hypot(pose.x - start.x, pose.y - start.y), 1e-6);

  // Distance from body origin to the world-frame pivot must equal |pivot| = 1.
  const double dist_to_pivot = std::hypot(pose.x - pivot_world_x, pose.y - pivot_world_y);
  EXPECT_THAT(dist_to_pivot, DoubleNear(1.0, 1e-9));
}

}  // namespace
}  // namespace stdr_simulation::motion
