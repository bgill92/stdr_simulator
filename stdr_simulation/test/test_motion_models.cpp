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

}  // namespace
}  // namespace stdr_simulation::motion
