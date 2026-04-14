#include <stdr_gui/teleop_controller.hpp>

#include <gtest/gtest.h>

namespace stdr_gui
{

TEST(TeleopControllerTest, NoKeysHeldReturnsZero)
{
  const TeleopKeys keys{};
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(TeleopControllerTest, ForwardProducesPositiveLinearX)
{
  const TeleopKeys keys{ .forward = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, speeds.linear);
  EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(TeleopControllerTest, BackwardProducesNegativeLinearX)
{
  const TeleopKeys keys{ .backward = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, -speeds.linear);
  EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(TeleopControllerTest, OpposingForwardBackwardCancel)
{
  const TeleopKeys keys{ .forward = true, .backward = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, 0.0);
}

TEST(TeleopControllerTest, TurnLeftRightMapToAngularZ)
{
  const TeleopSpeeds speeds{};
  {
    const TeleopKeys keys{ .turn_left = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
    EXPECT_DOUBLE_EQ(twist.angular_z, speeds.angular);
  }
  {
    const TeleopKeys keys{ .turn_right = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
    EXPECT_DOUBLE_EQ(twist.angular_z, -speeds.angular);
  }
}

TEST(TeleopControllerTest, OpposingTurnsCancel)
{
  const TeleopKeys keys{ .turn_left = true, .turn_right = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.angular_z, 0.0);
}

TEST(TeleopControllerTest, DifferentialIgnoresStrafe)
{
  const TeleopSpeeds speeds{};
  {
    const TeleopKeys keys{ .left = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
    EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
  }
  {
    const TeleopKeys keys{ .right = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
    EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
  }
}

TEST(TeleopControllerTest, OmniStrafeLeftProducesPositiveLinearY)
{
  const TeleopSpeeds speeds{};
  {
    const TeleopKeys keys{ .left = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Omni, speeds);
    EXPECT_DOUBLE_EQ(twist.linear_y, speeds.linear);
  }
  {
    const TeleopKeys keys{ .right = true };
    const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Omni, speeds);
    EXPECT_DOUBLE_EQ(twist.linear_y, -speeds.linear);
  }
}

TEST(TeleopControllerTest, OmniOpposingStrafesCancel)
{
  const TeleopKeys keys{ .left = true, .right = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Omni, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_y, 0.0);
}

TEST(TeleopControllerTest, CombinedForwardAndTurnProducesBoth)
{
  const TeleopKeys keys{ .forward = true, .turn_left = true };
  const TeleopSpeeds speeds{};
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, speeds.linear);
  EXPECT_DOUBLE_EQ(twist.angular_z, speeds.angular);
}

TEST(TeleopControllerTest, SpeedsScaleOutput)
{
  const TeleopKeys keys{ .forward = true, .turn_left = true };
  const TeleopSpeeds speeds{ .linear = 2.5, .angular = 3.0 };
  const stdr_simulation::Twist2D twist = compute_teleop_twist(keys, KinematicType::Differential, speeds);
  EXPECT_DOUBLE_EQ(twist.linear_x, 2.5);
  EXPECT_DOUBLE_EQ(twist.angular_z, 3.0);
}

TEST(TeleopControllerTest, KinematicTypeFromStringRecognizesOmni)
{
  EXPECT_EQ(kinematic_type_from_string("omni"), KinematicType::Omni);
}

TEST(TeleopControllerTest, KinematicTypeFromStringDefaultsToDifferential)
{
  EXPECT_EQ(kinematic_type_from_string("ideal"), KinematicType::Differential);
  EXPECT_EQ(kinematic_type_from_string(""), KinematicType::Differential);
  // Guard against the previous bug where "omni_drive" was accepted instead of "omni".
  EXPECT_EQ(kinematic_type_from_string("omni_drive"), KinematicType::Differential);
}

}  // namespace stdr_gui
