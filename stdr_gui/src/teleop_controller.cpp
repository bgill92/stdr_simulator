#include <stdr_gui/teleop_controller.hpp>

namespace stdr_gui
{

KinematicType kinematic_type_from_string(const std::string& type_str)
{
  // Only "omni" enables strafe; every other model (including
  // "differential" and empty string) falls back to Differential
  // so that unrecognised values do not accidentally enable lateral motion.
  if (type_str == "omni")
  {
    return KinematicType::Omni;
  }
  return KinematicType::Differential;
}

stdr_simulation::Twist2D compute_teleop_twist(const TeleopKeys& keys, KinematicType kinematics,
                                              const TeleopSpeeds& speeds)
{
  stdr_simulation::Twist2D twist{};

  if (keys.forward != keys.backward)
  {
    twist.linear_x = keys.forward ? speeds.linear : -speeds.linear;
  }

  if (keys.turn_left != keys.turn_right)
  {
    // Positive angular_z = CCW = turn left per REP-103.
    twist.angular_z = keys.turn_left ? speeds.angular : -speeds.angular;
  }

  if (kinematics == KinematicType::Omni && (keys.left != keys.right))
  {
    // Positive linear_y = left per REP-103.
    twist.linear_y = keys.left ? speeds.linear : -speeds.linear;
  }

  return twist;
}

stdr_simulation::Twist2D TeleopController::update(const TeleopKeys& keys, KinematicType kinematics) const
{
  return compute_teleop_twist(keys, kinematics, speeds_);
}

}  // namespace stdr_gui
