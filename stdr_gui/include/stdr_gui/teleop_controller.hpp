#pragma once

/** @file Pure-logic teleop velocity computation, decoupled from any GUI
 *  framework so it can be unit-tested without a window. */

#include <stdr_simulation/types.hpp>

#include <string>

namespace stdr_gui
{

/** @brief Kinematic model the teleop controller should respect. */
enum class KinematicType
{
  Differential,
  Omni
};

/** @brief Which teleop keys are currently held. */
struct TeleopKeys
{
  bool forward{ false };     // W / Up
  bool backward{ false };    // S / Down
  bool left{ false };        // A / Left  — strafe left (Omni) or no-op (Differential)
  bool right{ false };       // D / Right — strafe right (Omni) or no-op (Differential)
  bool turn_left{ false };   // Q
  bool turn_right{ false };  // E
};

/** @brief Configurable speed limits. */
struct TeleopSpeeds
{
  double linear{ 0.5 };   // m/s
  double angular{ 1.0 };  // rad/s
};

/** @brief Map a kinematic model type string from RobotConfig to KinematicType.
 *
 *  "omni" maps to Omni; any other string (including "differential"
 *  and empty) falls back to Differential so unrecognised models behave
 *  conservatively rather than enabling lateral motion. */
[[nodiscard]] KinematicType kinematic_type_from_string(const std::string& type_str);

/** @brief Compute a velocity command from key state and kinematics.
 *
 *  Rules:
 *   - forward/backward map to linear_x (+/-). Both held → 0.
 *   - turn_left/turn_right map to angular_z (+/-). Both held → 0.
 *   - For Omni, left/right map to linear_y (+/-). Both held → 0.
 *   - For Differential, left/right are ignored (strafe not supported). linear_y stays 0.
 *   - No keys held → all zeros. */
[[nodiscard]] stdr_simulation::Twist2D compute_teleop_twist(const TeleopKeys& keys, KinematicType kinematics,
                                                            const TeleopSpeeds& speeds);

/** @brief Thin stateful wrapper that owns TeleopSpeeds and delegates to
 *  compute_teleop_twist. The class exists so callers can update speed settings
 *  without threading them through every call site. */
class TeleopController
{
public:
  TeleopSpeeds& speeds()
  {
    return speeds_;
  }
  [[nodiscard]] const TeleopSpeeds& speeds() const
  {
    return speeds_;
  }

  [[nodiscard]] stdr_simulation::Twist2D update(const TeleopKeys& keys, KinematicType kinematics) const;

private:
  TeleopSpeeds speeds_{};
};

}  // namespace stdr_gui
