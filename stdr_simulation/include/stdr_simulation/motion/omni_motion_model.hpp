#pragma once

/** @file Omnidirectional motion model with velocity-dependent noise. */

#include <stdr_simulation/types.hpp>

#include <cstdint>
#include <random>

namespace stdr_simulation::motion
{

/**
 * @brief Holonomic (omnidirectional) motion model with optional Gaussian velocity noise.
 *
 * Integrates a full Twist2D command — including lateral linear_y — over a
 * timestep using decoupled x/y/theta kinematics.  Noise follows the same
 * velocity-dependent model as IdealMotionModel; when all KinematicConfig
 * alpha coefficients are zero the model is noiseless.
 *
 * @warning Not thread-safe — each thread should own its own instance because
 *          the internal RNG is mutated on every call to update(). */
class OmniMotionModel
{
public:
  OmniMotionModel();

  /**
   * @brief Construct with a fixed RNG seed for deterministic, repeatable
   * noise draws — intended for tests that need to distinguish Velocity-mode
   * output from integrate() reproducibly rather than statistically.
   */
  explicit OmniMotionModel(std::uint32_t seed);

  ~OmniMotionModel() = default;

  /**
   * @brief Propagate a pose forward by one timestep under a velocity command.
   *
   * @param current             The robot's current 2D pose.
   * @param cmd                 Commanded velocity (all three components are used).
   * @param dt                  Timestep in seconds.
   * @param noise_params        KinematicConfig encoding the alpha noise coefficients.
   * @param center_of_rotation  Rotation pivot expressed in the robot body frame.
   *                            Defaults to (0, 0) — the body origin — which
   *                            reproduces the previous behaviour exactly.
   * @return                    The predicted next pose.
   */
  [[nodiscard]] Pose2D update(const Pose2D& current, const Twist2D& cmd, double dt, const KinematicConfig& noise_params,
                              const Point2D& center_of_rotation = {}) const;

  /**
   * @brief Noise-free decoupled integration of a command over one timestep.
   *
   * This is the same kinematics update() applies to its noisy twist, exposed
   * directly so callers can integrate a clean odometry belief pose alongside
   * a (possibly noisy) true pose from the same commanded velocity — see
   * SimulationEngine::step, which calls this on the odometry pose and
   * update() on the true pose every tick.
   *
   * @param current             The pose to integrate forward.
   * @param cmd                 Commanded velocity (all three components are used).
   * @param dt                  Timestep in seconds.
   * @param center_of_rotation  Rotation pivot expressed in the robot body frame.
   * @return                    The integrated next pose.
   */
  [[nodiscard]] Pose2D integrate(const Pose2D& current, const Twist2D& cmd, double dt,
                                 const Point2D& center_of_rotation = {}) const;

private:
  // Shared decoupled kinematics used by both update() (noisy) and integrate()
  // (noise-free) so the geometry lives in exactly one place.
  [[nodiscard]] static Pose2D integrate_kinematics(const Pose2D& current, double vx, double vy, double w, double dt,
                                                   const Point2D& center_of_rotation);

  // Mutable because the RNG is internal implementation state, not observable
  // robot state — callers see a logically const model.
  mutable std::mt19937 rng_;
};

}  // namespace stdr_simulation::motion
