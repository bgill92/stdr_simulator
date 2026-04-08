#pragma once

/** @file Geometric utilities for 2D pose composition. */

#include <stdr_simulation/types.hpp>

namespace stdr_simulation
{

/**
 * @brief Compute the world-frame pose of a sensor mounted on a robot.
 *
 * Rotates the sensor's local offset by the robot heading before translating,
 * then adds the sensor's local orientation to the robot's heading.
 *
 * @param robot_pose The robot's pose in the world frame.
 * @param sensor_local The sensor's pose relative to the robot.
 * @return The sensor's pose in the world frame.
 */
[[nodiscard]] Pose2D compute_sensor_world_pose(const Pose2D& robot_pose, const Pose2D& sensor_local);

}  // namespace stdr_simulation
