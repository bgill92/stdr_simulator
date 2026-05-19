#pragma once

/** @file Geometric utilities for 2D pose composition and footprint queries. */

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

/**
 * @brief Convert a body-origin pose to the world pose of a pivot point.
 *
 * Given a robot body pose and a pivot offset expressed in the body frame,
 * returns the world-frame pose of that pivot.  The pivot's heading equals the
 * body heading — only the position differs.
 *
 * When @p pivot is the zero vector the returned pose equals @p body_pose
 * exactly, preserving bit-for-bit equivalence with the no-offset case.
 *
 * @param body_pose The robot's current pose in the world frame.
 * @param pivot     Pivot offset in the body (robot) frame.
 * @return          World-frame pose of the pivot point.
 */
[[nodiscard]] Pose2D body_to_pivot_pose(const Pose2D& body_pose, const Point2D& pivot);

/**
 * @brief Recover the body-origin pose from the world pose of the pivot point.
 *
 * Inverse of body_to_pivot_pose: given the new world-frame pivot pose after
 * integration and the (fixed) body-frame pivot offset, returns the new
 * body-origin world pose.
 *
 * When @p pivot is the zero vector the returned pose equals @p pivot_pose
 * exactly.
 *
 * @param pivot_pose World-frame pose of the pivot point after integration.
 * @param pivot      Pivot offset in the body (robot) frame.
 * @return           World-frame pose of the robot body origin.
 */
[[nodiscard]] Pose2D pivot_to_body_pose(const Pose2D& pivot_pose, const Point2D& pivot);

/**
 * @brief Test whether a point lies inside (or on the boundary of) a footprint.
 *
 * Both @p p and @p fp are expressed in the same frame (typically robot body
 * frame).  The footprint is treated as a circle when @c fp.points is empty,
 * and as a closed polygon otherwise.  On-boundary points are reported as
 * inside so that grazing contacts are not silently ignored during collision
 * checking.  Degenerate polygons (fewer than 3 vertices) return @c false —
 * they cannot enclose area and treating them as inside would be unsafe.
 *
 * @param p  Point to test.
 * @param fp Footprint (polygon or circle) in the same frame as @p p.
 * @return @c true if @p p is inside or on the boundary of @p fp.
 */
[[nodiscard]] bool point_in_footprint(const Point2D& p, const Footprint& fp);

}  // namespace stdr_simulation
