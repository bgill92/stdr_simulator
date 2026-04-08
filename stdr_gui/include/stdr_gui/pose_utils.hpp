#pragma once

#include <stdr_simulation/types.hpp>

namespace stdr_gui
{

/** @brief Transform a child pose from a parent's local frame to the world frame.
 *
 *  Applies a 2D rigid-body transform: rotates the child offset by the parent's
 *  orientation, translates by the parent's position, and adds orientations. */
[[nodiscard]] stdr_simulation::Pose2D transform_to_world(const stdr_simulation::Pose2D& parent,
                                                         const stdr_simulation::Pose2D& child_offset);

}  // namespace stdr_gui
