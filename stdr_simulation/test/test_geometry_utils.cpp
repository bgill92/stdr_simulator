#include <stdr_simulation/geometry_utils.hpp>
#include <stdr_simulation/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace stdr_simulation
{
namespace
{

using ::testing::DoubleNear;
using ::testing::IsFalse;
using ::testing::IsTrue;

constexpr double kTol = 1e-12;

// ---------------------------------------------------------------------------
// Footprint helpers
// ---------------------------------------------------------------------------

// Unit square polygon: vertices at (0,0), (1,0), (1,1), (0,1).
Footprint unit_square_footprint()
{
  Footprint fp;
  fp.points = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 1.0, 1.0 }, { 0.0, 1.0 } };
  return fp;
}

// Circle of radius 1.0 centred at the robot body-frame origin.
Footprint unit_circle_footprint()
{
  Footprint fp;
  fp.radius = 1.0;
  return fp;
}

// Polygon with only 2 vertices — degenerate.
Footprint degenerate_footprint()
{
  Footprint fp;
  fp.points = { { 0.0, 0.0 }, { 1.0, 0.0 } };
  return fp;
}

// ---------------------------------------------------------------------------
// Degenerate polygon tests (simplest — error path first)
// ---------------------------------------------------------------------------

TEST(PointInFootprintTest, DegeneratePolygonReturnsFalse)
{
  // A polygon with fewer than 3 vertices cannot enclose area; must not
  // silently report a point as inside.
  EXPECT_THAT(point_in_footprint({ 0.0, 0.0 }, degenerate_footprint()), IsFalse());
}

// ---------------------------------------------------------------------------
// Circle footprint tests
// ---------------------------------------------------------------------------

TEST(PointInFootprintTest, CircleInteriorPointIsInside)
{
  EXPECT_THAT(point_in_footprint({ 0.0, 0.0 }, unit_circle_footprint()), IsTrue());
}

TEST(PointInFootprintTest, CircleExteriorPointIsOutside)
{
  EXPECT_THAT(point_in_footprint({ 2.0, 0.0 }, unit_circle_footprint()), IsFalse());
}

TEST(PointInFootprintTest, CircleOnEdgePointIsInside)
{
  // Point exactly on the circumference must be treated as inside.
  EXPECT_THAT(point_in_footprint({ 1.0, 0.0 }, unit_circle_footprint()), IsTrue());
}

// ---------------------------------------------------------------------------
// Polygon footprint tests
// ---------------------------------------------------------------------------

TEST(PointInFootprintTest, PolygonInteriorPointIsInside)
{
  EXPECT_THAT(point_in_footprint({ 0.5, 0.5 }, unit_square_footprint()), IsTrue());
}

TEST(PointInFootprintTest, PolygonExteriorPointIsOutside)
{
  EXPECT_THAT(point_in_footprint({ 2.0, 2.0 }, unit_square_footprint()), IsFalse());
}

TEST(PointInFootprintTest, PolygonOnEdgePointIsInside)
{
  // Midpoint of the bottom edge of the unit square.
  EXPECT_THAT(point_in_footprint({ 0.5, 0.0 }, unit_square_footprint()), IsTrue());
}

TEST(PointInFootprintTest, PolygonOnCornerPointIsInside)
{
  // Corner vertex is a degenerate on-edge case that must also be inside.
  EXPECT_THAT(point_in_footprint({ 0.0, 0.0 }, unit_square_footprint()), IsTrue());
}

// ---------------------------------------------------------------------------
// body_to_pivot_pose / pivot_to_body_pose round-trip tests
// ---------------------------------------------------------------------------

// With a zero pivot offset, body_to_pivot_pose must return the body pose
// unchanged — the pivot coincides with the body origin.
TEST(BodyToPivotPoseTest, ZeroPivotReturnsPoseUnchanged)
{
  const Pose2D body{ 1.0, 2.0, 0.5 };
  const Point2D zero_pivot{ 0.0, 0.0 };
  const Pose2D result = body_to_pivot_pose(body, zero_pivot);
  EXPECT_THAT(result.x, DoubleNear(body.x, kTol));
  EXPECT_THAT(result.y, DoubleNear(body.y, kTol));
  EXPECT_THAT(result.theta, DoubleNear(body.theta, kTol));
}

// pivot_to_body_pose must be the exact inverse of body_to_pivot_pose: applying
// both in sequence must recover the original body pose for a non-trivial pose
// and a non-origin pivot.
TEST(PivotToBodyPoseTest, RoundTripRecoverOriginalPose)
{
  const Pose2D body{ 3.0, -1.5, 0.8 };
  const Point2D pivot{ 0.4, -0.2 };
  const Pose2D pivot_pose = body_to_pivot_pose(body, pivot);
  const Pose2D recovered = pivot_to_body_pose(pivot_pose, pivot);
  EXPECT_THAT(recovered.x, DoubleNear(body.x, kTol));
  EXPECT_THAT(recovered.y, DoubleNear(body.y, kTol));
  EXPECT_THAT(recovered.theta, DoubleNear(body.theta, kTol));
}

}  // namespace
}  // namespace stdr_simulation
