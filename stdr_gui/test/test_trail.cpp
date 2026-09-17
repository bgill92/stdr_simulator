#include <stdr_gui/plot/trail.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace stdr::plot
{
namespace
{

using ::testing::SizeIs;

TEST(Trail, FirstPointIsAlwaysAccepted)
{
  Trail trail;
  trail.push_if_moved(stdr_simulation::Pose2D{ 0.0, 0.0, 0.0 });
  ASSERT_THAT(trail.points, SizeIs(1));
}

TEST(Trail, PointBelowSpacingIsDropped)
{
  Trail trail;
  trail.push_if_moved(stdr_simulation::Pose2D{ 0.0, 0.0, 0.0 });
  // Half of kMarkerSpacingMeters — must not be accepted.
  trail.push_if_moved(stdr_simulation::Pose2D{ kMarkerSpacingMeters / 2.0, 0.0, 0.0 });
  EXPECT_THAT(trail.points, SizeIs(1));
}

TEST(Trail, PointAtOrAboveSpacingIsAccepted)
{
  Trail trail;
  trail.push_if_moved(stdr_simulation::Pose2D{ 0.0, 0.0, 0.0 });
  trail.push_if_moved(stdr_simulation::Pose2D{ kMarkerSpacingMeters, 0.0, 0.0 });
  ASSERT_THAT(trail.points, SizeIs(2));
  EXPECT_DOUBLE_EQ(trail.points.back().x, kMarkerSpacingMeters);
}

TEST(Trail, CapIsRespected)
{
  Trail trail;
  // Push kMaxMarkers + 10 points spaced well above kMarkerSpacingMeters apart
  // so every push is accepted, then verify the cap holds and the oldest
  // points were dropped in favor of the newest.
  for (std::size_t i = 0; i < kMaxMarkers + 10; ++i)
  {
    trail.push_if_moved(stdr_simulation::Pose2D{ static_cast<double>(i) * kMarkerSpacingMeters * 2.0, 0.0, 0.0 });
  }
  ASSERT_THAT(trail.points, SizeIs(kMaxMarkers));
  EXPECT_DOUBLE_EQ(trail.points.back().x, static_cast<double>(kMaxMarkers + 9) * kMarkerSpacingMeters * 2.0);
}

TEST(Trail, ClearEmptiesTheTrail)
{
  Trail trail;
  trail.push_if_moved(stdr_simulation::Pose2D{ 0.0, 0.0, 0.0 });
  trail.clear();
  EXPECT_THAT(trail.points, SizeIs(0));
}

}  // namespace
}  // namespace stdr::plot
