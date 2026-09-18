#include <stdr_gui/plot/helpers.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <stdr_simulation/types.hpp>
#include <stdr_simulation/world/world_model.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace stdr::plot::helpers
{
namespace
{

using ::testing::IsEmpty;
using ::testing::SizeIs;

// ---------------------------------------------------------------------------
// Minimal SimView stub — only robot_ids / laser_sensors / sonar_sensors
// need to work for iteration helper tests.  All other methods return safe
// defaults so the stub satisfies the abstract interface.
// ---------------------------------------------------------------------------

class FakeSimView : public SimView
{
public:
  explicit FakeSimView(std::vector<RobotId> robot_ids, std::vector<SensorId> lasers = {},
                       std::vector<SensorId> sonars = {})
    : robot_ids_(std::move(robot_ids)), lasers_(std::move(lasers)), sonars_(std::move(sonars))
  {
  }

  [[nodiscard]] std::size_t num_robots() const override
  {
    return robot_ids_.size();
  }
  [[nodiscard]] std::vector<RobotId> robot_ids() const override
  {
    return robot_ids_;
  }
  [[nodiscard]] std::vector<SensorId> laser_sensors(const RobotId& /*robot_id*/) const override
  {
    return lasers_;
  }
  [[nodiscard]] std::vector<SensorId> sonar_sensors(const RobotId& /*robot_id*/) const override
  {
    return sonars_;
  }
  [[nodiscard]] stdr_simulation::Pose2D pose(const RobotId& /*robot_id*/) const override
  {
    return {};
  }
  [[nodiscard]] stdr_simulation::Pose2D odom_pose(const RobotId& /*robot_id*/) const override
  {
    return {};
  }
  [[nodiscard]] stdr_simulation::Twist2D twist(const RobotId& /*robot_id*/) const override
  {
    return {};
  }
  [[nodiscard]] bool collided(const RobotId& /*robot_id*/) const override
  {
    return false;
  }
  [[nodiscard]] std::vector<stdr_simulation::Point2D> footprint(const RobotId& /*robot_id*/) const override
  {
    return {};
  }
  [[nodiscard]] stdr_simulation::Point2D center_of_rotation(const RobotId& /*robot_id*/) const override
  {
    return {};
  }
  [[nodiscard]] double sim_time() const override
  {
    return 0.0;
  }
  [[nodiscard]] const stdr_simulation::OccupancyGrid& map() const override
  {
    static const stdr_simulation::OccupancyGrid kEmpty{};
    return kEmpty;
  }
  [[nodiscard]] std::optional<stdr_simulation::LaserScan> latest_laser(const RobotId& /*robot_id*/,
                                                                       const SensorId& /*sensor_id*/) const override
  {
    return std::nullopt;
  }
  [[nodiscard]] std::optional<stdr_simulation::SonarScan> latest_sonar(const RobotId& /*robot_id*/,
                                                                       const SensorId& /*sensor_id*/) const override
  {
    return std::nullopt;
  }
  [[nodiscard]] std::optional<stdr_simulation::Pose2D> laser_pose(const RobotId& /*robot_id*/,
                                                                  const SensorId& /*sensor_id*/) const override
  {
    return std::nullopt;
  }
  [[nodiscard]] DrainedLaser new_laser_scans(const RobotId& /*robot_id*/, const SensorId& /*sensor_id*/) override
  {
    return {};
  }
  [[nodiscard]] DrainedSonar new_sonar_readings(const RobotId& /*robot_id*/, const SensorId& /*sensor_id*/) override
  {
    return {};
  }
  void cmd_velocity(const RobotId& /*robot_id*/, double /*v*/, double /*w*/) override
  {
  }
  void teleport(const RobotId& /*robot_id*/, stdr_simulation::Pose2D /*pose*/) override
  {
  }
  void pause() override
  {
  }
  void resume() override
  {
  }

private:
  std::vector<RobotId> robot_ids_;
  std::vector<SensorId> lasers_;
  std::vector<SensorId> sonars_;
};

// ---------------------------------------------------------------------------
// Unit conversion round-trips
// ---------------------------------------------------------------------------

TEST(PlotHelpers, RadDegRoundTrip)
{
  // Round-trip: rad_to_deg then deg_to_rad should recover the original value.
  constexpr double kInput = 1.23456789;
  const double recovered = deg_to_rad(rad_to_deg(kInput));
  EXPECT_NEAR(recovered, kInput, 1e-12);
}

TEST(PlotHelpers, MeterMillimeterRoundTrip)
{
  // Round-trip: m_to_mm then mm_to_m should recover the original value.
  constexpr double kInput = 3.14159;
  const double recovered = mm_to_m(m_to_mm(kInput));
  EXPECT_NEAR(recovered, kInput, 1e-12);
}

// ---------------------------------------------------------------------------
// wrapped_angle_diff
// ---------------------------------------------------------------------------

TEST(PlotHelpers, WrappedAngleDiffZeroWhenEqual)
{
  EXPECT_NEAR(wrapped_angle_diff(0.5, 0.5), 0.0, 1e-10);
}

TEST(PlotHelpers, WrappedAngleDiffPositiveWrap)
{
  // 0.9*pi - (-0.9*pi) = 1.8*pi, which wraps to 1.8*pi - 2*pi = -0.2*pi.
  const double result = wrapped_angle_diff(0.9 * std::numbers::pi, -0.9 * std::numbers::pi);
  EXPECT_NEAR(result, -0.2 * std::numbers::pi, 1e-10);
}

TEST(PlotHelpers, WrappedAngleDiffNegativeWrap)
{
  // -0.9*pi - 0.9*pi = -1.8*pi, which wraps to -1.8*pi + 2*pi = 0.2*pi.
  const double result = wrapped_angle_diff(-0.9 * std::numbers::pi, 0.9 * std::numbers::pi);
  EXPECT_NEAR(result, 0.2 * std::numbers::pi, 1e-10);
}

TEST(PlotHelpers, WrappedAngleDiffFullTurnIsZero)
{
  // A difference of exactly 2*pi must wrap back to 0, not stay at 2*pi.
  const double result = wrapped_angle_diff(2.0 * std::numbers::pi, 0.0);
  EXPECT_NEAR(result, 0.0, 1e-10);
}

TEST(PlotHelpers, WrappedAngleDiffExactlyPositivePi)
{
  // The documented convention is (-pi, pi], so a diff of exactly +pi should
  // land on the positive boundary. On this toolchain's libm,
  // atan2(sin(pi), cos(pi)) returns std::numbers::pi bit-for-bit; the
  // tolerance is only a defensive margin in case another libm does not
  // represent sin(pi) as exactly 0.
  const double result = wrapped_angle_diff(std::numbers::pi, 0.0);
  EXPECT_NEAR(result, std::numbers::pi, 1e-10);
}

TEST(PlotHelpers, WrappedAngleDiffExactlyNegativePi)
{
  // Symmetric to the +pi case: on this toolchain's libm,
  // atan2(sin(-pi), cos(-pi)) returns -std::numbers::pi bit-for-bit. -pi
  // itself is technically excluded by the (-pi, pi] convention, but this
  // documents the observed behaviour rather than the theoretical boundary.
  const double result = wrapped_angle_diff(-std::numbers::pi, 0.0);
  EXPECT_NEAR(result, -std::numbers::pi, 1e-10);
}

// ---------------------------------------------------------------------------
// Coordinate transforms
// ---------------------------------------------------------------------------

TEST(PlotHelpers, MapToRobotIdentityAtOrigin)
{
  // Robot at origin, zero heading: map frame == robot frame.
  const stdr_simulation::Pose2D robot_pose{ 0.0, 0.0, 0.0 };
  const stdr_simulation::Pose2D p_map{ 3.0, 4.0, 0.5 };
  const stdr_simulation::Pose2D result = map_to_robot(p_map, robot_pose);
  EXPECT_NEAR(result.x, 3.0, 1e-10);
  EXPECT_NEAR(result.y, 4.0, 1e-10);
  EXPECT_NEAR(result.theta, 0.5, 1e-10);
}

TEST(PlotHelpers, MapToRobotKnownPose)
{
  // Robot at (1, 2) facing pi/2 (pointing in the +Y map direction).
  // A map point at (2, 2) is (1, 0) ahead of the robot in map X, which in
  // the robot frame lies along the robot's right side (robot Y negative).
  //
  // Hand calculation:
  //   dx = 2 - 1 = 1,  dy = 2 - 2 = 0
  //   cos(pi/2) = 0,  sin(pi/2) = 1
  //   robot.x =  0*1 + 1*0 =  0
  //   robot.y = -1*1 + 0*0 = -1
  const stdr_simulation::Pose2D robot_in_map{ 1.0, 2.0, std::numbers::pi / 2.0 };
  const stdr_simulation::Pose2D p_map{ 2.0, 2.0, 0.0 };
  const stdr_simulation::Pose2D result = map_to_robot(p_map, robot_in_map);
  EXPECT_NEAR(result.x, 0.0, 1e-10);
  EXPECT_NEAR(result.y, -1.0, 1e-10);
  // Theta: input 0 minus robot heading pi/2 = -pi/2.
  EXPECT_NEAR(result.theta, -std::numbers::pi / 2.0, 1e-10);
}

TEST(PlotHelpers, RobotToMapInvertsMapToRobot)
{
  // Round-trip: converting to robot frame then back to map must recover input.
  const stdr_simulation::Pose2D robot_in_map{ 3.0, -1.5, 0.7 };
  const stdr_simulation::Pose2D p_map{ 5.0, 2.0, 1.1 };
  const stdr_simulation::Pose2D in_robot = map_to_robot(p_map, robot_in_map);
  const stdr_simulation::Pose2D recovered = robot_to_map(in_robot, robot_in_map);
  EXPECT_NEAR(recovered.x, p_map.x, 1e-10);
  EXPECT_NEAR(recovered.y, p_map.y, 1e-10);
  EXPECT_NEAR(recovered.theta, p_map.theta, 1e-10);
}

// ---------------------------------------------------------------------------
// scan_to_map_points
// ---------------------------------------------------------------------------

TEST(PlotHelpers, ScanToMapPointsEmptyScanIsEmpty)
{
  const stdr_simulation::LaserScan scan{};
  const std::vector<Point2> points = scan_to_map_points(scan, {}, {});
  EXPECT_THAT(points, IsEmpty());
}

TEST(PlotHelpers, ScanToMapPointsSkipsOutOfRangeAndNonFiniteRays)
{
  // All rays share angle 0 (angle_increment = 0) so only the range value
  // determines which ray survives filtering.
  stdr_simulation::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.1;
  scan.range_max = 5.0;
  scan.ranges = { 0.05F, 6.0F, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), 2.0F };

  const std::vector<Point2> points = scan_to_map_points(scan, {}, {});
  ASSERT_THAT(points, SizeIs(1));
  EXPECT_NEAR(points[0].x, 2.0, 1e-9);
  EXPECT_NEAR(points[0].y, 0.0, 1e-9);
}

TEST(PlotHelpers, ScanToMapPointsSingleRayAtIdentityPosesLandsOnXAxis)
{
  stdr_simulation::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.0;
  scan.range_max = 10.0;
  scan.ranges = { 3.0F };

  const std::vector<Point2> points = scan_to_map_points(scan, {}, {});
  ASSERT_THAT(points, SizeIs(1));
  EXPECT_NEAR(points[0].x, 3.0, 1e-9);
  EXPECT_NEAR(points[0].y, 0.0, 1e-9);
}

TEST(PlotHelpers, ScanToMapPointsRotatedRobotWithOffsetLaser)
{
  // Robot rotated 90 degrees at the map origin; laser mounted 0.2 m ahead
  // along the robot's local X axis with no mount rotation.
  //
  // Hand calculation:
  //   sensor_in_map = robot_to_map(laser_in_robot={0.2,0,0}, robot_in_map={0,0,pi/2})
  //     cos(pi/2) = 0, sin(pi/2) = 1
  //     sensor.x = 0 + 0*0.2 - 1*0 = 0
  //     sensor.y = 0 + 1*0.2 + 0*0 = 0.2
  //     sensor.theta = 0 + pi/2 = pi/2
  //   ray angle = sensor.theta + angle_min(0) = pi/2
  //   point.x = sensor.x + 1.0*cos(pi/2) = 0
  //   point.y = sensor.y + 1.0*sin(pi/2) = 0.2 + 1.0 = 1.2
  const stdr_simulation::Pose2D laser_in_robot{ 0.2, 0.0, 0.0 };
  const stdr_simulation::Pose2D robot_in_map{ 0.0, 0.0, std::numbers::pi / 2.0 };

  stdr_simulation::LaserScan scan;
  scan.angle_min = 0.0;
  scan.angle_increment = 0.0;
  scan.range_min = 0.0;
  scan.range_max = 10.0;
  scan.ranges = { 1.0F };

  const std::vector<Point2> points = scan_to_map_points(scan, laser_in_robot, robot_in_map);
  ASSERT_THAT(points, SizeIs(1));
  EXPECT_NEAR(points[0].x, 0.0, 1e-9);
  EXPECT_NEAR(points[0].y, 1.2, 1e-9);
}

// ---------------------------------------------------------------------------
// should_sample
// ---------------------------------------------------------------------------

TEST(PlotHelpers, ShouldSampleFiresAtPeriod)
{
  // Period of 0.1 s.  Three sim times: 0.0, 0.05, 0.1.
  //   t=0.0: 0.0 - (-inf) would fire on the first call since last was never set.
  //           We initialise last_t to a value before t=0.0 to force the first fire.
  //   t=0.0 fires (delta = 0.0 >= 0.1 only if last was set to -0.1)
  //
  // Simpler: initialise last_t to -1.0 so the very first call fires, then
  // verify the second (at 0.05, delta=0.05) does not, and the third (at 0.1,
  // delta=0.1) does.
  constexpr double kPeriod = 0.1;
  double last_t = -1.0;

  // First call at t=0.0: delta = 0.0 - (-1.0) = 1.0 >= 0.1 → fires.
  EXPECT_TRUE(should_sample(0.0, last_t, kPeriod));
  EXPECT_DOUBLE_EQ(last_t, 0.0);

  // Second call at t=0.05: delta = 0.05 - 0.0 = 0.05 < 0.1 → does not fire.
  EXPECT_FALSE(should_sample(0.05, last_t, kPeriod));
  EXPECT_DOUBLE_EQ(last_t, 0.0);  // Unchanged.

  // Third call at t=0.1: delta = 0.1 - 0.0 = 0.1 >= 0.1 → fires.
  EXPECT_TRUE(should_sample(0.1, last_t, kPeriod));
  EXPECT_DOUBLE_EQ(last_t, 0.1);
}

TEST(PlotHelpers, ShouldSampleFiresWhenTimeGoesBackwards)
{
  // Simulation reset: now_sim_time (0.5) is earlier than last_sample_sim_time
  // (10.0).  Without the backwards check this would stall until sim_time
  // caught back up past 10.0 + period; instead it must fire immediately and
  // rebase last_sample_sim_time to the new, earlier time.
  constexpr double kPeriod = 1.0;
  double last_t = 10.0;

  EXPECT_TRUE(should_sample(0.5, last_t, kPeriod));
  EXPECT_DOUBLE_EQ(last_t, 0.5);
}

// ---------------------------------------------------------------------------
// Iteration helpers
// ---------------------------------------------------------------------------

TEST(PlotHelpers, ForEachRobotVisitsAllRobots)
{
  FakeSimView sim({ "robot0", "robot1" });

  std::vector<std::string> visited;
  for_each_robot(sim, [&visited](const RobotId& id) { visited.push_back(id); });

  ASSERT_THAT(visited, SizeIs(2));
  // Both robots must have been visited with distinct IDs.
  EXPECT_NE(visited[0], visited[1]);
  EXPECT_TRUE(std::find(visited.begin(), visited.end(), "robot0") != visited.end());
  EXPECT_TRUE(std::find(visited.begin(), visited.end(), "robot1") != visited.end());
}

TEST(PlotHelpers, ForEachLaserVisitsAllSensors)
{
  FakeSimView sim({ "robot0" }, { "laser0", "laser1" });

  std::vector<std::string> visited;
  for_each_laser(sim, "robot0", [&visited](const SensorId& id) { visited.push_back(id); });

  ASSERT_THAT(visited, SizeIs(2));
  EXPECT_EQ(visited[0], "laser0");
  EXPECT_EQ(visited[1], "laser1");
}

TEST(PlotHelpers, ForEachSonarVisitsAllSensors)
{
  FakeSimView sim({ "robot0" }, {}, { "sonar0", "sonar1" });

  std::vector<std::string> visited;
  for_each_sonar(sim, "robot0", [&visited](const SensorId& id) { visited.push_back(id); });

  ASSERT_THAT(visited, SizeIs(2));
  EXPECT_EQ(visited[0], "sonar0");
  EXPECT_EQ(visited[1], "sonar1");
}

// ---------------------------------------------------------------------------
// SimulationSnapshot::robot(name)
// ---------------------------------------------------------------------------

// Build a snapshot with two robots so lookup tests share the same setup.
stdr_gui::SimulationSnapshot make_snapshot_with_two_robots()
{
  stdr_gui::SimulationSnapshot snap;
  stdr_simulation::world::RobotState r0;
  r0.name = "robot0";
  r0.pose = { 1.0, 2.0, 0.0 };
  stdr_simulation::world::RobotState r1;
  r1.name = "robot1";
  r1.pose = { 3.0, 4.0, 0.5 };
  snap.robots.push_back(r0);
  snap.robots.push_back(r1);
  return snap;
}

TEST(SimulationSnapshot, RobotReturnsNullptrForMissing)
{
  const stdr_gui::SimulationSnapshot snap = make_snapshot_with_two_robots();
  EXPECT_EQ(snap.robot("does_not_exist"), nullptr);
}

TEST(SimulationSnapshot, RobotReturnsValidPointerForExisting)
{
  const stdr_gui::SimulationSnapshot snap = make_snapshot_with_two_robots();

  const stdr_simulation::world::RobotState* r = snap.robot("robot0");
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->name, "robot0");
  EXPECT_DOUBLE_EQ(r->pose.x, 1.0);
  EXPECT_DOUBLE_EQ(r->pose.y, 2.0);

  const stdr_simulation::world::RobotState* r1 = snap.robot("robot1");
  ASSERT_NE(r1, nullptr);
  EXPECT_EQ(r1->name, "robot1");
  EXPECT_DOUBLE_EQ(r1->pose.x, 3.0);
}

}  // namespace
}  // namespace stdr::plot::helpers
