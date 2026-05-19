#include <stdr_simulation/sensors/co2_simulator.hpp>
#include <stdr_simulation/sensors/laser_simulator.hpp>
#include <stdr_simulation/sensors/rfid_simulator.hpp>
#include <stdr_simulation/sensors/sonar_simulator.hpp>
#include <stdr_simulation/types.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <vector>

namespace stdr_simulation::sensors
{
namespace
{

using ::testing::DoubleNear;
using ::testing::Each;
using ::testing::FloatNear;
using ::testing::Gt;
using ::testing::IsEmpty;
using ::testing::Lt;
using ::testing::SizeIs;

// Builds a 10×10 grid at 0.1 m/cell with walls on the four edges (col/row 0
// and 9) and free space in the interior.  The sensor sits in the centre.
OccupancyGrid make_walled_grid()
{
  constexpr int kSize = 10;
  constexpr double kRes = 0.1;

  OccupancyGrid grid;
  grid.width = kSize;
  grid.height = kSize;
  grid.resolution = kRes;
  grid.origin = { 0.0, 0.0, 0.0 };
  grid.data.assign(static_cast<std::size_t>(kSize * kSize), 0);

  for (int i = 0; i < kSize; ++i)
  {
    // Top and bottom rows.
    grid.data[static_cast<std::size_t>(0 * kSize + i)] = 100;
    grid.data[static_cast<std::size_t>(9 * kSize + i)] = 100;
    // Left and right columns.
    grid.data[static_cast<std::size_t>(i * kSize + 0)] = 100;
    grid.data[static_cast<std::size_t>(i * kSize + 9)] = 100;
  }
  return grid;
}

// A fully free grid — no obstacles anywhere.
OccupancyGrid make_free_grid()
{
  constexpr int kSize = 10;
  OccupancyGrid grid;
  grid.width = kSize;
  grid.height = kSize;
  grid.resolution = 0.1;
  grid.origin = { 0.0, 0.0, 0.0 };
  grid.data.assign(static_cast<std::size_t>(kSize * kSize), 0);
  return grid;
}

// ---- LaserSimulatorTest -----------------------------------------------------

TEST(LaserSimulatorTest, RayCountMatchesConfig)
{
  const LaserSimulator sim;
  const OccupancyGrid map = make_free_grid();

  LaserConfig cfg;
  cfg.min_angle = -std::numbers::pi / 2.0;
  cfg.max_angle = std::numbers::pi / 2.0;
  cfg.min_range = 0.1;
  cfg.max_range = 5.0;
  cfg.num_rays = 9;

  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const LaserScan scan = sim.simulate(pose, cfg, map);
  ASSERT_THAT(scan.ranges, SizeIs(9));
}

TEST(LaserSimulatorTest, EmptyMapAllMaxRange)
{
  const LaserSimulator sim;
  const OccupancyGrid map = make_free_grid();

  LaserConfig cfg;
  cfg.min_angle = -std::numbers::pi / 4.0;
  cfg.max_angle = std::numbers::pi / 4.0;
  cfg.min_range = 0.05;
  cfg.max_range = 100.0;
  cfg.num_rays = 5;

  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const LaserScan scan = sim.simulate(pose, cfg, map);
  ASSERT_THAT(scan.ranges, SizeIs(5));
  // With no obstacles every ray should return max_range.
  EXPECT_THAT(scan.ranges, Each(static_cast<float>(cfg.max_range)));
}

TEST(LaserSimulatorTest, WallInFront)
{
  const LaserSimulator sim;
  const OccupancyGrid map = make_walled_grid();

  LaserConfig cfg;
  // Single ray pointing forward (+x direction) — will hit the right wall
  // (column 9) which is ~0.4 m away from the centre.
  cfg.min_angle = 0.0;
  cfg.max_angle = 0.0;
  cfg.min_range = 0.05;
  cfg.max_range = 5.0;
  cfg.num_rays = 1;

  const Pose2D pose{ 0.5, 0.5, 0.0 };  // Grid centre.
  const LaserScan scan = sim.simulate(pose, cfg, map);
  ASSERT_THAT(scan.ranges, SizeIs(1));
  EXPECT_THAT(scan.ranges[0], Lt(static_cast<float>(cfg.max_range)));
}

TEST(LaserSimulatorTest, NoiseIsZeroMeanNotBiased)
{
  // Verify that config.noise.mean does NOT act as a range bias.  We place the
  // sensor so that a known obstacle is exactly 0.4 m away, enable noise with a
  // non-zero mean (0.5) and a tiny std_dev, and confirm the measured range is
  // close to the true obstacle distance — not shifted by 0.5 m.
  const LaserSimulator sim;
  const OccupancyGrid map = make_walled_grid();

  LaserConfig cfg;
  cfg.min_angle = 0.0;
  cfg.max_angle = 0.0;
  cfg.min_range = 0.05;
  cfg.max_range = 5.0;
  cfg.num_rays = 1;
  cfg.noise.enabled = true;
  cfg.noise.mean = 0.5;       // Must NOT bias the result.
  cfg.noise.std_dev = 0.001;  // Tiny so individual samples stay near 0.

  // Sensor at grid centre (0.5, 0.5); right wall is at grid column 9 → x=0.9 m,
  // so the true range is approximately 0.4 m.
  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const LaserScan scan = sim.simulate(pose, cfg, map);
  ASSERT_THAT(scan.ranges, SizeIs(1));
  // 0.01 m window is ~10 sigma — flake-free but tight enough to catch bias ≥ 0.01 m.
  EXPECT_THAT(scan.ranges[0], FloatNear(0.4f, 0.01f));
}

// ---- SonarSimulatorTest -----------------------------------------------------

TEST(SonarSimulatorTest, EmptyMapMaxRange)
{
  const SonarSimulator sim;
  const OccupancyGrid map = make_free_grid();

  SonarConfig cfg;
  cfg.min_range = 0.1;
  cfg.max_range = 100.0;
  cfg.cone_angle = std::numbers::pi / 4.0;

  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const SonarScan scan = sim.simulate(pose, cfg, map);
  // The implementation signals "no obstacle found" with +infinity rather than
  // returning the configured max_range value directly.
  EXPECT_TRUE(std::isinf(scan.range));
  EXPECT_GT(scan.range, 0.0);
}

TEST(SonarSimulatorTest, ObstacleInCone)
{
  const SonarSimulator sim;
  const OccupancyGrid map = make_walled_grid();

  SonarConfig cfg;
  cfg.min_range = 0.05;
  cfg.max_range = 5.0;
  cfg.cone_angle = std::numbers::pi / 4.0;

  // Sensor at centre pointing toward the right wall.
  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const SonarScan scan = sim.simulate(pose, cfg, map);
  EXPECT_LT(scan.range, cfg.max_range);
}

TEST(SonarSimulatorTest, NoiseIsZeroMeanNotBiased)
{
  // Same intent as the laser test: config.noise.mean must not bias the range.
  const SonarSimulator sim;
  const OccupancyGrid map = make_walled_grid();

  SonarConfig cfg;
  cfg.min_range = 0.05;
  cfg.max_range = 5.0;
  cfg.cone_angle = std::numbers::pi / 4.0;
  cfg.noise.enabled = true;
  cfg.noise.mean = 0.5;       // Must NOT bias the result.
  cfg.noise.std_dev = 0.001;  // Tiny so individual samples stay near 0.

  // Sensor at centre (0.5, 0.5) pointing toward the right wall; the cone
  // samples oblique rays so the minimum range is ~0.5 m, not 0.4 m.
  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const SonarScan scan = sim.simulate(pose, cfg, map);
  // The sonar cone (π/4) samples oblique rays, so the true range is ~0.5 m
  // (not 0.4 m as in the straight-line laser case).  A 0.02 m window is still
  // tight enough to catch any bias ≥ 0.02 m while accommodating cone geometry.
  EXPECT_THAT(scan.range, DoubleNear(0.5, 0.02));
}

TEST(SonarSimulatorTest, NoHitNoiseOffReturnsInfinity)
{
  // Regression test for the off-by-one sentinel bug, distinct from the generic
  // no-hit coverage in EmptyMapMaxRange.  max_range = 0.3 m with resolution 0.1 m
  // gives max_steps = (int)(0.3 / 0.1) = 2 in IEEE arithmetic (0.3/0.1 rounds to
  // 2.999..., which truncates to 2), so the old sentinel of max_steps+1 = 3
  // exactly equalled one extra cell — the precise case the numeric sentinel
  // mishandled.  The result must be exactly +infinity.
  const SonarSimulator sim;
  const OccupancyGrid map = make_free_grid();

  SonarConfig cfg;
  cfg.min_range = 0.1;
  cfg.max_range = 0.3;  // Chosen for the FP-truncation corner case above.
  cfg.cone_angle = std::numbers::pi / 4.0;

  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const SonarScan scan = sim.simulate(pose, cfg, map);
  EXPECT_TRUE(std::isinf(scan.range));
  EXPECT_GT(scan.range, 0.0);
}

TEST(SonarSimulatorTest, NoHitNoiseOnReturnsInfinity)
{
  // Regression test: with noise enabled and no obstacle in range, the result
  // must still be exactly +infinity — not a finite biased value.  The old
  // sentinel (max_steps + 1) mapped back to max_range + resolution = 0.4 m;
  // with std_dev = 0.05 the noise could push it below 0.4 m to a finite
  // reading that passed the upper clamp, silently returning a bogus distance.
  const SonarSimulator sim;
  const OccupancyGrid map = make_free_grid();

  SonarConfig cfg;
  cfg.min_range = 0.1;
  cfg.max_range = 0.3;  // Old sentinel mapped to 0.4 m; noise could push it finite.
  cfg.cone_angle = std::numbers::pi / 4.0;
  cfg.noise.enabled = true;
  cfg.noise.std_dev = 0.05;

  const Pose2D pose{ 0.5, 0.5, 0.0 };
  const SonarScan scan = sim.simulate(pose, cfg, map);
  EXPECT_TRUE(std::isinf(scan.range));
  EXPECT_GT(scan.range, 0.0);
}

// ---- RfidSimulatorTest ------------------------------------------------------

TEST(RfidSimulatorTest, TagInRange)
{
  const RfidSimulator sim;

  RfidSensorConfig cfg;
  cfg.max_range = 2.0;
  cfg.angle_span = std::numbers::pi * 2.0;  // Full 360° detection.

  RfidTag tag;
  tag.tag_id = "tag_001";
  tag.message = "hello";
  tag.pose = { 0.5, 0.5, 0.0 };

  // Sensor at origin — tag is 0.5√2 ≈ 0.71 m away, well within range.
  const Pose2D sensor_pose{ 0.0, 0.0, 0.0 };
  const RfidMeasurement result = sim.simulate(sensor_pose, cfg, { tag });
  ASSERT_THAT(result.tag_ids, SizeIs(1));
  EXPECT_EQ(result.tag_ids[0], "tag_001");
}

TEST(RfidSimulatorTest, TagOutOfRange)
{
  const RfidSimulator sim;

  RfidSensorConfig cfg;
  cfg.max_range = 0.3;  // Only 0.3 m reach.
  cfg.angle_span = std::numbers::pi * 2.0;

  RfidTag tag;
  tag.tag_id = "tag_far";
  tag.message = "";
  tag.pose = { 5.0, 5.0, 0.0 };  // 7+ m away.

  const Pose2D sensor_pose{ 0.0, 0.0, 0.0 };
  const RfidMeasurement result = sim.simulate(sensor_pose, cfg, { tag });
  EXPECT_THAT(result.tag_ids, IsEmpty());
}

// ---- Co2SimulatorTest -------------------------------------------------------

TEST(Co2SimulatorTest, NoSourcesZeroPpm)
{
  const Co2Simulator sim;

  CO2SensorConfig cfg;
  cfg.max_range = 5.0;

  const Pose2D sensor_pose{ 0.0, 0.0, 0.0 };
  const CO2Measurement result = sim.simulate(sensor_pose, cfg, {});
  EXPECT_THAT(result.ppm, testing::DoubleNear(0.0, 1e-9));
}

TEST(Co2SimulatorTest, SourceNearbyPositivePpm)
{
  const Co2Simulator sim;

  CO2SensorConfig cfg;
  cfg.max_range = 5.0;

  CO2Source source;
  source.id = "src_0";
  source.ppm = 400.0;
  source.pose = { 0.1, 0.0, 0.0 };  // Very close — within saturation zone.

  const Pose2D sensor_pose{ 0.0, 0.0, 0.0 };
  const CO2Measurement result = sim.simulate(sensor_pose, cfg, { source });
  EXPECT_GT(result.ppm, 0.0);
}

}  // namespace
}  // namespace stdr_simulation::sensors
