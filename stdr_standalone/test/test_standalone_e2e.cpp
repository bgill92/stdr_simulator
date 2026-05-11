/** @file End-to-end scenario tests for StandaloneBackend.
 *
 *  These tests complement the exhaustive unit coverage in
 *  test_standalone_backend.cpp by exercising multi-step drive sequences that
 *  involve real map collision geometry and laser sensing.  Each test drives the
 *  simulation through a meaningful physical scenario rather than just checking
 *  API surface. */

#include "stdr_standalone/standalone_backend.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace stdr_standalone
{
namespace
{

using ::testing::SizeIs;

constexpr auto kResourcesDir = STDR_RESOURCES_DIR;
constexpr auto kTestDir = STDR_TEST_DIR;

// -- Path helpers ------------------------------------------------------------

[[nodiscard]] std::string simple_rooms_map_path()
{
  return std::string(kResourcesDir) + "/maps/simple_rooms.yaml";
}

[[nodiscard]] std::string maze1_map_path()
{
  return std::string(kResourcesDir) + "/maps/maze1.yaml";
}

[[nodiscard]] std::string laser_robot_path()
{
  // Minimal inline laser robot: 180 rays, ±π/2 FOV, 4 m max range, frame_id laser_0.
  return std::string(kTestDir) + "/test_laser_robot.yaml";
}

[[nodiscard]] std::string trin_bot_path()
{
  return std::string(kResourcesDir) + "/resources/robots/trin_bot.yaml";
}

// -- Polling helper ----------------------------------------------------------

/** Poll @p condition every 10 ms until it returns true or @p deadline elapses.
 *
 *  Returns true if the condition was satisfied before the deadline. Prefer this
 *  over unconditional sleep_for to avoid flakiness under CI load. */
template <typename Pred>
[[nodiscard]] bool poll_until(Pred condition, std::chrono::milliseconds deadline = std::chrono::seconds(2))
{
  const auto start = std::chrono::steady_clock::now();
  while (!condition())
  {
    if (std::chrono::steady_clock::now() - start >= deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

// -- RAII guard for environment variables ------------------------------------

/** Saves and restores a single environment variable across a scope.
 *
 *  Useful when a test must temporarily override an env var that other tests or
 *  the process itself may depend on. */
struct EnvGuard
{
  explicit EnvGuard(const char* name) : name_(name)
  {
    const char* prior = std::getenv(name);
    if (prior != nullptr)
    {
      prior_value_ = prior;
    }
  }

  ~EnvGuard()
  {
    if (prior_value_.has_value())
    {
      ::setenv(name_.c_str(), prior_value_->c_str(), /*overwrite=*/1);
    }
    else
    {
      ::unsetenv(name_.c_str());
    }
  }

  // Non-copyable, non-movable — lifetime must match the enclosing scope.
  EnvGuard(const EnvGuard&) = delete;
  EnvGuard& operator=(const EnvGuard&) = delete;

  std::string name_;
  std::optional<std::string> prior_value_;
};

// -- Scenario 1 --------------------------------------------------------------

// Spawn the laser robot at (5.5, 10.0, 0.0) in simple_rooms.yaml.
//
// Map layout: simple_rooms.yaml is a 20 m × 15 m map (400×300 px, 0.05 m/px).
// There is an internal vertical wall band from x ≈ 6.30 to x ≈ 6.95 at y = 10.0,
// so the center laser ray at θ = 0 (along +x) starts at roughly 0.80 m range.
// Driving 0.1 m forward shrinks that range to ≈ 0.70 m — a 0.10 m drop, safely
// above the 0.05 m assertion margin.
TEST(StandaloneE2E, LaserRangesShrinkWhenDrivingTowardWall)
{
  StandaloneBackend backend;
  ASSERT_TRUE(backend.load_map(simple_rooms_map_path()).has_value());

  const stdr_simulation::Pose2D spawn{ .x = 5.5, .y = 10.0, .theta = 0.0 };
  const std::string name = backend.spawn_robot(laser_robot_path(), spawn).value();

  // Warm up: let the sim produce at least one laser scan before sampling.
  backend.start();
  ASSERT_TRUE(poll_until([&] { return backend.latest_laser(name, "laser_0").has_value(); }));
  backend.pause();

  const std::optional<stdr_simulation::LaserScan> scan_before = backend.latest_laser(name, "laser_0");
  ASSERT_TRUE(scan_before.has_value());
  ASSERT_THAT(scan_before->ranges, SizeIs(180));

  // The center ray (index 90 for 180 rays) looks straight ahead along +x.
  // size() / 2 yields index 90 for a 180-ray scan — the boresight ray.
  const double range_before = scan_before->ranges[scan_before->ranges.size() / 2];
  EXPECT_GT(range_before, 0.0);

  // Drive forward at 0.3 m/s until the robot advances at least 0.1 m.
  backend.set_cmd_vel(name, stdr_simulation::Twist2D{ .linear_x = 0.3, .linear_y = 0.0, .angular_z = 0.0 });
  backend.start();
  ASSERT_TRUE(poll_until([&] {
    const std::optional<stdr_simulation::Pose2D> p = backend.pose(name);
    return p.has_value() && (p->x - spawn.x) >= 0.1;
  })) << "Robot did not advance 0.1 m within the poll deadline";
  backend.pause();

  const std::optional<stdr_simulation::LaserScan> scan_after = backend.latest_laser(name, "laser_0");
  ASSERT_TRUE(scan_after.has_value());
  ASSERT_THAT(scan_after->ranges, SizeIs(180));

  const double range_after = scan_after->ranges[scan_after->ranges.size() / 2];

  // Moving toward the wall must shrink the center-ray range by at least 0.05 m.
  EXPECT_LT(range_after, range_before - 0.05) << "range_before=" << range_before << " range_after=" << range_after
                                              << " — expected center ray to shrink by at least 0.05 m";
}

// -- Scenario 2 --------------------------------------------------------------

// Load simple_rooms.yaml (20 m x 15 m, 0.05 m/px) and spawn two trin_bot
// instances in the open horizontal corridor at y = 7.0 (between internal wall
// bands at y ≈ 6.60 and y ≈ 8.50).
//
// At y = 7.0 there are no vertical wall dividers — the corridor spans from
// x ≈ 0.75 to x ≈ 18.90.  Both robots have over 0.2 m clearance from every
// wall, satisfying the collision checker's 3x3 neighbourhood at 0.05 m/px.
//   Robot A: (3.0, 7.0, 0.0)  — faces +x, drives away from the left boundary.
//   Robot B: (15.0, 7.0, pi)  — faces -x, drives away from the right boundary.
//
// Giving both robots linear_x = +0.2 drives A toward +x and B (facing -x)
// toward -x — the two robots move in opposite directions and do not interact.
//
// trin_bot.yaml uses filename: sub-config references that the backend resolves
// relative to STDR_RESOURCES_DIR.  The loader expects paths like
// "range_sensors/VL53L0X.yaml" to be relative to stdr_resources/resources/, so
// we set the environment variable to that subdirectory before spawning.
TEST(StandaloneE2E, MultiRobotFromLauncherConfigDriveIndependently)
{
  // Restore STDR_RESOURCES_DIR to its original value when the test exits, regardless
  // of how it exits, so other tests in the same process are not affected.
  const EnvGuard resources_dir_guard("STDR_RESOURCES_DIR");

  // Ensure the config loader can resolve sub-config filenames.
  const std::string resources_subdir = std::string(kResourcesDir) + "/resources";
  ::setenv("STDR_RESOURCES_DIR", resources_subdir.c_str(), /*overwrite=*/1);

  StandaloneBackend backend;
  ASSERT_TRUE(backend.load_map(simple_rooms_map_path()).has_value());

  const stdr_simulation::Pose2D spawn_a{ .x = 3.0, .y = 7.0, .theta = 0.0 };
  const stdr_simulation::Pose2D spawn_b{ .x = 15.0, .y = 7.0, .theta = M_PI };

  const tl::expected<std::string, std::string> result_a = backend.spawn_robot(trin_bot_path(), spawn_a);
  const tl::expected<std::string, std::string> result_b = backend.spawn_robot(trin_bot_path(), spawn_b);
  ASSERT_TRUE(result_a.has_value()) << result_a.error();
  ASSERT_TRUE(result_b.has_value()) << result_b.error();
  const std::string name_a = result_a.value();
  const std::string name_b = result_b.value();

  // Drive A in the +x direction, drive B in the −x direction (B faces −x so
  // positive linear_x still moves it forward in its own frame → −x world).
  const stdr_simulation::Twist2D twist_a{ .linear_x = 0.2, .linear_y = 0.0, .angular_z = 0.0 };
  const stdr_simulation::Twist2D twist_b{ .linear_x = 0.2, .linear_y = 0.0, .angular_z = 0.0 };
  backend.set_cmd_vel(name_a, twist_a);
  backend.set_cmd_vel(name_b, twist_b);

  backend.start();
  ASSERT_TRUE(poll_until([&] {
    const std::optional<stdr_simulation::Pose2D> pa = backend.pose(name_a);
    const std::optional<stdr_simulation::Pose2D> pb = backend.pose(name_b);
    if (!pa.has_value() || !pb.has_value())
    {
      return false;
    }
    const bool a_moved = std::abs(pa->x - spawn_a.x) >= 0.02;
    const bool b_moved = std::abs(pb->x - spawn_b.x) >= 0.02;
    return a_moved && b_moved;
  })) << "One or both robots failed to move 0.02 m within the poll deadline";
  backend.pause();

  const std::optional<stdr_simulation::Pose2D> final_a = backend.pose(name_a);
  const std::optional<stdr_simulation::Pose2D> final_b = backend.pose(name_b);
  ASSERT_TRUE(final_a.has_value());
  ASSERT_TRUE(final_b.has_value());

  // Robot A drove in the +x direction — its x must have increased.
  EXPECT_GT(final_a->x, spawn_a.x) << "Robot A did not advance in the expected +x direction";

  // Robot B faced −x and drove forward — its x must have decreased.
  EXPECT_LT(final_b->x, spawn_b.x) << "Robot B did not advance in the expected −x direction";

  // Independence check: A moved in the +x direction, not pulled toward B's corridor.
  // Both robots' y coordinates should remain close to their spawn y (straight corridor).
  EXPECT_NEAR(final_a->y, spawn_a.y, 0.1) << "Robot A drifted off its corridor";
  EXPECT_NEAR(final_b->y, spawn_b.y, 0.1) << "Robot B drifted off its corridor";
}

// -- Scenario 3 --------------------------------------------------------------

// Spawn the laser robot in maze1.yaml at (0.6, 1.0, 0.0), facing a thin wall
// band at x ≈ 0.741 — only 0.141 m ahead.  At 0.3 m/s the robot reaches the
// wall in under 0.5 s, well within the 5 s deadline.
//
// The simulator sets the collision flag when a robot's footprint overlaps an
// occupied cell; it does not necessarily halt motion (the physics integrator may
// step through thin walls at coarse dt).  We assert the flag becomes true and
// add a sanity-check that the robot did not teleport more than twice the map
// width (~5 m), which would indicate a physics runaway rather than a real
// collision.
TEST(StandaloneE2E, RobotStopsAtWall)
{
  StandaloneBackend backend;
  ASSERT_TRUE(backend.load_map(maze1_map_path()).has_value());

  // Spawn the laser robot 0.141 m ahead of the internal wall at x ≈ 0.741.
  const stdr_simulation::Pose2D spawn{ .x = 0.6, .y = 1.0, .theta = 0.0 };
  const std::string name = backend.spawn_robot(laser_robot_path(), spawn).value();

  backend.set_cmd_vel(name, stdr_simulation::Twist2D{ .linear_x = 0.3, .linear_y = 0.0, .angular_z = 0.0 });

  // Poll for the collision flag with a generous 5 s deadline.
  backend.start();
  const bool hit_wall = poll_until(
      [&] {
        const std::optional<bool> c = backend.collided(name);
        return c.has_value() && *c;
      },
      std::chrono::seconds(5));
  backend.pause();

  // The collision flag must have become true before the deadline.
  ASSERT_TRUE(hit_wall) << "Robot did not register a collision within 5 s — "
                           "wall at x ≈ 0.741 was not reached from spawn x = 0.6 at 0.3 m/s";

  // Check that the final pose is physically consistent with the robot having
  // stopped near the wall band.  The robot spawned at x = 0.6 and the wall
  // starts at x ≈ 0.741; the robot must not have gone backwards (x < 0.6) nor
  // passed clean through the wall (x > 0.85 adds a small margin for the
  // footprint centre crossing the occupied cell boundary).
  const std::optional<stdr_simulation::Pose2D> final_pose = backend.pose(name);
  ASSERT_TRUE(final_pose.has_value());
  EXPECT_GE(final_pose->x, spawn.x) << "Robot moved backwards from its spawn — unexpected";
  // A runaway (the robot tunneling through the wall) would put the final pose
  // far past the wall band; the upper bound here ensures the collision check
  // actually halted motion.
  EXPECT_LT(final_pose->x, 0.85) << "Robot final x=" << final_pose->x
                                 << " is past the wall band (x ≈ 0.741) — "
                                    "collision detection may not have engaged correctly";
}

}  // namespace
}  // namespace stdr_standalone
