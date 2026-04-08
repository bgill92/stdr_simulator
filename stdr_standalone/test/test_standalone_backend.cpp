#include "stdr_standalone/standalone_backend.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

namespace stdr_standalone
{
namespace
{

using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::SizeIs;

constexpr auto kResourcesDir = STDR_RESOURCES_DIR;

std::string map_path()
{
  return std::string(kResourcesDir) + "/maps/maze1.yaml";
}

std::string robot_path()
{
  // khepera2.yaml uses only inline specs (no filename: references), so it loads
  // correctly regardless of the base_dir passed to load_robot_config.
  return std::string(kResourcesDir) + "/resources/robots/khepera2.yaml";
}

// --- Error cases first ---

TEST(StandaloneBackend, LoadMapFailsOnMissingFile)
{
  StandaloneBackend backend;
  const auto result = backend.load_map("/nonexistent/map.yaml");
  EXPECT_FALSE(result.has_value());
}

TEST(StandaloneBackend, SpawnRobotFailsOnMissingFile)
{
  StandaloneBackend backend;
  const auto result = backend.spawn_robot("/nonexistent/robot.yaml", {});
  EXPECT_FALSE(result.has_value());
}

// --- Construction ---

TEST(StandaloneBackend, ConstructsAndDestructs)
{
  StandaloneBackend backend;
  // Destructor should cleanly stop the sim thread.
}

TEST(StandaloneBackend, InitialSnapshotIsEmpty)
{
  StandaloneBackend backend;
  const auto snapshot = backend.get_snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_THAT(snapshot->robots, IsEmpty());
  EXPECT_EQ(snapshot->elapsed_time, 0.0);
  EXPECT_FALSE(snapshot->running);
}

// --- Map loading ---

TEST(StandaloneBackend, LoadMapSucceeds)
{
  StandaloneBackend backend;
  const auto result = backend.load_map(map_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto snapshot = backend.get_snapshot();
  EXPECT_GT(snapshot->map.width, 0);
  EXPECT_GT(snapshot->map.height, 0);
}

// --- Robot spawning ---

TEST(StandaloneBackend, SpawnRobotSucceeds)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const stdr_simulation::Pose2D pose{ 1.0, 1.0, 0.0 };
  const auto result = backend.spawn_robot(robot_path(), pose);
  ASSERT_TRUE(result.has_value()) << result.error();

  const auto snapshot = backend.get_snapshot();
  ASSERT_THAT(snapshot->robots, SizeIs(1));
  EXPECT_EQ(snapshot->robots[0].name, result.value());
}

TEST(StandaloneBackend, DeleteRobotRemovesIt)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();
  backend.delete_robot(name);

  const auto snapshot = backend.get_snapshot();
  EXPECT_THAT(snapshot->robots, IsEmpty());
}

// --- Simulation control ---

TEST(StandaloneBackend, StartAndPauseToggleRunningState)
{
  StandaloneBackend backend;
  EXPECT_FALSE(backend.get_snapshot()->running);

  backend.start();
  EXPECT_TRUE(backend.get_snapshot()->running);

  backend.pause();
  EXPECT_FALSE(backend.get_snapshot()->running);
}

TEST(StandaloneBackend, ResetClearsRobotsAndElapsedTime)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  std::ignore = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 });

  backend.start();
  // Let simulation run briefly to accumulate elapsed time.
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  backend.reset();

  const auto snapshot = backend.get_snapshot();
  EXPECT_THAT(snapshot->robots, IsEmpty());
  EXPECT_EQ(snapshot->elapsed_time, 0.0);
  EXPECT_FALSE(snapshot->running);
}

TEST(StandaloneBackend, SetRobotPoseUpdatesPose)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());
  const std::string name = backend.spawn_robot(robot_path(), { 1.0, 1.0, 0.0 }).value();

  const stdr_simulation::Pose2D new_pose{ 2.0, 3.0, 1.57 };
  backend.set_robot_pose(name, new_pose);

  const auto snapshot = backend.get_snapshot();
  ASSERT_THAT(snapshot->robots, SizeIs(1));
  EXPECT_DOUBLE_EQ(snapshot->robots[0].pose.x, 2.0);
  EXPECT_DOUBLE_EQ(snapshot->robots[0].pose.y, 3.0);
}

// --- Messages ---

TEST(StandaloneBackend, PollMessagesReturnsAndDrains)
{
  StandaloneBackend backend;
  std::ignore = backend.load_map(map_path());

  const std::vector<std::string> messages = backend.poll_messages();
  EXPECT_THAT(messages, Not(IsEmpty()));

  // Second call should be empty since we drained.
  const std::vector<std::string> messages2 = backend.poll_messages();
  EXPECT_THAT(messages2, IsEmpty());
}

}  // namespace
}  // namespace stdr_standalone
