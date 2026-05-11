/** @file End-to-end scenario tests for the ROS2 stack in-process.
 *
 *  Exercises StdrServerNode + StdrRobotNode + Ros2Backend together in a single
 *  process, driving the simulation through physical drive scenarios using a real
 *  map with wall geometry (simple_rooms.yaml).
 *
 *  Scenarios:
 *    1. LaserRangesShrinkWhenDrivingTowardWall — spawn one robot at (5.5, 10.0)
 *       facing +x, drive forward, assert center-ray range shrinks by ≥ 0.05 m.
 *    2. MultiRobotIndependentDrive — spawn two robots in the open corridor at
 *       y = 7.0 facing opposite directions, drive both forward, assert each
 *       advances along its own heading without y drift.
 *
 *  Scenario 3 (RobotStopsAtWall) lives in
 *  stdr_standalone/test/test_standalone_e2e.cpp because
 *  Ros2Backend::collided() is not yet implemented — the method would need to
 *  subscribe to a collision topic published by StdrRobotNode, which does not
 *  exist yet. Move this scenario here once that topic is available.
 *
 *  Fixture properties: MultiThreadedExecutor (4 threads), suite-level rclcpp
 *  init/shutdown, background spinner thread, no tf2 spin threads. */

#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_robot/stdr_robot_node.hpp>
#include <stdr_server/map_loader.hpp>
#include <stdr_server/stdr_server_node.hpp>

#include <stdr_msgs/action/spawn_robot.hpp>
#include <stdr_msgs/msg/laser_sensor_msg.hpp>
#include <stdr_msgs/srv/load_external_map.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace stdr_gui_ros
{
namespace
{

using ::testing::SizeIs;
using namespace std::chrono_literals;

constexpr auto kResourcesDir = STDR_RESOURCES_DIR;

// ─── Fixture ──────────────────────────────────────────────────────────────────

class Ros2E2E : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  Ros2E2E()
    : executor_(rclcpp::ExecutorOptions{}, /*number_of_threads=*/4)
    , server_node_(std::make_shared<stdr_server::StdrServerNode>())
    , backend_node_(std::make_shared<rclcpp::Node>("e2e_test_backend"))
    , backend_(backend_node_)
  {
    executor_.add_node(server_node_);
    executor_.add_node(backend_node_);
    // Start the executor on a background thread so wait_until calls on the
    // test thread can block while the background thread delivers messages.
    spinner_ = std::thread([this]() { executor_.spin(); });
  }

  ~Ros2E2E() override
  {
    executor_.cancel();
    if (spinner_.joinable())
    {
      spinner_.join();
    }
  }

  // Poll the predicate every 20 ms until it returns true or the timeout elapses.
  // Safe to call from the test thread while the background spinner is running.
  template <typename Pred>
  [[nodiscard]] bool wait_until(Pred predicate, std::chrono::milliseconds timeout) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate())
    {
      if (std::chrono::steady_clock::now() >= deadline)
      {
        return false;
      }
      std::this_thread::sleep_for(20ms);
    }
    return true;
  }

  // Load a map from a YAML file into the server via LoadExternalMap.
  // Uses stdr_server::load_map() to parse the file, then sends the resulting
  // OccupancyGrid through the already-tested LoadExternalMap call path.
  void load_map_into_server(const std::string& yaml_path)
  {
    const tl::expected<nav_msgs::msg::OccupancyGrid, std::string> map_result = stdr_server::load_map(yaml_path);
    ASSERT_TRUE(map_result.has_value()) << "Failed to load map from file: " << map_result.error();

    auto client_node = std::make_shared<rclcpp::Node>("e2e_map_loader");
    executor_.add_node(client_node);

    // RAII guard removes the client node from the executor on all return paths.
    struct NodeGuard
    {
      rclcpp::executors::MultiThreadedExecutor& exec;
      rclcpp::Node::SharedPtr node;
      ~NodeGuard()
      {
        exec.remove_node(node);
      }
    } guard{ executor_, client_node };

    const auto client = client_node->create_client<stdr_msgs::srv::LoadExternalMap>("stdr_server/load_external_map");

    ASSERT_TRUE(wait_until([&client]() { return client->service_is_ready(); }, 3s))
        << "LoadExternalMap service never became ready";

    auto request = std::make_shared<stdr_msgs::srv::LoadExternalMap::Request>();
    request->map = map_result.value();

    auto future = client->async_send_request(request);
    ASSERT_TRUE(wait_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 3s))
        << "LoadExternalMap service call did not complete";
  }

  // Spawn a robot via the SpawnRobot action with one laser sensor.
  // Returns the assigned robot name, or empty string on failure.
  [[nodiscard]] std::string spawn_laser_robot(const stdr_simulation::Pose2D& pose)
  {
    using SpawnRobot = stdr_msgs::action::SpawnRobot;

    auto action_client_node = std::make_shared<rclcpp::Node>("e2e_spawn_client");
    executor_.add_node(action_client_node);

    // RAII guard removes the spawn client node on all return paths.
    struct NodeGuard
    {
      rclcpp::executors::MultiThreadedExecutor& exec;
      rclcpp::Node::SharedPtr node;
      ~NodeGuard()
      {
        exec.remove_node(node);
      }
    } guard{ executor_, action_client_node };

    const auto spawn_client = rclcpp_action::create_client<SpawnRobot>(action_client_node, "stdr_server/spawn_robot");

    if (!wait_until([&spawn_client]() { return spawn_client->action_server_is_ready(); }, 3s))
    {
      ADD_FAILURE() << "SpawnRobot action server never became ready";
      return {};
    }

    SpawnRobot::Goal goal;
    goal.description.footprint.radius = 0.2f;

    stdr_msgs::msg::LaserSensorMsg laser_msg;
    laser_msg.frame_id = "laser_0";
    laser_msg.min_angle = static_cast<float>(-M_PI / 2.0);
    laser_msg.max_angle = static_cast<float>(M_PI / 2.0);
    laser_msg.num_rays = 180;
    laser_msg.max_range = 4.0f;
    laser_msg.min_range = 0.05f;
    laser_msg.frequency = 10.0f;
    goal.description.laser_sensors.push_back(laser_msg);

    goal.description.initial_pose.x = pose.x;
    goal.description.initial_pose.y = pose.y;
    goal.description.initial_pose.theta = pose.theta;

    auto goal_handle_future = spawn_client->async_send_goal(goal);
    if (!wait_until([&goal_handle_future]() { return goal_handle_future.wait_for(0s) == std::future_status::ready; },
                    3s))
    {
      ADD_FAILURE() << "SpawnRobot goal was not accepted";
      return {};
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle)
    {
      ADD_FAILURE() << "SpawnRobot goal handle is null";
      return {};
    }

    auto result_future = spawn_client->async_get_result(goal_handle);
    if (!wait_until([&result_future]() { return result_future.wait_for(0s) == std::future_status::ready; }, 3s))
    {
      ADD_FAILURE() << "SpawnRobot action did not complete";
      return {};
    }

    const rclcpp_action::ClientGoalHandle<SpawnRobot>::WrappedResult spawn_result = result_future.get();
    if (spawn_result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
      ADD_FAILURE() << "SpawnRobot action did not succeed";
      return {};
    }

    return spawn_result.result->indexed_description.name;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<stdr_server::StdrServerNode> server_node_;
  std::shared_ptr<rclcpp::Node> backend_node_;
  Ros2Backend backend_;
  std::thread spinner_;
};

// ─── Scenario 1 ──────────────────────────────────────────────────────────────

// Spawn one laser robot at (5.5, 10.0, 0.0) facing +x in simple_rooms.yaml.
// Map layout: simple_rooms.yaml is a 20 m × 15 m map (400×300 px, 0.05 m/px).
// There is an internal vertical wall band from x ≈ 6.30 to x ≈ 6.95 at y = 10.0
// so the center laser ray at θ = 0 starts at roughly 0.80 m range.
// Driving 0.1 m forward shrinks that range to ≈ 0.70 m — a 0.10 m drop, safely
// above the 0.05 m assertion margin.
TEST_F(Ros2E2E, LaserRangesShrinkWhenDrivingTowardWall)
{
  const std::string yaml_path = std::string(kResourcesDir) + "/maps/simple_rooms.yaml";
  load_map_into_server(yaml_path);

  const stdr_simulation::Pose2D spawn{ .x = 5.5, .y = 10.0, .theta = 0.0 };
  const std::string name = spawn_laser_robot(spawn);
  ASSERT_FALSE(name.empty()) << "SpawnRobot action did not return a name";

  // Bring up the robot node — it self-registers via RegisterRobot action.
  rclcpp::NodeOptions robot_options;
  robot_options.append_parameter_override("robot_name", name);
  std::shared_ptr<stdr_robot::StdrRobotNode> robot_node = std::make_shared<stdr_robot::StdrRobotNode>(robot_options);
  executor_.add_node(robot_node);

  // RAII guard removes the robot node on all exit paths (normal or ASSERT abort)
  // and keeps the shared_ptr alive until after remove_node returns so the
  // executor's internal cleanup has a valid node object to work with.
  struct NodeGuard
  {
    rclcpp::executors::MultiThreadedExecutor& exec;
    std::shared_ptr<stdr_robot::StdrRobotNode> node;
    ~NodeGuard()
    {
      exec.remove_node(node);
    }
  } robot_guard{ executor_, robot_node };

  // Wait for the backend to observe the spawned robot via active_robots.
  ASSERT_TRUE(wait_until([this]() { return backend_.num_robots() > 0; }, 5s))
      << "Backend never observed any robots on active_robots topic";

  // Wait for the first laser scan to arrive (robot timer fires at 10 Hz).
  ASSERT_TRUE(wait_until([this, &name]() { return backend_.latest_laser(name, "laser_0").has_value(); }, 5s))
      << "Timed out waiting for laser scan from the robot node";

  const std::optional<stdr_simulation::LaserScan> scan_before = backend_.latest_laser(name, "laser_0");
  ASSERT_TRUE(scan_before.has_value());
  ASSERT_THAT(scan_before->ranges, SizeIs(180));

  // The center ray (index 90) looks straight ahead along +x — the boresight ray.
  const double range_before = scan_before->ranges[scan_before->ranges.size() / 2];
  EXPECT_GT(range_before, 0.0);

  // Drive forward at 0.3 m/s until the robot advances at least 0.1 m.
  // Allow 5 s to account for the ROS2 topic round-trip added by cmd_vel publishing.
  backend_.set_cmd_vel(name, stdr_simulation::Twist2D{ .linear_x = 0.3, .linear_y = 0.0, .angular_z = 0.0 });

  ASSERT_TRUE(wait_until(
      [this, &name, &spawn]() {
        const std::optional<stdr_simulation::Pose2D> p = backend_.pose(name);
        return p.has_value() && (p->x - spawn.x) >= 0.1;
      },
      5s))
      << "Robot did not advance 0.1 m in +x direction within the deadline";

  const std::optional<stdr_simulation::LaserScan> scan_after = backend_.latest_laser(name, "laser_0");
  ASSERT_TRUE(scan_after.has_value());
  ASSERT_THAT(scan_after->ranges, SizeIs(180));

  const double range_after = scan_after->ranges[scan_after->ranges.size() / 2];

  // Moving toward the wall must shrink the center-ray range by at least 0.05 m.
  EXPECT_LT(range_after, range_before - 0.05) << "range_before=" << range_before << " range_after=" << range_after
                                              << " — expected center ray to shrink by at least 0.05 m";
  // robot_guard destructor removes robot_node from executor_.
}

// ─── Scenario 2 ──────────────────────────────────────────────────────────────

// Load simple_rooms.yaml (20 m × 15 m, 0.05 m/px) and spawn two robots in the
// open horizontal corridor at y = 7.0 (between internal wall bands at y ≈ 6.60
// and y ≈ 8.50).  At y = 7.0 there are no vertical wall dividers — the corridor
// spans from x ≈ 0.75 to x ≈ 18.90.  Both robots have over 0.2 m clearance.
//   Robot A: (3.0, 7.0, 0.0)  — faces +x, drives toward +x.
//   Robot B: (15.0, 7.0, π)   — faces -x, drives toward -x (linear_x = +0.2 in
//     body frame translates to -x in world frame because B faces π).
TEST_F(Ros2E2E, MultiRobotIndependentDrive)
{
  const std::string yaml_path = std::string(kResourcesDir) + "/maps/simple_rooms.yaml";
  load_map_into_server(yaml_path);

  const stdr_simulation::Pose2D spawn_a{ .x = 3.0, .y = 7.0, .theta = 0.0 };
  const stdr_simulation::Pose2D spawn_b{ .x = 15.0, .y = 7.0, .theta = M_PI };

  const std::string name_a = spawn_laser_robot(spawn_a);
  ASSERT_FALSE(name_a.empty()) << "SpawnRobot for robot A did not return a name";

  const std::string name_b = spawn_laser_robot(spawn_b);
  ASSERT_FALSE(name_b.empty()) << "SpawnRobot for robot B did not return a name";

  // Bring up robot node A — self-registers via RegisterRobot action.
  rclcpp::NodeOptions robot_options_a;
  robot_options_a.append_parameter_override("robot_name", name_a);
  std::shared_ptr<stdr_robot::StdrRobotNode> robot_node_a =
      std::make_shared<stdr_robot::StdrRobotNode>(robot_options_a);
  executor_.add_node(robot_node_a);

  // RAII guard for robot A — removes it on all exit paths, keeping the
  // shared_ptr alive so the executor cleanup has a valid node object.
  struct NodeGuard
  {
    rclcpp::executors::MultiThreadedExecutor& exec;
    std::shared_ptr<stdr_robot::StdrRobotNode> node;
    ~NodeGuard()
    {
      exec.remove_node(node);
    }
  } robot_guard_a{ executor_, robot_node_a };

  // Bring up robot node B with its own laser sensor config.
  rclcpp::NodeOptions robot_options_b;
  robot_options_b.append_parameter_override("robot_name", name_b);
  std::shared_ptr<stdr_robot::StdrRobotNode> robot_node_b =
      std::make_shared<stdr_robot::StdrRobotNode>(robot_options_b);
  executor_.add_node(robot_node_b);

  // RAII guard for robot B — same pattern as robot A above.
  NodeGuard robot_guard_b{ executor_, robot_node_b };

  // Wait for both robots to be observed by the backend.
  ASSERT_TRUE(wait_until([this]() { return backend_.num_robots() == 2; }, 5s))
      << "Backend did not observe both robots on active_robots topic";

  // Drive both robots forward in their respective body frames.
  backend_.set_cmd_vel(name_a, stdr_simulation::Twist2D{ .linear_x = 0.2, .linear_y = 0.0, .angular_z = 0.0 });
  backend_.set_cmd_vel(name_b, stdr_simulation::Twist2D{ .linear_x = 0.2, .linear_y = 0.0, .angular_z = 0.0 });

  // Wait for both robots to have moved at least 0.02 m in their intended
  // directions.  Signed checks ensure we detect backwards motion immediately
  // rather than accepting spurious movement in the wrong direction.
  //   Robot A faces +x: world x must increase by at least 0.02 m.
  //   Robot B faces -x: world x must decrease by at least 0.02 m.
  ASSERT_TRUE(wait_until(
      [this, &name_a, &name_b, &spawn_a, &spawn_b]() {
        const std::optional<stdr_simulation::Pose2D> pa = backend_.pose(name_a);
        const std::optional<stdr_simulation::Pose2D> pb = backend_.pose(name_b);
        if (!pa.has_value() || !pb.has_value())
        {
          return false;
        }
        const bool a_moved = (pa->x - spawn_a.x) >= 0.02;
        const bool b_moved = (spawn_b.x - pb->x) >= 0.02;
        return a_moved && b_moved;
      },
      5s))
      << "One or both robots failed to move 0.02 m within the deadline";

  const std::optional<stdr_simulation::Pose2D> final_a = backend_.pose(name_a);
  const std::optional<stdr_simulation::Pose2D> final_b = backend_.pose(name_b);
  ASSERT_TRUE(final_a.has_value());
  ASSERT_TRUE(final_b.has_value());

  // Robot A drove in the +x direction — its x must have increased.
  EXPECT_GT(final_a->x, spawn_a.x) << "Robot A did not advance in the expected +x direction";

  // Robot B faced -x and drove forward — its x must have decreased.
  EXPECT_LT(final_b->x, spawn_b.x) << "Robot B did not advance in the expected -x direction";

  // Independence check: neither robot should drift off the corridor (y stable).
  EXPECT_NEAR(final_a->y, spawn_a.y, 0.1) << "Robot A drifted off its corridor";
  EXPECT_NEAR(final_b->y, spawn_b.y, 0.1) << "Robot B drifted off its corridor";
  // robot_guard_a and robot_guard_b destructors remove both nodes from executor_.
}

}  // namespace
}  // namespace stdr_gui_ros
