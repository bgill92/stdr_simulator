/**
 * Integration smoke test: StdrServerNode + StdrRobotNode + Ros2Backend in a
 * single process, exercising the full introspection and sensor-data pipeline
 * against the real server and robot nodes rather than synthetic mock publishers.
 *
 * Registration flow:
 *   1. Spawn robot config via SpawnRobot action  → server stores config in world model.
 *   2. Construct StdrRobotNode with the assigned name  → node sends RegisterRobot
 *      goal automatically in its constructor.
 *   3. Server returns config to robot node  → node calls configure() internally.
 *   4. Server publishes updated active_robots  → Ros2Backend picks it up and
 *      creates per-sensor subscriptions.
 *   5. Robot simulation timer fires  → publishes laser/sonar data with a map.
 *
 * Executor: MultiThreadedExecutor so that the robot's RegisterRobot action
 * client and the server's action server can exchange messages concurrently on
 * separate threads.  No tf2 spin threads are used to avoid deadlocks.
 */

#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_robot/stdr_robot_node.hpp>
#include <stdr_server/stdr_server_node.hpp>

#include <stdr_msgs/action/spawn_robot.hpp>
#include <stdr_msgs/srv/load_external_map.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
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

using ::testing::Contains;
using ::testing::IsEmpty;
using ::testing::SizeIs;
using namespace std::chrono_literals;

// ─── Helpers ──────────────────────────────────────────────────────────────────

// Build a minimal 100x100 all-free OccupancyGrid that gives the laser
// simulator a valid map to ray-cast into and produce non-infinite ranges.
[[nodiscard]] nav_msgs::msg::OccupancyGrid make_integration_map()
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = 100;
  map.info.height = 100;
  map.info.resolution = 0.05f;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.data.assign(static_cast<std::size_t>(100 * 100), 0);  // All free.
  return map;
}

// ─── Fixture ──────────────────────────────────────────────────────────────────

class Ros2BackendIntegrationTest : public ::testing::Test
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

  Ros2BackendIntegrationTest()
    : executor_(rclcpp::ExecutorOptions{}, /*number_of_threads=*/4)
    , server_node_(std::make_shared<stdr_server::StdrServerNode>())
    , backend_node_(std::make_shared<rclcpp::Node>("integration_test_backend"))
    , backend_(backend_node_)
  {
    executor_.add_node(server_node_);
    executor_.add_node(backend_node_);
    // Start the executor on a background thread so spin_until() calls on the
    // test thread can block and let the background thread deliver messages.
    spinner_ = std::thread([this]() { executor_.spin(); });
  }

  ~Ros2BackendIntegrationTest() override
  {
    executor_.cancel();
    if (spinner_.joinable())
    {
      spinner_.join();
    }
  }

  // Spin the fixture's executor in spin_some bursts until the predicate fires.
  // This variant is safe to call from the test thread while the background
  // spinner is running — spin_some is not called here; we just sleep-poll.
  [[nodiscard]] bool wait_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout) const
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

  // Load the map into the server via the LoadExternalMap service.
  void load_map_into_server()
  {
    // Use a throw-away node on the same executor to call the service.
    // The service call must not block the executor, so we use async_send_request.
    auto client_node = std::make_shared<rclcpp::Node>("integration_map_loader");
    executor_.add_node(client_node);

    const auto client = client_node->create_client<stdr_msgs::srv::LoadExternalMap>("stdr_server/load_external_map");

    ASSERT_TRUE(wait_until([&client]() { return client->service_is_ready(); }, 3s))
        << "LoadExternalMap service never became ready";

    auto request = std::make_shared<stdr_msgs::srv::LoadExternalMap::Request>();
    request->map = make_integration_map();

    auto future = client->async_send_request(request);
    ASSERT_TRUE(wait_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }, 3s))
        << "LoadExternalMap service call did not complete";

    executor_.remove_node(client_node);
  }

  // Send a SpawnRobot action goal with one laser and one sonar.
  // Returns the assigned robot name, or empty on failure.
  [[nodiscard]] std::string spawn_robot_with_sensors()
  {
    using SpawnRobot = stdr_msgs::action::SpawnRobot;

    auto action_client_node = std::make_shared<rclcpp::Node>("integration_spawn_client");
    executor_.add_node(action_client_node);

    // RAII guard ensures the node is removed from the executor on every return
    // path, including those triggered by fatal assertion failures (ASSERT_*).
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

    // One laser sensor.
    stdr_msgs::msg::LaserSensorMsg laser_msg;
    laser_msg.frame_id = "laser0";
    laser_msg.max_angle = 1.57f;
    laser_msg.min_angle = -1.57f;
    laser_msg.max_range = 5.0f;
    laser_msg.min_range = 0.05f;
    laser_msg.num_rays = 10;
    laser_msg.frequency = 10.0f;
    goal.description.laser_sensors.push_back(laser_msg);

    // One sonar sensor.
    stdr_msgs::msg::SonarSensorMsg sonar_msg;
    sonar_msg.frame_id = "sonar0";
    sonar_msg.max_range = 5.0f;
    sonar_msg.min_range = 0.05f;
    sonar_msg.cone_angle = 0.5f;
    sonar_msg.frequency = 10.0f;
    goal.description.sonar_sensors.push_back(sonar_msg);

    // Initial pose at a spot well inside the 100x100 (5mx5m) free map.
    goal.description.initial_pose.x = 2.0;
    goal.description.initial_pose.y = 2.0;
    goal.description.initial_pose.theta = 0.0;

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
      return {};
    }
    return spawn_result.result->indexed_description.name;
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<stdr_server::StdrServerNode> server_node_;
  std::shared_ptr<stdr_robot::StdrRobotNode> robot_node_;
  std::shared_ptr<rclcpp::Node> backend_node_;
  Ros2Backend backend_;
  std::thread spinner_;
};

// ─── Integration smoke test ───────────────────────────────────────────────────

TEST_F(Ros2BackendIntegrationTest, BackendObservesSpawnedRobotAndSensorData)
{
  // ── Step 1: Load a map into the server ──────────────────────────────────────
  load_map_into_server();

  // ── Step 2: Spawn a robot with one laser and one sonar ──────────────────────
  const std::string robot_name = spawn_robot_with_sensors();
  ASSERT_FALSE(robot_name.empty()) << "SpawnRobot action did not return a name";

  const std::string laser_frame_id = "laser0";
  const std::string sonar_frame_id = "sonar0";

  // ── Step 3: Bring up the robot node (it will self-register via RegisterRobot)
  // The robot node is added to the already-spinning executor so that its
  // RegisterRobot action client and the server action server can exchange
  // messages on the background threads.
  rclcpp::NodeOptions robot_options;
  robot_options.append_parameter_override("robot_name", robot_name);
  robot_node_ = std::make_shared<stdr_robot::StdrRobotNode>(robot_options);
  executor_.add_node(robot_node_);

  // ── Step 4: Wait for the backend to observe the spawned robot ────────────────
  // The server publishes active_robots when SpawnRobot succeeds; the backend
  // subscribes with transient-local QoS so it will receive the latched message
  // once its subscription is matched.
  ASSERT_TRUE(wait_until([this, &robot_name]() { return backend_.num_robots() > 0; }, 5s))
      << "Backend never observed any robots on active_robots topic";

  // ── Step 5: Assert introspection ────────────────────────────────────────────
  const std::vector<std::string> ids = backend_.robot_ids();
  ASSERT_THAT(ids, SizeIs(1));
  EXPECT_THAT(ids, Contains(robot_name));

  const std::vector<std::string> lasers = backend_.laser_sensors(robot_name);
  ASSERT_THAT(lasers, SizeIs(1));
  EXPECT_EQ(lasers[0], laser_frame_id);

  const std::vector<std::string> sonars = backend_.sonar_sensors(robot_name);
  ASSERT_THAT(sonars, SizeIs(1));
  EXPECT_EQ(sonars[0], sonar_frame_id);

  // ── Step 6: Wait for laser data to arrive ───────────────────────────────────
  // The robot simulation timer fires at 10 Hz.  Allow 5 s for registration to
  // complete, publishers/subscriptions to match, and the first scan to arrive.
  ASSERT_TRUE(wait_until([this, &robot_name,
                          &laser_frame_id]() { return backend_.latest_laser(robot_name, laser_frame_id).has_value(); },
                         5s))
      << "Timed out waiting for laser scan from the robot node";

  // ── Step 7: Wait for sonar data to arrive ───────────────────────────────────
  ASSERT_TRUE(wait_until([this, &robot_name,
                          &sonar_frame_id]() { return backend_.latest_sonar(robot_name, sonar_frame_id).has_value(); },
                         5s))
      << "Timed out waiting for sonar reading from the robot node";

  // ── Step 8: Verify poll_laser_events drains at least one scan ───────────────
  std::uint64_t laser_cursor = 0;
  const stdr_gui::DrainedLaserResult laser_result =
      backend_.poll_laser_events(laser_cursor, robot_name, laser_frame_id);
  ASSERT_FALSE(laser_result.scans.empty()) << "poll_laser_events returned no scans";
  // Robot starts at (2.0, 2.0) on a fully-free map with max_range = 5.0.
  // No obstacle is within range, so all rays must return exactly max_range.
  ASSERT_THAT(laser_result.scans[0].scan.ranges, ::testing::Each(::testing::FloatEq(5.0f)))
      << "Expected all laser rays to return max_range on an obstacle-free map";

  // ── Step 9: Verify poll_sonar_events drains at least one reading ─────────────
  std::uint64_t sonar_cursor = 0;
  const stdr_gui::DrainedSonarResult sonar_result =
      backend_.poll_sonar_events(sonar_cursor, robot_name, sonar_frame_id);
  EXPECT_FALSE(sonar_result.scans.empty()) << "poll_sonar_events returned no readings";

  // ── Cleanup: remove robot node from executor before it is destroyed ──────────
  executor_.remove_node(robot_node_);
}

}  // namespace
}  // namespace stdr_gui_ros
