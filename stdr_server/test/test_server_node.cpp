#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_server/stdr_server_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <stdr_msgs/action/delete_robot.hpp>
#include <stdr_msgs/action/spawn_robot.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace stdr_server
{
namespace
{

using ::testing::Contains;
using namespace std::chrono_literals;

// Spin the executor until predicate returns true or the timeout elapses.
// Returns true if predicate was satisfied before the timeout.
[[nodiscard]] bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                              const std::function<bool()>& predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate())
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return false;
    }
    executor.spin_some(50ms);
  }
  return true;
}

// ─── Fixture ──────────────────────────────────────────────────────────────────

class ServerNodeTest : public ::testing::Test
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

  ServerNodeTest() : node_(std::make_shared<StdrServerNode>())
  {
  }

  std::shared_ptr<StdrServerNode> node_;
};

// ─── Basic lifecycle ─────────────────────────────────────────────────────────

TEST_F(ServerNodeTest, CreatesSuccessfully)
{
  EXPECT_NE(node_, nullptr);
  EXPECT_EQ(node_->get_name(), std::string("stdr_server"));
}

// ─── Services ────────────────────────────────────────────────────────────────

TEST_F(ServerNodeTest, HasExpectedServices)
{
  // Collect the flat list of service names from the node.
  // get_service_names_and_types() returns a map<name, vector<type>>.
  const std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();

  std::vector<std::string> names;
  names.reserve(services.size());
  for (const auto& [name, types] : services)
  {
    names.push_back(name);
  }

  EXPECT_THAT(names, Contains("/stdr_server/load_static_map"));
  EXPECT_THAT(names, Contains("/stdr_server/load_external_map"));
  EXPECT_THAT(names, Contains("/stdr_server/move_robot"));
  EXPECT_THAT(names, Contains("/stdr_server/add_rfid_tag"));
  EXPECT_THAT(names, Contains("/stdr_server/delete_rfid_tag"));
  EXPECT_THAT(names, Contains("/stdr_server/add_co2_source"));
  EXPECT_THAT(names, Contains("/stdr_server/delete_co2_source"));
  EXPECT_THAT(names, Contains("/stdr_server/add_thermal_source"));
  EXPECT_THAT(names, Contains("/stdr_server/delete_thermal_source"));
  EXPECT_THAT(names, Contains("/stdr_server/add_sound_source"));
  EXPECT_THAT(names, Contains("/stdr_server/delete_sound_source"));
}

// ─── Actions ─────────────────────────────────────────────────────────────────

TEST_F(ServerNodeTest, HasExpectedActions)
{
  // Action servers expose a set of hidden services following the pattern
  // /<action>/{send_goal,cancel_goal,get_result}. Check the canonical names.
  const std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();

  std::vector<std::string> names;
  names.reserve(services.size());
  for (const auto& [name, types] : services)
  {
    names.push_back(name);
  }

  EXPECT_THAT(names, Contains("/stdr_server/spawn_robot/_action/send_goal"));
  EXPECT_THAT(names, Contains("/stdr_server/delete_robot/_action/send_goal"));
  EXPECT_THAT(names, Contains("/stdr_server/register_robot/_action/send_goal"));
}

// ─── Spawn and delete robot ───────────────────────────────────────────────────

TEST_F(ServerNodeTest, SpawnAndDeleteRobot)
{
  using SpawnRobot = stdr_msgs::action::SpawnRobot;
  using DeleteRobot = stdr_msgs::action::DeleteRobot;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  // ── Spawn ──────────────────────────────────────────────────────────────────
  auto spawn_client = rclcpp_action::create_client<SpawnRobot>(node_, "stdr_server/spawn_robot");

  ASSERT_TRUE(spin_until(
      executor, [&spawn_client]() { return spawn_client->action_server_is_ready(); }, 2s))
      << "Spawn action server never became ready";

  auto spawn_goal = SpawnRobot::Goal{};
  // A minimal robot description with no sensors is sufficient for spawning.
  spawn_goal.description.footprint.radius = 0.1f;

  std::string spawned_name;

  auto spawn_goal_handle_future = spawn_client->async_send_goal(spawn_goal);
  ASSERT_TRUE(spin_until(
      executor,
      [&spawn_goal_handle_future]() { return spawn_goal_handle_future.wait_for(0s) == std::future_status::ready; }, 2s))
      << "Spawn goal was not accepted";

  auto spawn_goal_handle = spawn_goal_handle_future.get();
  ASSERT_NE(spawn_goal_handle, nullptr) << "Spawn goal was rejected";

  auto spawn_result_future = spawn_client->async_get_result(spawn_goal_handle);
  ASSERT_TRUE(spin_until(
      executor, [&spawn_result_future]() { return spawn_result_future.wait_for(0s) == std::future_status::ready; }, 2s))
      << "Spawn action did not complete";

  const rclcpp_action::ClientGoalHandle<SpawnRobot>::WrappedResult spawn_result = spawn_result_future.get();
  ASSERT_EQ(spawn_result.code, rclcpp_action::ResultCode::SUCCEEDED)
      << "Spawn failed: " << spawn_result.result->message;

  spawned_name = spawn_result.result->indexed_description.name;
  EXPECT_FALSE(spawned_name.empty());

  // ── Delete ─────────────────────────────────────────────────────────────────

  auto delete_client = rclcpp_action::create_client<DeleteRobot>(node_, "stdr_server/delete_robot");

  ASSERT_TRUE(spin_until(
      executor, [&delete_client]() { return delete_client->action_server_is_ready(); }, 2s))
      << "Delete action server never became ready";

  auto delete_goal = DeleteRobot::Goal{};
  delete_goal.name = spawned_name;

  auto delete_goal_handle_future = delete_client->async_send_goal(delete_goal);
  ASSERT_TRUE(spin_until(
      executor,
      [&delete_goal_handle_future]() { return delete_goal_handle_future.wait_for(0s) == std::future_status::ready; },
      2s))
      << "Delete goal was not accepted";

  auto delete_goal_handle = delete_goal_handle_future.get();
  ASSERT_NE(delete_goal_handle, nullptr) << "Delete goal was rejected";

  auto delete_result_future = delete_client->async_get_result(delete_goal_handle);
  ASSERT_TRUE(spin_until(
      executor, [&delete_result_future]() { return delete_result_future.wait_for(0s) == std::future_status::ready; },
      2s))
      << "Delete action did not complete";

  const rclcpp_action::ClientGoalHandle<DeleteRobot>::WrappedResult delete_result = delete_result_future.get();
  EXPECT_EQ(delete_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(delete_result.result->success);
}

}  // namespace
}  // namespace stdr_server
