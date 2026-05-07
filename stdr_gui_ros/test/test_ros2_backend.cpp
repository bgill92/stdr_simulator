#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_gui/simulator_backend.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <memory>

namespace stdr_gui_ros
{
namespace
{

using ::testing::SizeIs;

// ─── Fixture ──────────────────────────────────────────────────────────────────

class Ros2BackendTest : public ::testing::Test
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

  Ros2BackendTest() : node_(std::make_shared<rclcpp::Node>("test_stdr_gui")), backend_(node_)
  {
  }

  std::shared_ptr<rclcpp::Node> node_;
  Ros2Backend backend_;
};

// ─── Construction ─────────────────────────────────────────────────────────────

TEST_F(Ros2BackendTest, DefaultStepDtIsKDefaultStepDt)
{
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kDefaultStepDt);
}

// ─── set_step_dt clamping ─────────────────────────────────────────────────────

TEST_F(Ros2BackendTest, SetStepDtClampsBelowMin)
{
  backend_.set_step_dt(0.0);
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kMinStepDt);
}

TEST_F(Ros2BackendTest, SetStepDtClampsAboveMax)
{
  backend_.set_step_dt(100.0);
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kMaxStepDt);
}

// ─── Map subscription integration test ───────────────────────────────────────

// Publishes a latched OccupancyGrid on "map" from a separate helper node,
// lets the backend's map subscription receive it, then checks that
// get_snapshot() reflects the published dimensions. This exercises the real
// ROS2 topic plumbing without needing a stdr_server process running.
TEST_F(Ros2BackendTest, MapSubscriptionUpdatesSnapshot)
{
  // Publisher node that mimics stdr_server's latched map topic.
  std::shared_ptr<rclcpp::Node> pub_node = std::make_shared<rclcpp::Node>("test_map_publisher");

  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub =
      pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", latched_qos);

  // Build a minimal map message.
  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.info.width = 50;
  map_msg.info.height = 40;
  map_msg.info.resolution = 0.05f;
  map_msg.info.origin.position.x = -1.25;
  map_msg.info.origin.position.y = -1.0;
  map_msg.data.assign(static_cast<std::size_t>(50 * 40), 0);

  map_pub->publish(map_msg);

  // Spin both the publisher node and the backend node so the latched message
  // is delivered and the backend's subscription callback fires.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(node_);

  // Spin long enough for the transient-local delivery to complete. A tight
  // deadline is fine here; the message is queued immediately by the middleware.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));

    // Exit early once the snapshot reflects the published map.
    const std::shared_ptr<const stdr_gui::SimulationSnapshot> snapshot = backend_.get_snapshot();
    if (snapshot->map.width == 50)
    {
      break;
    }
  }

  const std::shared_ptr<const stdr_gui::SimulationSnapshot> snapshot = backend_.get_snapshot();
  ASSERT_EQ(snapshot->map.width, 50);
  EXPECT_EQ(snapshot->map.height, 40);
  // resolution is stored as float32 in OccupancyGrid; compare as float to
  // avoid false failures from float→double widening.
  EXPECT_FLOAT_EQ(snapshot->map.resolution, 0.05f);
  ASSERT_THAT(snapshot->map.data, SizeIs(50 * 40));
}

}  // namespace
}  // namespace stdr_gui_ros
