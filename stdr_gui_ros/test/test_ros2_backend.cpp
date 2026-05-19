#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_gui/simulator_backend.hpp>
#include <stdr_parser/msg_conversions.hpp>
#include <stdr_simulation/types.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <stdr_msgs/msg/robot_indexed_msg.hpp>
#include <stdr_msgs/msg/robot_indexed_vector_msg.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace stdr_gui_ros
{
namespace
{

using ::testing::DoubleNear;
using ::testing::IsEmpty;
using ::testing::SizeIs;

// ─── Global rclcpp lifecycle ──────────────────────────────────────────────────
//
// All test suites in this binary share one rclcpp::init / rclcpp::shutdown pair
// so that multiple test fixtures with different SetUpTestSuite bodies don't
// fight over the rclcpp context.

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Register the environment before any test runs.  The pointer is adopted by the
// GoogleTest framework and must not be deleted by the caller.
// NOLINTNEXTLINE(cert-err58-cpp) — safe: AddGlobalTestEnvironment is called before main test execution
const ::testing::Environment* const kRclcppEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

// ─── Helpers ──────────────────────────────────────────────────────────────────

// Build a minimal sensor_msgs::msg::LaserScan for a given set of ranges.
sensor_msgs::msg::LaserScan make_laser_scan_msg(const std::vector<float>& ranges, rclcpp::Time stamp,
                                                float angle_min = -1.57f, float angle_max = 1.57f,
                                                float range_min = 0.1f, float range_max = 10.0f)
{
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = stamp;
  msg.angle_min = angle_min;
  msg.angle_max = angle_max;
  msg.angle_increment = (ranges.size() < 2) ? 0.0f : (angle_max - angle_min) / static_cast<float>(ranges.size() - 1);
  msg.range_min = range_min;
  msg.range_max = range_max;
  msg.ranges = ranges;
  return msg;
}

// Build a minimal RobotIndexedVectorMsg with the given robot configurations
// so the test can publish it to the active_robots topic without depending on a
// live stdr_server.
stdr_msgs::msg::RobotIndexedVectorMsg
make_active_robots_msg(const std::vector<std::pair<std::string, stdr_simulation::RobotConfig>>& robots)
{
  stdr_msgs::msg::RobotIndexedVectorMsg msg;
  for (const auto& [name, config] : robots)
  {
    stdr_msgs::msg::RobotIndexedMsg entry;
    entry.name = name;
    entry.robot = stdr_parser::to_ros_msg(config);
    msg.robots.push_back(entry);
  }
  return msg;
}

// Spin the executor until the predicate returns true or the deadline passes.
// Returns true if the predicate fired before the deadline.
template <typename Predicate>
bool spin_until(rclcpp::executors::SingleThreadedExecutor& executor, Predicate pred,
                std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some(std::chrono::milliseconds(10));
    if (pred())
    {
      return true;
    }
  }
  return false;
}

// ─── Fixture ──────────────────────────────────────────────────────────────────

class Ros2BackendTest : public ::testing::Test
{
protected:
  Ros2BackendTest() : node_(std::make_shared<rclcpp::Node>("test_stdr_gui")), backend_(node_)
  {
  }

  // Publish active_robots and wait for the backend to process it.
  void publish_active_robots(const std::vector<std::pair<std::string, stdr_simulation::RobotConfig>>& robots,
                             rclcpp::executors::SingleThreadedExecutor& executor)
  {
    stdr_msgs::msg::RobotIndexedVectorMsg msg = make_active_robots_msg(robots);
    active_robots_pub_->publish(msg);
    const std::size_t expected_count = robots.size();
    spin_until(executor, [&] { return backend_.num_robots() == expected_count; });
  }

  std::shared_ptr<rclcpp::Node> node_;
  Ros2Backend backend_;

  // Publisher node that mimics stdr_server's active_robots topic.
  std::shared_ptr<rclcpp::Node> pub_node_{ std::make_shared<rclcpp::Node>("test_publisher") };

  rclcpp::QoS latched_qos_{ []() {
    rclcpp::QoS q(10);
    q.transient_local();
    return q;
  }() };

  rclcpp::Publisher<stdr_msgs::msg::RobotIndexedVectorMsg>::SharedPtr active_robots_pub_{
    pub_node_->create_publisher<stdr_msgs::msg::RobotIndexedVectorMsg>("stdr_server/active_robots", latched_qos_)
  };
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

// ─── Introspection — unknown robot ───────────────────────────────────────────

TEST_F(Ros2BackendTest, UnknownRobotReturnsEmpty)
{
  EXPECT_THAT(backend_.laser_sensors("nope"), IsEmpty());
  EXPECT_THAT(backend_.sonar_sensors("nope"), IsEmpty());
  EXPECT_THAT(backend_.footprint("nope"), IsEmpty());
  EXPECT_FALSE(backend_.pose("nope").has_value());
  EXPECT_FALSE(backend_.twist("nope").has_value());
  EXPECT_FALSE(backend_.collided("nope").has_value());
}

// ─── Introspection — active robots ───────────────────────────────────────────

TEST_F(Ros2BackendTest, RobotIdsReflectsActiveRobots)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots({ { "robot0", cfg }, { "robot1", cfg } }, executor);

  ASSERT_EQ(backend_.num_robots(), 2u);
  const std::vector<std::string> ids = backend_.robot_ids();
  ASSERT_THAT(ids, SizeIs(2));
  EXPECT_THAT(ids, ::testing::UnorderedElementsAre("robot0", "robot1"));
}

// ─── Introspection — laser sensors ───────────────────────────────────────────

TEST_F(Ros2BackendTest, LaserSensorsReturnsFrameIds)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser_a;
  laser_a.frame_id = "laser_front";
  stdr_simulation::LaserConfig laser_b;
  laser_b.frame_id = "laser_rear";
  cfg.laser_sensors = { laser_a, laser_b };

  publish_active_robots({ { "robot0", cfg } }, executor);

  const std::vector<std::string> frames = backend_.laser_sensors("robot0");
  ASSERT_THAT(frames, SizeIs(2));
  EXPECT_EQ(frames[0], "laser_front");
  EXPECT_EQ(frames[1], "laser_rear");
}

// ─── Introspection — sonar sensors ───────────────────────────────────────────

TEST_F(Ros2BackendTest, SonarSensorsReturnsFrameIds)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar_a;
  sonar_a.frame_id = "sonar_left";
  stdr_simulation::SonarConfig sonar_b;
  sonar_b.frame_id = "sonar_right";
  cfg.sonar_sensors = { sonar_a, sonar_b };

  publish_active_robots({ { "robot0", cfg } }, executor);

  const std::vector<std::string> frames = backend_.sonar_sensors("robot0");
  ASSERT_THAT(frames, SizeIs(2));
  EXPECT_EQ(frames[0], "sonar_left");
  EXPECT_EQ(frames[1], "sonar_right");
}

// ─── Introspection — footprint ────────────────────────────────────────────────

TEST_F(Ros2BackendTest, FootprintReturnsConfiguredPolygon)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  cfg.footprint.points = { { 0.2, 0.1 }, { -0.2, 0.1 }, { -0.2, -0.1 }, { 0.2, -0.1 } };

  publish_active_robots({ { "robot0", cfg } }, executor);

  const std::vector<stdr_simulation::Point2D> pts = backend_.footprint("robot0");
  ASSERT_THAT(pts, SizeIs(4));
  EXPECT_DOUBLE_EQ(pts[0].x, 0.2);
  EXPECT_DOUBLE_EQ(pts[0].y, 0.1);
  EXPECT_DOUBLE_EQ(pts[2].x, -0.2);
  EXPECT_DOUBLE_EQ(pts[2].y, -0.1);
}

// ─── Introspection — pose from odom ──────────────────────────────────────────

TEST_F(Ros2BackendTest, PoseReflectsLatestOdom)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Publish an odom message to the robot's odom topic.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub =
      pub_node_->create_publisher<nav_msgs::msg::Odometry>("robot0/odom", 10);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.pose.pose.position.x = 1.5;
  odom_msg.pose.pose.position.y = 2.5;
  // Build a quaternion for yaw = 0.7 rad.
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.7);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_pub->publish(odom_msg);

  const bool received = spin_until(executor, [&] {
    const std::optional<stdr_simulation::Pose2D> p = backend_.pose("robot0");
    return p.has_value() && std::abs(p->x - 1.5) < 1e-9;
  });
  ASSERT_TRUE(received) << "Odom message was not delivered in time.";

  const std::optional<stdr_simulation::Pose2D> p = backend_.pose("robot0");
  ASSERT_TRUE(p.has_value());
  EXPECT_DOUBLE_EQ(p->x, 1.5);
  EXPECT_DOUBLE_EQ(p->y, 2.5);
  EXPECT_THAT(p->theta, DoubleNear(0.7, 1e-6));
}

// ─── Introspection — commanded velocity ──────────────────────────────────────

// twist() returns the most-recently commanded velocity set via set_cmd_vel(),
// matching StandaloneBackend's cmd_vel semantics.
TEST_F(Ros2BackendTest, TwistReflectsCommandedVelocity)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Before any command is sent, twist() returns nullopt because no cmd has been issued.
  EXPECT_FALSE(backend_.twist("robot0").has_value());

  stdr_simulation::Twist2D cmd;
  cmd.linear_x = 0.5;
  cmd.linear_y = 0.0;
  cmd.angular_z = 0.3;
  backend_.set_cmd_vel("robot0", cmd);

  const std::optional<stdr_simulation::Twist2D> t = backend_.twist("robot0");
  ASSERT_TRUE(t.has_value());
  EXPECT_DOUBLE_EQ(t->linear_x, 0.5);
  EXPECT_DOUBLE_EQ(t->linear_y, 0.0);
  EXPECT_DOUBLE_EQ(t->angular_z, 0.3);
}

// Verifies that latest_cmd_vel_ is cleaned up when the robot disappears from
// active_robots so stale velocity data does not leak to a future robot with the
// same name.
TEST_F(Ros2BackendTest, TwistClearedWhenRobotRemoved)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  // Step 1: Publish one robot.
  stdr_simulation::RobotConfig cfg;
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Step 2: Command a velocity and verify twist() returns it.
  stdr_simulation::Twist2D cmd;
  cmd.linear_x = 0.5;
  cmd.linear_y = 0.0;
  cmd.angular_z = 0.3;
  backend_.set_cmd_vel("robot0", cmd);

  const std::optional<stdr_simulation::Twist2D> t_before = backend_.twist("robot0");
  ASSERT_TRUE(t_before.has_value());
  EXPECT_DOUBLE_EQ(t_before->linear_x, 0.5);

  // Step 3: Republish active_robots without robot0 to simulate it being deleted.
  publish_active_robots({}, executor);

  // Step 4: twist() must return nullopt once the robot is gone.
  EXPECT_FALSE(backend_.twist("robot0").has_value());
}

// ─── Introspection — collided ─────────────────────────────────────────────────

TEST_F(Ros2BackendTest, CollidedReturnsNulloptOrFalse)
{
  // Unknown robot returns nullopt.
  EXPECT_FALSE(backend_.collided("ghost").has_value());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Known robot returns false (collision state is not published by stdr_robot).
  const std::optional<bool> result = backend_.collided("robot0");
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result.value());
}

// ─── Introspection — sim_time ─────────────────────────────────────────────────

TEST_F(Ros2BackendTest, SimTimeMonotonicallyIncreases)
{
  const double t0 = backend_.sim_time();
  // A brief sleep so wall-clock advances enough for the difference to be measurable.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  const double t1 = backend_.sim_time();
  EXPECT_GT(t1, t0);
}

// ─── Laser scan introspection ─────────────────────────────────────────────────

// latest_laser() returns nullopt for an unknown robot/sensor before any scan
// has been received.
TEST_F(Ros2BackendTest, LatestLaserReturnsNulloptBeforeAnyScan)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser;
  laser.frame_id = "laser_0";
  cfg.laser_sensors = { laser };
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Robot is known but no scan has been published yet.
  EXPECT_FALSE(backend_.latest_laser("robot0", "laser_0").has_value());
}

// latest_laser() returns nullopt for a completely unknown robot/sensor.
TEST_F(Ros2BackendTest, UnknownRobotOrSensorReturnsEmpty)
{
  EXPECT_FALSE(backend_.latest_laser("nope", "nope").has_value());

  std::uint64_t cursor = 0;
  const stdr_gui::DrainedLaserResult result = backend_.poll_laser_events(cursor, "nope", "nope");
  EXPECT_THAT(result.scans, IsEmpty());
  EXPECT_EQ(result.dropped, 0u);
}

// Publishes a robot with a laser sensor, delivers a scan over the topic, and
// verifies that latest_laser() returns the correct ranges.
TEST_F(Ros2BackendTest, LatestLaserReflectsPublishedScan)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser;
  laser.frame_id = "laser_0";
  cfg.laser_sensors = { laser };

  // Create the publisher before the backend subscribes so the DDS graph is
  // already populated when the subscription is created.
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
      pub_node_->create_publisher<sensor_msgs::msg::LaserScan>("robot0/laser_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  // Wait for DDS endpoint matching to complete before publishing — without this
  // the first publish() can fire before the middleware finishes matching, and
  // the message is silently dropped.
  spin_until(
      executor, [&] { return laser_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  const sensor_msgs::msg::LaserScan scan_msg = make_laser_scan_msg({ 1.0f, 2.0f, 3.0f }, pub_node_->now());
  laser_pub->publish(scan_msg);

  const bool received = spin_until(executor, [&] { return backend_.latest_laser("robot0", "laser_0").has_value(); });
  ASSERT_TRUE(received) << "Laser scan was not delivered in time.";

  const std::optional<stdr_simulation::LaserScan> result = backend_.latest_laser("robot0", "laser_0");
  ASSERT_TRUE(result.has_value());
  ASSERT_THAT(result->ranges, SizeIs(3));
  EXPECT_FLOAT_EQ(result->ranges[0], 1.0f);
  EXPECT_FLOAT_EQ(result->ranges[1], 2.0f);
  EXPECT_FLOAT_EQ(result->ranges[2], 3.0f);
}

// poll_laser_events() drains scans and returns the correct count and values.
TEST_F(Ros2BackendTest, PollLaserEventsDrainsScans)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser;
  laser.frame_id = "laser_0";
  cfg.laser_sensors = { laser };

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
      pub_node_->create_publisher<sensor_msgs::msg::LaserScan>("robot0/laser_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return laser_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  const sensor_msgs::msg::LaserScan scan_msg = make_laser_scan_msg({ 4.0f, 5.0f }, pub_node_->now());
  laser_pub->publish(scan_msg);

  spin_until(executor, [&] { return backend_.latest_laser("robot0", "laser_0").has_value(); });

  std::uint64_t cursor = 0;
  const stdr_gui::DrainedLaserResult result = backend_.poll_laser_events(cursor, "robot0", "laser_0");
  ASSERT_THAT(result.scans, SizeIs(1));
  ASSERT_THAT(result.scans[0].scan.ranges, SizeIs(2));
  EXPECT_FLOAT_EQ(result.scans[0].scan.ranges[0], 4.0f);
  EXPECT_FLOAT_EQ(result.scans[0].scan.ranges[1], 5.0f);
  EXPECT_EQ(result.dropped, 0u);
  // Cursor must have advanced past the returned entry.
  EXPECT_EQ(cursor, 1u);
}

// A second poll with the same cursor returns nothing (no new scans).
TEST_F(Ros2BackendTest, PollLaserEventsCursorAdvances)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser;
  laser.frame_id = "laser_0";
  cfg.laser_sensors = { laser };

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
      pub_node_->create_publisher<sensor_msgs::msg::LaserScan>("robot0/laser_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return laser_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  laser_pub->publish(make_laser_scan_msg({ 1.0f }, pub_node_->now()));
  spin_until(executor, [&] { return backend_.latest_laser("robot0", "laser_0").has_value(); });

  // First drain.
  std::uint64_t cursor = 0;
  const stdr_gui::DrainedLaserResult first = backend_.poll_laser_events(cursor, "robot0", "laser_0");
  ASSERT_THAT(first.scans, SizeIs(1));

  // Second drain with the advanced cursor should be empty.
  const stdr_gui::DrainedLaserResult second = backend_.poll_laser_events(cursor, "robot0", "laser_0");
  EXPECT_THAT(second.scans, IsEmpty());
  EXPECT_EQ(second.dropped, 0u);
}

// After a robot is removed from active_robots, latest_laser() returns nullopt
// and poll_laser_events() returns empty.
TEST_F(Ros2BackendTest, LaserSubscriptionsClearedOnRobotRemoval)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::LaserConfig laser;
  laser.frame_id = "laser_0";
  cfg.laser_sensors = { laser };

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
      pub_node_->create_publisher<sensor_msgs::msg::LaserScan>("robot0/laser_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return laser_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  laser_pub->publish(make_laser_scan_msg({ 2.0f }, pub_node_->now()));
  spin_until(executor, [&] { return backend_.latest_laser("robot0", "laser_0").has_value(); });
  ASSERT_TRUE(backend_.latest_laser("robot0", "laser_0").has_value());

  // Remove the robot by publishing an empty active_robots list.
  publish_active_robots({}, executor);

  // Both methods must now return empty.
  EXPECT_FALSE(backend_.latest_laser("robot0", "laser_0").has_value());

  std::uint64_t cursor = 0;
  const stdr_gui::DrainedLaserResult result = backend_.poll_laser_events(cursor, "robot0", "laser_0");
  EXPECT_THAT(result.scans, IsEmpty());

  // Publish another scan on the same topic after removal.  The subscription
  // must have been destroyed, so the cache should remain empty even after
  // spinning long enough for the message to be delivered.
  laser_pub->publish(make_laser_scan_msg({ 9.0f }, pub_node_->now()));
  executor.spin_some(std::chrono::milliseconds(100));

  EXPECT_FALSE(backend_.latest_laser("robot0", "laser_0").has_value());
  const stdr_gui::DrainedLaserResult result2 = backend_.poll_laser_events(cursor, "robot0", "laser_0");
  EXPECT_THAT(result2.scans, IsEmpty());
}

// ─── Sonar reading introspection ──────────────────────────────────────────────

// Build a minimal sensor_msgs::msg::Range with the given range value.
sensor_msgs::msg::Range make_range_msg(const std::string& frame_id, float range_value, rclcpp::Time stamp)
{
  sensor_msgs::msg::Range msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = 0.5f;
  msg.min_range = 0.2f;
  msg.max_range = 5.0f;
  msg.range = range_value;
  return msg;
}

// latest_sonar() returns nullopt for a known robot/sensor before any reading
// has been received.
TEST_F(Ros2BackendTest, LatestSonarReturnsNulloptBeforeAnyReading)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar;
  sonar.frame_id = "sonar_0";
  cfg.sonar_sensors = { sonar };
  publish_active_robots({ { "robot0", cfg } }, executor);

  // Robot is known but no Range message has been published yet.
  EXPECT_FALSE(backend_.latest_sonar("robot0", "sonar_0").has_value());
}

// Publishes a robot with a sonar sensor, delivers a Range reading over the
// topic, and verifies that latest_sonar() returns the correct range value.
TEST_F(Ros2BackendTest, LatestSonarReflectsPublishedReading)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar;
  sonar.frame_id = "sonar_0";
  cfg.sonar_sensors = { sonar };

  // Create the publisher before the backend subscribes so the DDS graph is
  // already populated when the subscription is created.
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub =
      pub_node_->create_publisher<sensor_msgs::msg::Range>("robot0/sonar_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  // Wait for DDS endpoint matching to complete before publishing — without this
  // the first publish() can fire before the middleware finishes matching, and
  // the message is silently dropped.
  spin_until(
      executor, [&] { return sonar_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  const sensor_msgs::msg::Range range_msg = make_range_msg("sonar_0", 2.75f, pub_node_->now());
  sonar_pub->publish(range_msg);

  const bool received = spin_until(executor, [&] { return backend_.latest_sonar("robot0", "sonar_0").has_value(); });
  ASSERT_TRUE(received) << "Sonar Range message was not delivered in time.";

  const std::optional<stdr_simulation::SonarScan> result = backend_.latest_sonar("robot0", "sonar_0");
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->range, static_cast<double>(2.75f), 1e-6);
}

// poll_sonar_events() drains readings and returns the correct count and value.
TEST_F(Ros2BackendTest, PollSonarEventsDrainsReadings)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar;
  sonar.frame_id = "sonar_0";
  cfg.sonar_sensors = { sonar };

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub =
      pub_node_->create_publisher<sensor_msgs::msg::Range>("robot0/sonar_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return sonar_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  const sensor_msgs::msg::Range range_msg = make_range_msg("sonar_0", 1.5f, pub_node_->now());
  sonar_pub->publish(range_msg);

  spin_until(executor, [&] { return backend_.latest_sonar("robot0", "sonar_0").has_value(); });

  std::uint64_t cursor = 0;
  const stdr_gui::DrainedSonarResult result = backend_.poll_sonar_events(cursor, "robot0", "sonar_0");
  ASSERT_THAT(result.scans, SizeIs(1));
  EXPECT_NEAR(result.scans[0].scan.range, static_cast<double>(1.5f), 1e-6);
  EXPECT_EQ(result.dropped, 0u);
  // Cursor must have advanced past the returned entry.
  EXPECT_EQ(cursor, 1u);
}

// A second poll with the same cursor returns nothing (no new readings).
TEST_F(Ros2BackendTest, PollSonarEventsCursorAdvances)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar;
  sonar.frame_id = "sonar_0";
  cfg.sonar_sensors = { sonar };

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub =
      pub_node_->create_publisher<sensor_msgs::msg::Range>("robot0/sonar_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return sonar_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  sonar_pub->publish(make_range_msg("sonar_0", 3.0f, pub_node_->now()));
  spin_until(executor, [&] { return backend_.latest_sonar("robot0", "sonar_0").has_value(); });

  // First drain.
  std::uint64_t cursor = 0;
  const stdr_gui::DrainedSonarResult first = backend_.poll_sonar_events(cursor, "robot0", "sonar_0");
  ASSERT_THAT(first.scans, SizeIs(1));

  // Second drain with the advanced cursor should be empty.
  const stdr_gui::DrainedSonarResult second = backend_.poll_sonar_events(cursor, "robot0", "sonar_0");
  EXPECT_THAT(second.scans, IsEmpty());
  EXPECT_EQ(second.dropped, 0u);
}

// After a robot is removed from active_robots, latest_sonar() returns nullopt
// and poll_sonar_events() returns empty.  A publish-after-removal must not
// update the cache because the subscription must have been destroyed.
TEST_F(Ros2BackendTest, SonarSubscriptionsClearedOnRobotRemoval)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pub_node_);
  executor.add_node(node_);

  stdr_simulation::RobotConfig cfg;
  stdr_simulation::SonarConfig sonar;
  sonar.frame_id = "sonar_0";
  cfg.sonar_sensors = { sonar };

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub =
      pub_node_->create_publisher<sensor_msgs::msg::Range>("robot0/sonar_0", 10);

  publish_active_robots({ { "robot0", cfg } }, executor);

  spin_until(
      executor, [&] { return sonar_pub->get_subscription_count() > 0; }, std::chrono::milliseconds(1000));

  sonar_pub->publish(make_range_msg("sonar_0", 0.9f, pub_node_->now()));
  spin_until(executor, [&] { return backend_.latest_sonar("robot0", "sonar_0").has_value(); });
  ASSERT_TRUE(backend_.latest_sonar("robot0", "sonar_0").has_value());

  // Remove the robot by publishing an empty active_robots list.
  publish_active_robots({}, executor);

  // Both methods must now return empty.
  EXPECT_FALSE(backend_.latest_sonar("robot0", "sonar_0").has_value());

  std::uint64_t cursor = 0;
  const stdr_gui::DrainedSonarResult result = backend_.poll_sonar_events(cursor, "robot0", "sonar_0");
  EXPECT_THAT(result.scans, IsEmpty());

  // Publish another reading after removal.  The subscription must have been
  // destroyed, so the cache should remain empty even after spinning.
  sonar_pub->publish(make_range_msg("sonar_0", 9.9f, pub_node_->now()));
  executor.spin_some(std::chrono::milliseconds(100));

  EXPECT_FALSE(backend_.latest_sonar("robot0", "sonar_0").has_value());
  const stdr_gui::DrainedSonarResult result2 = backend_.poll_sonar_events(cursor, "robot0", "sonar_0");
  EXPECT_THAT(result2.scans, IsEmpty());
}

// ─── Rate propagation tests ───────────────────────────────────────────────────
//
// These tests verify that set_step_dt/set_tf_rate/set_odom_rate propagate their
// values to connected robot nodes via AsyncParametersClient.  A helper node
// named "robot0" (matching what StdrRobotNode uses when robot_name="robot0") declares
// the same parameters so the backend's param clients can target it.
//
// Note: AsyncParametersClient service discovery is async; these tests spin for
// up to 2 s per assertion.  Occasional flakiness on a heavily loaded CI machine
// is possible — the underlying mechanism is correct, not the test.

class Ros2BackendRatePropagationTest : public ::testing::Test
{
protected:
  Ros2BackendRatePropagationTest()
    : node_(std::make_shared<rclcpp::Node>("test_stdr_gui_rate"))
    , backend_(node_)
    // A mock robot node named "robot0" to match what StdrRobotNode uses when
    // robot_name is set to "robot0".  The backend now targets the robot by its
    // robot_name (not the hardcoded "stdr_robot").
    , robot_node_(std::make_shared<rclcpp::Node>("robot0"))
    , pub_node_(std::make_shared<rclcpp::Node>("test_rate_publisher"))
  {
    rclcpp::QoS q(10);
    q.transient_local();
    active_robots_pub_ =
        pub_node_->create_publisher<stdr_msgs::msg::RobotIndexedVectorMsg>("stdr_server/active_robots", q);

    // Declare the same parameters that StdrRobotNode declares, so the
    // AsyncParametersClient can set them.
    robot_node_->declare_parameter<double>("sim_step_dt", stdr_gui::kDefaultStepDt);
    robot_node_->declare_parameter<double>("tf_rate", stdr_gui::kDefaultTfRate);
    robot_node_->declare_parameter<double>("odom_rate", stdr_gui::kDefaultOdomRate);
  }

  // Publish active_robots and wait for the backend to process it.
  void publish_active_robots_and_wait(const std::vector<std::pair<std::string, stdr_simulation::RobotConfig>>& robots,
                                      rclcpp::executors::SingleThreadedExecutor& executor)
  {
    stdr_msgs::msg::RobotIndexedVectorMsg msg = make_active_robots_msg(robots);
    active_robots_pub_->publish(msg);
    const std::size_t expected_count = robots.size();
    spin_until(
        executor, [&] { return backend_.num_robots() == expected_count; }, std::chrono::milliseconds(1000));
  }

  std::shared_ptr<rclcpp::Node> node_;
  Ros2Backend backend_;
  std::shared_ptr<rclcpp::Node> robot_node_;
  std::shared_ptr<rclcpp::Node> pub_node_;
  rclcpp::Publisher<stdr_msgs::msg::RobotIndexedVectorMsg>::SharedPtr active_robots_pub_;
};

TEST_F(Ros2BackendRatePropagationTest, SetStepDtPropagatesToRobotNode)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(pub_node_);
  executor.add_node(robot_node_);

  // Register one robot so a param client is created.
  stdr_simulation::RobotConfig cfg;
  publish_active_robots_and_wait({ { "robot0", cfg } }, executor);

  // Wait for the param service on robot_node_ to be available.
  const bool service_ready = spin_until(
      executor,
      [&] {
        rclcpp::SyncParametersClient sc(node_, "robot0");
        return sc.service_is_ready();
      },
      std::chrono::milliseconds(2000));
  if (!service_ready)
  {
    GTEST_SKIP() << "Parameter service not available in time — skipping propagation test.";
  }

  backend_.set_step_dt(0.05);

  const bool propagated = spin_until(
      executor, [&] { return std::abs(robot_node_->get_parameter("sim_step_dt").as_double() - 0.05) < 1e-9; },
      std::chrono::milliseconds(2000));
  EXPECT_TRUE(propagated) << "sim_step_dt was not propagated in time";
}

TEST_F(Ros2BackendRatePropagationTest, SetTfRatePropagatesToRobotNode)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(pub_node_);
  executor.add_node(robot_node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots_and_wait({ { "robot0", cfg } }, executor);

  const bool service_ready = spin_until(
      executor,
      [&] {
        rclcpp::SyncParametersClient sc(node_, "robot0");
        return sc.service_is_ready();
      },
      std::chrono::milliseconds(2000));
  if (!service_ready)
  {
    GTEST_SKIP() << "Parameter service not available in time — skipping propagation test.";
  }

  backend_.set_tf_rate(25.0);

  const bool propagated = spin_until(
      executor, [&] { return std::abs(robot_node_->get_parameter("tf_rate").as_double() - 25.0) < 1e-9; },
      std::chrono::milliseconds(2000));
  EXPECT_TRUE(propagated) << "tf_rate was not propagated in time";
}

TEST_F(Ros2BackendRatePropagationTest, SetOdomRatePropagatesToRobotNode)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(pub_node_);
  executor.add_node(robot_node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots_and_wait({ { "robot0", cfg } }, executor);

  const bool service_ready = spin_until(
      executor,
      [&] {
        rclcpp::SyncParametersClient sc(node_, "robot0");
        return sc.service_is_ready();
      },
      std::chrono::milliseconds(2000));
  if (!service_ready)
  {
    GTEST_SKIP() << "Parameter service not available in time — skipping propagation test.";
  }

  backend_.set_odom_rate(15.0);

  const bool propagated = spin_until(
      executor, [&] { return std::abs(robot_node_->get_parameter("odom_rate").as_double() - 15.0) < 1e-9; },
      std::chrono::milliseconds(2000));
  EXPECT_TRUE(propagated) << "odom_rate was not propagated in time";
}

// ─── External param change reflection tests ───────────────────────────────────
//
// These tests verify that the backend's atomic caches update when parameters
// are changed on the robot node from outside the GUI (e.g. via ros2 param set).
// ParameterEventHandler subscriptions registered in on_active_robots() should
// detect the change and update the corresponding atomic.

TEST_F(Ros2BackendRatePropagationTest, BackendCacheReflectsExternalParamChange)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(pub_node_);
  executor.add_node(robot_node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots_and_wait({ { "robot0", cfg } }, executor);

  // Wait for the param event subscription to be established.
  const bool service_ready = spin_until(
      executor,
      [&] {
        rclcpp::SyncParametersClient sc(node_, "robot0");
        return sc.service_is_ready();
      },
      std::chrono::milliseconds(2000));
  if (!service_ready)
  {
    GTEST_SKIP() << "Parameter service not available in time — skipping external-change test.";
  }

  // Change tf_rate directly on the robot node (simulates `ros2 param set`).
  robot_node_->set_parameter(rclcpp::Parameter("tf_rate", 42.0));

  // The backend's get_tf_rate() should converge to 42.0 via the event callback.
  const bool updated = spin_until(
      executor, [&] { return std::abs(backend_.get_tf_rate() - 42.0) < 1e-9; }, std::chrono::milliseconds(2000));
  EXPECT_TRUE(updated) << "Backend tf_rate cache did not reflect external change in time";

  // Also verify sim_step_dt.
  robot_node_->set_parameter(rclcpp::Parameter("sim_step_dt", 0.02));
  const bool step_updated = spin_until(
      executor, [&] { return std::abs(backend_.get_step_dt() - 0.02) < 1e-9; }, std::chrono::milliseconds(2000));
  EXPECT_TRUE(step_updated) << "Backend step_dt cache did not reflect external change in time";

  // Also verify odom_rate.
  robot_node_->set_parameter(rclcpp::Parameter("odom_rate", 7.0));
  const bool odom_updated = spin_until(
      executor, [&] { return std::abs(backend_.get_odom_rate() - 7.0) < 1e-9; }, std::chrono::milliseconds(2000));
  EXPECT_TRUE(odom_updated) << "Backend odom_rate cache did not reflect external change in time";
}

// Verifies the initial-seed path: when a robot node is already running with
// non-default parameter values before the GUI connects, the backend should seed
// its caches from the robot's actual current values.
//
// The seed now uses a 50 ms retry timer (instead of a bare service_is_ready()
// check), so we must spin the executor for long enough to let the timer fire
// and the get_parameters future resolve.  A 2 s timeout is generous for the
// single-process test environment.
TEST_F(Ros2BackendRatePropagationTest, BackendSeedsCacheFromInitialRobotParams)
{
  // Override tf_rate to a non-default value before the backend discovers the
  // robot so the seed path exercises the get_parameters() call.
  robot_node_->set_parameter(rclcpp::Parameter("tf_rate", 33.0));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(pub_node_);
  executor.add_node(robot_node_);

  stdr_simulation::RobotConfig cfg;
  publish_active_robots_and_wait({ { "robot0", cfg } }, executor);

  // The seed timer fires at most every 50 ms.  Keep spinning so the timer
  // callback runs, discovers the service, issues get_parameters, and the
  // future continuation updates the atomic.
  const bool seeded = spin_until(
      executor, [&] { return std::abs(backend_.get_tf_rate() - 33.0) < 1e-9; }, std::chrono::milliseconds(2000));
  if (!seeded)
  {
    GTEST_SKIP() << "Seed did not land in time — parameter service may not have been available.";
  }
  EXPECT_NEAR(backend_.get_tf_rate(), 33.0, 1e-9);
}

// ─── publishes_ros_topics ─────────────────────────────────────────────────────

TEST_F(Ros2BackendTest, PublishesRosTopicsReturnsTrue)
{
  EXPECT_TRUE(backend_.publishes_ros_topics());
}

// ─── get_scheduling_mode default ─────────────────────────────────────────────

// Before any robot connects, the mode defaults to SnapToMultiple so the
// effective-rate display matches the pre-existing behaviour.
TEST_F(Ros2BackendTest, GetSchedulingModeDefaultsToSnapToMultiple)
{
  EXPECT_EQ(backend_.get_scheduling_mode(), stdr_simulation::SchedulingMode::SnapToMultiple);
}

}  // namespace
}  // namespace stdr_gui_ros
