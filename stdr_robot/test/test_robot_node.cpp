#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_robot/stdr_robot_node.hpp>
#include <stdr_simulation/rate_scheduler.hpp>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace stdr_robot
{
namespace
{

using ::testing::Contains;
using ::testing::Gt;
using ::testing::SizeIs;
using namespace std::chrono_literals;

// Spin the executor until predicate returns true or the timeout elapses.
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

class RobotNodeTest : public ::testing::Test
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

  RobotNodeTest()
  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("robot_name", "robot0");
    node_ = std::make_shared<StdrRobotNode>(options);
  }

  std::shared_ptr<StdrRobotNode> node_;
};

// ─── Helper: create a minimal robot config ──────────────────────────────────

[[nodiscard]] stdr_simulation::RobotConfig make_test_config()
{
  stdr_simulation::RobotConfig config;
  config.initial_pose = { 1.0, 2.0, 0.0 };
  config.footprint.radius = 0.2;
  config.kinematic_model.type = "ideal";

  stdr_simulation::LaserConfig laser;
  laser.max_angle = 1.57;
  laser.min_angle = -1.57;
  laser.max_range = 10.0;
  laser.min_range = 0.1;
  laser.num_rays = 5;
  laser.frequency = 10.0;
  laser.frame_id = "laser0";
  laser.pose = { 0.1, 0.0, 0.0 };
  config.laser_sensors.push_back(laser);

  return config;
}

// ─── Basic lifecycle ────────────────────────────────────────────────────────

TEST_F(RobotNodeTest, CreatesSuccessfully)
{
  EXPECT_NE(node_, nullptr);
}

// ─── Subscriptions ──────────────────────────────────────────────────────────

TEST_F(RobotNodeTest, HasExpectedSubscriptions)
{
  const std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();

  // Map subscription (the node subscribes to "map").
  EXPECT_THAT(topics,
              Contains(std::pair{ std::string("/map"), std::vector<std::string>{ "nav_msgs/msg/OccupancyGrid" } }));

  // cmd_vel subscription.
  EXPECT_THAT(topics, Contains(std::pair{ std::string("/robot0/cmd_vel"),
                                          std::vector<std::string>{ "geometry_msgs/msg/Twist" } }));
}

// ─── Services ───────────────────────────────────────────────────────────────

TEST_F(RobotNodeTest, HasExpectedService)
{
  const std::map<std::string, std::vector<std::string>> services = node_->get_service_names_and_types();
  EXPECT_THAT(services, Contains(std::pair{ std::string("/robot0/replace"),
                                            std::vector<std::string>{ "stdr_msgs/srv/MoveRobot" } }));
}

// ─── Simulation inactive before registration ────────────────────────────────

TEST_F(RobotNodeTest, SimulationInactiveBeforeRegistration)
{
  // The node is not registered (no server), so the timer should early-return.
  // Verify no odom is published.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  bool received_odom = false;
  auto helper_node = rclcpp::Node::make_shared("test_helper");
  executor.add_node(helper_node);
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&received_odom](const nav_msgs::msg::Odometry::SharedPtr) { received_odom = true; });

  // Spin for a bit — should NOT receive odom since robot is not registered.
  executor.spin_some(300ms);
  EXPECT_FALSE(received_odom);
}

// ─── Configure creates publishers ───────────────────────────────────────────

TEST_F(RobotNodeTest, ConfigureCreatesPublishers)
{
  node_->configure(make_test_config());

  const std::map<std::string, std::vector<std::string>> topics = node_->get_topic_names_and_types();

  EXPECT_THAT(topics,
              Contains(std::pair{ std::string("/robot0/odom"), std::vector<std::string>{ "nav_msgs/msg/Odometry" } }));
  EXPECT_THAT(topics, Contains(std::pair{ std::string("/robot0/laser0"),
                                          std::vector<std::string>{ "sensor_msgs/msg/LaserScan" } }));
}

// ─── Motion integration produces odometry ───────────────────────────────────

TEST_F(RobotNodeTest, MotionIntegrationProducesOdometry)
{
  node_->configure(make_test_config());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  nav_msgs::msg::Odometry last_odom;
  bool received_odom = false;
  auto helper_node = rclcpp::Node::make_shared("test_helper_odom");
  executor.add_node(helper_node);
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&last_odom, &received_odom](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom = *msg;
        received_odom = true;
      });

  // Spin until odom is received.
  const bool ok = spin_until(
      executor, [&received_odom]() { return received_odom; }, 2000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for odometry";

  // The robot should be at its initial pose (no cmd_vel sent).
  EXPECT_NEAR(last_odom.pose.pose.position.x, 1.0, 0.5);
  EXPECT_NEAR(last_odom.pose.pose.position.y, 2.0, 0.5);
  EXPECT_EQ(last_odom.header.frame_id, "map_static");
  EXPECT_EQ(last_odom.child_frame_id, "robot0");
}

// ─── Laser publishes data with map ──────────────────────────────────────────

TEST_F(RobotNodeTest, LaserPublishesWithMap)
{
  node_->configure(make_test_config());

  // Provide a small empty map (all free space).
  auto map_pub_node = rclcpp::Node::make_shared("map_publisher");
  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();
  auto map_pub = map_pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", latched_qos);

  nav_msgs::msg::OccupancyGrid map_msg;
  map_msg.info.width = 100;
  map_msg.info.height = 100;
  map_msg.info.resolution = 0.05;
  map_msg.info.origin.position.x = 0.0;
  map_msg.info.origin.position.y = 0.0;
  map_msg.data.assign(100 * 100, 0);  // All free.
  map_pub->publish(map_msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(map_pub_node);

  bool received_laser = false;
  auto helper_node = rclcpp::Node::make_shared("test_helper_laser");
  executor.add_node(helper_node);
  auto laser_sub = helper_node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/robot0/laser0", 10, [&received_laser](const sensor_msgs::msg::LaserScan::SharedPtr) { received_laser = true; });

  const bool ok = spin_until(
      executor, [&received_laser]() { return received_laser; }, 3000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for laser scan";
}

// ─── Rate parameter declarations ─────────────────────────────────────────────

TEST_F(RobotNodeTest, DeclaresRateParametersWithDefaults)
{
  EXPECT_DOUBLE_EQ(node_->get_parameter("sim_step_dt").as_double(), stdr_simulation::kDefaultStepDt);
  EXPECT_DOUBLE_EQ(node_->get_parameter("tf_rate").as_double(), stdr_simulation::kDefaultTfRateHz);
  EXPECT_DOUBLE_EQ(node_->get_parameter("odom_rate").as_double(), stdr_simulation::kDefaultOdomRateHz);
}

// ─── sim_step_dt bounds validation ──────────────────────────────────────────

TEST_F(RobotNodeTest, RejectsSimStepDtBelowMin)
{
  const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("sim_step_dt", 0.0));
  EXPECT_FALSE(result.successful);
}

TEST_F(RobotNodeTest, RejectsSimStepDtAboveMax)
{
  const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("sim_step_dt", 2.0));
  EXPECT_FALSE(result.successful);
}

// ─── tf_rate bounds validation ───────────────────────────────────────────────

TEST_F(RobotNodeTest, RejectsTfRateBelowMin)
{
  // Negative value is below kMinRateHz = 0.0.
  const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("tf_rate", -1.0));
  EXPECT_FALSE(result.successful);
}

TEST_F(RobotNodeTest, RejectsTfRateAboveMax)
{
  const rcl_interfaces::msg::SetParametersResult result =
      node_->set_parameter(rclcpp::Parameter("tf_rate", stdr_simulation::kMaxRateHz + 1.0));
  EXPECT_FALSE(result.successful);
}

// ─── odom_rate bounds validation ────────────────────────────────────────────

TEST_F(RobotNodeTest, RejectsOdomRateBelowMin)
{
  const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("odom_rate", -1.0));
  EXPECT_FALSE(result.successful);
}

TEST_F(RobotNodeTest, RejectsOdomRateAboveMax)
{
  const rcl_interfaces::msg::SetParametersResult result =
      node_->set_parameter(rclcpp::Parameter("odom_rate", stdr_simulation::kMaxRateHz + 1.0));
  EXPECT_FALSE(result.successful);
}

// ─── Valid rate change is accepted ──────────────────────────────────────────

TEST_F(RobotNodeTest, AcceptsValidRateChange)
{
  const rcl_interfaces::msg::SetParametersResult result = node_->set_parameter(rclcpp::Parameter("tf_rate", 25.0));
  ASSERT_TRUE(result.successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("tf_rate").as_double(), 25.0);
}

// ─── Scheduler period math after reconfiguration ────────────────────────────

// With sim_step_dt=0.05, tf_rate=20 Hz: period = round(1/(20*0.05)) = round(1) = 1 tick.
// With odom_rate=10 Hz: period = round(1/(10*0.05)) = round(2) = 2 ticks.
// After configure(), the scheduler streams are registered with the default rates;
// then we reconfigure the rates to verify.
TEST_F(RobotNodeTest, SchedulerPeriodMatchesAfterReconfigure)
{
  // First configure the node so scheduler streams are registered.
  node_->configure(make_test_config());

  // Reconfigure to a new sim_step_dt and rates.
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("sim_step_dt", 0.05)).successful);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("tf_rate", 20.0)).successful);
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("odom_rate", 10.0)).successful);

  // With dt=0.05 and tf_rate=20: period = max(1, round(1/(20*0.05))) = 1.
  EXPECT_EQ(node_->effective_tf_rate(), 1.0 / (1 * 0.05));
  // With dt=0.05 and odom_rate=10: period = max(1, round(1/(10*0.05))) = 2.
  EXPECT_EQ(node_->effective_odom_rate(), 1.0 / (2 * 0.05));
}

// ─── Per-sensor scheduler registration ──────────────────────────────────────

// After configuring with a laser at 5 Hz and a sonar at 10 Hz, the scheduler
// must reflect those rates for the respective sensor indices.
TEST_F(RobotNodeTest, PerSensorRegistrationFromConfig)
{
  stdr_simulation::RobotConfig config;
  config.kinematic_model.type = "ideal";

  stdr_simulation::LaserConfig laser;
  laser.frequency = 5.0;
  laser.frame_id = "laser0";
  config.laser_sensors.push_back(laser);

  stdr_simulation::SonarConfig sonar;
  sonar.frequency = 10.0;
  sonar.frame_id = "sonar0";
  config.sonar_sensors.push_back(sonar);

  node_->configure(config);

  // Effective laser rate with default dt=0.1: period = max(1, round(1/(5*0.1))) = 2 → 5 Hz.
  EXPECT_DOUBLE_EQ(node_->effective_laser_rate(0), 5.0);
  // Effective sonar rate: period = max(1, round(1/(10*0.1))) = 1 → 10 Hz.
  EXPECT_DOUBLE_EQ(node_->effective_sonar_rate(0), 10.0);
}

}  // namespace
}  // namespace stdr_robot
