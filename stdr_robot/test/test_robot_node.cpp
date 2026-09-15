#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <stdr_robot/stdr_robot_node.hpp>
#include <stdr_simulation/rate_scheduler.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <future>
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

// All test suites in this binary share one rclcpp::init / rclcpp::shutdown pair
// to avoid conflicts between fixtures that each declare SetUpTestSuite.
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

// NOLINTNEXTLINE(cert-err58-cpp) — safe: AddGlobalTestEnvironment is called before main test execution
const ::testing::Environment* const kRclcppEnv = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

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

// ─── Node name derivation from robot_name override ──────────────────────────

// When robot_name is passed as a parameter override, the node must adopt it
// as the ROS node name so each robot surfaces as a distinct node rather than
// all competing for the same /stdr_robot name.
TEST(RobotNodeNameTest, NodeNameDerivedFromRobotNameOverride)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_name", "robot7");
  const std::shared_ptr<StdrRobotNode> node = std::make_shared<StdrRobotNode>(options);
  EXPECT_EQ(std::string(node->get_name()), std::string("robot7"));
}

// When no robot_name override is present the node falls back to "stdr_robot".
TEST(RobotNodeNameTest, FallsBackToStdrRobotWhenNoOverride)
{
  const std::shared_ptr<StdrRobotNode> node = std::make_shared<StdrRobotNode>(rclcpp::NodeOptions());
  EXPECT_EQ(std::string(node->get_name()), std::string("stdr_robot"));
}

// The spawn server guarantees valid robot names; a digit-leading name is a
// contract violation that should fail loudly rather than silently producing
// broken topic names (e.g. "1robot/cmd_vel" which ROS2 rejects at runtime).
TEST(RobotNodeNameTest, ThrowsOnDigitLeadingName)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_name", "1robot");
  EXPECT_THROW((stdr_robot::StdrRobotNode(options)), std::invalid_argument);
}

// A slash in robot_name is equally invalid — ROS2 topic names built from it
// would be misrouted as nested namespaces rather than rejected outright.
TEST(RobotNodeNameTest, ThrowsOnInvalidCharactersInName)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_name", "robot/0");
  EXPECT_THROW((stdr_robot::StdrRobotNode(options)), std::invalid_argument);
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
  EXPECT_THAT(topics, Contains(std::pair{ std::string("/robot0/ground_truth"),
                                          std::vector<std::string>{ "nav_msgs/msg/Odometry" } }));
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
  EXPECT_EQ(last_odom.header.frame_id, "robot0/odom");
  EXPECT_EQ(last_odom.child_frame_id, "robot0/base_link");
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

// ─── scheduling_mode parameter ──────────────────────────────────────────────

TEST_F(RobotNodeTest, SchedulingModeDefaultsToSnapToMultiple)
{
  EXPECT_EQ(node_->get_parameter("scheduling_mode").as_string(), std::string("snap_to_multiple"));
}

TEST_F(RobotNodeTest, SchedulingModeRejectsInvalidValue)
{
  const rcl_interfaces::msg::SetParametersResult result =
      node_->set_parameter(rclcpp::Parameter("scheduling_mode", "garbage"));
  EXPECT_FALSE(result.successful);
  // Rejection reason must mention at least one valid option.
  EXPECT_NE(result.reason.find("snap_to_multiple"), std::string::npos);
}

TEST_F(RobotNodeTest, SchedulingModeAcceptsAccumulator)
{
  const rcl_interfaces::msg::SetParametersResult result =
      node_->set_parameter(rclcpp::Parameter("scheduling_mode", "accumulator"));
  ASSERT_TRUE(result.successful);
  EXPECT_EQ(node_->get_parameter("scheduling_mode").as_string(), std::string("accumulator"));
}

// ─── Center-of-rotation integration ─────────────────────────────────────────

// A robot node configured with a non-origin center_of_rotation and a pure
// rotation command (zero linear velocity) must pivot its body origin about the
// configured world-frame pivot point.  The odom topic carries the body position
// so we can verify: the origin moved AND the distance from the body origin to
// the fixed world-frame pivot equals the body-frame pivot magnitude.
TEST_F(RobotNodeTest, OffsetCenterOfRotationPivotsPoseAboutConfiguredPoint)
{
  // Pivot is 1 m ahead of the body origin in the body frame.  The robot starts
  // at (0, 0, 0), so the world-frame pivot starts at (1, 0).
  stdr_simulation::RobotConfig cfg;
  cfg.initial_pose = { 0.0, 0.0, 0.0 };
  cfg.footprint.radius = 0.05;
  cfg.kinematic_model.type = "ideal";
  // Direct RobotConfig construction bypasses load_robot_config footprint
  // validation; the wide pivot is intentional to produce a measurable arc.
  cfg.center_of_rotation = { 1.0, 0.0 };

  node_->configure(cfg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  auto helper_node = rclcpp::Node::make_shared("test_helper_cor");
  executor.add_node(helper_node);

  // Publish a pure rotation command.  The sim timer fires at 0.1 s intervals;
  // the cmd_vel subscription processes the message in between ticks.
  auto cmd_pub = helper_node->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", 10);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 1.0;

  // Collect odom samples.  We wait until we have received enough messages to
  // guarantee that at least several ticks have run with the rotation command
  // applied, making the arc displacement unambiguous.
  constexpr int kMinOdomCount = 5;
  nav_msgs::msg::Odometry last_odom;
  int odom_count = 0;
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&last_odom, &odom_count](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom = *msg;
        ++odom_count;
      });

  // Spin briefly to let subscriptions wire up, then publish the command.
  executor.spin_some(50ms);
  cmd_pub->publish(cmd);

  const bool ok = spin_until(
      executor, [&odom_count]() { return odom_count >= kMinOdomCount; }, 3000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for odometry after offset center-of-rotation command";

  const double body_x = last_odom.pose.pose.position.x;
  const double body_y = last_odom.pose.pose.position.y;

  // The world-frame pivot starts at (initial_x + pivot_x, initial_y + pivot_y)
  // = (1, 0).  For v=0, the ideal model keeps the pivot fixed in world frame
  // throughout, so the distance from body origin to pivot equals the pivot magnitude.
  constexpr double kPivotWorldX = 1.0;
  constexpr double kPivotWorldY = 0.0;
  constexpr double kPivotRadius = 1.0;  // std::hypot(1.0, 0.0)

  // After several ticks with w=1 rad/s the body must have moved from (0, 0).
  const double displacement = std::hypot(body_x, body_y);
  EXPECT_GT(displacement, 1e-6) << "Body origin should have moved with offset pivot during pure rotation";

  // Distance from body origin to the fixed world-frame pivot must equal the
  // pivot radius — the arc radius is preserved regardless of heading.
  const double dist_to_pivot = std::hypot(body_x - kPivotWorldX, body_y - kPivotWorldY);
  EXPECT_NEAR(dist_to_pivot, kPivotRadius, 1e-3);
}

// ─── Ground truth topic ──────────────────────────────────────────────────────

TEST_F(RobotNodeTest, GroundTruthPublishedInMapStaticFrame)
{
  node_->configure(make_test_config());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  nav_msgs::msg::Odometry last_ground_truth;
  bool received = false;
  auto helper_node = rclcpp::Node::make_shared("test_helper_ground_truth");
  executor.add_node(helper_node);
  auto ground_truth_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/ground_truth", 10, [&last_ground_truth, &received](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_ground_truth = *msg;
        received = true;
      });

  const bool ok = spin_until(
      executor, [&received]() { return received; }, 2000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for ground truth";

  EXPECT_EQ(last_ground_truth.header.frame_id, "map_static");
  EXPECT_EQ(last_ground_truth.child_frame_id, "robot0");
  EXPECT_NEAR(last_ground_truth.pose.pose.position.x, 1.0, 0.5);
  EXPECT_NEAR(last_ground_truth.pose.pose.position.y, 2.0, 0.5);
}

// ─── Odometry model semantics: odom vs. ground truth ────────────────────────

// Under the default OdometryModel::Perfect, odom_pose_ integrates the exact
// same command as the true pose with no noise drawn — the two must match
// exactly at every tick (mirrors IdealMotionModelIntegrateTest.PerfectMatchesUpdateExactly).
TEST_F(RobotNodeTest, PerfectOdometryMatchesGroundTruth)
{
  node_->configure(make_test_config());  // odometry_model defaults to Perfect.

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  auto helper_node = rclcpp::Node::make_shared("test_helper_perfect_odom");
  executor.add_node(helper_node);

  auto cmd_pub = helper_node->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", 10);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  cmd.angular.z = 0.3;

  nav_msgs::msg::Odometry last_odom;
  nav_msgs::msg::Odometry last_ground_truth;
  int odom_count = 0;
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&last_odom, &odom_count](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom = *msg;
        ++odom_count;
      });
  auto ground_truth_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/ground_truth", 10,
      [&last_ground_truth](const nav_msgs::msg::Odometry::SharedPtr msg) { last_ground_truth = *msg; });

  executor.spin_some(50ms);
  cmd_pub->publish(cmd);

  constexpr int kMinOdomCount = 10;
  const bool ok = spin_until(
      executor, [&odom_count]() { return odom_count >= kMinOdomCount; }, 3000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for odometry";

  EXPECT_NEAR(last_odom.pose.pose.position.x, last_ground_truth.pose.pose.position.x, 1e-9);
  EXPECT_NEAR(last_odom.pose.pose.position.y, last_ground_truth.pose.pose.position.y, 1e-9);
  EXPECT_NEAR(tf2::getYaw(last_odom.pose.pose.orientation), tf2::getYaw(last_ground_truth.pose.pose.orientation), 1e-9);
}

// Under OdometryModel::Velocity with nonzero alphas, odom_pose_ (clean) must
// separate from the noisy true pose over many ticks, and the odom covariance
// diagonal must reflect a nonzero accumulated variance.
TEST_F(RobotNodeTest, VelocityOdometryDivergesFromGroundTruth)
{
  stdr_simulation::RobotConfig config = make_test_config();
  config.kinematic_model.odometry_model = stdr_simulation::OdometryModel::Velocity;
  config.kinematic_model.a_ux_ux = 0.5;
  config.kinematic_model.a_w_w = 0.5;
  node_->configure(config);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  auto helper_node = rclcpp::Node::make_shared("test_helper_velocity_odom");
  executor.add_node(helper_node);

  auto cmd_pub = helper_node->create_publisher<geometry_msgs::msg::Twist>("/robot0/cmd_vel", 10);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 1.0;
  cmd.angular.z = 0.5;

  nav_msgs::msg::Odometry last_odom;
  nav_msgs::msg::Odometry last_ground_truth;
  int ground_truth_count = 0;
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&last_odom](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom = *msg; });
  auto ground_truth_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/ground_truth", 10,
      [&last_ground_truth, &ground_truth_count](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_ground_truth = *msg;
        ++ground_truth_count;
      });

  executor.spin_some(50ms);
  cmd_pub->publish(cmd);

  constexpr int kMinCount = 50;
  const bool ok = spin_until(
      executor, [&ground_truth_count]() { return ground_truth_count >= kMinCount; }, 5000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for odometry";

  const double dx = last_odom.pose.pose.position.x - last_ground_truth.pose.pose.position.x;
  const double dy = last_odom.pose.pose.position.y - last_ground_truth.pose.pose.position.y;
  EXPECT_GT(std::hypot(dx, dy), 1e-3) << "Velocity-mode noise should have separated odom from ground truth";

  EXPECT_GT(last_odom.pose.covariance[0], 0.0);
  EXPECT_GT(last_odom.pose.covariance[35], 0.0);
}

// ─── MoveRobot resets odometry ───────────────────────────────────────────────

// Teleporting the robot must collapse the belief pose onto the requested pose,
// same as SimulationEngine's teleport handling.
TEST_F(RobotNodeTest, MoveRobotResetsOdometry)
{
  node_->configure(make_test_config());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  auto helper_node = rclcpp::Node::make_shared("test_helper_move_robot");
  executor.add_node(helper_node);

  auto client = helper_node->create_client<stdr_msgs::srv::MoveRobot>("/robot0/replace");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<stdr_msgs::srv::MoveRobot::Request>();
  request->new_pose.x = 5.0;
  request->new_pose.y = 6.0;
  request->new_pose.theta = 1.0;

  auto future = client->async_send_request(request);
  const bool service_ok = spin_until(
      executor, [&future]() { return future.wait_for(0s) == std::future_status::ready; }, 2000ms);
  ASSERT_TRUE(service_ok) << "MoveRobot service call did not complete in time";

  nav_msgs::msg::Odometry last_odom;
  bool received = false;
  auto odom_sub = helper_node->create_subscription<nav_msgs::msg::Odometry>(
      "/robot0/odom", 10, [&last_odom, &received](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom = *msg;
        received = true;
      });

  const bool odom_ok = spin_until(
      executor, [&received]() { return received; }, 2000ms);
  ASSERT_TRUE(odom_ok) << "Timed out waiting for odometry after MoveRobot";

  EXPECT_NEAR(last_odom.pose.pose.position.x, 5.0, 1e-3);
  EXPECT_NEAR(last_odom.pose.pose.position.y, 6.0, 1e-3);
}

// ─── Belief and correction TF ────────────────────────────────────────────────

TEST_F(RobotNodeTest, PublishesBeliefAndCorrectionTf)
{
  stdr_simulation::RobotConfig config = make_test_config();
  config.initial_pose = { 1.0, 2.0, 0.3 };
  node_->configure(config);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  auto helper_node = rclcpp::Node::make_shared("test_helper_tf");
  executor.add_node(helper_node);

  tf2_ros::Buffer buffer(helper_node->get_clock());
  tf2_ros::TransformListener listener(buffer, helper_node, /*spin_thread=*/false);

  const bool ok = spin_until(
      executor,
      [&buffer]() {
        return buffer.canTransform("robot0/odom", "robot0/base_link", tf2::TimePointZero) &&
               buffer.canTransform("map_static", "robot0/odom", tf2::TimePointZero) &&
               buffer.canTransform("map_static", "robot0", tf2::TimePointZero);
      },
      3000ms);
  ASSERT_TRUE(ok) << "Timed out waiting for belief/correction/truth TF";

  // tf2 composes the map_static -> robot0/odom -> robot0/base_link chain
  // automatically; the composed result must recover the truth transform.
  const geometry_msgs::msg::TransformStamped composed =
      buffer.lookupTransform("map_static", "robot0/base_link", tf2::TimePointZero);
  const geometry_msgs::msg::TransformStamped truth = buffer.lookupTransform("map_static", "robot0", tf2::TimePointZero);

  EXPECT_NEAR(composed.transform.translation.x, truth.transform.translation.x, 1e-3);
  EXPECT_NEAR(composed.transform.translation.y, truth.transform.translation.y, 1e-3);
  EXPECT_NEAR(tf2::getYaw(composed.transform.rotation), tf2::getYaw(truth.transform.rotation), 1e-3);
}

// ─── publish_map_to_odom_tf parameter ────────────────────────────────────────

TEST(RobotNodeMapToOdomTfTest, MapToOdomTfDisabledByParameter)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("robot_name", "robot0");
  options.append_parameter_override("publish_map_to_odom_tf", false);
  const std::shared_ptr<StdrRobotNode> node = std::make_shared<StdrRobotNode>(options);
  node->configure(make_test_config());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto helper_node = rclcpp::Node::make_shared("test_helper_no_map_to_odom");
  executor.add_node(helper_node);

  tf2_ros::Buffer buffer(helper_node->get_clock());
  tf2_ros::TransformListener listener(buffer, helper_node, /*spin_thread=*/false);

  // The belief TF must still appear even with the correction TF disabled.
  const bool belief_ok = spin_until(
      executor, [&buffer]() { return buffer.canTransform("robot0/odom", "robot0/base_link", tf2::TimePointZero); },
      3000ms);
  ASSERT_TRUE(belief_ok) << "Timed out waiting for belief TF";

  // Give the correction TF ample opportunity to appear before asserting its absence.
  executor.spin_some(500ms);
  EXPECT_FALSE(buffer.canTransform("map_static", "robot0/odom", tf2::TimePointZero))
      << "map_static -> robot0/odom must not be published when publish_map_to_odom_tf is false";
}

// publish_map_to_odom_tf is read once at construction with no reconfiguration
// callback path — it must be declared read-only so a runtime set_parameter()
// is rejected instead of silently succeeding with no effect.
TEST_F(RobotNodeTest, PublishMapToOdomTfParameterIsReadOnly)
{
  const rcl_interfaces::msg::SetParametersResult result =
      node_->set_parameter(rclcpp::Parameter("publish_map_to_odom_tf", false));
  EXPECT_FALSE(result.successful);
}

}  // namespace
}  // namespace stdr_robot
