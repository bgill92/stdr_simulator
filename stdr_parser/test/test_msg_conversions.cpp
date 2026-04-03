#include <stdr_parser/msg_conversions.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace stdr_parser {
namespace {

using ::testing::SizeIs;

// --- Pose2D ---

TEST(MsgConversions, Pose2DRoundTrip)
{
  const stdr_simulation::Pose2D original{1.5, 2.5, 0.75};
  const geometry_msgs::msg::Pose2D msg = to_ros_msg(original);

  EXPECT_DOUBLE_EQ(msg.x, 1.5);
  EXPECT_DOUBLE_EQ(msg.y, 2.5);
  EXPECT_DOUBLE_EQ(msg.theta, 0.75);

  const stdr_simulation::Pose2D roundtripped = from_ros_msg(msg);

  EXPECT_DOUBLE_EQ(roundtripped.x, 1.5);
  EXPECT_DOUBLE_EQ(roundtripped.y, 2.5);
  EXPECT_DOUBLE_EQ(roundtripped.theta, 0.75);
}

// --- Point2D ---

TEST(MsgConversions, PointRoundTrip)
{
  const stdr_simulation::Point2D original{3.0, 4.0};
  const geometry_msgs::msg::Point msg = to_ros_point(original);

  EXPECT_DOUBLE_EQ(msg.x, 3.0);
  EXPECT_DOUBLE_EQ(msg.y, 4.0);
  EXPECT_DOUBLE_EQ(msg.z, 0.0);

  // z is dropped on the way back.
  const stdr_simulation::Point2D roundtripped = from_ros_point(msg);

  EXPECT_DOUBLE_EQ(roundtripped.x, 3.0);
  EXPECT_DOUBLE_EQ(roundtripped.y, 4.0);
}

// --- NoiseConfig ---

TEST(MsgConversions, NoiseConfigFieldMapping)
{
  stdr_simulation::NoiseConfig cfg;
  cfg.enabled = true;
  cfg.mean = 0.1;
  cfg.std_dev = 0.05;

  const stdr_msgs::msg::Noise msg = to_ros_msg(cfg);

  EXPECT_EQ(msg.noise, true);
  EXPECT_NEAR(msg.noise_mean, 0.1f, 1e-5f);
  EXPECT_NEAR(msg.noise_std, 0.05f, 1e-5f);

  const stdr_simulation::NoiseConfig roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.enabled, true);
  EXPECT_NEAR(roundtripped.mean, 0.1, 1e-5);
  EXPECT_NEAR(roundtripped.std_dev, 0.05, 1e-5);
}

// --- Footprint ---

TEST(MsgConversions, FootprintWithPoints)
{
  stdr_simulation::Footprint fp;
  fp.radius = 0.35;
  fp.points = {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}};

  const stdr_msgs::msg::FootprintMsg msg = to_ros_msg(fp);

  EXPECT_NEAR(msg.radius, 0.35f, 1e-5f);
  ASSERT_THAT(msg.points, SizeIs(3));
  EXPECT_DOUBLE_EQ(msg.points[0].z, 0.0);
  EXPECT_DOUBLE_EQ(msg.points[1].z, 0.0);
  EXPECT_DOUBLE_EQ(msg.points[2].z, 0.0);
  EXPECT_DOUBLE_EQ(msg.points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(msg.points[0].y, 0.0);
  EXPECT_DOUBLE_EQ(msg.points[1].x, 0.0);
  EXPECT_DOUBLE_EQ(msg.points[1].y, 1.0);
  EXPECT_DOUBLE_EQ(msg.points[2].x, -1.0);
  EXPECT_DOUBLE_EQ(msg.points[2].y, 0.0);

  const stdr_simulation::Footprint roundtripped = from_ros_msg(msg);

  ASSERT_THAT(roundtripped.points, SizeIs(3));
  EXPECT_NEAR(roundtripped.radius, 0.35, 1e-5);
  EXPECT_NEAR(roundtripped.points[0].x, 1.0, 1e-5);
  EXPECT_NEAR(roundtripped.points[1].y, 1.0, 1e-5);
  EXPECT_NEAR(roundtripped.points[2].x, -1.0, 1e-5);
}

// --- KinematicConfig ---

TEST(MsgConversions, KinematicConfigRoundTrip)
{
  stdr_simulation::KinematicConfig kin;
  kin.type = "omni";
  kin.a_ux_ux = 1.0;
  kin.a_ux_uy = 2.0;
  kin.a_ux_w = 3.0;
  kin.a_uy_ux = 4.0;
  kin.a_uy_uy = 5.0;
  kin.a_uy_w = 6.0;
  kin.a_w_ux = 7.0;
  kin.a_w_uy = 8.0;
  kin.a_w_w = 9.0;
  kin.a_g_ux = 10.0;
  kin.a_g_uy = 11.0;
  kin.a_g_w = 12.0;

  const stdr_msgs::msg::KinematicMsg msg = to_ros_msg(kin);

  EXPECT_EQ(msg.type, "omni");
  EXPECT_NEAR(msg.a_ux_ux, 1.0f, 1e-5f);
  EXPECT_NEAR(msg.a_g_w, 12.0f, 1e-5f);

  const stdr_simulation::KinematicConfig roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.type, "omni");
  EXPECT_NEAR(roundtripped.a_ux_ux, 1.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_ux_uy, 2.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_ux_w, 3.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_uy_ux, 4.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_uy_uy, 5.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_uy_w, 6.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_w_ux, 7.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_w_uy, 8.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_w_w, 9.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_g_ux, 10.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_g_uy, 11.0, 1e-5);
  EXPECT_NEAR(roundtripped.a_g_w, 12.0, 1e-5);
}

// --- LaserConfig ---

TEST(MsgConversions, LaserConfigRoundTrip)
{
  stdr_simulation::LaserConfig cfg;
  cfg.max_angle = 1.57;
  cfg.min_angle = -1.57;
  cfg.max_range = 10.0;
  cfg.min_range = 0.1;
  cfg.num_rays = 180;
  cfg.noise.enabled = true;
  cfg.noise.mean = 0.0;
  cfg.noise.std_dev = 0.01;
  cfg.frequency = 10.0;
  cfg.frame_id = "laser_frame";
  cfg.pose = {0.1, 0.0, 0.0};

  const stdr_msgs::msg::LaserSensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.max_angle, 1.57f, 1e-5f);
  EXPECT_EQ(msg.num_rays, 180);
  EXPECT_EQ(msg.frame_id, "laser_frame");
  EXPECT_EQ(msg.noise.noise, true);

  const stdr_simulation::LaserConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_angle, 1.57, 1e-5);
  EXPECT_NEAR(roundtripped.min_angle, -1.57, 1e-5);
  EXPECT_NEAR(roundtripped.max_range, 10.0, 1e-5);
  EXPECT_NEAR(roundtripped.min_range, 0.1, 1e-5);
  EXPECT_EQ(roundtripped.num_rays, 180);
  EXPECT_EQ(roundtripped.noise.enabled, true);
  EXPECT_NEAR(roundtripped.noise.std_dev, 0.01, 1e-5);
  EXPECT_NEAR(roundtripped.frequency, 10.0, 1e-5);
  EXPECT_EQ(roundtripped.frame_id, "laser_frame");
  EXPECT_NEAR(roundtripped.pose.x, 0.1, 1e-5);
}

// --- SonarConfig ---

TEST(MsgConversions, SonarConfigRoundTrip)
{
  stdr_simulation::SonarConfig cfg;
  cfg.max_range = 5.0;
  cfg.min_range = 0.2;
  cfg.cone_angle = 0.5;
  cfg.frequency = 5.0;
  cfg.noise.enabled = false;
  cfg.noise.mean = 0.0;
  cfg.noise.std_dev = 0.0;
  cfg.frame_id = "sonar_frame";
  cfg.pose = {0.0, 0.15, 1.57};

  const stdr_msgs::msg::SonarSensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.cone_angle, 0.5f, 1e-5f);
  EXPECT_EQ(msg.frame_id, "sonar_frame");

  const stdr_simulation::SonarConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_range, 5.0, 1e-5);
  EXPECT_NEAR(roundtripped.min_range, 0.2, 1e-5);
  EXPECT_NEAR(roundtripped.cone_angle, 0.5, 1e-5);
  EXPECT_NEAR(roundtripped.frequency, 5.0, 1e-5);
  EXPECT_EQ(roundtripped.noise.enabled, false);
  EXPECT_EQ(roundtripped.frame_id, "sonar_frame");
  EXPECT_NEAR(roundtripped.pose.y, 0.15, 1e-5);
  EXPECT_NEAR(roundtripped.pose.theta, 1.57, 1e-5);
}

// --- RfidSensorConfig ---

TEST(MsgConversions, RfidSensorConfigRoundTrip)
{
  stdr_simulation::RfidSensorConfig cfg;
  cfg.max_range = 3.0;
  cfg.angle_span = 1.2;
  cfg.signal_cutoff = -70.0;
  cfg.frequency = 1.0;
  cfg.frame_id = "rfid_frame";
  cfg.pose = {0.05, 0.0, 0.0};

  const stdr_msgs::msg::RfidSensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.signal_cutoff, -70.0f, 1e-3f);
  EXPECT_EQ(msg.frame_id, "rfid_frame");

  const stdr_simulation::RfidSensorConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_range, 3.0, 1e-5);
  EXPECT_NEAR(roundtripped.angle_span, 1.2, 1e-5);
  EXPECT_NEAR(roundtripped.signal_cutoff, -70.0, 1e-3);
  EXPECT_NEAR(roundtripped.frequency, 1.0, 1e-5);
  EXPECT_EQ(roundtripped.frame_id, "rfid_frame");
  EXPECT_NEAR(roundtripped.pose.x, 0.05, 1e-5);
}

// --- CO2SensorConfig ---

TEST(MsgConversions, CO2SensorConfigRoundTrip)
{
  stdr_simulation::CO2SensorConfig cfg;
  cfg.max_range = 8.0;
  cfg.frequency = 2.0;
  cfg.frame_id = "co2_frame";
  cfg.pose = {0.0, 0.05, 0.0};

  const stdr_msgs::msg::CO2SensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.max_range, 8.0f, 1e-5f);
  EXPECT_EQ(msg.frame_id, "co2_frame");

  const stdr_simulation::CO2SensorConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_range, 8.0, 1e-5);
  EXPECT_NEAR(roundtripped.frequency, 2.0, 1e-5);
  EXPECT_EQ(roundtripped.frame_id, "co2_frame");
  EXPECT_NEAR(roundtripped.pose.y, 0.05, 1e-5);
}

// --- SoundSensorConfig ---

TEST(MsgConversions, SoundSensorConfigRoundTrip)
{
  stdr_simulation::SoundSensorConfig cfg;
  cfg.max_range = 12.0;
  cfg.frequency = 4.0;
  cfg.angle_span = 2.0;
  cfg.frame_id = "sound_frame";
  cfg.pose = {0.1, 0.1, 0.0};

  const stdr_msgs::msg::SoundSensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.angle_span, 2.0f, 1e-5f);
  EXPECT_EQ(msg.frame_id, "sound_frame");

  const stdr_simulation::SoundSensorConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_range, 12.0, 1e-5);
  EXPECT_NEAR(roundtripped.frequency, 4.0, 1e-5);
  EXPECT_NEAR(roundtripped.angle_span, 2.0, 1e-5);
  EXPECT_EQ(roundtripped.frame_id, "sound_frame");
  EXPECT_NEAR(roundtripped.pose.x, 0.1, 1e-5);
  EXPECT_NEAR(roundtripped.pose.y, 0.1, 1e-5);
}

// --- ThermalSensorConfig ---

TEST(MsgConversions, ThermalSensorConfigRoundTrip)
{
  stdr_simulation::ThermalSensorConfig cfg;
  cfg.max_range = 6.0;
  cfg.frequency = 3.0;
  cfg.angle_span = 1.5;
  cfg.frame_id = "thermal_frame";
  cfg.pose = {-0.1, 0.0, 3.14};

  const stdr_msgs::msg::ThermalSensorMsg msg = to_ros_msg(cfg);

  EXPECT_NEAR(msg.angle_span, 1.5f, 1e-5f);
  EXPECT_EQ(msg.frame_id, "thermal_frame");

  const stdr_simulation::ThermalSensorConfig roundtripped = from_ros_msg(msg);

  EXPECT_NEAR(roundtripped.max_range, 6.0, 1e-5);
  EXPECT_NEAR(roundtripped.frequency, 3.0, 1e-5);
  EXPECT_NEAR(roundtripped.angle_span, 1.5, 1e-5);
  EXPECT_EQ(roundtripped.frame_id, "thermal_frame");
  EXPECT_NEAR(roundtripped.pose.x, -0.1, 1e-5);
  EXPECT_NEAR(roundtripped.pose.theta, 3.14, 1e-5);
}

// --- RobotConfig ---

TEST(MsgConversions, RobotConfigRoundTrip)
{
  stdr_simulation::RobotConfig cfg;
  cfg.initial_pose = {1.0, 2.0, 0.5};
  cfg.footprint.radius = 0.2;
  cfg.footprint.points = {{0.2, 0.0}, {-0.2, 0.0}};
  cfg.kinematic_model.type = "unicycle";
  cfg.kinematic_model.a_ux_ux = 1.0;

  stdr_simulation::LaserConfig laser;
  laser.max_range = 10.0;
  laser.num_rays = 360;
  laser.frame_id = "laser0";
  laser.pose = {0.1, 0.0, 0.0};
  cfg.laser_sensors.push_back(laser);

  stdr_simulation::SonarConfig sonar;
  sonar.max_range = 3.0;
  sonar.frame_id = "sonar0";
  sonar.pose = {-0.1, 0.0, 3.14};
  cfg.sonar_sensors.push_back(sonar);

  const stdr_msgs::msg::RobotMsg msg = to_ros_msg(cfg);

  EXPECT_DOUBLE_EQ(msg.initial_pose.x, 1.0);
  ASSERT_THAT(msg.laser_sensors, SizeIs(1));
  ASSERT_THAT(msg.sonar_sensors, SizeIs(1));
  EXPECT_EQ(msg.laser_sensors[0].frame_id, "laser0");
  EXPECT_EQ(msg.kinematic_model.type, "unicycle");

  const stdr_simulation::RobotConfig roundtripped = from_ros_msg(msg);

  EXPECT_DOUBLE_EQ(roundtripped.initial_pose.x, 1.0);
  EXPECT_DOUBLE_EQ(roundtripped.initial_pose.y, 2.0);
  EXPECT_DOUBLE_EQ(roundtripped.initial_pose.theta, 0.5);
  EXPECT_NEAR(roundtripped.footprint.radius, 0.2, 1e-5);
  ASSERT_THAT(roundtripped.footprint.points, SizeIs(2));
  ASSERT_THAT(roundtripped.laser_sensors, SizeIs(1));
  EXPECT_NEAR(roundtripped.laser_sensors[0].max_range, 10.0, 1e-5);
  EXPECT_EQ(roundtripped.laser_sensors[0].num_rays, 360);
  ASSERT_THAT(roundtripped.sonar_sensors, SizeIs(1));
  EXPECT_NEAR(roundtripped.sonar_sensors[0].max_range, 3.0, 1e-5);
  EXPECT_EQ(roundtripped.kinematic_model.type, "unicycle");
  EXPECT_NEAR(roundtripped.kinematic_model.a_ux_ux, 1.0, 1e-5);
}

// --- RfidTag ---

TEST(MsgConversions, RfidTagRoundTrip)
{
  stdr_simulation::RfidTag tag;
  tag.tag_id = "tag42";
  tag.message = "hello";
  tag.pose = {3.0, 4.0, 0.0};

  const stdr_msgs::msg::RfidTag msg = to_ros_msg(tag);

  EXPECT_EQ(msg.tag_id, "tag42");
  EXPECT_EQ(msg.message, "hello");
  EXPECT_DOUBLE_EQ(msg.pose.x, 3.0);

  const stdr_simulation::RfidTag roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.tag_id, "tag42");
  EXPECT_EQ(roundtripped.message, "hello");
  EXPECT_DOUBLE_EQ(roundtripped.pose.x, 3.0);
  EXPECT_DOUBLE_EQ(roundtripped.pose.y, 4.0);
}

// --- CO2Source ---

TEST(MsgConversions, CO2SourceRoundTrip)
{
  stdr_simulation::CO2Source src;
  src.id = "co2_src1";
  src.ppm = 400.0;
  src.pose = {5.0, 6.0, 0.0};

  const stdr_msgs::msg::CO2Source msg = to_ros_msg(src);

  EXPECT_EQ(msg.id, "co2_src1");
  EXPECT_NEAR(msg.ppm, 400.0f, 0.1f);

  const stdr_simulation::CO2Source roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.id, "co2_src1");
  EXPECT_NEAR(roundtripped.ppm, 400.0, 0.1);
  EXPECT_NEAR(roundtripped.pose.x, 5.0, 1e-5);
  EXPECT_NEAR(roundtripped.pose.y, 6.0, 1e-5);
}

// --- SoundSource ---

TEST(MsgConversions, SoundSourceRoundTrip)
{
  stdr_simulation::SoundSource src;
  src.id = "sound_src1";
  src.dbs = 65.0;
  src.pose = {1.5, 2.5, 0.0};

  const stdr_msgs::msg::SoundSource msg = to_ros_msg(src);

  EXPECT_EQ(msg.id, "sound_src1");
  EXPECT_NEAR(msg.dbs, 65.0f, 1e-3f);

  const stdr_simulation::SoundSource roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.id, "sound_src1");
  EXPECT_NEAR(roundtripped.dbs, 65.0, 1e-3);
  EXPECT_NEAR(roundtripped.pose.x, 1.5, 1e-5);
}

// --- ThermalSource ---

TEST(MsgConversions, ThermalSourceRoundTrip)
{
  stdr_simulation::ThermalSource src;
  src.id = "thermal_src1";
  src.degrees = 37.5;
  src.pose = {0.5, 1.5, 0.0};

  const stdr_msgs::msg::ThermalSource msg = to_ros_msg(src);

  EXPECT_EQ(msg.id, "thermal_src1");
  EXPECT_NEAR(msg.degrees, 37.5f, 1e-3f);

  const stdr_simulation::ThermalSource roundtripped = from_ros_msg(msg);

  EXPECT_EQ(roundtripped.id, "thermal_src1");
  EXPECT_NEAR(roundtripped.degrees, 37.5, 1e-3);
  EXPECT_NEAR(roundtripped.pose.x, 0.5, 1e-5);
}

// --- FloatPrecisionRoundTrip ---

TEST(MsgConversions, FloatPrecisionRoundTrip)
{
  // Verify that double→float32→double round-trips use EXPECT_NEAR (not exact),
  // because float32 has only ~7 significant decimal digits.
  stdr_simulation::LaserConfig cfg;
  cfg.max_range = 10.123456789;  // More precision than float32 can represent.
  cfg.min_range = 0.0;
  cfg.num_rays = 1;
  cfg.frequency = 10.0;

  const stdr_msgs::msg::LaserSensorMsg msg = to_ros_msg(cfg);
  const stdr_simulation::LaserConfig roundtripped = from_ros_msg(msg);

  // Float32 precision is roughly 1e-6 relative, so 1e-5 absolute is safe here.
  EXPECT_NEAR(roundtripped.max_range, 10.123456789, 1e-4);
  // Pose2D uses double throughout, so exact equality holds.
  EXPECT_DOUBLE_EQ(roundtripped.pose.x, 0.0);
}

}  // namespace
}  // namespace stdr_parser
