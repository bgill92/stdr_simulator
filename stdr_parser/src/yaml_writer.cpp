#include <stdr_parser/yaml_writer.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <stdr_msgs/msg/co2_sensor_msg.hpp>
#include <stdr_msgs/msg/footprint_msg.hpp>
#include <stdr_msgs/msg/kinematic_msg.hpp>
#include <stdr_msgs/msg/laser_sensor_msg.hpp>
#include <stdr_msgs/msg/noise.hpp>
#include <stdr_msgs/msg/rfid_sensor_msg.hpp>
#include <stdr_msgs/msg/robot_msg.hpp>
#include <stdr_msgs/msg/sonar_sensor_msg.hpp>
#include <stdr_msgs/msg/sound_sensor_msg.hpp>
#include <stdr_msgs/msg/thermal_sensor_msg.hpp>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>

namespace stdr_parser {
namespace {

void emit_pose(YAML::Emitter& out, const geometry_msgs::msg::Pose2D& pose)
{
  out << YAML::Key << "pose" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << pose.x;
  out << YAML::Key << "y" << YAML::Value << pose.y;
  out << YAML::Key << "theta" << YAML::Value << pose.theta;
  out << YAML::EndMap;
}

void emit_noise(YAML::Emitter& out, const stdr_msgs::msg::Noise& noise)
{
  out << YAML::Key << "noise" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "noise_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "noise_mean" << YAML::Value << noise.noise_mean;
  out << YAML::Key << "noise_std" << YAML::Value << noise.noise_std;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_footprint(YAML::Emitter& out, const stdr_msgs::msg::FootprintMsg& fp)
{
  out << YAML::BeginMap;
  out << YAML::Key << "footprint" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "footprint_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "radius" << YAML::Value << fp.radius;
  if (!fp.points.empty()) {
    out << YAML::Key << "points" << YAML::Value;
    out << YAML::BeginSeq;
    for (const geometry_msgs::msg::Point& pt : fp.points) {
      out << YAML::BeginMap;
      out << YAML::Key << "point" << YAML::Value;
      out << YAML::BeginMap;
      out << YAML::Key << "x" << YAML::Value << pt.x;
      out << YAML::Key << "y" << YAML::Value << pt.y;
      out << YAML::EndMap;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_initial_pose(YAML::Emitter& out, const geometry_msgs::msg::Pose2D& pose)
{
  out << YAML::BeginMap;
  out << YAML::Key << "initial_pose" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << pose.x;
  out << YAML::Key << "y" << YAML::Value << pose.y;
  out << YAML::Key << "theta" << YAML::Value << pose.theta;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_laser(YAML::Emitter& out, const stdr_msgs::msg::LaserSensorMsg& laser)
{
  out << YAML::BeginMap;
  out << YAML::Key << "laser" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "laser_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "max_angle" << YAML::Value << laser.max_angle;
  out << YAML::Key << "min_angle" << YAML::Value << laser.min_angle;
  out << YAML::Key << "max_range" << YAML::Value << laser.max_range;
  out << YAML::Key << "min_range" << YAML::Value << laser.min_range;
  out << YAML::Key << "num_rays" << YAML::Value << laser.num_rays;
  emit_noise(out, laser.noise);
  out << YAML::Key << "frequency" << YAML::Value << laser.frequency;
  out << YAML::Key << "frame_id" << YAML::Value << laser.frame_id;
  emit_pose(out, laser.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_sonar(YAML::Emitter& out, const stdr_msgs::msg::SonarSensorMsg& sonar)
{
  out << YAML::BeginMap;
  out << YAML::Key << "sonar" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "sonar_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "max_range" << YAML::Value << sonar.max_range;
  out << YAML::Key << "min_range" << YAML::Value << sonar.min_range;
  out << YAML::Key << "cone_angle" << YAML::Value << sonar.cone_angle;
  out << YAML::Key << "frequency" << YAML::Value << sonar.frequency;
  emit_noise(out, sonar.noise);
  out << YAML::Key << "frame_id" << YAML::Value << sonar.frame_id;
  emit_pose(out, sonar.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_rfid_reader(YAML::Emitter& out, const stdr_msgs::msg::RfidSensorMsg& rfid)
{
  out << YAML::BeginMap;
  out << YAML::Key << "rfid_reader" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "rfid_reader_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "angle_span" << YAML::Value << rfid.angle_span;
  out << YAML::Key << "max_range" << YAML::Value << rfid.max_range;
  out << YAML::Key << "signal_cutoff" << YAML::Value << rfid.signal_cutoff;
  out << YAML::Key << "frequency" << YAML::Value << rfid.frequency;
  out << YAML::Key << "frame_id" << YAML::Value << rfid.frame_id;
  emit_pose(out, rfid.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_co2_sensor(YAML::Emitter& out, const stdr_msgs::msg::CO2SensorMsg& msg)
{
  out << YAML::BeginMap;
  out << YAML::Key << "co2_sensor" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "co2_sensor_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "max_range" << YAML::Value << msg.max_range;
  out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
  out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
  emit_pose(out, msg.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_thermal_sensor(YAML::Emitter& out, const stdr_msgs::msg::ThermalSensorMsg& msg)
{
  out << YAML::BeginMap;
  out << YAML::Key << "thermal_sensor" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "thermal_sensor_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "max_range" << YAML::Value << msg.max_range;
  out << YAML::Key << "angle_span" << YAML::Value << msg.angle_span;
  out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
  out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
  emit_pose(out, msg.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_sound_sensor(YAML::Emitter& out, const stdr_msgs::msg::SoundSensorMsg& msg)
{
  out << YAML::BeginMap;
  out << YAML::Key << "sound_sensor" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "sound_sensor_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "max_range" << YAML::Value << msg.max_range;
  out << YAML::Key << "angle_span" << YAML::Value << msg.angle_span;
  out << YAML::Key << "frequency" << YAML::Value << msg.frequency;
  out << YAML::Key << "frame_id" << YAML::Value << msg.frame_id;
  emit_pose(out, msg.pose);
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

void emit_kinematic(YAML::Emitter& out, const stdr_msgs::msg::KinematicMsg& kin)
{
  out << YAML::BeginMap;
  out << YAML::Key << "kinematic" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "kinematic_specifications" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "kinematic_model" << YAML::Value << kin.type;
  out << YAML::Key << "kinematic_parameters" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "a_ux_ux" << YAML::Value << kin.a_ux_ux;
  out << YAML::Key << "a_ux_uy" << YAML::Value << kin.a_ux_uy;
  out << YAML::Key << "a_ux_w" << YAML::Value << kin.a_ux_w;
  out << YAML::Key << "a_uy_ux" << YAML::Value << kin.a_uy_ux;
  out << YAML::Key << "a_uy_uy" << YAML::Value << kin.a_uy_uy;
  out << YAML::Key << "a_uy_w" << YAML::Value << kin.a_uy_w;
  out << YAML::Key << "a_w_ux" << YAML::Value << kin.a_w_ux;
  out << YAML::Key << "a_w_uy" << YAML::Value << kin.a_w_uy;
  out << YAML::Key << "a_w_w" << YAML::Value << kin.a_w_w;
  out << YAML::Key << "a_g_ux" << YAML::Value << kin.a_g_ux;
  out << YAML::Key << "a_g_uy" << YAML::Value << kin.a_g_uy;
  out << YAML::Key << "a_g_w" << YAML::Value << kin.a_g_w;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;
}

}  // namespace

tl::expected<void, std::string> write_robot_yaml(
    const stdr_msgs::msg::RobotMsg& msg,
    const std::string& file_path)
{
  std::ofstream file{file_path};
  if (!file.is_open()) {
    return tl::unexpected<std::string>("Cannot open file for writing: " + file_path);
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "robot" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "robot_specifications" << YAML::Value;
  out << YAML::BeginSeq;

  emit_footprint(out, msg.footprint);
  emit_initial_pose(out, msg.initial_pose);

  for (const stdr_msgs::msg::LaserSensorMsg& laser : msg.laser_sensors) {
    emit_laser(out, laser);
  }
  for (const stdr_msgs::msg::SonarSensorMsg& sonar : msg.sonar_sensors) {
    emit_sonar(out, sonar);
  }
  for (const stdr_msgs::msg::RfidSensorMsg& rfid : msg.rfid_sensors) {
    emit_rfid_reader(out, rfid);
  }
  for (const stdr_msgs::msg::CO2SensorMsg& co2 : msg.co2_sensors) {
    emit_co2_sensor(out, co2);
  }
  for (const stdr_msgs::msg::ThermalSensorMsg& thermal : msg.thermal_sensors) {
    emit_thermal_sensor(out, thermal);
  }
  for (const stdr_msgs::msg::SoundSensorMsg& sound : msg.sound_sensors) {
    emit_sound_sensor(out, sound);
  }

  emit_kinematic(out, msg.kinematic_model);

  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::EndMap;

  if (!out.good()) {
    return tl::unexpected<std::string>("YAML emitter error: " + std::string{out.GetLastError()});
  }

  file << out.c_str();
  if (!file) {
    return tl::unexpected<std::string>("Write error for file: " + file_path);
  }

  return {};
}

}  // namespace stdr_parser
