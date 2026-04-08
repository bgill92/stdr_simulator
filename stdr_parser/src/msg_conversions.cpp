#include <stdr_parser/msg_conversions.hpp>

namespace stdr_parser
{

// --- Pose2D ---

geometry_msgs::msg::Pose2D to_ros_msg(const stdr_simulation::Pose2D& pose)
{
  geometry_msgs::msg::Pose2D msg;
  msg.x = pose.x;
  msg.y = pose.y;
  msg.theta = pose.theta;
  return msg;
}

stdr_simulation::Pose2D from_ros_msg(const geometry_msgs::msg::Pose2D& msg)
{
  stdr_simulation::Pose2D pose;
  pose.x = msg.x;
  pose.y = msg.y;
  pose.theta = msg.theta;
  return pose;
}

// --- Point2D ---

geometry_msgs::msg::Point to_ros_point(const stdr_simulation::Point2D& pt)
{
  geometry_msgs::msg::Point msg;
  msg.x = pt.x;
  msg.y = pt.y;
  msg.z = 0.0;
  return msg;
}

stdr_simulation::Point2D from_ros_point(const geometry_msgs::msg::Point& msg)
{
  stdr_simulation::Point2D pt;
  pt.x = msg.x;
  pt.y = msg.y;
  // z is intentionally ignored: the simulator operates in 2D only.
  return pt;
}

// --- NoiseConfig ---

stdr_msgs::msg::Noise to_ros_msg(const stdr_simulation::NoiseConfig& noise)
{
  stdr_msgs::msg::Noise msg;
  msg.noise = noise.enabled;
  msg.noise_mean = static_cast<float>(noise.mean);
  msg.noise_std = static_cast<float>(noise.std_dev);
  return msg;
}

stdr_simulation::NoiseConfig from_ros_msg(const stdr_msgs::msg::Noise& msg)
{
  stdr_simulation::NoiseConfig noise;
  noise.enabled = msg.noise;
  noise.mean = static_cast<double>(msg.noise_mean);
  noise.std_dev = static_cast<double>(msg.noise_std);
  return noise;
}

// --- Footprint ---

stdr_msgs::msg::FootprintMsg to_ros_msg(const stdr_simulation::Footprint& fp)
{
  stdr_msgs::msg::FootprintMsg msg;
  msg.radius = static_cast<float>(fp.radius);
  msg.points.reserve(fp.points.size());
  for (const stdr_simulation::Point2D& pt : fp.points)
  {
    msg.points.push_back(to_ros_point(pt));
  }
  return msg;
}

stdr_simulation::Footprint from_ros_msg(const stdr_msgs::msg::FootprintMsg& msg)
{
  stdr_simulation::Footprint fp;
  fp.radius = static_cast<double>(msg.radius);
  fp.points.reserve(msg.points.size());
  for (const geometry_msgs::msg::Point& pt : msg.points)
  {
    fp.points.push_back(from_ros_point(pt));
  }
  return fp;
}

// --- KinematicConfig ---

stdr_msgs::msg::KinematicMsg to_ros_msg(const stdr_simulation::KinematicConfig& kin)
{
  stdr_msgs::msg::KinematicMsg msg;
  msg.type = kin.type;
  msg.a_ux_ux = static_cast<float>(kin.a_ux_ux);
  msg.a_ux_uy = static_cast<float>(kin.a_ux_uy);
  msg.a_ux_w = static_cast<float>(kin.a_ux_w);
  msg.a_uy_ux = static_cast<float>(kin.a_uy_ux);
  msg.a_uy_uy = static_cast<float>(kin.a_uy_uy);
  msg.a_uy_w = static_cast<float>(kin.a_uy_w);
  msg.a_w_ux = static_cast<float>(kin.a_w_ux);
  msg.a_w_uy = static_cast<float>(kin.a_w_uy);
  msg.a_w_w = static_cast<float>(kin.a_w_w);
  msg.a_g_ux = static_cast<float>(kin.a_g_ux);
  msg.a_g_uy = static_cast<float>(kin.a_g_uy);
  msg.a_g_w = static_cast<float>(kin.a_g_w);
  return msg;
}

stdr_simulation::KinematicConfig from_ros_msg(const stdr_msgs::msg::KinematicMsg& msg)
{
  stdr_simulation::KinematicConfig kin;
  kin.type = msg.type;
  kin.a_ux_ux = static_cast<double>(msg.a_ux_ux);
  kin.a_ux_uy = static_cast<double>(msg.a_ux_uy);
  kin.a_ux_w = static_cast<double>(msg.a_ux_w);
  kin.a_uy_ux = static_cast<double>(msg.a_uy_ux);
  kin.a_uy_uy = static_cast<double>(msg.a_uy_uy);
  kin.a_uy_w = static_cast<double>(msg.a_uy_w);
  kin.a_w_ux = static_cast<double>(msg.a_w_ux);
  kin.a_w_uy = static_cast<double>(msg.a_w_uy);
  kin.a_w_w = static_cast<double>(msg.a_w_w);
  kin.a_g_ux = static_cast<double>(msg.a_g_ux);
  kin.a_g_uy = static_cast<double>(msg.a_g_uy);
  kin.a_g_w = static_cast<double>(msg.a_g_w);
  return kin;
}

// --- LaserConfig ---

stdr_msgs::msg::LaserSensorMsg to_ros_msg(const stdr_simulation::LaserConfig& cfg)
{
  stdr_msgs::msg::LaserSensorMsg msg;
  msg.max_angle = static_cast<float>(cfg.max_angle);
  msg.min_angle = static_cast<float>(cfg.min_angle);
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.min_range = static_cast<float>(cfg.min_range);
  msg.num_rays = cfg.num_rays;
  msg.noise = to_ros_msg(cfg.noise);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::LaserConfig from_ros_msg(const stdr_msgs::msg::LaserSensorMsg& msg)
{
  stdr_simulation::LaserConfig cfg;
  cfg.max_angle = static_cast<double>(msg.max_angle);
  cfg.min_angle = static_cast<double>(msg.min_angle);
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.min_range = static_cast<double>(msg.min_range);
  cfg.num_rays = msg.num_rays;
  cfg.noise = from_ros_msg(msg.noise);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- SonarConfig ---

stdr_msgs::msg::SonarSensorMsg to_ros_msg(const stdr_simulation::SonarConfig& cfg)
{
  stdr_msgs::msg::SonarSensorMsg msg;
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.min_range = static_cast<float>(cfg.min_range);
  msg.cone_angle = static_cast<float>(cfg.cone_angle);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.noise = to_ros_msg(cfg.noise);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::SonarConfig from_ros_msg(const stdr_msgs::msg::SonarSensorMsg& msg)
{
  stdr_simulation::SonarConfig cfg;
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.min_range = static_cast<double>(msg.min_range);
  cfg.cone_angle = static_cast<double>(msg.cone_angle);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.noise = from_ros_msg(msg.noise);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- RfidSensorConfig ---

stdr_msgs::msg::RfidSensorMsg to_ros_msg(const stdr_simulation::RfidSensorConfig& cfg)
{
  stdr_msgs::msg::RfidSensorMsg msg;
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.angle_span = static_cast<float>(cfg.angle_span);
  msg.signal_cutoff = static_cast<float>(cfg.signal_cutoff);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::RfidSensorConfig from_ros_msg(const stdr_msgs::msg::RfidSensorMsg& msg)
{
  stdr_simulation::RfidSensorConfig cfg;
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.angle_span = static_cast<double>(msg.angle_span);
  cfg.signal_cutoff = static_cast<double>(msg.signal_cutoff);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- CO2SensorConfig ---

stdr_msgs::msg::CO2SensorMsg to_ros_msg(const stdr_simulation::CO2SensorConfig& cfg)
{
  stdr_msgs::msg::CO2SensorMsg msg;
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::CO2SensorConfig from_ros_msg(const stdr_msgs::msg::CO2SensorMsg& msg)
{
  stdr_simulation::CO2SensorConfig cfg;
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- SoundSensorConfig ---

stdr_msgs::msg::SoundSensorMsg to_ros_msg(const stdr_simulation::SoundSensorConfig& cfg)
{
  stdr_msgs::msg::SoundSensorMsg msg;
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.angle_span = static_cast<float>(cfg.angle_span);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::SoundSensorConfig from_ros_msg(const stdr_msgs::msg::SoundSensorMsg& msg)
{
  stdr_simulation::SoundSensorConfig cfg;
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.angle_span = static_cast<double>(msg.angle_span);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- ThermalSensorConfig ---

stdr_msgs::msg::ThermalSensorMsg to_ros_msg(const stdr_simulation::ThermalSensorConfig& cfg)
{
  stdr_msgs::msg::ThermalSensorMsg msg;
  msg.max_range = static_cast<float>(cfg.max_range);
  msg.frequency = static_cast<float>(cfg.frequency);
  msg.angle_span = static_cast<float>(cfg.angle_span);
  msg.frame_id = cfg.frame_id;
  msg.pose = to_ros_msg(cfg.pose);
  return msg;
}

stdr_simulation::ThermalSensorConfig from_ros_msg(const stdr_msgs::msg::ThermalSensorMsg& msg)
{
  stdr_simulation::ThermalSensorConfig cfg;
  cfg.max_range = static_cast<double>(msg.max_range);
  cfg.frequency = static_cast<double>(msg.frequency);
  cfg.angle_span = static_cast<double>(msg.angle_span);
  cfg.frame_id = msg.frame_id;
  cfg.pose = from_ros_msg(msg.pose);
  return cfg;
}

// --- RobotConfig ---

stdr_msgs::msg::RobotMsg to_ros_msg(const stdr_simulation::RobotConfig& cfg)
{
  stdr_msgs::msg::RobotMsg msg;
  msg.initial_pose = to_ros_msg(cfg.initial_pose);
  msg.footprint = to_ros_msg(cfg.footprint);
  msg.laser_sensors.reserve(cfg.laser_sensors.size());
  for (const stdr_simulation::LaserConfig& s : cfg.laser_sensors)
  {
    msg.laser_sensors.push_back(to_ros_msg(s));
  }
  msg.sonar_sensors.reserve(cfg.sonar_sensors.size());
  for (const stdr_simulation::SonarConfig& s : cfg.sonar_sensors)
  {
    msg.sonar_sensors.push_back(to_ros_msg(s));
  }
  msg.rfid_sensors.reserve(cfg.rfid_sensors.size());
  for (const stdr_simulation::RfidSensorConfig& s : cfg.rfid_sensors)
  {
    msg.rfid_sensors.push_back(to_ros_msg(s));
  }
  msg.co2_sensors.reserve(cfg.co2_sensors.size());
  for (const stdr_simulation::CO2SensorConfig& s : cfg.co2_sensors)
  {
    msg.co2_sensors.push_back(to_ros_msg(s));
  }
  msg.sound_sensors.reserve(cfg.sound_sensors.size());
  for (const stdr_simulation::SoundSensorConfig& s : cfg.sound_sensors)
  {
    msg.sound_sensors.push_back(to_ros_msg(s));
  }
  msg.thermal_sensors.reserve(cfg.thermal_sensors.size());
  for (const stdr_simulation::ThermalSensorConfig& s : cfg.thermal_sensors)
  {
    msg.thermal_sensors.push_back(to_ros_msg(s));
  }
  msg.kinematic_model = to_ros_msg(cfg.kinematic_model);
  return msg;
}

stdr_simulation::RobotConfig from_ros_msg(const stdr_msgs::msg::RobotMsg& msg)
{
  stdr_simulation::RobotConfig cfg;
  cfg.initial_pose = from_ros_msg(msg.initial_pose);
  cfg.footprint = from_ros_msg(msg.footprint);
  cfg.laser_sensors.reserve(msg.laser_sensors.size());
  for (const stdr_msgs::msg::LaserSensorMsg& s : msg.laser_sensors)
  {
    cfg.laser_sensors.push_back(from_ros_msg(s));
  }
  cfg.sonar_sensors.reserve(msg.sonar_sensors.size());
  for (const stdr_msgs::msg::SonarSensorMsg& s : msg.sonar_sensors)
  {
    cfg.sonar_sensors.push_back(from_ros_msg(s));
  }
  cfg.rfid_sensors.reserve(msg.rfid_sensors.size());
  for (const stdr_msgs::msg::RfidSensorMsg& s : msg.rfid_sensors)
  {
    cfg.rfid_sensors.push_back(from_ros_msg(s));
  }
  cfg.co2_sensors.reserve(msg.co2_sensors.size());
  for (const stdr_msgs::msg::CO2SensorMsg& s : msg.co2_sensors)
  {
    cfg.co2_sensors.push_back(from_ros_msg(s));
  }
  cfg.sound_sensors.reserve(msg.sound_sensors.size());
  for (const stdr_msgs::msg::SoundSensorMsg& s : msg.sound_sensors)
  {
    cfg.sound_sensors.push_back(from_ros_msg(s));
  }
  cfg.thermal_sensors.reserve(msg.thermal_sensors.size());
  for (const stdr_msgs::msg::ThermalSensorMsg& s : msg.thermal_sensors)
  {
    cfg.thermal_sensors.push_back(from_ros_msg(s));
  }
  cfg.kinematic_model = from_ros_msg(msg.kinematic_model);
  return cfg;
}

// --- RfidTag ---

stdr_msgs::msg::RfidTag to_ros_msg(const stdr_simulation::RfidTag& tag)
{
  stdr_msgs::msg::RfidTag msg;
  msg.tag_id = tag.tag_id;
  msg.message = tag.message;
  msg.pose = to_ros_msg(tag.pose);
  return msg;
}

stdr_simulation::RfidTag from_ros_msg(const stdr_msgs::msg::RfidTag& msg)
{
  stdr_simulation::RfidTag tag;
  tag.tag_id = msg.tag_id;
  tag.message = msg.message;
  tag.pose = from_ros_msg(msg.pose);
  return tag;
}

// --- CO2Source ---

stdr_msgs::msg::CO2Source to_ros_msg(const stdr_simulation::CO2Source& src)
{
  stdr_msgs::msg::CO2Source msg;
  msg.id = src.id;
  msg.ppm = static_cast<float>(src.ppm);
  msg.pose = to_ros_msg(src.pose);
  return msg;
}

stdr_simulation::CO2Source from_ros_msg(const stdr_msgs::msg::CO2Source& msg)
{
  stdr_simulation::CO2Source src;
  src.id = msg.id;
  src.ppm = static_cast<double>(msg.ppm);
  src.pose = from_ros_msg(msg.pose);
  return src;
}

// --- SoundSource ---

stdr_msgs::msg::SoundSource to_ros_msg(const stdr_simulation::SoundSource& src)
{
  stdr_msgs::msg::SoundSource msg;
  msg.id = src.id;
  msg.dbs = static_cast<float>(src.dbs);
  msg.pose = to_ros_msg(src.pose);
  return msg;
}

stdr_simulation::SoundSource from_ros_msg(const stdr_msgs::msg::SoundSource& msg)
{
  stdr_simulation::SoundSource src;
  src.id = msg.id;
  src.dbs = static_cast<double>(msg.dbs);
  src.pose = from_ros_msg(msg.pose);
  return src;
}

// --- ThermalSource ---

stdr_msgs::msg::ThermalSource to_ros_msg(const stdr_simulation::ThermalSource& src)
{
  stdr_msgs::msg::ThermalSource msg;
  msg.id = src.id;
  msg.degrees = static_cast<float>(src.degrees);
  msg.pose = to_ros_msg(src.pose);
  return msg;
}

stdr_simulation::ThermalSource from_ros_msg(const stdr_msgs::msg::ThermalSource& msg)
{
  stdr_simulation::ThermalSource src;
  src.id = msg.id;
  src.degrees = static_cast<double>(msg.degrees);
  src.pose = from_ros_msg(msg.pose);
  return src;
}

// --- Sensor measurements ---

sensor_msgs::msg::LaserScan to_ros_msg(const stdr_simulation::LaserScan& scan)
{
  sensor_msgs::msg::LaserScan msg;
  msg.angle_min = static_cast<float>(scan.angle_min);
  msg.angle_max = static_cast<float>(scan.angle_max);
  msg.angle_increment = static_cast<float>(scan.angle_increment);
  msg.range_min = static_cast<float>(scan.range_min);
  msg.range_max = static_cast<float>(scan.range_max);
  msg.ranges = scan.ranges;  // Already float.
  return msg;
}

sensor_msgs::msg::Range to_ros_sonar_msg(const stdr_simulation::SonarScan& scan,
                                         const stdr_simulation::SonarConfig& config)
{
  sensor_msgs::msg::Range msg;
  msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  msg.field_of_view = static_cast<float>(config.cone_angle);
  msg.min_range = static_cast<float>(config.min_range);
  msg.max_range = static_cast<float>(config.max_range);
  msg.range = static_cast<float>(scan.range);
  return msg;
}

stdr_msgs::msg::RfidSensorMeasurementMsg to_ros_msg(const stdr_simulation::RfidMeasurement& meas)
{
  stdr_msgs::msg::RfidSensorMeasurementMsg msg;
  msg.rfid_tags_ids = meas.tag_ids;
  msg.rfid_tags_msgs = meas.tag_messages;
  msg.rfid_tags_dbs.reserve(meas.tag_dbs.size());
  for (const double db : meas.tag_dbs)
  {
    msg.rfid_tags_dbs.push_back(static_cast<float>(db));
  }
  return msg;
}

stdr_msgs::msg::CO2SensorMeasurementMsg to_ros_msg(const stdr_simulation::CO2Measurement& meas)
{
  stdr_msgs::msg::CO2SensorMeasurementMsg msg;
  msg.co2_ppm = static_cast<float>(meas.ppm);
  return msg;
}

stdr_msgs::msg::SoundSensorMeasurementMsg to_ros_msg(const stdr_simulation::SoundMeasurement& meas)
{
  stdr_msgs::msg::SoundSensorMeasurementMsg msg;
  msg.sound_dbs = static_cast<float>(meas.dbs);
  return msg;
}

stdr_msgs::msg::ThermalSensorMeasurementMsg to_ros_msg(const stdr_simulation::ThermalMeasurement& meas)
{
  stdr_msgs::msg::ThermalSensorMeasurementMsg msg;
  msg.thermal_source_degrees.reserve(meas.source_degrees.size());
  for (const double deg : meas.source_degrees)
  {
    msg.thermal_source_degrees.push_back(static_cast<float>(deg));
  }
  return msg;
}

}  // namespace stdr_parser
