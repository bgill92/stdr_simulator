#pragma once

/** @file Core data types for the simulation library. These mirror the ROS
 *  message definitions but are completely ROS-free, using double precision
 *  throughout. */

#include <cstdint>
#include <string>
#include <vector>

namespace stdr_simulation
{

/** 2D pose in world frame: position and orientation. */
struct Pose2D
{
  double x{ 0.0 };
  double y{ 0.0 };
  double theta{ 0.0 };
};

/** 2D velocity command: planar linear velocities and angular rate. */
struct Twist2D
{
  double linear_x{ 0.0 };
  double linear_y{ 0.0 };
  double angular_z{ 0.0 };
};

/** Gaussian noise configuration for sensor simulation. Field names (`mean`,
 *  `std_dev`) are intentionally simplified from the ROS msg (`noise_mean`,
 *  `noise_std`); conversion is handled in stdr_parser (Phase 4). */
struct NoiseConfig
{
  bool enabled{ false };
  double mean{ 0.0 };
  double std_dev{ 0.0 };
};

/** 2D point in Cartesian space. */
struct Point2D
{
  double x{ 0.0 };
  double y{ 0.0 };
};

/** Robot footprint: either a polygon or a circular approximation. `points`
 *  uses `Point2D` (z dropped) because the simulator operates in 2D only. */
struct Footprint
{
  std::vector<Point2D> points;
  double radius{ 0.0 };
};

/** Kinematic model parameters encoding velocity coupling coefficients. */
struct KinematicConfig
{
  std::string type;
  double a_ux_ux{ 0.0 };
  double a_ux_uy{ 0.0 };
  double a_ux_w{ 0.0 };
  double a_uy_ux{ 0.0 };
  double a_uy_uy{ 0.0 };
  double a_uy_w{ 0.0 };
  double a_w_ux{ 0.0 };
  double a_w_uy{ 0.0 };
  double a_w_w{ 0.0 };
  double a_g_ux{ 0.0 };
  double a_g_uy{ 0.0 };
  double a_g_w{ 0.0 };
};

/** Configuration for a laser range-finder sensor. */
struct LaserConfig
{
  double max_angle{ 0.0 };
  double min_angle{ 0.0 };
  double max_range{ 0.0 };
  double min_range{ 0.0 };
  std::int32_t num_rays{ 0 };
  NoiseConfig noise;
  double frequency{ 0.0 };
  std::string frame_id;
  Pose2D pose;
};

/** Configuration for an ultrasonic sonar sensor. */
struct SonarConfig
{
  double max_range{ 0.0 };
  double min_range{ 0.0 };
  double cone_angle{ 0.0 };
  double frequency{ 0.0 };
  NoiseConfig noise;
  std::string frame_id;
  Pose2D pose;
};

/** Configuration for an RFID reader sensor. */
struct RfidSensorConfig
{
  double max_range{ 0.0 };
  double angle_span{ 0.0 };
  double signal_cutoff{ 0.0 };
  double frequency{ 0.0 };
  std::string frame_id;
  Pose2D pose;
};

/** Configuration for a CO2 concentration sensor. */
struct CO2SensorConfig
{
  double max_range{ 0.0 };
  double frequency{ 0.0 };
  std::string frame_id;
  Pose2D pose;
};

/** Configuration for a sound-level sensor. */
struct SoundSensorConfig
{
  double max_range{ 0.0 };
  double frequency{ 0.0 };
  double angle_span{ 0.0 };
  std::string frame_id;
  Pose2D pose;
};

/** Configuration for a thermal (infrared) sensor. */
struct ThermalSensorConfig
{
  double max_range{ 0.0 };
  double frequency{ 0.0 };
  double angle_span{ 0.0 };
  std::string frame_id;
  Pose2D pose;
};

/** An RFID tag placed in the environment. */
struct RfidTag
{
  std::string tag_id;
  std::string message;
  Pose2D pose;
};

/** A CO2 emission source placed in the environment. */
struct CO2Source
{
  std::string id;
  double ppm{ 0.0 };
  Pose2D pose;
};

/** A sound emission source placed in the environment. */
struct SoundSource
{
  std::string id;
  double dbs{ 0.0 };
  Pose2D pose;
};

/** A thermal emission source placed in the environment. */
struct ThermalSource
{
  std::string id;
  double degrees{ 0.0 };
  Pose2D pose;
};

/** Result of a simulated laser scan. Uses float for ranges to reduce memory
 *  for dense scans. */
struct LaserScan
{
  double angle_min{ 0.0 };
  double angle_max{ 0.0 };
  double angle_increment{ 0.0 };
  double range_min{ 0.0 };
  double range_max{ 0.0 };
  std::vector<float> ranges;
};

/** Sonar range measurement result. */
struct SonarScan
{
  double range{ 0.0 };
};

/** RFID reader measurement: all tags detected in a single sweep. Field names
 *  (`tag_ids`, `tag_messages`, `tag_dbs`) are intentionally simplified from
 *  the ROS msg (`rfid_tags_ids`, `rfid_tags_msgs`, `rfid_tags_dbs`). */
struct RfidMeasurement
{
  std::vector<std::string> tag_ids;
  std::vector<std::string> tag_messages;
  std::vector<double> tag_dbs;
};

/** CO2 concentration measurement result. `ppm` maps to `co2_ppm` in the ROS msg. */
struct CO2Measurement
{
  double ppm{ 0.0 };
};

/** Sound level measurement result. `dbs` maps to `sound_dbs` in the ROS msg. */
struct SoundMeasurement
{
  double dbs{ 0.0 };
};

/** Thermal measurement: one reading per source detected. `source_degrees`
 *  maps to `thermal_source_degrees` in the ROS msg. */
struct ThermalMeasurement
{
  std::vector<double> source_degrees;
};

/** 2D occupancy grid for map representation. Uses int8_t to match the
 *  nav_msgs/OccupancyGrid convention: 0=free, 100=occupied, -1=unknown. */
struct OccupancyGrid
{
  std::int32_t width{ 0 };
  std::int32_t height{ 0 };
  double resolution{ 0.0 };
  Pose2D origin;
  std::vector<std::int8_t> data;
};

/** Metadata parsed from a ROS-style map YAML file. The image_path is resolved
 *  to an absolute path so the caller can load it without knowing the YAML's
 *  directory. Image loading itself is left to the caller. */
struct MapMetadata
{
  std::string image_path;
  double resolution{ 0.0 };
  Pose2D origin;
  double occupied_thresh{ 0.65 };
  double free_thresh{ 0.196 };
  bool negate{ false };
};

/** Full configuration for a single simulated robot instance. */
struct RobotConfig
{
  Pose2D initial_pose;
  Footprint footprint;
  /** Point about which the robot rotates, expressed in robot body frame.
   *  Defaults to (0,0), i.e. the robot's geometric centre. Override when
   *  the physical pivot differs from the coordinate origin (e.g. rear-axle
   *  steering). */
  Point2D center_of_rotation;
  std::vector<LaserConfig> laser_sensors;
  std::vector<SonarConfig> sonar_sensors;
  std::vector<RfidSensorConfig> rfid_sensors;
  std::vector<CO2SensorConfig> co2_sensors;
  std::vector<SoundSensorConfig> sound_sensors;
  std::vector<ThermalSensorConfig> thermal_sensors;
  KinematicConfig kinematic_model;
};

}  // namespace stdr_simulation
