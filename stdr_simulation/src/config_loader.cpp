#include <stdr_simulation/config_loader.hpp>

#include <yaml-cpp/yaml.h>

#include <tl_expected/expected.hpp>

#include <filesystem>
#include <string>

namespace stdr_simulation
{
namespace
{

Pose2D parse_pose(const YAML::Node& node)
{
  Pose2D p;
  if (node["x"])
    p.x = node["x"].as<double>();
  if (node["y"])
    p.y = node["y"].as<double>();
  if (node["theta"])
    p.theta = node["theta"].as<double>();
  return p;
}

// Reads noise.noise_specifications.{noise_mean, noise_std} from a sensor
// specifications node. Returns an enabled NoiseConfig only if noise is present.
NoiseConfig parse_noise(const YAML::Node& node)
{
  NoiseConfig cfg;
  if (!node["noise"])
    return cfg;
  const YAML::Node specs = node["noise"]["noise_specifications"];
  if (!specs)
    return cfg;
  cfg.enabled = true;
  if (specs["noise_mean"])
    cfg.mean = specs["noise_mean"].as<double>();
  if (specs["noise_std"])
    cfg.std_dev = specs["noise_std"].as<double>();
  return cfg;
}

LaserConfig parse_laser_specs(const YAML::Node& specs)
{
  LaserConfig cfg;
  if (specs["max_angle"])
    cfg.max_angle = specs["max_angle"].as<double>();
  if (specs["min_angle"])
    cfg.min_angle = specs["min_angle"].as<double>();
  if (specs["max_range"])
    cfg.max_range = specs["max_range"].as<double>();
  if (specs["min_range"])
    cfg.min_range = specs["min_range"].as<double>();
  if (specs["num_rays"])
    cfg.num_rays = specs["num_rays"].as<std::int32_t>();
  if (specs["frequency"])
    cfg.frequency = specs["frequency"].as<double>();
  if (specs["frame_id"])
    cfg.frame_id = specs["frame_id"].as<std::string>();
  if (specs["pose"])
    cfg.pose = parse_pose(specs["pose"]);
  cfg.noise = parse_noise(specs);
  return cfg;
}

// Loads base laser config from an external file, then applies inline overrides.
[[nodiscard]] tl::expected<LaserConfig, std::string> parse_laser(const YAML::Node& node, const std::string& base_dir)
{
  LaserConfig cfg;
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["laser"]["laser_specifications"];
      if (file_specs)
        cfg = parse_laser_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load laser file " + full.string() + ": " + e.what());
    }
  }
  if (node["laser_specifications"])
  {
    const YAML::Node specs = node["laser_specifications"];
    if (specs["max_angle"])
      cfg.max_angle = specs["max_angle"].as<double>();
    if (specs["min_angle"])
      cfg.min_angle = specs["min_angle"].as<double>();
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["min_range"])
      cfg.min_range = specs["min_range"].as<double>();
    if (specs["num_rays"])
      cfg.num_rays = specs["num_rays"].as<std::int32_t>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["frame_id"])
      cfg.frame_id = specs["frame_id"].as<std::string>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
    const NoiseConfig override_noise = parse_noise(specs);
    if (override_noise.enabled)
      cfg.noise = override_noise;
  }
  return cfg;
}

SonarConfig parse_sonar_specs(const YAML::Node& specs)
{
  SonarConfig cfg;
  if (specs["max_range"])
    cfg.max_range = specs["max_range"].as<double>();
  if (specs["min_range"])
    cfg.min_range = specs["min_range"].as<double>();
  if (specs["cone_angle"])
    cfg.cone_angle = specs["cone_angle"].as<double>();
  if (specs["frequency"])
    cfg.frequency = specs["frequency"].as<double>();
  if (specs["frame_id"])
    cfg.frame_id = specs["frame_id"].as<std::string>();
  if (specs["pose"])
    cfg.pose = parse_pose(specs["pose"]);
  cfg.noise = parse_noise(specs);
  return cfg;
}

[[nodiscard]] tl::expected<SonarConfig, std::string> parse_sonar(const YAML::Node& node, const std::string& base_dir)
{
  SonarConfig cfg;
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["sonar"]["sonar_specifications"];
      if (file_specs)
        cfg = parse_sonar_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load sonar file " + full.string() + ": " + e.what());
    }
  }
  if (node["sonar_specifications"])
  {
    const YAML::Node specs = node["sonar_specifications"];
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["min_range"])
      cfg.min_range = specs["min_range"].as<double>();
    if (specs["cone_angle"])
      cfg.cone_angle = specs["cone_angle"].as<double>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["frame_id"])
      cfg.frame_id = specs["frame_id"].as<std::string>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
    const NoiseConfig override_noise = parse_noise(specs);
    if (override_noise.enabled)
      cfg.noise = override_noise;
  }
  return cfg;
}

[[nodiscard]] tl::expected<KinematicConfig, std::string> parse_kinematic(const YAML::Node& node,
                                                                         const std::string& base_dir)
{
  KinematicConfig cfg;
  const auto apply_kinematic_specs = [&](const YAML::Node& specs) {
    if (specs["kinematic_model"])
      cfg.type = specs["kinematic_model"].as<std::string>();
    if (specs["kinematic_parameters"])
    {
      const YAML::Node p = specs["kinematic_parameters"];
      if (p["a_ux_ux"])
        cfg.a_ux_ux = p["a_ux_ux"].as<double>();
      if (p["a_ux_uy"])
        cfg.a_ux_uy = p["a_ux_uy"].as<double>();
      if (p["a_ux_w"])
        cfg.a_ux_w = p["a_ux_w"].as<double>();
      if (p["a_uy_ux"])
        cfg.a_uy_ux = p["a_uy_ux"].as<double>();
      if (p["a_uy_uy"])
        cfg.a_uy_uy = p["a_uy_uy"].as<double>();
      if (p["a_uy_w"])
        cfg.a_uy_w = p["a_uy_w"].as<double>();
      if (p["a_w_ux"])
        cfg.a_w_ux = p["a_w_ux"].as<double>();
      if (p["a_w_uy"])
        cfg.a_w_uy = p["a_w_uy"].as<double>();
      if (p["a_w_w"])
        cfg.a_w_w = p["a_w_w"].as<double>();
      if (p["a_g_ux"])
        cfg.a_g_ux = p["a_g_ux"].as<double>();
      if (p["a_g_uy"])
        cfg.a_g_uy = p["a_g_uy"].as<double>();
      if (p["a_g_w"])
        cfg.a_g_w = p["a_g_w"].as<double>();
    }
  };

  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["kinematic"]["kinematic_specifications"];
      if (file_specs)
        apply_kinematic_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load kinematic file " + full.string() + ": " + e.what());
    }
  }
  if (node["kinematic_specifications"])
  {
    apply_kinematic_specs(node["kinematic_specifications"]);
  }
  return cfg;
}

[[nodiscard]] tl::expected<RfidSensorConfig, std::string> parse_rfid(const YAML::Node& node, const std::string& base_dir)
{
  RfidSensorConfig cfg;
  const auto apply_specs = [&](const YAML::Node& specs) {
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["angle_span"])
      cfg.angle_span = specs["angle_span"].as<double>();
    if (specs["signal_cutoff"])
      cfg.signal_cutoff = specs["signal_cutoff"].as<double>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
  };
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["rfid_reader"]["rfid_reader_specifications"];
      if (file_specs)
        apply_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load rfid file " + full.string() + ": " + e.what());
    }
  }
  if (node["rfid_reader_specifications"])
    apply_specs(node["rfid_reader_specifications"]);
  return cfg;
}

[[nodiscard]] tl::expected<CO2SensorConfig, std::string> parse_co2(const YAML::Node& node, const std::string& base_dir)
{
  CO2SensorConfig cfg;
  const auto apply_specs = [&](const YAML::Node& specs) {
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
  };
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["co2_sensor"]["co2_sensor_specifications"];
      if (file_specs)
        apply_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load co2 file " + full.string() + ": " + e.what());
    }
  }
  if (node["co2_sensor_specifications"])
    apply_specs(node["co2_sensor_specifications"]);
  return cfg;
}

[[nodiscard]] tl::expected<ThermalSensorConfig, std::string> parse_thermal(const YAML::Node& node,
                                                                           const std::string& base_dir)
{
  ThermalSensorConfig cfg;
  const auto apply_specs = [&](const YAML::Node& specs) {
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["angle_span"])
      cfg.angle_span = specs["angle_span"].as<double>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
  };
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["thermal_sensor"]["thermal_sensor_specifications"];
      if (file_specs)
        apply_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load thermal file " + full.string() + ": " + e.what());
    }
  }
  if (node["thermal_sensor_specifications"])
    apply_specs(node["thermal_sensor_specifications"]);
  return cfg;
}

[[nodiscard]] tl::expected<SoundSensorConfig, std::string> parse_sound(const YAML::Node& node,
                                                                       const std::string& base_dir)
{
  SoundSensorConfig cfg;
  const auto apply_specs = [&](const YAML::Node& specs) {
    if (specs["max_range"])
      cfg.max_range = specs["max_range"].as<double>();
    if (specs["frequency"])
      cfg.frequency = specs["frequency"].as<double>();
    if (specs["angle_span"])
      cfg.angle_span = specs["angle_span"].as<double>();
    if (specs["pose"])
      cfg.pose = parse_pose(specs["pose"]);
  };
  if (node["filename"])
  {
    const std::string rel = node["filename"].as<std::string>();
    const std::filesystem::path full = std::filesystem::path(base_dir) / rel;
    try
    {
      const YAML::Node file_root = YAML::LoadFile(full.string());
      const YAML::Node file_specs = file_root["sound_sensor"]["sound_sensor_specifications"];
      if (file_specs)
        apply_specs(file_specs);
    }
    catch (const YAML::Exception& e)
    {
      return tl::unexpected("Failed to load sound file " + full.string() + ": " + e.what());
    }
  }
  if (node["sound_sensor_specifications"])
    apply_specs(node["sound_sensor_specifications"]);
  return cfg;
}

}  // namespace

tl::expected<RobotConfig, std::string> load_robot_config(const std::string& yaml_path, const std::string& base_dir)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node robot_specs = root["robot"]["robot_specifications"];
    if (!robot_specs || !robot_specs.IsSequence())
    {
      return tl::unexpected("Missing robot.robot_specifications sequence in " + yaml_path);
    }

    RobotConfig config;
    for (const YAML::Node& item : robot_specs)
    {
      if (item["footprint"])
      {
        const YAML::Node fp = item["footprint"]["footprint_specifications"];
        if (fp)
        {
          if (fp["radius"])
            config.footprint.radius = fp["radius"].as<double>();
          if (fp["points"])
          {
            for (const YAML::Node& pt_node : fp["points"])
            {
              // Unwrap the optional `point:` key to support both wrapped and
              // bare point entries in the YAML.
              const YAML::Node pt = pt_node["point"] ? pt_node["point"] : pt_node;
              Point2D p;
              if (pt["x"])
                p.x = pt["x"].as<double>();
              if (pt["y"])
                p.y = pt["y"].as<double>();
              config.footprint.points.push_back(p);
            }
          }
        }
      }
      else if (item["initial_pose"])
      {
        config.initial_pose = parse_pose(item["initial_pose"]);
      }
      else if (item["laser"])
      {
        tl::expected<LaserConfig, std::string> laser = parse_laser(item["laser"], base_dir);
        if (!laser)
          return tl::unexpected(laser.error());
        config.laser_sensors.push_back(std::move(*laser));
      }
      else if (item["sonar"])
      {
        tl::expected<SonarConfig, std::string> sonar = parse_sonar(item["sonar"], base_dir);
        if (!sonar)
          return tl::unexpected(sonar.error());
        config.sonar_sensors.push_back(std::move(*sonar));
      }
      else if (item["kinematic"])
      {
        tl::expected<KinematicConfig, std::string> kin = parse_kinematic(item["kinematic"], base_dir);
        if (!kin)
          return tl::unexpected(kin.error());
        config.kinematic_model = std::move(*kin);
      }
      else if (item["rfid_reader"])
      {
        tl::expected<RfidSensorConfig, std::string> rfid = parse_rfid(item["rfid_reader"], base_dir);
        if (!rfid)
          return tl::unexpected(rfid.error());
        config.rfid_sensors.push_back(std::move(*rfid));
      }
      else if (item["co2_sensor"])
      {
        tl::expected<CO2SensorConfig, std::string> co2 = parse_co2(item["co2_sensor"], base_dir);
        if (!co2)
          return tl::unexpected(co2.error());
        config.co2_sensors.push_back(std::move(*co2));
      }
      else if (item["thermal_sensor"])
      {
        tl::expected<ThermalSensorConfig, std::string> thermal = parse_thermal(item["thermal_sensor"], base_dir);
        if (!thermal)
          return tl::unexpected(thermal.error());
        config.thermal_sensors.push_back(std::move(*thermal));
      }
      else if (item["sound_sensor"])
      {
        tl::expected<SoundSensorConfig, std::string> sound = parse_sound(item["sound_sensor"], base_dir);
        if (!sound)
          return tl::unexpected(sound.error());
        config.sound_sensors.push_back(std::move(*sound));
      }
    }
    return config;
  }
  catch (const YAML::Exception& e)
  {
    return tl::unexpected("Failed to load robot config " + yaml_path + ": " + e.what());
  }
}

tl::expected<MapMetadata, std::string> load_map_metadata(const std::string& yaml_path)
{
  try
  {
    const YAML::Node root = YAML::LoadFile(yaml_path);

    if (!root["image"])
    {
      return tl::unexpected("Missing 'image' key in map YAML " + yaml_path);
    }
    if (!root["resolution"])
    {
      return tl::unexpected("Missing 'resolution' key in map YAML " + yaml_path);
    }

    MapMetadata meta;

    const std::string image_rel = root["image"].as<std::string>();
    const std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
    meta.image_path = (yaml_dir / image_rel).lexically_normal().string();

    meta.resolution = root["resolution"].as<double>();

    if (root["origin"] && root["origin"].IsSequence() && root["origin"].size() >= 2)
    {
      meta.origin.x = root["origin"][0].as<double>();
      meta.origin.y = root["origin"][1].as<double>();
      if (root["origin"].size() >= 3)
      {
        meta.origin.theta = root["origin"][2].as<double>();
      }
    }

    if (root["occupied_thresh"])
      meta.occupied_thresh = root["occupied_thresh"].as<double>();
    if (root["free_thresh"])
      meta.free_thresh = root["free_thresh"].as<double>();
    if (root["negate"])
      meta.negate = (root["negate"].as<int>() != 0);

    return meta;
  }
  catch (const YAML::Exception& e)
  {
    return tl::unexpected("Failed to load map metadata " + yaml_path + ": " + e.what());
  }
}

}  // namespace stdr_simulation
