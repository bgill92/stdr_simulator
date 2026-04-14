#include <stdr_simulation/world/world_model.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace stdr_simulation::world
{

void WorldModel::set_map(stdr_simulation::OccupancyGrid map)
{
  map_ = std::move(map);
}

const stdr_simulation::OccupancyGrid* WorldModel::get_map() const
{
  if (!map_.has_value())
  {
    return nullptr;
  }
  return &map_.value();
}

std::string WorldModel::add_robot(const stdr_simulation::RobotConfig& config)
{
  const std::string name = "robot" + std::to_string(next_robot_id_++);
  robots_[name] = RobotState{
    .name = name,
    .config = config,
    .pose = config.initial_pose,
    .cmd_vel = {},
  };
  return name;
}

void WorldModel::remove_robot(const std::string& name)
{
  robots_.erase(name);
}

void WorldModel::set_robot_pose(const std::string& name, const stdr_simulation::Pose2D& pose)
{
  const auto it = robots_.find(name);
  if (it != robots_.end())
  {
    it->second.pose = pose;
  }
}

void WorldModel::set_robot_cmd_vel(const std::string& name, const stdr_simulation::Twist2D& cmd)
{
  const auto it = robots_.find(name);
  if (it != robots_.end())
  {
    it->second.cmd_vel = cmd;
  }
}

const RobotState* WorldModel::get_robot(const std::string& name) const
{
  const auto it = robots_.find(name);
  if (it == robots_.end())
  {
    return nullptr;
  }
  return &it->second;
}

std::vector<RobotState> WorldModel::get_all_robots() const
{
  std::vector<RobotState> result;
  result.reserve(robots_.size());
  for (const auto& [name, state] : robots_)
  {
    result.push_back(state);
  }
  return result;
}

void WorldModel::add_rfid_tag(const stdr_simulation::RfidTag& tag)
{
  rfid_tags_.push_back(tag);
}

void WorldModel::remove_rfid_tag(const std::string& id)
{
  std::erase_if(rfid_tags_, [&id](const stdr_simulation::RfidTag& t) { return t.tag_id == id; });
}

const std::vector<stdr_simulation::RfidTag>& WorldModel::get_rfid_tags() const
{
  return rfid_tags_;
}

void WorldModel::add_co2_source(const stdr_simulation::CO2Source& source)
{
  co2_sources_.push_back(source);
}

void WorldModel::remove_co2_source(const std::string& id)
{
  std::erase_if(co2_sources_, [&id](const stdr_simulation::CO2Source& s) { return s.id == id; });
}

const std::vector<stdr_simulation::CO2Source>& WorldModel::get_co2_sources() const
{
  return co2_sources_;
}

void WorldModel::add_thermal_source(const stdr_simulation::ThermalSource& source)
{
  thermal_sources_.push_back(source);
}

void WorldModel::remove_thermal_source(const std::string& id)
{
  std::erase_if(thermal_sources_, [&id](const stdr_simulation::ThermalSource& s) { return s.id == id; });
}

const std::vector<stdr_simulation::ThermalSource>& WorldModel::get_thermal_sources() const
{
  return thermal_sources_;
}

void WorldModel::add_sound_source(const stdr_simulation::SoundSource& source)
{
  sound_sources_.push_back(source);
}

void WorldModel::remove_sound_source(const std::string& id)
{
  std::erase_if(sound_sources_, [&id](const stdr_simulation::SoundSource& s) { return s.id == id; });
}

const std::vector<stdr_simulation::SoundSource>& WorldModel::get_sound_sources() const
{
  return sound_sources_;
}

}  // namespace stdr_simulation::world
