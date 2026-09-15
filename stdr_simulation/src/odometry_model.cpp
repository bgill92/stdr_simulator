#include <stdr_simulation/odometry_model.hpp>

namespace stdr_simulation
{

tl::expected<OdometryModel, std::string> parse_odometry_model(const std::string_view value)
{
  if (value == "perfect")
  {
    return OdometryModel::Perfect;
  }
  if (value == "velocity")
  {
    return OdometryModel::Velocity;
  }
  return tl::unexpected("Unknown odometry_model '" + std::string(value) + "'. Allowed values: perfect, velocity.");
}

std::string_view to_string(const OdometryModel model)
{
  switch (model)
  {
    case OdometryModel::Perfect:
      return "perfect";
    case OdometryModel::Velocity:
      return "velocity";
  }
  // Unreachable for a valid enumerator; keeps the function total rather than UB.
  return "perfect";
}

}  // namespace stdr_simulation
