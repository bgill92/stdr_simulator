#pragma once

namespace stdr_simulation::world {

/**
 * @brief Holds the occupancy grid and all environment entities for simulation.
 *
 * Owns the static obstacle map (loaded from a YAML/PGM map file) and the
 * dynamic entity collections (robots, sensor sources).  Provides accessors
 * used by sensor simulators and the collision checker.
 */
class WorldModel {
public:
  WorldModel() = default;
  ~WorldModel() = default;
};

}  // namespace stdr_simulation::world
