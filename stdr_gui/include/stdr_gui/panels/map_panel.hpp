#pragma once

#include <stdr_gui/map_transform.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <cstdint>
#include <string>
#include <vector>

typedef unsigned int GLuint;

namespace stdr_gui
{

/** @brief Renders the occupancy grid map with pan/zoom, robot visualization,
 *  and sensor overlays.
 *
 *  @warning Not thread-safe. Must be called from the render thread only. */
class MapPanel
{
public:
  MapPanel() = default;
  ~MapPanel();

  MapPanel(const MapPanel&) = delete;
  MapPanel& operator=(const MapPanel&) = delete;
  MapPanel(MapPanel&&) = delete;
  MapPanel& operator=(MapPanel&&) = delete;

  /** @brief Render the map panel as a dockable ImGui window. */
  void render(const SimulationSnapshot& snapshot, SimulatorBackend& backend);

  /** @brief Get the currently selected robot name (empty if none). */
  [[nodiscard]] const std::string& selected_robot() const;

private:
  void update_texture(const stdr_simulation::OccupancyGrid& grid);
  void handle_input(SimulatorBackend& backend);
  void render_map_image();
  void render_robots(const SimulationSnapshot& snapshot);
  void render_sensor_overlays(const SimulationSnapshot& snapshot);
  void render_environment_sources(const SimulationSnapshot& snapshot);
  void render_context_menu(SimulatorBackend& backend, const SimulationSnapshot& snapshot);

  MapTransform transform_;
  GLuint map_texture_{ 0 };
  std::int32_t cached_map_width_{ 0 };
  std::int32_t cached_map_height_{ 0 };
  std::vector<std::int8_t> cached_map_data_;
  double cached_origin_x_{ 0.0 };
  double cached_origin_y_{ 0.0 };
  double cached_resolution_{ 1.0 };

  std::string selected_robot_;
  bool show_grid_{ false };
  bool dragging_robot_{ false };
};

}  // namespace stdr_gui
