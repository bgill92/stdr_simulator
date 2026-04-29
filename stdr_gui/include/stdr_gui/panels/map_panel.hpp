#pragma once

#include <stdr_gui/map_transform.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <imgui.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
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
  void render(const SimulationSnapshot& snapshot, SimulatorBackend& backend,
              const std::unordered_set<std::string>& show_sensors_for);

  /** @brief Get the currently selected robot name (empty if none). */
  [[nodiscard]] const std::string& selected_robot() const;

  /** @brief Set the robot name that is currently under teleop control.
   *
   *  Call this each frame before render() so the velocity overlay knows which
   *  robot to show.  Pass an empty string to hide the overlay. */
  void set_teleop_target(std::string_view name);

private:
  void update_texture(const stdr_simulation::OccupancyGrid& grid);
  void handle_input(SimulatorBackend& backend);
  void render_map_image();
  void render_robots(const SimulationSnapshot& snapshot);
  void render_sensor_overlays(const SimulationSnapshot& snapshot,
                              const std::unordered_set<std::string>& show_sensors_for);
  void render_environment_sources(const SimulationSnapshot& snapshot);
  void render_map_info_overlay(const SimulationSnapshot& snapshot);
  void render_context_menu(SimulatorBackend& backend, const SimulationSnapshot& snapshot);
  void render_velocity_overlay(const SimulationSnapshot& snapshot);

  MapTransform transform_;
  // Top-left and size of the content region (below the title bar), captured at
  // the start of each render() call.  All draw-list calls that anchor
  // world-space coordinates into screen space use content_origin_ so that
  // drawings are never shifted up under the title bar.
  ImVec2 content_origin_{};
  ImVec2 content_size_{};
  GLuint map_texture_{ 0 };
  std::int32_t cached_map_width_{ 0 };
  std::int32_t cached_map_height_{ 0 };
  std::vector<std::int8_t> cached_map_data_;
  double cached_origin_x_{ 0.0 };
  double cached_origin_y_{ 0.0 };
  double cached_resolution_{ 1.0 };

  std::string selected_robot_;
  std::string teleop_target_;
  bool show_grid_{ false };
  bool locked_view_{ true };
  bool dragging_robot_{ false };

  // Screen position where the context menu was opened, captured on right-click
  // so "Teleport here" targets the click location rather than the menu item position.
  float context_click_x_{ 0.0f };
  float context_click_y_{ 0.0f };
};

}  // namespace stdr_gui
