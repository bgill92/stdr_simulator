#pragma once

#include <stdr_gui/panels/file_dialog.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <string>
#include <vector>

namespace stdr_gui
{

/** @brief Top menu bar and toolbar for simulation control.
 *
 *  @warning Not thread-safe. Must be called from the render thread only. */
class Toolbar
{
public:
  /** @brief Render the main menu bar and toolbar. */
  void render(SimulatorBackend& backend, FileDialog& file_dialog, const SimulationSnapshot& snapshot);

private:
  void render_menu_bar(SimulatorBackend& backend, FileDialog& file_dialog);
  void render_status_bar(const SimulationSnapshot& snapshot, SimulatorBackend& backend);

  double speed_multiplier_{ 1.0 };
  std::vector<std::string> messages_;
};

/** @brief Format elapsed seconds as HH:MM:SS.cs (centiseconds). */
[[nodiscard]] std::string format_elapsed_time(double seconds);

}  // namespace stdr_gui
