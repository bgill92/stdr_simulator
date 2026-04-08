#pragma once

#include <stdr_gui/map_transform.hpp>
#include <stdr_gui/panels/file_dialog.hpp>
#include <stdr_gui/panels/map_panel.hpp>
#include <stdr_gui/panels/robot_info_panel.hpp>
#include <stdr_gui/panels/sensor_window.hpp>
#include <stdr_gui/panels/toolbar.hpp>
#include <stdr_gui/simulator_backend.hpp>
#include <tl_expected/expected.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;

namespace stdr_gui
{

/** @brief Main application class managing the GLFW window and ImGui lifecycle.
 *
 *  @warning Not thread-safe. Must be constructed and used on the main thread. */
class GuiApp
{
public:
  explicit GuiApp(std::unique_ptr<SimulatorBackend> backend);
  ~GuiApp();

  GuiApp(const GuiApp&) = delete;
  GuiApp& operator=(const GuiApp&) = delete;
  GuiApp(GuiApp&&) = delete;
  GuiApp& operator=(GuiApp&&) = delete;

  /** @brief Initialize GLFW, OpenGL context, ImGui, and ImPlot.
   *  @return Error string on failure. */
  [[nodiscard]] tl::expected<void, std::string> init(int width = 1280, int height = 720);

  /** @brief Run the main render loop. Blocks until the window is closed.
   *  @return Exit code (0 = normal). */
  [[nodiscard]] int run();

  /** @brief Request the render loop to stop. Thread-safe via atomic. */
  void request_shutdown();

private:
  void render_frame(const SimulationSnapshot& snapshot);
  void setup_docking_layout();
  void handle_file_dialog_result();

  std::unique_ptr<SimulatorBackend> backend_;
  GLFWwindow* window_{ nullptr };
  std::atomic<bool> shutdown_requested_{ false };
  bool first_frame_{ true };

  MapPanel map_panel_;
  RobotInfoPanel robot_info_panel_;
  Toolbar toolbar_;
  FileDialog file_dialog_;
  std::vector<std::unique_ptr<SensorWindow>> sensor_windows_;
};

}  // namespace stdr_gui
