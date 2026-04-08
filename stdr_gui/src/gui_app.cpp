#include <stdr_gui/gui_app.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>

namespace stdr_gui
{

GuiApp::GuiApp(std::unique_ptr<SimulatorBackend> backend) : backend_(std::move(backend))
{
}

GuiApp::~GuiApp()
{
  if (window_)
  {
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    glfwTerminate();
  }
}

tl::expected<void, std::string> GuiApp::init(int width, int height)
{
  if (!glfwInit())
  {
    return tl::make_unexpected(std::string("Failed to initialize GLFW"));
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(width, height, "STDR Simulator", nullptr, nullptr);
  if (!window_)
  {
    glfwTerminate();
    return tl::make_unexpected(std::string("Failed to create GLFW window"));
  }

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);  // VSync.

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  return {};
}

int GuiApp::run()
{
  while (!glfwWindowShouldClose(window_) && !shutdown_requested_.load())
  {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (first_frame_)
    {
      setup_docking_layout();
      first_frame_ = false;
    }

    const std::shared_ptr<const SimulationSnapshot> snapshot = backend_->get_snapshot();
    if (snapshot)
    {
      render_frame(*snapshot);
    }

    ImGui::Render();

    int display_w = 0;
    int display_h = 0;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window_);
  }
  return 0;
}

void GuiApp::request_shutdown()
{
  shutdown_requested_.store(true);
}

void GuiApp::render_frame(const SimulationSnapshot& snapshot)
{
  toolbar_.render(*backend_, file_dialog_, snapshot);
  map_panel_.render(snapshot, *backend_);
  robot_info_panel_.render(snapshot, *backend_, sensor_windows_);
  handle_file_dialog_result();

  // Render sensor windows, removing any that have been closed.
  std::erase_if(sensor_windows_, [&snapshot](const std::unique_ptr<SensorWindow>& w) { return !w->render(snapshot); });
}

void GuiApp::setup_docking_layout()
{
  // Docking is not available in this imgui build. This method is a no-op
  // placeholder for when it becomes available.
}

void GuiApp::handle_file_dialog_result()
{
  if (!file_dialog_.render())
  {
    return;
  }

  const std::string& path = file_dialog_.selected_path();
  switch (file_dialog_.purpose())
  {
    case FileDialogPurpose::kLoadMap:
    {
      // Errors are surfaced through backend_->poll_messages() in the toolbar.
      std::ignore = backend_->load_map(path);
      break;
    }
    case FileDialogPurpose::kLoadRobot:
    {
      // Errors are surfaced through backend_->poll_messages() in the toolbar.
      std::ignore = backend_->spawn_robot(path, stdr_simulation::Pose2D{});
      break;
    }
  }
}

}  // namespace stdr_gui
