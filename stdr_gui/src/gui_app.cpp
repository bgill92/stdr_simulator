#include <stdr_gui/gui_app.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <numbers>

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

  // Use the viewport work area so panel layout adapts to the menu bar height
  // at any DPI, without querying the GLFW window size in logical coordinates.
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float content_y = viewport->WorkPos.y;
  const float content_h = viewport->WorkSize.y;
  const float total_w = viewport->WorkSize.x;

  constexpr float kRightPanelFraction = 0.25f;
  const float left_w = total_w * (1.0f - kRightPanelFraction);
  const float right_w = total_w * kRightPanelFraction;

  ImGui::SetNextWindowPos(ImVec2(0.0f, content_y), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(left_w, content_h), ImGuiCond_Always);
  map_panel_.render(snapshot, *backend_);

  ImGui::SetNextWindowPos(ImVec2(left_w, content_y), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(right_w, content_h), ImGuiCond_Always);
  robot_info_panel_.render(snapshot, *backend_, sensor_windows_);

  handle_file_dialog_result();
  render_spawn_dialog();

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
      // Store the path and show the spawn dialog so the user can set an
      // initial pose before committing the spawn.
      pending_robot_path_ = path;
      spawn_x_ = 0.0f;
      spawn_y_ = 0.0f;
      spawn_theta_ = 0.0f;
      show_spawn_dialog_ = true;
      open_spawn_popup_ = true;
      break;
    }
  }
}

void GuiApp::render_spawn_dialog()
{
  if (!show_spawn_dialog_)
  {
    return;
  }

  // OpenPopup is called once when the dialog is first requested so ImGui
  // registers the open transition exactly once.
  if (open_spawn_popup_)
  {
    ImGui::OpenPopup("Spawn Robot");
    open_spawn_popup_ = false;
  }

  if (ImGui::BeginPopupModal("Spawn Robot", &show_spawn_dialog_, ImGuiWindowFlags_AlwaysAutoResize))
  {
    ImGui::TextUnformatted("Set initial pose for robot:");
    ImGui::Separator();

    ImGui::InputFloat("X", &spawn_x_, 0.1f, 1.0f, "%.2f");
    ImGui::InputFloat("Y", &spawn_y_, 0.1f, 1.0f, "%.2f");
    constexpr float kPi = std::numbers::pi_v<float>;
    ImGui::SliderFloat("Theta", &spawn_theta_, -kPi, kPi, "%.2f rad");

    ImGui::Separator();

    if (ImGui::Button("Spawn", ImVec2(120.0f, 0.0f)))
    {
      const stdr_simulation::Pose2D pose{ static_cast<double>(spawn_x_), static_cast<double>(spawn_y_),
                                          static_cast<double>(spawn_theta_) };
      // Errors are surfaced through backend_->poll_messages() in the toolbar.
      std::ignore = backend_->spawn_robot(pending_robot_path_, pose);
      show_spawn_dialog_ = false;
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(120.0f, 0.0f)))
    {
      show_spawn_dialog_ = false;
      ImGui::CloseCurrentPopup();
    }

    ImGui::EndPopup();
  }
}

}  // namespace stdr_gui
