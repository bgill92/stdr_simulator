#include <stdr_gui/panels/toolbar.hpp>

#include <imgui.h>

#include <cmath>
#include <format>
#include <string>
#include <vector>

namespace stdr_gui
{

std::string format_elapsed_time(double seconds)
{
  if (seconds < 0.0)
  {
    seconds = 0.0;
  }

  const int total_cs = static_cast<int>(std::round(seconds * 100.0));
  const int cs = total_cs % 100;
  const int total_s = total_cs / 100;
  const int secs = total_s % 60;
  const int total_m = total_s / 60;
  const int mins = total_m % 60;
  const int hours = total_m / 60;

  return std::format("{:02d}:{:02d}:{:02d}.{:02d}", hours, mins, secs, cs);
}

void Toolbar::render(SimulatorBackend& backend, FileDialog& file_dialog, const SimulationSnapshot& snapshot)
{
  render_menu_bar(backend, file_dialog);
  render_status_bar(snapshot, backend);
}

void Toolbar::render_menu_bar(SimulatorBackend& backend, FileDialog& file_dialog)
{
  if (!ImGui::BeginMainMenuBar())
  {
    return;
  }

  if (ImGui::BeginMenu("File"))
  {
    if (ImGui::MenuItem("Load Map..."))
    {
      file_dialog.open(FileDialogPurpose::kLoadMap, ".");
    }
    if (ImGui::MenuItem("Load Robot..."))
    {
      file_dialog.open(FileDialogPurpose::kLoadRobot, ".");
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Exit"))
    {
      // FIXME-CLAUDE: NOT IMPLEMENTED - needs access to GLFWwindow* to call glfwSetWindowShouldClose.
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu("Simulation"))
  {
    if (ImGui::MenuItem("Start"))
    {
      backend.start();
    }
    if (ImGui::MenuItem("Pause"))
    {
      backend.pause();
    }
    if (ImGui::MenuItem("Reset"))
    {
      backend.reset();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Speed 0.5x"))
    {
      speed_multiplier_ = 0.5;
      backend.set_speed(speed_multiplier_);
    }
    if (ImGui::MenuItem("Speed 1x"))
    {
      speed_multiplier_ = 1.0;
      backend.set_speed(speed_multiplier_);
    }
    if (ImGui::MenuItem("Speed 2x"))
    {
      speed_multiplier_ = 2.0;
      backend.set_speed(speed_multiplier_);
    }
    if (ImGui::MenuItem("Speed 5x"))
    {
      speed_multiplier_ = 5.0;
      backend.set_speed(speed_multiplier_);
    }
    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();
}

void Toolbar::render_status_bar(const SimulationSnapshot& snapshot, SimulatorBackend& backend)
{
  // Render a thin toolbar strip just below the main menu bar.
  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  const float menu_height = ImGui::GetFrameHeight();
  ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x, viewport->WorkPos.y + menu_height));
  ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, 0.0f));

  constexpr ImGuiWindowFlags kToolbarFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                                             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollWithMouse |
                                             ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                             ImGuiWindowFlags_NoNav;

  ImGui::Begin("##Toolbar", nullptr, kToolbarFlags);

  const char* status = snapshot.running ? "Running" : "Paused";
  ImGui::Text("%s  |  Speed: %.1fx  |  Time: %s", status, speed_multiplier_,
              format_elapsed_time(snapshot.elapsed_time).c_str());

  // Show any backend messages.
  std::vector<std::string> new_messages = backend.poll_messages();
  for (std::string& msg : new_messages)
  {
    messages_.push_back(std::move(msg));
  }

  // Keep only the last few messages to avoid accumulation.
  constexpr std::size_t kMaxMessages = 5;
  if (messages_.size() > kMaxMessages)
  {
    messages_.erase(messages_.begin(), messages_.begin() + static_cast<std::ptrdiff_t>(messages_.size() - kMaxMessages));
  }

  for (const std::string& msg : messages_)
  {
    ImGui::SameLine();
    ImGui::Separator();
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "%s", msg.c_str());
  }

  ImGui::End();
}

}  // namespace stdr_gui
