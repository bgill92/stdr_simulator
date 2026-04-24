#include <stdr_gui/plot/plot_panel.hpp>

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>

#include <imgui.h>

#include <chrono>
#include <cstddef>
#include <exception>
#include <string>
#include <utility>

// make_backend_sim_view is defined in sim_view.cpp and declared here to avoid
// pulling the full BackendSimView implementation into this header.
namespace stdr::plot
{
std::unique_ptr<SimView> make_backend_sim_view(stdr_gui::SimulatorBackend& backend,
                                               std::unordered_map<std::string, std::uint64_t>& laser_cursors,
                                               std::unordered_map<std::string, std::uint64_t>& sonar_cursors);
}  // namespace stdr::plot

namespace stdr::plot
{

PlotPanel::PlotPanel(stdr_gui::SimulatorBackend& backend, PlotterRegistry& registry) : backend_(backend)
{
  // Build one slot per registered factory.  The factory is stored in the slot
  // so Remove can reinstantiate without going back to the registry.
  const std::vector<PlotterRegistry::Entry> entries = registry.entries();
  slots_.reserve(entries.size());
  for (const PlotterRegistry::Entry& entry : entries)
  {
    Slot& slot = slots_.emplace_back();
    slot.factory = entry.factory;
    init_slot(slot);
  }
}

// ---------------------------------------------------------------------------
// Public slot control API (used by tests and by render_menu_items)
// ---------------------------------------------------------------------------

void PlotPanel::pause_slot(std::size_t index)
{
  if (index < slots_.size())
  {
    slots_[index].paused = true;
  }
}

void PlotPanel::unpause_slot(std::size_t index)
{
  if (index < slots_.size())
  {
    slots_[index].paused = false;
  }
}

void PlotPanel::remove_slot(std::size_t index)
{
  if (index >= slots_.size())
  {
    return;
  }
  Slot& slot = slots_[index];

  // Reset all accumulated state before reinstantiating so the new plotter
  // starts from a clean baseline rather than inheriting stale cursors/errors.
  slot.sink = PlotSink{};
  slot.laser_cursors.clear();
  slot.sonar_cursors.clear();
  slot.paused = false;
  slot.error.clear();
  slot.last_sample = std::chrono::steady_clock::time_point{};
  slot.overran_budget = false;

  init_slot(slot);
}

std::vector<PlotterStatus> PlotPanel::slot_status() const
{
  std::vector<PlotterStatus> result;
  result.reserve(slots_.size());
  for (const Slot& slot : slots_)
  {
    PlotterStatus status;
    status.name = slot.name;
    status.paused = slot.paused;
    status.removed = false;  // Slots are always alive; Remove triggers immediate rebuild.
    status.error = slot.error;
    status.overran_budget = slot.overran_budget;
    result.push_back(std::move(status));
  }
  return result;
}

// ---------------------------------------------------------------------------
// Per-frame driving
// ---------------------------------------------------------------------------

void PlotPanel::pump_sample()
{
  for (Slot& slot : slots_)
  {
    // Skip paused slots — keep state, skip both sample and render.
    if (slot.paused)
    {
      continue;
    }

    // Slots with a recorded error are also skipped to avoid spamming the same
    // exception every frame.  The user must Remove/Reset to clear the error.
    if (!slot.error.empty())
    {
      continue;
    }

    if (!is_sample_due(slot))
    {
      continue;
    }

    // SimView is built per-plotter (not per-frame) because each plotter's
    // sensor-event cursor is held inside its own SimView instance — sharing
    // one view across plotters would break cursor isolation.
    std::unique_ptr<SimView> view = make_backend_sim_view(backend_, slot.laser_cursors, slot.sonar_cursors);

    const std::chrono::steady_clock::time_point sample_start = std::chrono::steady_clock::now();
    try
    {
      slot.plotter->on_sample(*view, slot.sink);
    }
    catch (const std::exception& e)
    {
      slot.error = e.what();
      continue;
    }
    catch (...)
    {
      slot.error = "Unknown exception in on_sample.";
      continue;
    }

    const std::chrono::steady_clock::duration elapsed = std::chrono::steady_clock::now() - sample_start;
    slot.overran_budget = (elapsed > kSampleBudget);
    slot.last_sample = sample_start;
  }
}

void PlotPanel::render()
{
  // Drive the sample phase headlessly first — no ImGui calls, safe to test.
  pump_sample();

  // Thin ImGui shell: iterate slots and emit one child window per active plotter.
  for (std::size_t i = 0; i < slots_.size(); ++i)
  {
    Slot& slot = slots_[i];

    // Skip paused slots entirely.
    if (slot.paused)
    {
      continue;
    }

    // Slots with a recorded error display an inline error child instead of the
    // plot so the user can read the message without opening the menu.
    if (!slot.error.empty())
    {
      ImGui::PushID(static_cast<int>(i));
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.3f, 0.0f, 0.0f, 1.0f));
      ImGui::BeginChild("##plot_error", ImVec2(-1.0f, 60.0f), true);
      ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[ERROR] %s: %s", slot.name.c_str(), slot.error.c_str());
      ImGui::EndChild();
      ImGui::PopStyleColor();
      ImGui::PopID();
      continue;
    }

    // --- Render phase (inside its own child so plotters scroll independently) ---

    // Reserve a fixed-height child so multiple plotters stack vertically
    // without each one consuming the full panel height.  200 px is a
    // reasonable default; users can scroll the outer panel for more.
    constexpr float kChildHeight = 200.0f;

    ImGui::PushID(static_cast<int>(i));
    ImGui::BeginChild("##plot_child", ImVec2(-1.0f, kChildHeight), false);

    // Budget-overrun indicator — yellow dot before the plotter name.
    if (slot.overran_budget)
    {
      ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.0f, 1.0f), "(!) ");
      if (ImGui::IsItemHovered())
      {
        ImGui::SetTooltip("on_sample exceeded the 5 ms budget.");
      }
      ImGui::SameLine();
    }

    ImGui::TextUnformatted(slot.name.c_str());
    ImGui::Separator();

    try
    {
      slot.plotter->on_render(PlotView{ slot.sink });
    }
    catch (const std::exception& e)
    {
      slot.error = e.what();
    }
    catch (...)
    {
      slot.error = "Unknown exception in on_render.";
    }

    ImGui::EndChild();
    ImGui::PopID();
  }
}

void PlotPanel::render_menu_items()
{
  for (std::size_t i = 0; i < slots_.size(); ++i)
  {
    Slot& slot = slots_[i];

    // Unique ImGui ID scope per slot so controls do not alias.
    ImGui::PushID(static_cast<int>(i));

    bool paused = slot.paused;
    if (ImGui::Checkbox("##pause", &paused))
    {
      slot.paused = paused;
    }
    if (ImGui::IsItemHovered())
    {
      ImGui::SetTooltip("Pause: keeps algorithm state but skips sampling and rendering.");
    }
    ImGui::SameLine();

    // Error indicator (red dot) shown inline in the menu.
    if (!slot.error.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "[!]");
      if (ImGui::IsItemHovered())
      {
        ImGui::SetTooltip("Error: %s", slot.error.c_str());
      }
      ImGui::SameLine();
    }

    ImGui::TextUnformatted(slot.name.c_str());
    ImGui::SameLine();

    if (ImGui::SmallButton("Remove"))
    {
      remove_slot(i);
    }
    if (ImGui::IsItemHovered())
    {
      ImGui::SetTooltip("Remove: resets algorithm state and restarts the plotter.");
    }

    ImGui::PopID();
  }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void PlotPanel::init_slot(Slot& slot) const
{
  slot.plotter = slot.factory();
  slot.name = std::string(slot.plotter->name());

  // Call on_init to let the plotter cache robot/sensor IDs before any frame.
  const SimIntrospection introspection(backend_);
  try
  {
    slot.plotter->on_init(introspection);
  }
  catch (const std::exception& e)
  {
    // Record init errors in the error field rather than propagating; the slot
    // will show as errored on the first render.
    slot.error = std::string("on_init: ") + e.what();
  }
  catch (...)
  {
    slot.error = "on_init: Unknown exception.";
  }
}

bool PlotPanel::is_sample_due(const Slot& slot) noexcept
{
  const std::chrono::milliseconds period = slot.plotter->sample_period();

  // A period of zero means "every frame" — always due.
  if (period.count() <= 0)
  {
    return true;
  }

  const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  return (now - slot.last_sample) >= period;
}

}  // namespace stdr::plot
