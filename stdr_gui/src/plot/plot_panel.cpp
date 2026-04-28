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
  if (index >= slots_.size())
  {
    return;
  }
  Slot& slot = slots_[index];
  // Idempotent — only fire the hook on the false→true transition.
  if (slot.paused)
  {
    return;
  }

  std::unique_ptr<SimView> view = make_backend_sim_view(backend_, slot.laser_cursors, slot.sonar_cursors);
  try
  {
    slot.plotter->on_pause(*view);
  }
  catch (const std::exception& e)
  {
    slot.error = e.what();
  }
  catch (...)
  {
    slot.error = "Unknown exception in on_pause.";
  }

  slot.paused = true;
}

void PlotPanel::unpause_slot(std::size_t index)
{
  if (index >= slots_.size())
  {
    return;
  }
  Slot& slot = slots_[index];
  // Idempotent — only fire the hook on the true→false transition.
  if (!slot.paused)
  {
    return;
  }

  // Clear the paused flag first so the plotter is considered active before
  // on_resume fires (on_resume may issue commands that the next pump_sample
  // should process).
  slot.paused = false;

  std::unique_ptr<SimView> view = make_backend_sim_view(backend_, slot.laser_cursors, slot.sonar_cursors);
  try
  {
    slot.plotter->on_resume(*view);
  }
  catch (const std::exception& e)
  {
    slot.error = e.what();
  }
  catch (...)
  {
    slot.error = "Unknown exception in on_resume.";
  }
}

void PlotPanel::remove_slot(std::size_t index)
{
  if (index >= slots_.size())
  {
    return;
  }
  Slot& slot = slots_[index];

  // on_pause is intentionally NOT called here: the slot is being torn down
  // entirely, not suspended.  Calling on_pause on a plotter that is about to
  // be destroyed would mislead it into thinking it can resume later.

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
  // Sampling must run for all non-paused slots regardless of which tab is
  // active so plotters accumulate historical data even when not visible.
  pump_sample();

  if (ImGui::BeginTabBar("##plot_tabs"))
  {
    for (std::size_t i = 0; i < slots_.size(); ++i)
    {
      Slot& slot = slots_[i];
      if (slot.paused)
      {
        continue;
      }

      ImGui::PushID(static_cast<int>(i));
      if (ImGui::BeginTabItem(slot.name.c_str()))
      {
        if (!slot.error.empty())
        {
          ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
          ImGui::TextWrapped("[ERROR] %s", slot.error.c_str());
          ImGui::PopStyleColor();
        }
        else
        {
          if (slot.overran_budget)
          {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.0f, 1.0f), "(!) on_sample exceeded 5 ms budget.");
          }
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
        }
        ImGui::EndTabItem();
      }
      ImGui::PopID();
    }
    ImGui::EndTabBar();
  }
}

void PlotPanel::render_menu_items()
{
  for (std::size_t i = 0; i < slots_.size(); ++i)
  {
    Slot& slot = slots_[i];

    // Unique ImGui ID scope per slot so controls do not alias.
    ImGui::PushID(static_cast<int>(i));

    const bool was_paused = slot.paused;
    bool paused = was_paused;
    if (ImGui::Checkbox("##pause", &paused))
    {
      if (paused && !was_paused)
      {
        pause_slot(i);
      }
      else if (!paused && was_paused)
      {
        unpause_slot(i);
      }
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
