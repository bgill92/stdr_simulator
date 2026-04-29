#pragma once

/** @file PlotPanel — the dockable ImGui/ImPlot widget that drives all registered
 *  plotter plugins each frame.
 *
 *  ## Responsibilities
 *
 *  - Own one `Slot` per registered plotter: plotter instance, plot sink, cursor
 *    maps, and per-slot control state (paused, removed, error, budget overrun).
 *  - Each frame: build a `SimView`, drive `on_sample` (rate-limited by
 *    `sample_period()`) then `on_render` for every non-paused, non-removed slot.
 *  - Wrap every `on_sample` and `on_render` call in try/catch so a buggy plotter
 *    cannot crash the app.  Record the exception message and mark the slot with
 *    an error indicator.
 *  - Enforce a 5 ms sampling budget per plotter.  Slots that consistently exceed
 *    it are flagged with a budget-overrun indicator (yellow dot + tooltip) in
 *    the panel so users know which plugin is costing frames.
 *  - Expose `render_menu_items()` so the main app can wire up a "View > Plots"
 *    submenu with Pause / Remove controls for each slot.
 *
 *  ## Remove vs Pause
 *
 *  Pause: keeps the plotter instance and its accumulated plot data alive;
 *  `on_sample` and `on_render` are skipped until unpaused.  Preferred UX for
 *  "I want to stop watching this temporarily" — a SLAM plotter keeps its map.
 *
 *  Remove: destroys the instance, clears the sink and all per-slot state, then
 *  reinstantiates a fresh plotter from the registry factory.  Used when the user
 *  explicitly wants to reset algorithm state.
 *
 *  @warning NOT thread-safe.  Must be created and called on the GUI (main) thread. */

#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/simulator_backend.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace stdr::plot
{

/** @brief Read-only view of one plotter slot's runtime state.
 *
 *  Returned by `PlotPanel::slot_status()` so tests and external code can
 *  inspect state without breaking encapsulation of the internal `Slot`. */
struct PlotterStatus
{
  /** Display name of the plotter (from `Plotter::name()`). */
  std::string name;

  /** True if the slot is paused: `on_sample` and `on_render` are skipped. */
  bool paused{ false };

  /** True if the slot has been removed and is pending reinstantiation.
   *  After removal the slot is immediately rebuilt from the registry factory,
   *  so this flag is transient — it is true only while the rebuild is in
   *  flight (currently always false after `remove()` returns). */
  bool removed{ false };

  /** Non-empty if the last `on_sample` or `on_render` threw an exception.
   *  The slot is kept alive but disabled on exception so users can still
   *  read the error message and decide whether to remove the plotter. */
  std::string error;

  /** True if the last `on_sample` call exceeded the 5 ms budget. */
  bool overran_budget{ false };
};

/** @brief Dockable panel that drives all registered plotter plugins each frame.
 *
 *  Construct once after the backend is ready.  Call `render()` inside the
 *  plot panel's ImGui window each frame, and `render_menu_items()` from the
 *  "View > Plots" submenu in the main menu bar.
 *
 *  @warning NOT thread-safe.  Must be created and called on the GUI thread. */
class PlotPanel
{
public:
  /** @brief Construct and populate slots from the global `PlotterRegistry`.
   *
   *  Each registered plotter is instantiated once and assigned a slot.
   *  `on_init` is called immediately so plotters can cache robot/sensor IDs
   *  before the first frame.
   *
   *  @param backend     Non-owning reference to the backend.  Must outlive this panel.
   *  @param registry    Registry to query for plotter factories.  Defaults to the
   *                     global singleton so call sites do not need to pass it, but
   *                     tests can supply a local registry to avoid global state. */
  explicit PlotPanel(stdr_gui::SimulatorBackend& backend, PlotterRegistry& registry = PlotterRegistry::instance());

  /** @brief Drive `on_sample` (rate-limited) for each enabled slot.
   *
   *  Builds a `SimView`, checks `sample_period()`, wraps each `on_sample`
   *  in try/catch, and records error/overrun in the slot status.  Does NOT
   *  call any ImGui functions — safe to call headlessly in tests.
   *
   *  Called automatically by `render()` before the ImGui child loop; exposed
   *  publicly so tests can drive the sample phase without a live ImGui context. */
  void pump_sample();

  /** @brief Drive `on_sample` (rate-limited) and `on_render` for each enabled slot.
   *
   *  Call once per frame, inside an `ImGui::Begin` / `ImGui::End` block.
   *  Calls `pump_sample()` first, then iterates slots to emit ImGui children.
   *  Skips slots that are paused or whose error flag is set. */
  void render();

  /** @brief Render "View > Plots" menu items — one entry per registered slot.
   *
   *  Call from inside an `ImGui::BeginMenu("Plots")` block.  Each entry renders:
   *    [checkbox Pause] SlotName [Remove]
   *
   *  Pause toggles `slot.paused`; Remove reinitialises the slot from the factory. */
  void render_menu_items();

  /** @brief Read-only view of every slot's current state.
   *
   *  Returns one `PlotterStatus` per slot in registration order.  Used by
   *  tests to verify pause / error / budget-overrun behaviour without needing
   *  a live ImGui context.
   *
   *  The vector is small (one entry per registered plotter) so returning by
   *  value is cheaper than the complexity of a mutable cache member. */
  [[nodiscard]] std::vector<PlotterStatus> slot_status() const;

  /** @brief Pause the slot at @p index (0-based).
   *
   *  No-op if @p index is out of range. */
  void pause_slot(std::size_t index);

  /** @brief Unpause the slot at @p index (0-based).
   *
   *  No-op if @p index is out of range. */
  void unpause_slot(std::size_t index);

  /** @brief Remove and reinstantiate the slot at @p index.
   *
   *  Clears the plotter, sink, cursors, and all error/budget state, then
   *  creates a fresh plotter from the registry factory and calls `on_init`. */
  void remove_slot(std::size_t index);

private:
  /** Maximum wall-clock duration allowed for a single `on_sample` call.
   *  Calls that exceed this threshold set `slot.overran_budget = true`. */
  static constexpr std::chrono::milliseconds kSampleBudget{ 5 };

  /** @brief Internal state for one registered plotter. */
  struct Slot
  {
    /** Plotter instance — owns the algorithm state. */
    std::unique_ptr<Plotter> plotter;

    /** Owned data sink written to by `on_sample` and read by `on_render`. */
    PlotSink sink;

    /** Per-sensor cursors for the SPSC laser event ring (one per "robot/sensor" key). */
    std::unordered_map<std::string, std::uint64_t> laser_cursors;

    /** Per-sensor cursors for the SPSC sonar event ring. */
    std::unordered_map<std::string, std::uint64_t> sonar_cursors;

    /** When true, `on_sample` and `on_render` are both skipped. */
    bool paused{ false };

    /** Non-empty when the last `on_sample` or `on_render` threw an exception.
     *  The slot is kept alive (not reinstantiated) so the user can see the
     *  message and decide whether to remove/reset the plotter. */
    std::string error;

    /** Wall-clock time of the last successful `on_sample` call.
     *  Used to gate calls by `sample_period()`. */
    std::chrono::steady_clock::time_point last_sample{};

    /** Set when the previous `on_sample` call exceeded `kSampleBudget`. */
    bool overran_budget{ false };

    /** Cached display name (from `plotter->name()`) so it can be shown even
     *  after the plotter has been replaced during Remove. */
    std::string name;

    /** Factory function captured from the registry at slot-creation time.
     *  Called by `remove_slot()` to produce a fresh plotter instance. */
    std::function<std::unique_ptr<Plotter>()> factory;
  };

  /** @brief Populate `slot.plotter`, `slot.name`, call `on_init`, and clear all state. */
  void init_slot(Slot& slot) const;

  /** @brief Check whether the slot is due for `on_sample` given its `sample_period()`. */
  [[nodiscard]] static bool is_sample_due(const Slot& slot) noexcept;

  stdr_gui::SimulatorBackend& backend_;

  /** All slots in registration order.  Size == `registry.size()`. */
  std::vector<Slot> slots_;
};

}  // namespace stdr::plot
