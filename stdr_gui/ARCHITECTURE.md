# stdr_gui — Architecture

This document describes the design of `stdr_gui` after the
`feat/phase-9-plot-panel` rewrite. The previous Qt5 / `rqt`
implementation is gone; nothing here applies to it.

## Goals

The rewrite was driven by three constraints:

1. **No ROS in the GUI.** The widget code links against
   `stdr_simulation` (a plain C++ library) and never sees a ROS node,
   topic, or message. ROS integration, when it returns, will be a
   `SimulatorBackend` implementation that lives outside this package.
2. **Backend-pluggable.** `GuiApp` takes a
   `std::unique_ptr<SimulatorBackend>` at construction time. Today the
   only concrete backend is `stdr_standalone::StandaloneBackend`; a
   ROS2 backend is sketched in `simulator_backend.hpp`'s default
   no-op overrides.
3. **User-extensible plots.** Researchers running this simulator
   want bespoke plots (pose error, residuals, SLAM-style maps)
   without forking the GUI. The plotter framework lets them drop a
   `.cpp` into `src/plotters/`, register it with one macro, and have
   it appear in the panel.

## High-level shape

```text
                 ┌────────────────────────────────────┐
                 │            stdr_gui                │
                 │                                    │
   ┌─────────┐   │  ┌──────────┐    ┌───────────────┐ │
   │ GuiApp  │──▶│  │  Panels  │    │ plot::PlotPanel│ │
   │ (GLFW + │   │  │ (Map,    │    │  (drives       │ │
   │  ImGui) │   │  │  Robot,  │    │   Plotter      │ │
   │         │   │  │  Sensor, │    │   plugins)     │ │
   │         │   │  │  Toolbar)│    │                │ │
   │         │   │  └────┬─────┘    └───────┬────────┘ │
   │         │   │       │                  │          │
   │         │   │       └──────┬───────────┘          │
   │         │   │              ▼                      │
   │         │   │      ┌────────────────┐             │
   │         │   │      │ SimulatorBackend│            │
   │         │   │      │   (abstract)    │            │
   │         │   │      └────────┬────────┘            │
   └─────────┘   └───────────────┼─────────────────────┘
                                 │
                       ┌─────────┴──────────┐
                       │                    │
              ┌──────────────────┐  ┌────────────────────┐
              │ StandaloneBackend│  │  Future Ros2Backend │
              │  (stdr_standalone)│ │                     │
              └──────────────────┘  └────────────────────┘
```

`GuiApp` owns the window, ImGui / ImPlot contexts, and one instance of
each panel. Each panel is handed the backend reference plus a
per-frame `SimulationSnapshot` and renders itself.

## SimulatorBackend

`include/stdr_gui/simulator_backend.hpp` defines the boundary between
the GUI and the simulation. Everything that crosses the boundary goes
through one of three channels:

1. **`get_snapshot()`** — returns a `shared_ptr<const
   SimulationSnapshot>`. The backend builds a snapshot on its thread
   and atomically swaps it into a member; the GUI reads it on the
   render thread for one frame. The snapshot owns its data, so the
   GUI never holds a backend lock while drawing.
2. **Mutating commands** — `load_map`, `spawn_robot`, `delete_robot`,
   `start`, `pause`, `reset`, `set_speed`, `set_step_dt`,
   `set_robot_pose`, `set_cmd_vel`. These are the toolbar / context
   menu actions. The backend is responsible for any internal
   synchronisation.
3. **Introspection methods** (`num_robots`, `robot_ids`, `pose`,
   `twist`, `latest_laser`, `latest_sonar`, `footprint`, `sim_time`,
   `collided`, …). Each call acquires the simulation lock for its
   own duration so the value is internally consistent. Cross-call
   consistency is the caller's responsibility — the framework
   provides snapshot-consistent views to plotters via `SimView`,
   described below.

All introspection methods have safe no-op defaults, so a partially
implemented backend (e.g. an early ROS2 port) compiles and runs
without crashing the GUI.

### Sensor event rings

For plotters that need every scan rather than the latest one, the
backend exposes `poll_laser_events` / `poll_sonar_events`. These
drain SPSC ring buffers (one per sensor) that the sim thread fills
after each physics step.

The drain methods return `Drained{Laser,Sonar}Result`, which **owns**
the scan data — the copy from the ring's internal buffer is performed
while the sensor ring lock is held, so callers never have a lifetime
dependency on the ring. This eliminates a subtle data race that would
exist if callers received a span over ring memory.

### Command queue

`push_command` is the asynchronous counterpart to the mutating
methods. Plotters call it (via `SimView::cmd_velocity`,
`teleport`, `pause`, `resume`) without ever taking `sim_mutex_`. The
sim thread drains the queue at the top of each step.

The default implementation is a no-op so backends that don't yet
support command injection compile.

## Threading model

There are two threads:

* **Sim thread** — owned by the backend. Owns `sim_mutex_`, runs
  `engine_.step()`, drains the command queue, fills snapshots and
  sensor rings.
* **GUI thread** — owned by `GuiApp`. Drives GLFW input, ImGui
  rendering, and all panel / plotter callbacks. Reads from the
  backend through `get_snapshot()` (lock-free atomic shared_ptr swap)
  and the introspection methods (short-held locks).

Plotter callbacks (`on_init`, `on_pause`, `on_resume`, `on_sample`,
`on_render`) all run on the GUI thread. The framework guarantees they
are serialised: every enabled plotter gets `on_sample` first, then
every enabled plotter gets `on_render`. There is no cross-thread
synchronisation between `PlotSink` (the write side) and `PlotView`
(the read side) — they are both touched on the same thread, just at
different phases of the frame.

## SimulationSnapshot

`SimulationSnapshot` is a value type containing everything the panels
need to draw a frame: the occupancy grid, the robot list, sensor
data, environmental sources, and timing metadata. It is exchanged
through a `shared_ptr<const SimulationSnapshot>` so the backend can
swap in a new one atomically while the GUI still holds the old one.

The struct exposes a small `robot(name)` helper that returns a
non-owning pointer into the `robots` vector — present so plotters
don't each reinvent the same `std::find_if` loop.

## Panels

Each panel is an independent class that takes the backend reference
and the current snapshot in its `render()` method.

* **`MapPanel`** — uploads the occupancy grid as a GL texture
  (`grid_utils::occupancy_to_rgba`), draws it through ImGui's draw
  list, then layers robots, sensor footprints, environmental sources,
  and a velocity overlay. Owns a `MapTransform` for world ↔ screen
  conversion. Captures the content region origin every frame so
  draw-list calls anchor under the title bar correctly. Drag-to-
  teleport and right-click "Teleport here" route through
  `set_robot_pose`.
* **`RobotInfoPanel`** — robot list with sensor-toggle checkboxes.
  Spawns `SensorWindow` instances on demand and hands ownership of
  them to `GuiApp` via a passed-in vector reference.
* **`SensorWindow`** — one per (robot, sensor-index, sensor-type).
  Self-closes by returning `false` from `render()` when the user
  closes the window.
* **`Toolbar`** — top menu bar plus a control bar (play / pause /
  reset / speed / step-dt) plus a bottom status strip. Accepts a
  `view_menu_extra` callback so `GuiApp` can plumb the plot panel's
  Pause / Remove menu items into "View > Plots" without the toolbar
  knowing about plotters.
* **`FileDialog`** — minimal YAML-only file browser, used to pick
  maps and robot configs.

`MapTransform`, `TeleopController`, and the small `*_utils` headers
all exist as **pure logic** so they can be unit-tested without an
ImGui context. This is a deliberate split: the panel classes are
thin wrappers over reusable helpers, and the helpers carry the
non-trivial behaviour.

## Plotter framework

`include/stdr_gui/plot/` defines the plugin system that the
`plot::PlotPanel` widget drives. There are five public types and
exactly one registration macro.

### Author-facing types

| Type             | Role                                                |
| ---------------- | --------------------------------------------------- |
| `Plotter`        | Abstract base class — subclass and override.        |
| `SimView`        | Snapshot-consistent simulation interface for `on_sample`. |
| `SimIntrospection` | Read-only roster lookup for `on_init`.            |
| `PlotSink`       | Write side of named ring buffers (called from `on_sample`). |
| `PlotView`       | Read side of those buffers (called from `on_render`). |

A complete plotter looks like:

```cpp
class MyPlotter : public stdr::plot::Plotter {
public:
  std::string_view name() const override { return "My Plotter"; }
  void on_sample(stdr::plot::SimView& sim,
                 stdr::plot::PlotSink& out) override {
    out.scalar("speed", sim.sim_time(), sim.twist("robot0").linear_x);
  }
  void on_render(const stdr::plot::PlotView& data) override {
    auto s = data.scalar("speed");
    if (ImPlot::BeginPlot("##speed")) {
      ImPlot::PlotLine("speed", &s[0].t, &s[0].v, s.size(), 0, 0,
                       sizeof(stdr::plot::TimedScalar));
      ImPlot::EndPlot();
    }
  }
};
REGISTER_PLOTTER(MyPlotter);
```

### Lifecycle

```text
   construction
        │
        ▼
   on_init(SimIntrospection)         ← once, before first frame
        │
        ▼
   ┌──────────────────────┐
   │ frame loop:          │
   │   if !paused:        │
   │     on_sample(...)   │   ← gated by sample_period()
   │     on_render(...)   │
   └──────────────────────┘
        │
        ▼
   on_pause / on_resume   ← on user pause/unpause transitions
        │
        ▼
   destruction (or rebuild via Remove)
```

The framework rate-limits `on_sample` per slot: the slot remembers
the wall-clock time of its last successful call and skips invocations
until `sample_period()` has elapsed. The default period is zero
(every frame).

### PlotSink / PlotView

`PlotSink` is a type-erased map of named channels. The three common
data shapes — scalar (`TimedScalar`), 2-D point (`Point2`), and
arbitrary user type (`channel<T>()`) — share a single
`detail::TypedChannel<T>` storage type so the same name dispatches by
type.

Channels grow unbounded (matching ImPlot's expectation of a full
history). The 4096 default is a `reserve()` hint, not a cap.

`PlotView` reads the same sink through const-only accessors. Returned
spans are valid until the next `on_sample` call writes to the same
channel — which can't happen concurrently because the framework
serialises `on_sample` and `on_render`.

### SimView

`SimView` is constructed fresh for each `on_sample` call. Its
introspection methods delegate to the backend (each call holds the
sim lock for its own duration); its history-drain methods accumulate
across the SPSC ring's segment boundaries until they're empty, and
return spans into a buffer that lives only as long as the `SimView`.

Plotters that need readings to outlive the call must copy them.

The `SimView` reference passed to `on_pause` / `on_resume` shares the
slot's live cursors, so calling `new_laser_scans` or
`new_sonar_readings` inside those hooks consumes events that the
next `on_sample` would otherwise have seen. This is intentional —
plotters that want to clear pending events on pause can.

### Plugin registration

`REGISTER_PLOTTER(ClassName)` produces a file-scope `static const
bool ClassName##_registered_` whose initialiser calls
`PlotterRegistry::instance().register_plotter<ClassName>("ClassName")`.

The singleton is a function-local static, so it is constructed on
first use even if the first use comes from a pre-`main` initialiser
in another TU. This sidesteps the static-initialisation-order fiasco
that a namespace-scope global would risk.

The registration trick works only if the linker keeps the TU. Under
`--gc-sections` (the ament default), an object file with no
referenced symbols is silently dropped, taking its registration with
it. The fix has two halves:

1. Plotter sources are GLOBbed into the **static** `stdr_gui_plotters`
   library at the bottom of `CMakeLists.txt`.
2. Consumers link it with
   `$<LINK_LIBRARY:WHOLE_ARCHIVE,stdr_gui_plotters>`, forcing every
   TU into the final link.

`stdr_standalone` does this in its CMake; the
`test_plotter_registry` gtest does the same and exists specifically
to catch a regression in the link configuration. If `REGISTER_PLOTTER`
ever stops working, that test will go red.

User code that ships its own plotters has the same options: drop a
`.cpp` into `src/plotters/` (and rebuild this package) or build a
private static library and link it with `WHOLE_ARCHIVE`.

### PlotPanel slot model

`PlotPanel` builds one `Slot` per registered plotter at construction
time. A slot bundles:

* the `Plotter` instance,
* a `PlotSink`,
* per-sensor ring cursors (laser / sonar), keyed by `"robot/sensor"`,
* the user-visible state (`paused`, `error`, `overran_budget`),
* the wall-clock time of the last `on_sample`,
* a cached display name and the registry factory for Remove.

Each frame `pump_sample()` walks the slots, builds a fresh
`BackendSimView` for each, and invokes `on_sample` if the slot is
enabled and due. `render()` calls `pump_sample()` first, then walks
the slots again to call `on_render` inside ImGui child windows.

#### Pause vs Remove

Pause keeps the plotter and all its accumulated data alive; only the
two callbacks are skipped. The `on_pause` / `on_resume` hooks fire on
the transitions so plotters that issue commands can latch a stop.
The slot's plot history survives a pause, which matters for SLAM-
style plotters that have spent compute building a map.

Remove destroys the plotter, clears the sink and cursors, then
reinstantiates a fresh plotter from the captured factory and calls
`on_init` again. This is the "reset algorithm state" path.

#### Error handling and budgets

Every `on_sample` and `on_render` call is wrapped in `try/catch`.
The exception message lands in `slot.error`; the slot is kept alive
but disabled so the user can still see the message and decide
whether to Remove. A buggy plugin cannot crash the host app.

`on_sample` is also clocked. Calls longer than `kSampleBudget`
(5 ms) flip `slot.overran_budget`, which the panel surfaces as a
yellow indicator with a tooltip. Overrun does not disable the
plotter — it's a hint, not a sentence.

## Testability

The architecture is shaped by what can be tested headlessly. A
running ImGui context requires a window, an OpenGL context, and the
docking branch's globals — none of which a CI build wants to spin
up.

The split is:

* **Pure logic** — `MapTransform`, `TeleopController`,
  `grid_utils`, `pose_utils`, `format_elapsed_time`, `FileDialog`'s
  directory listing, `plot::helpers`, `PlotSink` / `PlotView`,
  `PlotterRegistry` — fully covered by gtests with no ImGui
  dependency.
* **State machine without ImGui** — `PlotPanel::pump_sample`,
  `pause_slot`, `unpause_slot`, `remove_slot`, `slot_status` are all
  callable without a live ImGui context. `test_plot_panel` exercises
  pause / remove / error / budget flags this way.
* **Visual / interaction** — `MapPanel`, `RobotInfoPanel`,
  `SensorWindow`, `Toolbar`, plus the ImPlot half of `PlotPanel`,
  are exercised manually through `stdr_standalone`. There is no
  automated UI test.

This is why `pump_sample()` is exposed publicly and why
`PlotPanel`'s constructor accepts a `PlotterRegistry&` rather than
hard-coding the singleton: tests construct a local registry, register
fixture plotters into it, and drive the panel without touching
process-global state.

## Conventions

* Headers are `.hpp`; sources are `.cpp`. Public APIs use Doxygen
  `/** ... */` comments. Thread-unsafe classes carry a `@warning`.
* Errors use `tl::expected<T, std::string>` (project convention).
  Exceptions appear only in the plot framework, where they are
  caught at the slot boundary so a plugin author's bug doesn't crash
  the app.
* Owning IDs (`RobotId`, `SensorId`) are `std::string`, never
  `string_view` — this is documented at the top of `plotter.hpp` to
  prevent the dangling-view bug a plotter would hit if it cached
  `string_view` over a `vector<string>` returned by value.
* `MapTransform`, `TeleopController`, plot helpers, and similar
  reusable logic live as free functions or thin wrappers so they can
  be unit-tested. Panels stay thin.
