# stdr_gui

ImGui-based GUI library for the STDR Simulator. ROS-free; ships as an `ament_cmake` package that any application — ROS or otherwise — can link against to embed the simulator's interactive front-end.

## Status

This package was rewritten from the ground up on the `feat/phase-9-plot-panel` branch. The previous Qt5 / `rqt`-based GUI was removed and replaced with a Dear ImGui + ImPlot stack and a headless `SimulatorBackend` abstraction. The legacy widgets, `.ui` files, and icon resources are gone; do not look for them.

## What's in the package

* `GuiApp` — owns the GLFW window and the ImGui / ImPlot lifecycle. Built from a `SimulatorBackend` injected at construction time.
* `SimulatorBackend` — abstract interface to the simulation. Concrete implementations live outside this package (`stdr_standalone` ships `StandaloneBackend`; a ROS2 backend is planned but not yet wired in).
* Panels — dockable ImGui windows the app composes into a layout:
  * `MapPanel` — occupancy grid render with pan / zoom, robot visualisation, sensor overlays, drag-to-teleport, right-click context menu.
  * `RobotInfoPanel` — robot list, per-robot sensor toggles, spawns `SensorWindow` instances on demand.
  * `SensorWindow` — single-sensor visualiser (laser, sonar, RFID, CO₂, thermal, sound).
  * `Toolbar` — top menu bar plus play / pause / reset / speed / step-dt control bar and bottom status strip.
  * `FileDialog` — minimal file browser for loading maps and robot YAML configs.
  * `plot::PlotPanel` — dockable panel that hosts user-authored plot plugins (see "Plotters" below).
* Helpers — `MapTransform` (world ↔ screen), `TeleopController` (key-state → `Twist2D`), `grid_utils::occupancy_to_rgba`, `pose_utils::transform_to_world`, plus `plot::helpers` for plotter authors.

## Plotters

`stdr_gui_plotters` is a separate static library inside this package that holds the built-in plot plugins. Two ship today:

* `PoseErrorPlotter` (`src/plotters/pose_error_plotter.cpp`) — drives robot 0 in a circle and plots ground-truth vs dead-reckoning pose.
* `MapTracePlotter` (`src/plotters/map_trace_plotter.cpp`) — renders the map texture, the robot footprint, and an age-coloured breadcrumb trail.

Authoring a new plotter is three steps: subclass `stdr::plot::Plotter`, override `on_sample` / `on_render` / `name`, and call `REGISTER_PLOTTER(MyPlotter)` at file scope. See [`ARCHITECTURE.md`](ARCHITECTURE.md#plotter-framework) for the full contract, and `pose_error_plotter.cpp` for a complete worked example.

The library must be linked with `WHOLE_ARCHIVE` so the static registration initialisers survive `--gc-sections` — see [`ARCHITECTURE.md`](ARCHITECTURE.md#plugin-registration).

## Building

This package is part of the wider STDR colcon workspace and is built through the project's pixi environment.

```sh
pixi run build         # full workspace
pixi run test          # build + run all gtests
```

`pixi run build` invokes colcon under the hood; to build only this package:

```sh
pixi run colcon build --packages-select stdr_gui
```

### Dependencies

Declared in `package.xml` and resolved through the pixi environment — no manual setup needed:

* `stdr_simulation` — the headless physics / sensor library.
* `imgui` — Dear ImGui (docking branch).
* `implot` — ImPlot (used by the plot framework).
* `libgl-devel` — system OpenGL.
* `tl_expected` — `tl::expected<T, std::string>` for recoverable errors (project convention).

The package exports its headers and both library targets (`stdr_gui`, `stdr_gui_plotters`) via `ament_export_targets`, so downstream packages simply call `find_package(stdr_gui REQUIRED)`.

## Linking against `stdr_gui` from another package

```cmake
find_package(stdr_gui REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE
  stdr_gui::stdr_gui
  $<LINK_LIBRARY:WHOLE_ARCHIVE,stdr_gui::stdr_gui_plotters>)
```

The `WHOLE_ARCHIVE` wrap on `stdr_gui_plotters` is required to keep `REGISTER_PLOTTER` static initialisers alive. If you ship your own plotter library, link it the same way.

## Running

This package is a library — it has no executable of its own. The quickest way to drive it is the standalone front-end:

```sh
pixi run build-standalone
pixi run run-standalone -- --map <map.yaml> --robot <robot.yaml>
```

`stdr_standalone` constructs a `StandaloneBackend`, hands it to `GuiApp`, and runs the render loop.

## Testing

Tests live in `test/` and run as gtests via `ament_add_gtest`:

| Test binary                  | Covers                                      |
| ---------------------------- | ------------------------------------------- |
| `test_map_transform`         | World ↔ screen math, zoom / pan invariants. |
| `test_grid_utils`            | Occupancy → RGBA conversion.                |
| `test_pose_utils`            | 2D rigid transforms.                        |
| `test_format_elapsed_time`   | HH:MM:SS.cs formatting.                     |
| `test_file_dialog`           | YAML directory listing logic.               |
| `test_teleop_controller`     | Key-state → `Twist2D`, kinematic gating.    |
| `test_plot_helpers`          | Plotter helper free functions.              |
| `test_plot_sink_view`        | `PlotSink` / `PlotView` channel semantics.  |
| `test_plotter_registry`      | `REGISTER_PLOTTER` survives the link.       |
| `test_plot_panel`            | Slot lifecycle, pause / remove, error / budget flags. |

The plot-panel and plotter-registry tests run headless — no ImGui context is constructed, so `pump_sample()` is exercised directly. See [`ARCHITECTURE.md`](ARCHITECTURE.md#testability) for the testing contract these tests rely on.

## Layout (header layout mirrors `src/`)

```text
include/stdr_gui/
  gui_app.hpp                   GuiApp — window + main loop.
  simulator_backend.hpp         SimulatorBackend abstract interface
                                + SimulationSnapshot.
  map_transform.hpp             World ↔ screen coordinate math.
  grid_utils.hpp                occupancy_to_rgba.
  pose_utils.hpp                2D rigid transforms.
  teleop_controller.hpp         Pure-logic teleop.
  panels/                       MapPanel, RobotInfoPanel,
                                SensorWindow, Toolbar, FileDialog.
  plot/                         Plotter plugin framework
                                (see ARCHITECTURE.md).
src/
  gui_app.cpp
  panels/
  plot/                         Framework (PlotPanel, registry,
                                SimView, helpers).
  plotters/                     Built-in plugin sources, compiled
                                into stdr_gui_plotters.
test/
```

## Further reading

* [`ARCHITECTURE.md`](ARCHITECTURE.md) — design rationale, threading model, plotter framework, registration trick.
* `.claude/plans/PORT_PLAN.md` — context for the broader Qt → ROS2 + ImGui port this package landed in.
