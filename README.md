# STDR Simulator

A simple, flexible, scalable 2D multi-robot simulator — ROS2 Jazzy port with an optional ROS-free standalone mode.

## Status

ROS2 Jazzy port of [`stdr-simulator-ros-pkg/stdr_simulator`](https://github.com/stdr-simulator-ros-pkg/stdr_simulator).
Environment is managed by [pixi](https://pixi.sh); no system ROS2 install is required.
Implemented in C++20, built with ament\_cmake, and ships an ImGui-based GUI.

## Prerequisites

- [pixi](https://pixi.sh) installed.

That's it. No other system dependencies.

## Build

```bash
pixi install
pixi run build
```

`pixi run build` invokes `colcon build` under the hood with the project's RelWithDebInfo and compile-commands defaults.

## Run — standalone mode (no ROS)

```bash
pixi run standalone
```

Launches a single-process ImGui window; map and robot are loaded via the GUI's File menu.

## Run — ROS2 mode

```bash
pixi shell
ros2 launch stdr_launchers trin_bot_maze1.launch.py
```

This brings up the server, GUI, and a trin\_bot robot in the maze1 map.

### Other launch files

- `server_no_map.launch.py` — Bring up stdr\_server (no map) and a static world→map transform.
- `server_with_map.launch.py` — Bring up stdr\_server loaded with sparse\_obstacles.yaml and a static world→map transform.
- `server_with_map_and_gui.launch.py` — Bring up stdr\_server with sparse\_obstacles.yaml, the GUI, and a static world→map transform.
- `server_with_map_and_gui_plus_robot.launch.py` — Bring up stdr\_server, GUI, world→map TF, and spawn pandora\_robot at (1, 2, 0).
- `trin_bot_maze1.launch.py` — Bring up stdr\_server with maze1, the GUI, and spawn trin\_bot at a configurable pose.
- `rviz.launch.py` — Launch rviz2 with the STDR simulator config.

## Test

```bash
pixi run test
```

**Known issue:** `test_ros2_backend` in `stdr_gui_ros` exits with a SIGABRT during static teardown (heap corruption at process exit). All 25 gtest assertions still PASS; only the process exit is dirty. This is not a blocker.

## Package layout

| Package | Purpose |
|---|---|
| `stdr_msgs` | ROS2 message, service, and action definitions. |
| `stdr_simulation` | Pure C++ simulation library (no ROS deps). |
| `stdr_parser` | YAML config parser + simulation↔message conversion. |
| `stdr_resources` | Maps, robot configs, sensor specs (YAML). |
| `stdr_server` | Central ROS2 server node (map, spawn, world model). |
| `stdr_robot` | ROS2 composable robot node (one process per robot). |
| `stdr_gui` | ImGui-based GUI library (no ROS deps). |
| `stdr_gui_ros` | ROS2 GUI executable; subscribes to topics from the server and robots. |
| `stdr_standalone` | Single-binary entry point (no ROS). |
| `stdr_launchers` | ROS2 Python launch files. |
| `stdr_samples` | Example ROS2 nodes (obstacle avoidance). |
| `stdr_simulator` | Ament metapackage that depends on every package above. |

## Architecture

Both modes share the same `SimulationEngine` + GUI rendering code via a `SimulatorBackend` interface (standalone vs. ROS2).
See [`stdr_gui/ARCHITECTURE.md`](stdr_gui/ARCHITECTURE.md) for the backend interface design.

## License

GPLv3 — see [`LICENSE`](LICENSE).

## Upstream credits

Forked from [`stdr-simulator-ros-pkg/stdr_simulator`](https://github.com/stdr-simulator-ros-pkg/stdr_simulator) (Manos Tsardoulias, Chris Zalidis, Aris Thallas).
