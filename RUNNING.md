# Running STDR — three ways

Quick reference for the original ROS1 sim, the ROS-free standalone, and the ROS2 launch.

---

## 1. Original ROS1 code (`add-docker-support` branch)

The unported original, run via Docker — no system ROS1 install needed, just Docker.

**Prerequisites:** Docker + Docker Compose, an NVIDIA GPU with drivers, and the
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

```bash
git checkout add-docker-support

# compose + Dockerfile need these:
export UID=$(id -u)
export GID=$(id -g)

# Build the dev image:
docker compose -f compose.dev.yml build

# Allow X11 for GUI windows, then start the container:
xhost +local:docker
docker compose -f compose.dev.yml run development
```

You land in a bash shell at `~/ws` (a catkin workspace) with the repo bind-mounted
at `~/ws/src/stdr_simulator`. Inside the container:

```bash
catkin_make
source devel/setup.bash

# Server + GUI + a robot:
roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch
```

Build artifacts cache in `.catkin/` on the host, so they survive container restarts.
The container uses `network_mode: host` and mounts `/tmp/.X11-unix` for the display.

---

## 2. Standalone (no ROS) — robot + live plot

Builds and runs entirely inside pixi. No ROS install needed.

```bash
pixi install
pixi run build
```

Launch with a map + robot preloaded:

```bash
pixi shell
install/stdr_standalone/lib/stdr_standalone/stdr_standalone_exe \
  --map stdr_resources/maps/maze1.yaml \
  --robot stdr_resources/resources/robots/trin_bot.yaml \
  --x 1 --y 2 --theta 3.14
```

Flags: `--map`, `--robot`, `--x`, `--y`, `--theta` (radians). All optional —
`pixi run standalone` opens an empty window and you load via the GUI instead.

In the GUI window:

- **File → Load Map… / Load Robot…** — load if you didn't pass flags.
- **Simulation → Start** — begin stepping.
- **View → Plots → Pose Error** (or **Map Trace**) — toggle a live plot.
  *Pose Error* drives `robot0` in a circle and plots ground-truth vs dead-reckoning.

---

## 3. ROS2 — robot + RViz

Full ROS2 stack, also inside pixi (no system ROS2 needed). RViz is its own launch
file, so use **two terminals**.

Terminal 1 — server + GUI + a robot **with a lidar**. `pandora_robot` carries a
Hokuyo URG-04LX laser; `server_with_map_and_gui_plus_robot.launch.py` spawns it at
(1, 2, 0) on `sparse_obstacles`:

```bash
pixi shell
ros2 launch stdr_launchers server_with_map_and_gui_plus_robot.launch.py
```

Terminal 2 — RViz with the STDR config:

```bash
pixi shell
ros2 launch stdr_launchers rviz.launch.py
```

The shipped config already has a **LaserScan** display bound to `/robot0/laser_0`
(first robot spawned = `robot0`, laser frame = `laser_0`), so the lidar shows up with
no extra setup.

Other ROS2 launch files: `trin_bot_maze1.launch.py` (trin_bot — sonar only, no lidar),
`server_with_map_and_gui.launch.py`, `server_with_map.launch.py`,
`server_no_map.launch.py`.
