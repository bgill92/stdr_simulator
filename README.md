Fork of the stdr_simulator repo that is intended to be used for simulating algorithms for the Trinity Firefighting competition

stdr_simulator [![Build Status](https://travis-ci.org/stdr-simulator-ros-pkg/stdr_simulator.png?branch=indigo-devel)](https://travis-ci.org/stdr-simulator-ros-pkg/stdr_simulator)
==============

A simple, flexible and scalable 2D multi-robot simulator.

## Docker Development Setup

### Prerequisites

- Docker and Docker Compose
- NVIDIA GPU with drivers installed
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

### 1. Build the Docker container

```bash
# Ensure these env vars are set (needed by compose and Dockerfile)
export UID=$(id -u)
export GID=$(id -g)

# Build the development image
docker compose -f compose.dev.yml build
```

### 2. Run the container

```bash
# Allow X11 forwarding for GUI windows
xhost +local:docker

# Start the development container
docker compose -f compose.dev.yml run development
```

This drops you into a bash shell inside the container at `~/ws` (a catkin workspace), with the repo bind-mounted at `~/ws/src/stdr_simulator`.

### 3. Build and launch the simulator (inside the container)

```bash
# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash

# Launch the simulator
roslaunch stdr_simulator stdr_launchers server_with_map_and_gui_plus_robot.launch
```

### Notes

- Build artifacts are cached in `.catkin/` on the host, so they persist across container restarts.
- The container uses `network_mode: host` and mounts `/tmp/.X11-unix` for GUI display.

## Documentation and Tutorials
[ROS wiki page](http://wiki.ros.org/stdr_simulator)

## Further reference
[External website](http://stdr-simulator-ros-pkg.github.io/)

[Google Group](https://groups.google.com/forum/#!forum/stdr-simulator)
