---
name: build
description: "Build the code in the pixi environment"
context: fork
---

# Build

## Task

Build the workspace using pixi. If a package name is provided as an argument, build only that package. Otherwise build the full workspace.

## Commands

- **Full build**: `pixi run build`
- **Single package**: `pixi run colcon build --symlink-install --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

Run the appropriate command via Bash. Report success or failure, including any compiler errors or warnings.
