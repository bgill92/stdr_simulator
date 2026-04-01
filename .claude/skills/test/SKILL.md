---
name: test
description: "Build and run tests in the pixi environment"
context: fork
---

# Test

## Task

Build and run tests using pixi. If a package name is provided as an argument, test only that package. Otherwise test the full workspace.

## Commands

- **Full test**: `pixi run test`
- **Single package**: `pixi run colcon build --symlink-install --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && pixi run colcon test --packages-select <package_name> --event-handlers console_direct+`

Run the appropriate command via Bash. Report the pass/fail summary, including any test failures with details.
