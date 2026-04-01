---
name: dev-build
description: "Set up a new development environment from scratch"
context: fork
---

# Dev Build — Environment Setup

## Task

Set up a complete development environment for this project. Run each step sequentially and report any failures.

## Steps

1. **Verify pixi is installed**: Run `pixi --version`. If not found, tell the user to install pixi first.
2. **Install dependencies**: Run `pixi install`.
3. **Install pre-commit hooks**: Run `pixi run pre-commit install && pixi run pre-commit install --hook-type commit-msg`.
4. **Run initial build**: Run `pixi run build`.
5. **Verify environment**: Run `pixi run colcon list` to confirm packages are found, and `pixi run clang-format --version` to confirm tooling works.

Report the result of each step. If any step fails, stop and report the error.
