#!/usr/bin/env bash
# Build and run the standalone STDR simulator using plain CMake (no colcon).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_DIR="${ROOT_DIR}/build_standalone"
INSTALL_DIR="${BUILD_DIR}/install"
BUILD_TYPE="${1:-RelWithDebInfo}"

cmake_build() {
  local pkg="$1"
  echo "--- Building ${pkg} ---"
  cmake -G Ninja \
    -B "${BUILD_DIR}/${pkg}" \
    -S "${ROOT_DIR}/${pkg}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${INSTALL_DIR}" \
    -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    -DBUILD_TESTING=OFF
  cmake --build "${BUILD_DIR}/${pkg}" --parallel
  cmake --install "${BUILD_DIR}/${pkg}"
}

cmake_build stdr_simulation
cmake_build stdr_gui
cmake_build stdr_standalone

echo "--- Running stdr_standalone_exe ---"
export STDR_RESOURCES_DIR="${ROOT_DIR}/stdr_resources/resources"
exec "${INSTALL_DIR}/lib/stdr_standalone/stdr_standalone_exe"
