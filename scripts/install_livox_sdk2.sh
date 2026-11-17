#!/usr/bin/env bash
# Builds and installs Livox-SDK2 (git submodule third_party/Livox-SDK2) to /usr/local.
# livox_ros_driver2 links liblivox_lidar_sdk_shared.so from /usr/local/lib.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${ROOT}/third_party/Livox-SDK2"
SUDO=""
if [ "$(id -u)" != "0" ]; then SUDO="sudo"; fi
if [ ! -f "${SRC}/CMakeLists.txt" ]; then
  git -C "${ROOT}" submodule update --init third_party/Livox-SDK2
fi
cmake -S "${SRC}" -B "${SRC}/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "${SRC}/build" -j"$(nproc)"
$SUDO cmake --install "${SRC}/build"
$SUDO ldconfig
echo "Livox-SDK2 installed: $(ls /usr/local/lib/liblivox_lidar_sdk_shared.so)"
