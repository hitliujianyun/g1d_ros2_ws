#!/usr/bin/env bash
# Installs the build / test dependencies of the workspace inside a ros:jazzy container (CI) or on
# the 5090 host. Usage: scripts/ci/install_deps.sh   (run from the workspace root, as root or with sudo)
# no `set -u`: /opt/ros/*/setup.bash reads unset variables
set -eo pipefail

SUDO=""
if [ "$(id -u)" != "0" ]; then SUDO="sudo"; fi

export DEBIAN_FRONTEND=noninteractive
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  git cmake build-essential python3-pip python3-numpy python3-yaml python3-pytest \
  python3-colcon-common-extensions python3-rosdep python3-dev libeigen3-dev \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros-py ros-jazzy-tf2-ros ros-jazzy-sensor-msgs-py ros-jazzy-std-srvs \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros ros-jazzy-launch-testing-ament-cmake \
  ros-jazzy-ament-cmake-gtest

# Livox-SDK2 (system install, linked by livox_ros_driver2)
if [ ! -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then
  "$(dirname "$0")/../install_livox_sdk2.sh"
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init
fi
rosdep update --rosdistro jazzy
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# Vendor / external packages are built with BUILD_TESTING=OFF (colcon.meta): skip their linters.
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
  --skip-keys "ament_lint_auto ament_lint_common ament_cmake_clang_format ament_cmake_clang_tidy \
ament_cmake_black ament_cmake_xmllint ament_cmake_copyright"
