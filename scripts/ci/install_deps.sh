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
  git python3-pip python3-numpy python3-yaml python3-pytest python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-tf2-ros-py ros-jazzy-tf2-ros ros-jazzy-sensor-msgs-py ros-jazzy-std-srvs \
  ros-jazzy-launch-testing ros-jazzy-launch-testing-ros ros-jazzy-launch-testing-ament-cmake

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  $SUDO rosdep init
fi
rosdep update --rosdistro jazzy
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
  --skip-keys "ament_lint_auto ament_lint_common"
