#!/usr/bin/env bash
# Extras for the 5090 host that CI does not need: visualisation and MCAP recording.
set -eo pipefail
SUDO=""
if [ "$(id -u)" != "0" ]; then SUDO="sudo"; fi
$SUDO apt-get update
$SUDO apt-get install -y ros-jazzy-rviz2 ros-jazzy-foxglove-bridge ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-nav2-rviz-plugins
