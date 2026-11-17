#!/usr/bin/env bash
# Source on the 5090 host before running anything that talks to the robot:
#   export G1D_NET_IF=enp3s0        # interface on 192.168.123.0/24 (wired) or the WiFi interface
#   export G1D_LINK=wired           # wired | wifi
#   source install/share/g1d_bringup/scripts/g1d_env.sh
# Sets RMW_IMPLEMENTATION, ROS_DOMAIN_ID (Unitree SDK2 uses domain 0) and CYCLONEDDS_URI.

_g1d_share="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${G1D_DOMAIN:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

if [ -z "${G1D_NET_IF:-}" ]; then
  echo "g1d_env: set G1D_NET_IF to the interface facing the robot (ip -br addr)" >&2
  return 1 2>/dev/null || exit 1
fi
_g1d_link="${G1D_LINK:-wired}"
_g1d_xml="$(sed -e "s/G1D_NET_IF/${G1D_NET_IF}/" -e "s/G1D_PEER/${G1D_PEER:-192.168.123.164}/" \
  "${_g1d_share}/config/cyclonedds_${_g1d_link}.xml")"
export CYCLONEDDS_URI="${_g1d_xml}"
echo "g1d_env: RMW=cyclonedds domain=${ROS_DOMAIN_ID} if=${G1D_NET_IF} link=${_g1d_link}"
unset _g1d_share _g1d_link _g1d_xml
