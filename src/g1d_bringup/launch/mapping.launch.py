"""Mapping on top of a running LIO chain (lio.launch.py): pose graph (pgo) + map_manager.

    ros2 launch g1d_bringup mapping.launch.py [maps_dir:=~/g1d_maps] [pgo_config:=...]
    ros2 service call /g1d/save_map g1d_msgs/srv/SaveMap "{name: office}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("g1d_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("maps_dir", default_value="~/g1d_maps"),
        DeclareLaunchArgument("pgo_config", default_value=os.path.join(share, "config", "pgo.yaml")),
        Node(package="pgo", executable="pgo_node", name="pgo", output="screen",
             parameters=[{"config_path": LaunchConfiguration("pgo_config")}]),
        Node(package="g1d_mapping", executable="map_manager", name="map_manager", output="screen",
             parameters=[{"maps_dir": LaunchConfiguration("maps_dir")}]),
    ])
