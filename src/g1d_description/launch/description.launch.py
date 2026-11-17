"""robot_state_publisher for the G1-D (xacro + config/extrinsics.yaml).

    ros2 launch g1d_description description.launch.py [extrinsics:=/path/extrinsics.yaml]
"""
import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from g1d_core.description import xacro_args
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rsp(context):
    share = get_package_share_directory("g1d_description")
    path = LaunchConfiguration("extrinsics").perform(context) or \
        os.path.join(share, "config", "extrinsics.yaml")
    with open(path, encoding="utf-8") as f:
        extr = yaml.safe_load(f) or {}
    doc = xacro.process_file(os.path.join(share, "urdf", "g1d.urdf.xacro"),
                             mappings=xacro_args(extr))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    return [Node(package="robot_state_publisher", executable="robot_state_publisher",
                 name="robot_state_publisher", output="screen",
                 parameters=[{"robot_description": doc.toxml(), "publish_frequency": 50.0,
                              "use_sim_time": use_sim_time}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("extrinsics", default_value="",
                              description="extrinsics yaml (default: package config)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_rsp),
    ])
