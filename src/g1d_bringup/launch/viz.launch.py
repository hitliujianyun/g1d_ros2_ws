"""Visualisation: RViz2 with the g1d config and / or foxglove_bridge (ws://<host>:8765).

    ros2 launch g1d_bringup viz.launch.py [rviz:=true] [foxglove:=true]
Foxglove Studio: open ws://<host>:8765 and import config/foxglove_layout.json.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("g1d_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "g1d.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz"))),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge", output="screen",
             parameters=[{"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
                          "send_buffer_limit": 50000000}],
             condition=IfCondition(LaunchConfiguration("foxglove"))),
    ])
