"""Browser UI: http://<host>:8080 (point cloud, maps, costmaps, plans, goals, missions, teleop, e-stop,
initial pose / relocalization, map saving, acceptance tests T1-T7, guard parameters).

    ros2 launch g1d_bringup web.launch.py [port:=8080] [display_frame:=map] [root:=~/g1d_web]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("display_frame", default_value="map"),
        DeclareLaunchArgument("root", default_value="~/g1d_web"),
        Node(package="g1d_web", executable="g1d_web", name="g1d_web", output="screen",
             arguments=["--port", LaunchConfiguration("port"), "--display-frame",
                        LaunchConfiguration("display_frame"), "--root", LaunchConfiguration("root")]),
    ])
