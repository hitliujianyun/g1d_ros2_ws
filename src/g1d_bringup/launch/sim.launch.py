"""Closed-loop simulation without hardware: g1d_sim chassis + the real robot interface + guard.

    ros2 launch g1d_bringup sim.launch.py [world:=corridor] [http_port:=18080] [require_cloud:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interface = os.path.join(get_package_share_directory("g1d_bringup"), "launch",
                             "robot_interface.launch.py")
    port = LaunchConfiguration("http_port")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("http_port", default_value="18080"),
        DeclareLaunchArgument("require_cloud", default_value="false",
                              description="no lidar simulation before M3"),
        DeclareLaunchArgument("guard", default_value="true"),
        Node(package="g1d_sim", executable="chassis_sim", name="chassis_sim", output="screen",
             parameters=[{"world": LaunchConfiguration("world"),
                          "http_port": ParameterValue(port, value_type=int)}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(interface), launch_arguments={
            "chassis_http": ["http://127.0.0.1:", port],
            "require_cloud": LaunchConfiguration("require_cloud"),
            "guard": LaunchConfiguration("guard"),
        }.items()),
    ])
