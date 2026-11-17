"""Closed-loop simulation without hardware: g1d_sim chassis + the real robot interface + guard,
optionally with the simulated Mid-360 and the full LIO chain (lio:=true).

    ros2 launch g1d_bringup sim.launch.py [world:=room] [lio:=true] [http_port:=18080]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("g1d_bringup"), "launch")
    port = LaunchConfiguration("http_port")
    lio = LaunchConfiguration("lio")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty"),
        DeclareLaunchArgument("http_port", default_value="18080"),
        DeclareLaunchArgument("lio", default_value="false", description="simulated Mid-360 + FAST_LIO chain"),
        DeclareLaunchArgument("require_cloud", default_value=lio,
                              description="guard needs Mid-360 obstacle points (default: = lio)"),
        DeclareLaunchArgument("guard", default_value="true"),
        DeclareLaunchArgument("point_filter_num", default_value="1"),
        DeclareLaunchArgument("mapping", default_value="false", description="pgo + map_manager (needs lio:=true)"),
        DeclareLaunchArgument("maps_dir", default_value="~/g1d_maps"),
        Node(package="g1d_sim", executable="chassis_sim", name="chassis_sim", output="screen",
             parameters=[{"world": LaunchConfiguration("world"),
                          "http_port": ParameterValue(port, value_type=int)}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "robot_interface.launch.py")),
            launch_arguments={
                "chassis_http": ["http://127.0.0.1:", port],
                "require_cloud": LaunchConfiguration("require_cloud"),
                "guard": LaunchConfiguration("guard"),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "lio.launch.py")),
            condition=IfCondition(lio),
            launch_arguments={"lidar_source": "sim", "world": LaunchConfiguration("world"),
                              "point_filter_num": LaunchConfiguration("point_filter_num")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, "mapping.launch.py")),
            condition=IfCondition(LaunchConfiguration("mapping")),
            launch_arguments={"maps_dir": LaunchConfiguration("maps_dir"),
                              "pgo_config": os.path.join(
                                  get_package_share_directory("g1d_bringup"), "config", "pgo_sim.yaml")}.items()),
    ])
