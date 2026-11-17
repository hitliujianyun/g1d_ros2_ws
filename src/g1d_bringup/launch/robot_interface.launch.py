"""Robot interface: description (TF), SLAMWARE restamp, body joint states + lock, chassis events, guard.

    ros2 launch g1d_bringup robot_interface.launch.py [guard:=true] [require_cloud:=true]
        [chassis_http:=http://192.168.123.163:1448] [extrinsics:=/path/extrinsics.yaml]
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
    bridge_cfg = os.path.join(get_package_share_directory("g1d_robot_bridge"), "config", "robot_bridge.yaml")
    guard_cfg = os.path.join(get_package_share_directory("g1d_guard"), "config", "guard.yaml")
    description = os.path.join(get_package_share_directory("g1d_description"), "launch",
                               "description.launch.py")
    chassis_http = LaunchConfiguration("chassis_http")
    return LaunchDescription([
        DeclareLaunchArgument("guard", default_value="true"),
        DeclareLaunchArgument("require_cloud", default_value="true",
                              description="guard also needs fresh Mid-360S obstacle points"),
        DeclareLaunchArgument("chassis_http", default_value="http://192.168.123.163:1448"),
        DeclareLaunchArgument("extrinsics", default_value=""),
        DeclareLaunchArgument("other_writers", default_value="refuse"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description),
                                 launch_arguments={"extrinsics": LaunchConfiguration("extrinsics")}.items()),
        Node(package="g1d_robot_bridge", executable="chassis_restamp", name="chassis_restamp",
             parameters=[bridge_cfg], output="screen"),
        Node(package="g1d_robot_bridge", executable="body_state", name="body_state",
             parameters=[bridge_cfg], output="screen"),
        Node(package="g1d_robot_bridge", executable="chassis_events", name="chassis_events",
             parameters=[bridge_cfg, {"chassis_http": chassis_http}], output="screen"),
        Node(package="g1d_guard", executable="guard", name="g1d_guard", output="screen",
             condition=IfCondition(LaunchConfiguration("guard")),
             parameters=[guard_cfg, {
                 "chassis_http": chassis_http,
                 "require_cloud": ParameterValue(LaunchConfiguration("require_cloud"), value_type=bool),
                 "other_writers": LaunchConfiguration("other_writers")}]),
    ])
