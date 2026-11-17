"""Nav2 (planner, MPPI controller, behaviors, BT navigator, waypoint follower) + g1d_nav_manager.

    ros2 launch g1d_bringup navigation.launch.py global_frame:=map    # with localization mode:=map
    ros2 launch g1d_bringup navigation.launch.py global_frame:=odom   # no prior map (rolling global costmap)
Velocity commands go to /nav/cmd_vel; g1d_guard forwards them to /cmd_vel_no_limit.
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV2_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator", "waypoint_follower"]


def rewrite(params: dict, global_frame: str) -> dict:
    params["bt_navigator"]["ros__parameters"]["global_frame"] = global_frame
    params["behavior_server"]["ros__parameters"]["global_frame"] = global_frame
    gc = params["global_costmap"]["global_costmap"]["ros__parameters"]
    gc["global_frame"] = global_frame
    if global_frame != "map":                       # no prior map: rolling window, no static layer
        gc["plugins"] = [p for p in gc["plugins"] if p != "static_layer"]
        gc.pop("static_layer", None)
        gc.update({"rolling_window": True, "width": 20, "height": 20, "resolution": 0.1})
    return params


def _nodes(context):
    share = get_package_share_directory("g1d_bringup")
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    gf = arg("global_frame")
    with open(arg("params") or os.path.join(share, "config", "nav2.yaml"), encoding="utf-8") as f:
        params = rewrite(yaml.safe_load(f), gf)
    fd, path = tempfile.mkstemp(prefix="g1d_nav2_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(params, f, sort_keys=False)
    remap = [("cmd_vel", "/nav/cmd_vel")]
    nodes = [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[path], remappings=remap),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[path]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[path], remappings=remap),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[path]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower", name="waypoint_follower",
             output="screen", parameters=[path]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"autostart": True, "node_names": NAV2_NODES, "bond_timeout": 8.0}]),
        Node(package="g1d_nav_manager", executable="nav_manager", name="nav_manager", output="screen",
             parameters=[path, {"global_frame": gf}]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("global_frame", default_value="map", description="map | odom"),
        DeclareLaunchArgument("params", default_value="", description="Nav2 params (default config/nav2.yaml)"),
        OpaqueFunction(function=_nodes),
    ])
