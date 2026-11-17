"""Localization on top of a running LIO chain (lio.launch.py).

    ros2 launch g1d_bringup localization.launch.py mode:=map map:=~/g1d_maps/office [init:=auto|last|none]
    ros2 launch g1d_bringup localization.launch.py mode:=odom          # no prior map: map -> odom = I
map mode: small_gicp_relocalization (map -> odom), localization_monitor (/g1d/localization_status,
/g1d/relocalize), nav2 map_server (/map from map.yaml). Initial pose: RViz "2D Pose Estimate"
(/initialpose), /g1d/relocalize {mode: pose|auto|last}, or init:=...
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    mode = arg("mode")
    if mode == "odom":
        return [
            Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                 arguments=["--frame-id", "map", "--child-frame-id", "odom"]),
            Node(package="g1d_localization", executable="localization_monitor", name="localization_monitor",
                 output="screen", parameters=[{"mode": "odom"}]),
        ]
    if mode != "map":
        raise RuntimeError(f"mode must be map | odom, not {mode!r}")
    map_dir = os.path.expanduser(arg("map"))
    pcd, grid = os.path.join(map_dir, "map.pcd"), os.path.join(map_dir, "map.yaml")
    for f in (pcd, grid):
        if not os.path.isfile(f):
            raise RuntimeError(f"map file missing: {f} (save a map with /g1d/save_map)")
    init_pose = [float(v) for v in arg("init_pose").split(",")] if arg("init_pose") else [0.0, 0.0, 0.0]
    return [
        Node(package="small_gicp_relocalization", executable="small_gicp_relocalization_node",
             name="small_gicp_relocalization", output="screen",
             parameters=[{"num_threads": 4, "num_neighbors": 20, "global_leaf_size": 0.2,
                          "registered_leaf_size": 0.2, "max_dist_sq": 1.0,
                          "map_frame": "map", "odom_frame": "odom",
                          "base_frame": "base_link", "lidar_frame": "base_link",   # map.pcd already in map
                          "robot_base_frame": "base_link", "prior_pcd_file": pcd,
                          # T_map_odom at start = base pose in map (odom origin = base at LIO start)
                          "init_pose": [init_pose[0], init_pose[1], 0.0, 0.0, 0.0, init_pose[2]],
                          "input_cloud_topic": "/g1d/registered_scan"}]),
        Node(package="g1d_localization", executable="localization_monitor", name="localization_monitor",
             output="screen", parameters=[{"mode": "map", "map_dir": map_dir, "startup_init": arg("init")}]),
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": grid, "frame_id": "map"}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_map",
             output="screen", parameters=[{"autostart": True, "node_names": ["map_server"]}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="map", description="map | odom"),
        DeclareLaunchArgument("map", default_value="", description="map directory (map mode)"),
        DeclareLaunchArgument("init", default_value="none", description="none | last | auto"),
        DeclareLaunchArgument("init_pose", default_value="", description="x,y,yaw of base_link in map at start"),
        OpaqueFunction(function=_nodes),
    ])
