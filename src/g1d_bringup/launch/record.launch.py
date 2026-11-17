"""rosbag2 recording presets (MCAP).

    ros2 launch g1d_bringup record.launch.py preset:=mapping [out:=~/g1d_bags]
  mapping: raw sensors + body + chassis (re-run LIO / mapping offline)
  nav:     mapping + localization, costmaps, plans, guard, navigation status
  debug:   everything (-a)
"""
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

MAPPING = [
    "/livox/lidar_raw", "/livox/lidar", "/livox/imu", "/utlidar/cloud_livox_mid360", "/utlidar/imu_livox_mid360",
    "/slamware_ros_sdk_server_node/odom", "/slamware_ros_sdk_server_node/scan", "/chassis/odom", "/chassis/scan",
    "/hispeed_state", "/lowstate", "/waistpitchstate", "/joint_states", "/tf", "/tf_static", "/robot_description",
]
NAV = MAPPING + [
    "/Odometry", "/odometry", "/g1d/obstacle_cloud", "/g1d/lio_health", "/g1d/localization_status",
    "/g1d/body_lock", "/g1d/guard_state", "/g1d/chassis_event", "/g1d/estop", "/g1d/teleop_cmd",
    "/nav/cmd_vel", "/cmd_vel_no_limit", "/nav/status", "/nav/goal", "/plan", "/local_plan",
    "/local_costmap/costmap", "/global_costmap/costmap", "/map", "/initialpose", "/goal_pose",
    "/wirelesscontroller",
]


def _record(context):
    preset = LaunchConfiguration("preset").perform(context)
    out_dir = os.path.expanduser(LaunchConfiguration("out").perform(context))
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{preset}_{time.strftime('%Y%m%d_%H%M%S')}")
    cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", out]
    if preset == "debug":
        cmd += ["-a"]
    elif preset in ("mapping", "nav"):
        cmd += MAPPING if preset == "mapping" else NAV
    else:
        raise RuntimeError("preset must be mapping | nav | debug")
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("preset", default_value="nav", description="mapping | nav | debug"),
        DeclareLaunchArgument("out", default_value="~/g1d_bags"),
        OpaqueFunction(function=_record),
    ])
