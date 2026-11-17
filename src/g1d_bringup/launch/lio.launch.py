"""Mid-360S -> self filter -> FAST_LIO -> lio_bridge (odom -> base_link, /odometry, /g1d/lio_health).

    ros2 launch g1d_bringup lio.launch.py lidar_source:=livox host_ip:=192.168.123.222 [lidar_ip:=...]
    ros2 launch g1d_bringup lio.launch.py lidar_source:=unitree       # /utlidar/* (Unitree service owns the lidar)
    ros2 launch g1d_bringup lio.launch.py lidar_source:=sim world:=room   # with g1d_sim chassis_sim running
Outputs: /livox/lidar (filtered CustomMsg), /g1d/obstacle_cloud (de-skewed scan, livox_imu frame).
Needs the robot description (TF base_link ... livox_imu) to be running.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from g1d_core.sim2d import load_world
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Robot parts hit by the simulated lidar (base_link, x0 y0 z0 x1 y1 z1); self_filter_sim.yaml matches.
SIM_BODY_BOXES = [0.30, -0.20, 0.95, 0.45, 0.20, 1.15,      # "arm / carried object" above the bumper
                  -0.28, -0.26, 0.0, 0.28, 0.26, 0.62]       # chassis


def _nodes(context):
    share = get_package_share_directory("g1d_bringup")
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    source = arg("lidar_source")
    fast_lio_cfg = os.path.join(share, "config", "fast_lio.yaml")
    sf_cfg = arg("self_filter_config") or os.path.join(
        share, "config", "self_filter_sim.yaml" if source == "sim" else "self_filter.yaml")
    imu_topic = "/livox/imu"
    nodes = []
    if source == "livox":
        host_ip = arg("host_ip")
        if not host_ip:
            raise RuntimeError("lidar_source:=livox needs host_ip:=<this computer's IP on 192.168.123.0/24>")
        with open(os.path.join(share, "config", "livox_mid360s.json.in"), encoding="utf-8") as f:
            cfg = f.read().replace("@HOST_IP@", host_ip).replace("@LIDAR_IP@", arg("lidar_ip"))
        fd, path = tempfile.mkstemp(prefix="g1d_livox_", suffix=".json")
        with os.fdopen(fd, "w") as f:
            f.write(cfg)
        nodes.append(Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node", name="livox_lidar_publisher",
            output="screen", remappings=[("/livox/lidar", "/livox/lidar_raw")],
            parameters=[{"xfer_format": 1, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0,
                         "output_data_type": 0, "frame_id": "livox_frame", "user_config_path": path,
                         "cmdline_input_bd_code": "livox0000000001"}]))
    elif source == "unitree":
        imu_topic = arg("utlidar_imu_topic")
        nodes.append(Node(
            package="g1d_lidar", executable="utlidar_adapter", name="utlidar_adapter", output="screen",
            remappings=[("input", arg("utlidar_cloud_topic")), ("output", "/livox/lidar_raw")],
            parameters=[{"frame_id": "livox_frame"}]))
    elif source == "sim":
        world = arg("world")
        if not os.path.isfile(world):
            world = os.path.join(get_package_share_directory("g1d_sim"), "worlds", f"{world}.yaml")
        flat = [float(v) for b in load_world(world) for v in b]
        nodes.append(Node(
            package="g1d_lidar_sim", executable="lidar_sim", name="lidar_sim", output="screen",
            parameters=[{"world_boxes": flat, "body_boxes": SIM_BODY_BOXES,
                         "points_per_second": float(arg("sim_points_per_second"))}]))
    else:
        raise RuntimeError(f"lidar_source must be livox | unitree | sim, not {source!r}")

    nodes += [
        Node(package="g1d_lidar", executable="self_filter", name="self_filter", output="screen",
             parameters=[sf_cfg], remappings=[("input", "/livox/lidar_raw"), ("output", "/livox/lidar")]),
        Node(package="fast_lio", executable="fastlio_mapping", name="fast_lio", output="screen",
             parameters=[fast_lio_cfg, {"common.imu_topic": imu_topic,
                                        "point_filter_num": int(arg("point_filter_num"))}],
             remappings=[("/cloud_registered_body", "/g1d/obstacle_cloud")]),
        Node(package="g1d_localization", executable="lio_bridge", name="lio_bridge", output="screen"),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("lidar_source", default_value="livox", description="livox | unitree | sim"),
        DeclareLaunchArgument("host_ip", default_value="", description="this computer's IP (livox source)"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.123.120", description="Mid-360S IP (to measure)"),
        DeclareLaunchArgument("utlidar_cloud_topic", default_value="/utlidar/cloud_livox_mid360"),
        DeclareLaunchArgument("utlidar_imu_topic", default_value="/utlidar/imu_livox_mid360"),
        DeclareLaunchArgument("self_filter_config", default_value=""),
        DeclareLaunchArgument("point_filter_num", default_value="1"),
        DeclareLaunchArgument("world", default_value="room", description="sim world name or yaml path"),
        DeclareLaunchArgument("sim_points_per_second", default_value="100000"),
        OpaqueFunction(function=_nodes),
    ])
