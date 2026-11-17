"""Phase-0 interface probe: listens to the robot and g1d topics for a few seconds and reports rate,
header-stamp offset to the host clock and a decoded sample; lists every topic with its type, the
publishers of /cmd_vel_no_limit (non-ROS DDS writers included) and the chassis HTTP API.

    ros2 run g1d_tools check_interfaces [--duration 5] [--out logs/interfaces.json]
"""
import argparse
import json
import math
import os
import time

import rclpy
from g1d_core.chassis_http import ChassisHttp
from g1d_core.probe import RateStats
from g1d_msgs.msg import BodyLock, GuardState
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
from unitree_go.msg import WirelessController
from unitree_hg.msg import LowState, MotorState

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

PROBES = [
    ("odom", "/slamware_ros_sdk_server_node/odom", Odometry),
    ("scan", "/slamware_ros_sdk_server_node/scan", LaserScan),
    ("hispeed", "/hispeed_state", Point32),
    ("lowstate", "/lowstate", LowState),
    ("waistpitch", "/waistpitchstate", MotorState),
    ("joy", "/wirelesscontroller", WirelessController),
    ("utlidar_cloud", "/utlidar/cloud_livox_mid360", PointCloud2),
    ("utlidar_imu", "/utlidar/imu_livox_mid360", Imu),
    ("livox_imu", "/livox/imu", Imu),
    ("chassis_odom", "/chassis/odom", Odometry),
    ("chassis_scan", "/chassis/scan", LaserScan),
    ("joint_states", "/joint_states", JointState),
    ("body_lock", "/g1d/body_lock", BodyLock),
    ("guard_state", "/g1d/guard_state", GuardState),
    ("obstacle_cloud", "/g1d/obstacle_cloud", PointCloud2),
]


def summarise(name, m):
    if isinstance(m, Odometry):
        p, t = m.pose.pose.position, m.twist.twist
        return {"x": p.x, "y": p.y, "v": t.linear.x, "w": t.angular.z,
                "frame": m.header.frame_id, "child": m.child_frame_id}
    if isinstance(m, LaserScan):
        r = [v for v in m.ranges if math.isfinite(v) and v > 0.0]
        return {"beams": len(m.ranges), "valid": len(r), "angle_min": m.angle_min,
                "angle_increment": m.angle_increment, "min_range": min(r) if r else None,
                "frame": m.header.frame_id}
    if isinstance(m, Point32):
        return {"height_y": m.y, "speed_z": m.z, "x": m.x}
    if isinstance(m, LowState):
        q = [ms.q for ms in m.motor_state]
        return {"waist_yaw_q12": q[12], "q13": q[13], "q14": q[14],
                "arms_q15_28": [round(v, 4) for v in q[15:29]]}
    if isinstance(m, MotorState):
        return {"q": m.q, "dq": m.dq}
    if isinstance(m, WirelessController):
        return {"lx": m.lx, "ly": m.ly, "rx": m.rx, "ry": m.ry, "keys": m.keys}
    if isinstance(m, PointCloud2):
        return {"width": m.width, "height": m.height, "point_step": m.point_step,
                "fields": [(f.name, f.offset, f.datatype) for f in m.fields], "frame": m.header.frame_id}
    if isinstance(m, Imu):
        a = m.linear_acceleration
        return {"acc": [a.x, a.y, a.z], "acc_norm": math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2),
                "frame": m.header.frame_id}
    if isinstance(m, BodyLock):
        return {"locked": m.locked, "reasons": list(m.reasons), "lift": m.lift_height}
    if isinstance(m, GuardState):
        return {"blocked": m.blocked, "estop": m.estop, "manual": m.manual, "publishing": m.publishing}
    if isinstance(m, JointState):
        return dict(zip(m.name[:6], [round(v, 4) for v in m.position[:6]]))
    return {}


def stamp_sec(s):
    return s.sec + s.nanosec * 1e-9


def collect(node, duration: float) -> dict:
    """Subscribes to PROBES, spins `node` for `duration` seconds, returns the report."""
    stats, samples = {}, {}

    def make_cb(name):
        def cb(m):
            hdr = getattr(m, "header", None)
            now = stamp_sec(node.get_clock().now().to_msg())
            stats[name].add(now, stamp_sec(hdr.stamp) if hdr is not None else None)
            samples[name] = m
        return cb

    for name, topic, typ in PROBES:
        stats[name] = RateStats()
        node.create_subscription(typ, topic, make_cb(name), BEST_EFFORT)
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    topics = {}
    for name, topic, _ in PROBES:
        r = {"topic": topic, **stats[name].report()}
        if name in samples:
            try:
                r["sample"] = summarise(name, samples[name])
            except Exception as e:  # noqa: BLE001
                r["sample_error"] = repr(e)
        topics[name] = r
    writers = [{"node": f"{i.node_namespace.rstrip('/')}/{i.node_name}", "type": i.topic_type,
                "reliability": str(i.qos_profile.reliability)}
               for i in node.get_publishers_info_by_topic("/cmd_vel_no_limit")]
    return {"time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "domain": int(os.environ.get("ROS_DOMAIN_ID", "0")),
            "rmw": os.environ.get("RMW_IMPLEMENTATION", ""),
            "topics": topics,
            "cmd_vel_no_limit_publishers": writers,
            "all_topics": {n: t for n, t in node.get_topic_names_and_types()}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--out", default="logs/interfaces.json")
    ap.add_argument("--chassis-url", default="http://192.168.123.163:1448")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = rclpy.create_node("check_interfaces")
    print(f"listening {args.duration:.1f} s (domain {os.environ.get('ROS_DOMAIN_ID', '0')}) ...")
    report = collect(node, args.duration)
    try:
        ev = ChassisHttp(args.chassis_url, timeout=1.0).events()
        report["chassis_http"] = {"ok": True, "events": len(ev), "last": ev[-3:]}
    except Exception as e:  # noqa: BLE001
        report["chassis_http"] = {"ok": False, "error": repr(e)}
    for r in report["topics"].values():
        flag = "OK " if r["count"] else "-- "
        print(f"{flag}{r['topic']:<40} n={r['count']:<5} rate={r['rate_hz']} Hz "
              f"stamp_offset={r['stamp_offset_s']} s")
    print(f"publishers on /cmd_vel_no_limit: {report['cmd_vel_no_limit_publishers'] or 'none'}")
    print(f"chassis HTTP: {report['chassis_http']}")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, default=str)
    print(f"report: {args.out}")
    node.destroy_node()
    rclpy.try_shutdown()
