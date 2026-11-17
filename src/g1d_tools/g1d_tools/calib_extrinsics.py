"""Mid-360S extrinsic calibration on a flat floor (docs/CALIBRATION.md).

Robot static on a level floor in the navigation posture (waist / lift as during navigation), >= 2 m
of free floor in view, walls / boxes around for the yaw step. Needs the robot description (TF) and
the lidar driver (/livox/lidar or --topic) running; FAST_LIO is not needed.

  1. floor plane in the lidar frame (RANSAC) -> roll / pitch of the lidar and its height
  2. optional yaw: a horizontal lidar slice at the chassis lidar height is rotated onto /chassis/scan
  3. the corrected base_link -> livox_frame is converted into head_link -> livox_frame (current joints)
     and written to an extrinsics.yaml (z: the lidar height above the floor is matched by changing
     head_to_livox z; alternatively fix base_to_agv_z / lift_offset)

    ros2 run g1d_tools calib_extrinsics --yaw-from-scan --write src/g1d_description/config/extrinsics.yaml
"""
import argparse
import math
import time

import numpy as np
import rclpy
import yaml
from g1d_core.calib import estimate_yaw, fit_floor, level_rotation, matrix_to_rpy, rpy_to_matrix
from g1d_core.guard import scan_to_points
from g1d_core.lio import inv, make_tf
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def tf_matrix(t):
    tr, q = t.transform.translation, t.transform.rotation
    return make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/livox/lidar", help="Livox CustomMsg (self filter output or raw)")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--max-range", type=float, default=6.0)
    ap.add_argument("--yaw-from-scan", action="store_true")
    ap.add_argument("--band", type=float, default=0.03, help="slice half height around the chassis lidar (m)")
    ap.add_argument("--write", default="", help="extrinsics.yaml to update (prints only if empty)")
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = rclpy.create_node("calib_extrinsics")
    tf = Buffer()
    TransformListener(tf, node)
    clouds, scans = [], []

    def on_cloud(m):
        if len(clouds) < args.frames:
            clouds.append(np.array([(p.x, p.y, p.z) for p in m.points], dtype=np.float64).reshape(-1, 3))

    node.create_subscription(CustomMsg, args.topic, on_cloud, qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/chassis/scan", lambda m: scans.append(m), qos_profile_sensor_data)
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end and (len(clouds) < args.frames or (args.yaw_from_scan and len(scans) < 5)):
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(clouds) < args.frames:
        raise SystemExit(f"only {len(clouds)} frames on {args.topic}")
    try:
        t_base_lidar = tf_matrix(tf.lookup_transform("base_link", "livox_frame", Time()))
        t_base_head = tf_matrix(tf.lookup_transform("base_link", "head_link", Time()))
        t_base_laser = tf_matrix(tf.lookup_transform("base_link", "chassis_laser", Time()))
    except Exception as e:  # noqa: BLE001
        raise SystemExit(f"TF unavailable (robot description running?): {e}")
    pts = np.vstack(clouds)
    pts = pts[np.linalg.norm(pts, axis=1) < args.max_range]
    r_cfg = t_base_lidar[:3, :3]
    up_hint = r_cfg.T @ np.array([0.0, 0.0, 1.0])
    fit = fit_floor(pts, up_hint)
    if fit is None:
        raise SystemExit("no floor plane found (free floor in view? configured rpy roughly right?)")
    n, h, inliers = fit
    r_new = level_rotation(r_cfg, n)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(n, up_hint))))))
    print(f"floor: {inliers} inliers of {len(pts)}; lidar height measured {h:.4f} m, URDF {t_base_lidar[2, 3]:.4f} m "
          f"(error {h - t_base_lidar[2, 3]:+.4f} m); tilt error {tilt:.2f} deg")
    t_new = np.eye(4)
    t_new[:3, :3] = r_new
    t_new[:3, 3] = [t_base_lidar[0, 3], t_base_lidar[1, 3], h]
    if args.yaw_from_scan:
        s = scans[-1]
        scan_pts = scan_to_points(s.angle_min, s.angle_increment, s.ranges, t_base_laser[:3, 3],
                                  math.atan2(t_base_laser[1, 0], t_base_laser[0, 0]),
                                  max(0.05, s.range_min), s.range_max if s.range_max > 0 else 30.0)
        base = pts @ t_new[:3, :3].T + t_new[:3, 3]
        band = base[np.abs(base[:, 2] - t_base_laser[2, 3]) < args.band][:, :2]
        if len(band) < 50:
            raise SystemExit(f"only {len(band)} lidar points at the chassis lidar height (walls / boxes around?)")
        yaw, resid = estimate_yaw(band, scan_pts[:, :2], t_new[:2, 3])
        print(f"yaw correction vs chassis scan: {math.degrees(yaw):+.2f} deg (mean residual {resid:.3f} m)")
        t_new[:3, :3] = rpy_to_matrix(0.0, 0.0, yaw) @ t_new[:3, :3]
    t_head_lidar = inv(t_base_head) @ t_new
    rpy = matrix_to_rpy(t_head_lidar[:3, :3])
    xyz = t_head_lidar[:3, 3]
    print(f"head_to_livox_xyz: [{xyz[0]:.4f}, {xyz[1]:.4f}, {xyz[2]:.4f}]")
    print(f"head_to_livox_rpy: [{rpy[0]:.5f}, {rpy[1]:.5f}, {rpy[2]:.5f}]")
    if args.write:
        with open(args.write, encoding="utf-8") as f:
            extr = yaml.safe_load(f) or {}
        old = (extr.get("head_to_livox_xyz"), extr.get("head_to_livox_rpy"))
        extr["head_to_livox_xyz"] = [round(float(v), 4) for v in xyz]
        extr["head_to_livox_rpy"] = [round(float(v), 5) for v in rpy]
        with open(args.write, "w", encoding="utf-8") as f:
            f.write("# G1-D 外参（m / rad），calib_extrinsics 写回 " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n")
            yaml.safe_dump(extr, f, sort_keys=False)
        print(f"updated {args.write} (was xyz {old[0]}, rpy {old[1]}); restart the description launch")
    node.destroy_node()
    rclpy.try_shutdown()
