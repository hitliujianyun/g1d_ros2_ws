"""LIO closed loop: g1d_sim chassis + simulated Mid-360 / IMU + self filter + FAST_LIO + lio_bridge +
guard (sim.launch.py lio:=true, world room). Drives a 2 m square through the guard and compares the
LIO odometry with the ground truth; checks that self-returns never reach /g1d/obstacle_cloud.
"""
import json
import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from g1d_core.lio import between2, make_tf, wrap
from g1d_msgs.msg import GuardState, LioHealth
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

MAX_POS_ERR = 0.10
MAX_YAW_ERR = math.radians(2.0)
CARRIED_BOX = ((0.32, 0.43), (-0.18, 0.18), (0.97, 1.13))   # sim body box shrunk by 2 cm


REPORT_DIR = os.environ.get("G1D_TEST_REPORT_DIR", "/tmp/g1d_test_reports")
MAPS_DIR = os.path.join(REPORT_DIR, "maps")


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("g1d_bringup"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
            "world": "room", "lio": "true", "mapping": "true", "maps_dir": MAPS_DIR,
            "http_port": "18092"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def surface_distance(points, boxes):
    """Distance of each point to the nearest floor / box surface."""
    d = np.abs(points[:, 2])
    for b in boxes:
        lo, hi = np.array(b[:3]), np.array(b[3:])
        outside = np.maximum(np.maximum(lo - points, points - hi), 0.0)
        dist_out = np.linalg.norm(outside, axis=1)
        inside = np.all((points >= lo) & (points <= hi), axis=1)
        dist_in = np.min(np.minimum(points - lo, hi - points), axis=1)
        d = np.minimum(d, np.where(inside, dist_in, dist_out))
    return d


def write_report(name, report):
    os.makedirs(REPORT_DIR, exist_ok=True)
    with open(os.path.join(REPORT_DIR, name), "w", encoding="utf-8") as f:
        json.dump(report, f)


def planar(m: Odometry):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestLioClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        n = cls.node = rclpy.create_node("lio_loop_tester")
        cls.gt, cls.odom, cls.health, cls.guard = [], [], [], []
        cls.self_hits, cls.clouds = 0, 0
        n.create_subscription(Odometry, "/sim/ground_truth", lambda m: cls.gt.append(m), 100)
        n.create_subscription(Odometry, "/odometry", lambda m: cls.odom.append(m), 100)
        n.create_subscription(LioHealth, "/g1d/lio_health", cls.health.append, 10)
        n.create_subscription(GuardState, "/g1d/guard_state", cls.guard.append, 10)
        n.create_subscription(PointCloud2, "/g1d/obstacle_cloud", cls.on_cloud, qos_profile_sensor_data)
        cls.pub = n.create_publisher(Twist, "/nav/cmd_vel", 10)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, n)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_cloud(cls, m):
        try:
            t = cls.tf.lookup_transform("base_link", m.header.frame_id, Time())
        except Exception:  # noqa: BLE001
            return
        tr, q = t.transform.translation, t.transform.rotation
        T = make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        xyz = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        p = xyz.astype(np.float64) @ T[:3, :3].T + T[:3, 3]
        (x0, x1), (y0, y1), (z0, z1) = CARRIED_BOX
        inside = (p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1) & (p[:, 2] > z0) & (p[:, 2] < z1)
        cls.self_hits += int(inside.sum())
        cls.clouds += 1

    def spin(self, seconds, until=None, cmd=None):
        t_end = time.monotonic() + seconds
        t_next = 0.0
        while time.monotonic() < t_end:
            if cmd is not None and time.monotonic() >= t_next:
                t_next = time.monotonic() + 0.05
                c = Twist()
                c.linear.x, c.angular.z = cmd()
                self.pub.publish(c)
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def gt_pose(self):
        return planar(self.gt[-1])

    def leg(self, dist=None, angle=None, timeout=30.0):
        x0, y0, yaw0 = self.gt_pose()
        if dist is not None:
            done = lambda: math.hypot(self.gt_pose()[0] - x0, self.gt_pose()[1] - y0) >= dist  # noqa: E731
            ok = self.spin(timeout, done, cmd=lambda: (0.3, 0.0))
        else:
            done = lambda: abs(wrap(self.gt_pose()[2] - yaw0)) >= abs(angle) - 0.02  # noqa: E731
            ok = self.spin(timeout, done, cmd=lambda: (0.0, math.copysign(0.4, angle)))
        self.spin(1.5, cmd=lambda: (0.0, 0.0))
        g = self.guard[-1] if self.guard else None
        self.assertTrue(ok, f"leg did not finish (guard blocked={g.blocked if g else '?'})")

    def test_1_square_loop_accuracy_and_self_filter(self):
        ok = self.spin(90.0, until=lambda: self.health and self.health[-1].ok and self.odom and self.gt)
        self.assertTrue(ok, f"LIO not healthy: {list(self.health[-1].errors) if self.health else 'no health'}")
        self.spin(3.0)                                    # settle after IMU initialisation
        gt0, od0 = self.gt_pose(), planar(self.odom[-1])
        t0 = time.monotonic()
        for _ in range(4):
            self.leg(dist=2.0)
            self.leg(angle=-math.pi / 2)
        self.spin(1.0)
        gt1, od1 = self.gt_pose(), planar(self.odom[-1])
        d_gt, d_od = between2(gt0, gt1), between2(od0, od1)
        pos_err = math.hypot(d_gt[0] - d_od[0], d_gt[1] - d_od[1])
        yaw_err = abs(wrap(d_gt[2] - d_od[2]))
        report = {"pos_err_m": round(pos_err, 4), "yaw_err_deg": round(math.degrees(yaw_err), 3),
                  "gt_delta": [round(v, 3) for v in d_gt], "odom_delta": [round(v, 3) for v in d_od],
                  "duration_s": round(time.monotonic() - t0, 1), "self_hits": self.self_hits,
                  "clouds": self.clouds, "lio_ok_at_end": bool(self.health[-1].ok)}
        print(f"LIO square loop: {report}")
        write_report("lio_square_loop.json", report)
        self.assertGreater(self.clouds, 50)
        self.assertEqual(self.self_hits, 0)
        self.assertLess(pos_err, MAX_POS_ERR)
        self.assertLess(yaw_err, MAX_YAW_ERR)
        self.assertTrue(self.health[-1].ok, list(self.health[-1].errors))

    def test_2_save_map_and_quality(self):
        from g1d_core.grid import load_map, occupancy_scores, rasterize_boxes
        from g1d_core.pcd import read_pcd, xyz
        from g1d_core.sim2d import load_world
        from g1d_msgs.srv import SaveMap
        cli = self.node.create_client(SaveMap, "/g1d/save_map")
        self.assertTrue(cli.wait_for_service(timeout_sec=10.0), "no /g1d/save_map")
        fut = cli.call_async(SaveMap.Request(name="ci_room"))
        self.spin(150.0, until=fut.done)
        self.assertTrue(fut.done(), "save_map timed out")
        r = fut.result()
        self.assertTrue(r.success, r.message)
        out = r.path
        for f in ("map.pcd", "map.yaml", "map.pgm", "trajectory.csv", "meta.yaml", "keyframes/poses.csv"):
            self.assertTrue(os.path.isfile(os.path.join(out, f)), f)
        # The map frame is the odom frame of the session; the robot started at the world origin,
        # so map and world coincide up to the LIO start-up error.
        boxes = load_world(os.path.join(get_package_share_directory("g1d_sim"), "worlds", "room.yaml"))
        pts = xyz(read_pcd(os.path.join(out, "map.pcd")))
        d = surface_distance(pts, boxes)
        grid = load_map(os.path.join(out, "map.yaml"))
        precision, recall = occupancy_scores(grid, rasterize_boxes(boxes, grid), tol_cells=2)
        report = {"points": int(len(pts)), "keyframes": int(r.keyframes),
                  "surface_err_median_m": round(float(np.median(d)), 4),
                  "surface_err_p90_m": round(float(np.percentile(d, 90)), 4),
                  "grid_precision": round(precision, 3), "grid_recall_observed_outline": round(recall, 3)}
        print(f"map quality: {report}")
        write_report("map_quality.json", report)
        self.assertGreater(r.keyframes, 10)
        self.assertLess(np.median(d), 0.05)
        self.assertLess(np.percentile(d, 90), 0.15)
        self.assertGreater(precision, 0.8)
