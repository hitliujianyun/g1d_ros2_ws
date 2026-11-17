"""Relocalization closed loop: a prior map of the `room` world (built from the world's ground truth),
the robot powered up away from the map origin; localization.launch.py init:=auto must find the pose,
keep tracking while driving, and re-converge from a deliberately wrong /g1d/relocalize pose.
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
from g1d_core.grid import build_grid, save_map
from g1d_core.lio import wrap
from g1d_core.pcd import write_pcd
from g1d_core.sim2d import load_world
from g1d_msgs.msg import LocalizationStatus
from g1d_msgs.srv import Relocalize
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry

REPORT_DIR = os.environ.get("G1D_TEST_REPORT_DIR", "/tmp/g1d_test_reports")
MAP_DIR = os.path.join(REPORT_DIR, "reloc_map")
START = (2.0, -1.0, 0.6)
MAX_POS, MAX_YAW = 0.10, math.radians(2.0)


def make_prior_map(world_yaml, out, step=0.05):
    boxes = load_world(world_yaml)
    pts = [(x, y, 0.0) for x in np.arange(-3.0, 9.0, 0.1) for y in np.arange(-4.4, 4.4, 0.1)]
    for x0, y0, z0, x1, y1, z1 in boxes:
        zs = np.arange(z0, z1 + 1e-9, step)
        for z in zs:
            pts += [(x, y, z) for x in np.arange(x0, x1 + 1e-9, step) for y in (y0, y1)]
            pts += [(x, y, z) for y in np.arange(y0, y1 + 1e-9, step) for x in (x0, x1)]
        pts += [(x, y, z1) for x in np.arange(x0, x1 + 1e-9, step) for y in np.arange(y0, y1 + 1e-9, step)]
    pts = np.array(pts)
    os.makedirs(out, exist_ok=True)
    write_pcd(os.path.join(out, "map.pcd"), pts)
    save_map(build_grid(pts), out, "map")
    with open(os.path.join(out, "trajectory.csv"), "w", encoding="utf-8") as f:   # search seeds
        f.write("x,y,yaw\n")
        for x in np.arange(-2.5, 8.6, 0.5):
            for y in np.arange(-4.0, 4.01, 0.5):
                f.write(f"{x:.2f},{y:.2f},0.0\n")


@pytest.mark.launch_test
def generate_test_description():
    world = os.path.join(get_package_share_directory("g1d_sim"), "worlds", "room.yaml")
    make_prior_map(world, MAP_DIR)
    bringup = os.path.join(get_package_share_directory("g1d_bringup"), "launch")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "sim.launch.py")),
                                 launch_arguments={"world": "room", "lio": "true", "http_port": "18093",
                                                   "start_x": str(START[0]), "start_y": str(START[1]),
                                                   "start_yaw": str(START[2])}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "localization.launch.py")),
                                 launch_arguments={"mode": "map", "map": MAP_DIR, "init": "auto"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def gt_planar(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestRelocalization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        n = cls.node = rclpy.create_node("reloc_tester")
        cls.gt, cls.status = [], []
        n.create_subscription(Odometry, "/sim/ground_truth", cls.gt.append, 100)
        n.create_subscription(LocalizationStatus, "/g1d/localization_status", cls.status.append, 10)
        cls.pub = n.create_publisher(Twist, "/nav/cmd_vel", 10)
        cls.report = {}

    @classmethod
    def tearDownClass(cls):
        os.makedirs(REPORT_DIR, exist_ok=True)
        with open(os.path.join(REPORT_DIR, "relocalization.json"), "w", encoding="utf-8") as f:
            json.dump(cls.report, f)
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, until=None, cmd=None):
        t_end, t_next = time.monotonic() + seconds, 0.0
        while time.monotonic() < t_end:
            if cmd is not None and time.monotonic() >= t_next:
                t_next = time.monotonic() + 0.05
                c = Twist()
                c.linear.x, c.angular.z = cmd
                self.pub.publish(c)
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def error(self):
        """Status pose vs the ground truth at the status stamp (the status is published at 2 Hz)."""
        s = self.status[-1]
        t = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
        gt = min(self.gt[-200:], key=lambda m: abs(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t))
        x, y, yaw = gt_planar(gt)
        return math.hypot(s.pose.x - x, s.pose.y - y), abs(wrap(s.pose.theta - yaw))

    def converged(self):
        return bool(self.status) and self.status[-1].localized and bool(self.gt) and \
            self.error()[0] < MAX_POS and self.error()[1] < MAX_YAW

    def test_1_auto_initialization(self):
        t0 = time.monotonic()
        ok = self.spin(150.0, until=self.converged)
        pos, yaw = self.error() if self.status and self.gt else (math.inf, math.inf)
        self.report["auto"] = {"time_s": round(time.monotonic() - t0, 1), "pos_err_m": round(pos, 4),
                               "yaw_err_deg": round(math.degrees(yaw), 3),
                               "reasons": list(self.status[-1].reasons) if self.status else []}
        print(f"auto relocalization: {self.report['auto']}")
        self.assertTrue(ok, self.report["auto"])

    def test_2_tracking_while_driving(self):
        self.spin(5.0, until=self.converged)
        worst = 0.0
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end:
            self.spin(0.2, cmd=(0.25, 0.0))
            worst = max(worst, self.error()[0])
        self.spin(2.0, cmd=(0.0, 0.0))
        self.spin(5.0, until=self.converged)
        pos, yaw = self.error()
        self.report["tracking"] = {"worst_pos_err_m": round(worst, 4), "final_pos_err_m": round(pos, 4),
                                   "final_yaw_err_deg": round(math.degrees(yaw), 3)}
        print(f"tracking: {self.report['tracking']}")
        self.assertTrue(self.status[-1].localized)
        self.assertLess(worst, 0.2)
        self.assertLess(pos, MAX_POS)
        self.assertLess(yaw, MAX_YAW)

    def test_3_wrong_initial_pose_reconverges(self):
        x, y, yaw = gt_planar(self.gt[-1])
        cli = self.node.create_client(Relocalize, "/g1d/relocalize")
        self.assertTrue(cli.wait_for_service(timeout_sec=10.0))
        fut = cli.call_async(Relocalize.Request(mode="pose", x=x + 0.4, y=y - 0.3, yaw=yaw + math.radians(15)))
        self.spin(10.0, until=fut.done)
        self.assertTrue(fut.result().success, fut.result().message)
        t0 = time.monotonic()
        self.spin(3.0)                                       # let the wrong pose take effect
        ok = self.spin(30.0, until=self.converged)
        pos, yaw_e = self.error()
        self.report["reconverge"] = {"time_s": round(time.monotonic() - t0, 1), "pos_err_m": round(pos, 4),
                                     "yaw_err_deg": round(math.degrees(yaw_e), 3)}
        print(f"re-convergence: {self.report['reconverge']}")
        self.assertTrue(ok, self.report["reconverge"])
