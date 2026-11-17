"""Navigation closed loop (sim.launch.py lio:=true nav:=true, world room, odom mode):
NavGPT-style goals through g1d_nav_manager -> Nav2 (NavFn + RotationShim/MPPI) -> guard -> sim chassis.
"""
import json
import math
import os
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from g1d_core.lio import wrap
from g1d_msgs.msg import GuardState
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32

from g1d_nav_manager.client import NavClient

REPORT_DIR = os.environ.get("G1D_TEST_REPORT_DIR", "/tmp/g1d_test_reports")


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("g1d_bringup"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim), launch_arguments={
            "world": "room", "lio": "true", "nav": "true", "http_port": "18094"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def planar(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        n = cls.node = rclpy.create_node("nav_tester")
        cls.gt, cls.collided, cls.gap = [], [False], [math.inf]
        n.create_subscription(Odometry, "/sim/ground_truth", cls.gt.append, 100)
        n.create_subscription(Bool, "/sim/collided", lambda m: cls.collided.__setitem__(0, m.data), 10)
        n.create_subscription(Float32, "/sim/min_gap",
                              lambda m: cls.gap.__setitem__(0, min(cls.gap[0], m.data)), 10)
        cls.blocked, cls.n_cmd, cls.guard_log = {}, [0], []
        cls.odom_v, cls.plans = [], []
        n.create_subscription(Odometry, "/odometry", lambda m: cls.odom_v.append(m.twist.twist.linear.x), 50)
        n.create_subscription(Path, "/plan", cls.plans.append, 10)
        n.create_subscription(GuardState, "/g1d/guard_state", cls.on_guard, 10)
        n.create_subscription(Twist, "/nav/cmd_vel", lambda m: cls.n_cmd.__setitem__(0, cls.n_cmd[0] + 1), 50)
        cls.nav = NavClient(n)
        cls.report = {}

    @classmethod
    def on_guard(cls, m):
        key = m.blocked or ("estop" if m.estop else "manual" if m.manual else "")
        if key:
            cls.blocked[key] = cls.blocked.get(key, 0) + 1
        cls.guard_log.append((m.v_cmd, m.v_target, m.v, m.w_cmd, m.front_clear))

    def guard_summary(self):
        g = self.guard_log
        if not g:
            return {}
        mean = lambda i: round(sum(abs(s[i]) for s in g) / len(g), 3)  # noqa: E731
        slowed = sum(1 for s in g if s[0] > 0.05 and s[1] < 0.5 * s[0])
        return {"v_cmd": mean(0), "v_target": mean(1), "v_out": mean(2), "w_cmd": mean(3),
                "slowed_frac": round(slowed / len(g), 2),
                "front_clear_min": round(min(s[4] for s in g), 2)}

    @classmethod
    def tearDownClass(cls):
        os.makedirs(REPORT_DIR, exist_ok=True)
        with open(os.path.join(REPORT_DIR, "navigation.json"), "w", encoding="utf-8") as f:
            json.dump(cls.report, f)
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def gt_pose(self):
        return planar(self.gt[-1])

    def run_goal(self, name, **kw):
        t0 = time.monotonic()
        self.blocked.clear()
        self.guard_log.clear()
        self.odom_v.clear()
        self.plans.clear()
        n0, g0 = self.n_cmd[0], len(self.gt)
        gid = self.nav.send(**kw)
        r = self.nav.wait(gid, timeout=150.0)
        p = self.gt_pose()
        dt = time.monotonic() - t0
        track = [[round(v, 2) for v in planar(m)] for m in self.gt[g0::100]]  # ground truth ~1 Hz
        gt_v = [abs(m.twist.twist.linear.x) for m in self.gt[g0:]]
        mean = lambda xs: round(sum(xs) / len(xs), 3) if xs else None  # noqa: E731
        plan = []
        if self.plans:
            ps = self.plans[0].poses
            plan = [[round(q.pose.position.x, 2), round(q.pose.position.y, 2)] for q in ps[::max(1, len(ps) // 15)]]
        self.report[name] = {"result": r, "gt_final": [round(v, 3) for v in p],
                             "odom_v": mean([abs(v) for v in self.odom_v]), "gt_v": mean(gt_v),
                             "n_plans": len(self.plans), "first_plan": plan,
                             "time_s": round(dt, 1), "min_gap_m": round(self.gap[0], 3),
                             "collided": self.collided[0], "guard_blocked_10hz": dict(self.blocked),
                             "nav_cmd_hz": round((self.n_cmd[0] - n0) / max(dt, 1e-3), 1),
                             "guard": self.guard_summary(), "track": track}
        print(f"{name}: {self.report[name]}")
        return r, p

    def test_0_ready(self):
        """Wait for LIO, TF and Nav2 (the first goal would otherwise fail on start-up)."""
        t_end = time.monotonic() + 120.0
        while time.monotonic() < t_end:
            self.spin(1.0)
            if self.gt and self.nav.ac.server_is_ready():
                break
        self.spin(15.0)                                       # lifecycle activation, costmaps
        self.assertTrue(self.nav.ac.server_is_ready(), "nav_manager action server not ready")

    def test_1_go_around_box(self):
        r, p = self.run_goal("around_box", mode="navigate", x=6.0, y=0.0, frame="odom", use_yaw=False,
                             timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.assertLess(math.hypot(p[0] - 6.0, p[1]), 0.35)
        self.assertFalse(self.collided[0])

    def test_2_goal_inside_box_is_adjusted(self):
        r, p = self.run_goal("goal_in_box", mode="navigate", x=3.2, y=0.0, frame="odom", use_yaw=False,
                             timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.assertTrue(r["adjusted"], r)
        self.assertGreater(p[0], 3.4 + 0.25)                  # stopped on the near (east) side
        self.assertFalse(self.collided[0])

    def test_3_delay_compensation(self):
        r, _ = self.run_goal("to_open_area", mode="navigate", x=1.0, y=-1.0, yaw=0.0, frame="odom",
                             use_yaw=True, timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.spin(1.0)
        stamp = self.node.get_clock().now().to_msg()
        x0, y0, th0 = self.gt_pose()
        r, _ = self.run_goal("rotate", mode="rotate", yaw=math.pi / 2, frame="base", timeout_s=30.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        # NavGPT goal computed from the image taken before the rotation: 1 m ahead of the old heading
        r, p = self.run_goal("delayed_goal", mode="navigate", x=1.0, y=0.0, frame="base_at_stamp",
                             stamp=stamp, use_yaw=False, timeout_s=60.0)
        ex, ey = x0 + math.cos(th0), y0 + math.sin(th0)
        self.report["delayed_goal"]["expected"] = [round(ex, 3), round(ey, 3)]
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.assertLess(math.hypot(p[0] - ex, p[1] - ey), 0.35)

    def test_4_table_overhang_is_respected(self):
        r, _ = self.run_goal("before_table", mode="navigate", x=3.3, y=-1.9, frame="odom", use_yaw=False,
                             timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        r, p = self.run_goal("under_table", mode="navigate", x=3.3, y=-3.2, frame="odom", use_yaw=False,
                             timeout_s=90.0)
        self.assertTrue(r["state"] == "SUCCEEDED" and r["adjusted"] or r["state"] == "FAILED", r)
        self.assertGreater(p[1], -2.8 + 0.25)                 # never drove under the 0.72 m table top
        self.assertFalse(self.collided[0])

    def test_5_unreachable_reports_reason_and_sectors(self):
        r, _ = self.run_goal("unreachable", mode="navigate", x=11.5, y=0.0, frame="odom", use_yaw=False,
                             timeout_s=60.0)
        self.assertEqual(r["state"], "FAILED", r)
        self.assertIn(r["reason"], ("no_path", "blocked", "goal_in_obstacle", "goal_out_of_range", "timeout"))
        self.assertEqual(len(r["free_sectors"]), 7)
        self.assertFalse(self.collided[0])
