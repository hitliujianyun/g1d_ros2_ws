"""Navigation on a prior map, the way the robot is deployed: localization.launch.py mode:=map init:=auto
(small_gicp map -> odom, map_server) + navigation.launch.py global_frame:=map (static layer). The robot
powers up away from the map origin; goals are given in the map frame (= sim world frame).
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
from g1d_core.lio import wrap
from g1d_msgs.msg import LocalizationStatus
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry

from g1d_nav_manager.client import NavClient
from prior_map import make_prior_map

REPORT_DIR = os.environ.get("G1D_TEST_REPORT_DIR", "/tmp/g1d_test_reports")
MAP_DIR = os.path.join(REPORT_DIR, "map_nav_map")
START = (2.0, -1.0, 0.6)


@pytest.mark.launch_test
def generate_test_description():
    world = os.path.join(get_package_share_directory("g1d_sim"), "worlds", "room.yaml")
    make_prior_map(world, MAP_DIR)
    bringup = os.path.join(get_package_share_directory("g1d_bringup"), "launch")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "sim.launch.py")),
                                 launch_arguments={"world": "room", "lio": "true", "http_port": "18098",
                                                   "start_x": str(START[0]), "start_y": str(START[1]),
                                                   "start_yaw": str(START[2])}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "localization.launch.py")),
                                 launch_arguments={"mode": "map", "map": MAP_DIR, "init": "auto"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "navigation.launch.py")),
                                 launch_arguments={"global_frame": "map", "profile": "sim"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def planar(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    return p.x, p.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestMapNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        n = cls.node = rclpy.create_node("map_nav_tester")
        cls.gt, cls.status = [], []
        n.create_subscription(Odometry, "/sim/ground_truth", cls.gt.append, 100)
        n.create_subscription(LocalizationStatus, "/g1d/localization_status", cls.status.append, 10)
        cls.nav = NavClient(n)
        cls.report = {}

    @classmethod
    def tearDownClass(cls):
        os.makedirs(REPORT_DIR, exist_ok=True)
        with open(os.path.join(REPORT_DIR, "map_navigation.json"), "w", encoding="utf-8") as f:
            json.dump(cls.report, f)
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, until=None):
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if until is not None and until():
                return True
        return until() if until is not None else True

    def localized(self):
        if not (self.status and self.gt and self.status[-1].localized):
            return False
        s, (x, y, yaw) = self.status[-1], planar(self.gt[-1])
        return math.hypot(s.pose.x - x, s.pose.y - y) < 0.1 and abs(wrap(s.pose.theta - yaw)) < math.radians(2)

    def run_goal(self, name, **kw):
        t0 = time.monotonic()
        r = self.nav.wait(self.nav.send(**kw), timeout=150.0)
        p = planar(self.gt[-1])
        s = self.status[-1]
        self.report[name] = {"result": r, "gt_final": [round(v, 3) for v in p],
                             "time_s": round(time.monotonic() - t0, 1),
                             "localization": {"localized": s.localized, "reasons": list(s.reasons),
                                              "inlier_ratio": round(s.inlier_ratio, 3),
                                              "last_ok_age": round(s.last_ok_age, 2)}}
        print(f"{name}: {self.report[name]}")
        return r, p

    def test_0_localized_and_ready(self):
        t0 = time.monotonic()
        ok = self.spin(180.0, until=lambda: self.localized() and self.nav.ac.server_is_ready())
        self.report["ready_s"] = round(time.monotonic() - t0, 1)
        self.assertTrue(ok, "not localized / nav_manager not ready")
        self.spin(10.0)                                       # costmaps with the static layer

    def test_1_goal_in_map_frame(self):
        r, p = self.run_goal("around_box", mode="navigate", x=6.0, y=0.0, frame="map", use_yaw=False,
                             timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.assertLess(math.hypot(p[0] - 6.0, p[1]), 0.35)

    def test_2_goal_in_static_obstacle_is_adjusted(self):
        r, p = self.run_goal("goal_in_box", mode="navigate", x=3.2, y=0.0, frame="map", use_yaw=False,
                             timeout_s=120.0)
        self.assertEqual(r["state"], "SUCCEEDED", r)
        self.assertTrue(r["adjusted"], r)
        self.assertGreater(p[0], 3.4 + 0.25)                  # stopped on the robot's (east) side

    def test_3_goal_outside_the_map(self):
        r, _ = self.run_goal("outside_map", mode="navigate", x=11.5, y=0.0, frame="map", use_yaw=False,
                             timeout_s=30.0)
        self.assertEqual(r["state"], "FAILED", r)
        self.assertEqual(r["reason"], "goal_out_of_range", r)
