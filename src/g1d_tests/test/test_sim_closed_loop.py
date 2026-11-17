"""Closed loop: g1d_sim chassis + robot interface + guard (sim.launch.py), driven by a fake
navigation command stream. Checks the 0.6 s stop requirement, e-stop, remote override, TF chain,
SLAMWARE restamping and the body lock.
"""
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
from g1d_msgs.msg import BodyLock, GuardState
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray
from tf2_ros import Buffer, TransformListener

STOP_LIMIT_S = 0.6


@pytest.mark.launch_test
def generate_test_description():
    sim = os.path.join(get_package_share_directory("g1d_bringup"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                 launch_arguments={"world": "empty", "http_port": "18091"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def stamp_sec(s):
    return s.sec + s.nanosec * 1e-9


class TestSimClosedLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        n = cls.node = rclpy.create_node("closed_loop_tester")
        cls.gt, cls.guard, cls.out, cls.lock, cls.scan = [], [], [], [], []
        n.create_subscription(Odometry, "/sim/ground_truth", lambda m: cls.gt.append((time.monotonic(), m)), 100)
        n.create_subscription(GuardState, "/g1d/guard_state", cls.guard.append, 50)
        n.create_subscription(Twist, "/cmd_vel_no_limit", lambda m: cls.out.append((time.monotonic(), m)),
                              qos_profile_sensor_data)
        n.create_subscription(BodyLock, "/g1d/body_lock", cls.lock.append, 10)
        n.create_subscription(LaserScan, "/chassis/scan", cls.scan.append, qos_profile_sensor_data)
        cls.pub_cmd = n.create_publisher(Twist, "/nav/cmd_vel", 10)
        cls.pub_estop = n.create_publisher(Bool, "/g1d/estop", 10)
        cls.pub_joy = n.create_publisher(Float32MultiArray, "/sim/joystick", 10)
        cls.tf = Buffer()
        cls.tf_listener = TransformListener(cls.tf, n)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds, v=None, until=None):
        """Spins; streams /nav/cmd_vel at 20 Hz while v is given. Returns the last send time."""
        t_end = time.monotonic() + seconds
        t_next, t_sent = 0.0, None
        while time.monotonic() < t_end:
            now = time.monotonic()
            if v is not None and now >= t_next:
                cmd = Twist()
                cmd.linear.x = v
                self.pub_cmd.publish(cmd)
                t_sent, t_next = now, now + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if until is not None and until():
                break
        return t_sent

    def speed(self):
        return self.gt[-1][1].twist.twist.linear.x if self.gt else 0.0

    def wait_ready(self):
        ok = lambda: self.guard and 0.0 <= self.guard[-1].scan_age < 0.5  # noqa: E731
        self.spin(40.0, until=ok)
        self.assertTrue(ok(), "guard never saw a fresh chassis scan")

    def drive_to(self, v):
        t = self.spin(15.0, v=v, until=lambda: self.speed() >= v - 0.02)
        self.assertGreaterEqual(self.speed(), v - 0.02, "base did not reach the commanded speed")
        return t

    def test_1_tf_restamp_and_body_lock(self):
        self.wait_ready()
        self.spin(10.0, until=lambda: self.tf.can_transform("base_link", "livox_imu", Time()))
        t = self.tf.lookup_transform("base_link", "livox_imu", Time())
        # livox_frame z 1.3295 + IMU offset rotated by the upside-down mount (roll pi, pitch -0.04)
        self.assertAlmostEqual(t.transform.translation.z, 1.374034, delta=0.005)
        now = stamp_sec(self.node.get_clock().now().to_msg())
        self.assertLess(abs(now - stamp_sec(self.scan[-1].header.stamp)), 0.5)
        self.spin(5.0, until=lambda: self.lock and self.lock[-1].locked)
        self.assertTrue(self.lock[-1].locked, list(self.lock[-1].reasons) if self.lock else "no lock msg")

    def test_2_stop_after_nav_stops(self):
        self.wait_ready()
        t_last = self.drive_to(0.40)
        n_out = len(self.out)
        self.spin(3.0, until=lambda: self.speed() < 0.02)
        t_stop = next(t for t, m in self.gt if t > t_last and m.twist.twist.linear.x < 0.02)
        t_zero = next(t for t, m in self.out[n_out:] if t > t_last and abs(m.linear.x) < 1e-6)
        print(f"guard command zero after {t_zero - t_last:.3f} s, base stopped after {t_stop - t_last:.3f} s")
        self.assertLessEqual(t_zero - t_last, STOP_LIMIT_S)
        self.assertLessEqual(t_stop - t_last, STOP_LIMIT_S + 0.1)   # + simulated chassis lag

    def test_3_estop_latch(self):
        self.wait_ready()
        self.drive_to(0.30)
        self.pub_estop.publish(Bool(data=True))
        t0 = time.monotonic()
        self.spin(2.0, v=0.3, until=lambda: self.speed() < 0.02)
        self.assertLess(time.monotonic() - t0, 0.6)
        self.spin(0.5, v=0.3)
        self.assertTrue(self.guard[-1].estop)
        self.assertLess(self.speed(), 0.02)
        self.pub_estop.publish(Bool(data=False))
        self.drive_to(0.30)
        self.assertFalse(self.guard[-1].estop)
        self.spin(1.5)

    def test_4_remote_override(self):
        self.wait_ready()
        self.drive_to(0.30)
        self.pub_joy.publish(Float32MultiArray(data=[0.0, 0.8, 0.0, 0.0]))
        self.spin(1.5, v=0.3, until=lambda: self.guard[-1].manual)
        self.assertTrue(self.guard[-1].manual)
        self.assertEqual(self.guard[-1].manual_source, "remote")
        self.assertFalse(self.guard[-1].publishing)
        self.pub_joy.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, 0.0]))
        self.spin(1.0)
        self.assertTrue(math.isfinite(self.speed()))
