import json
import math
import os
import time
import urllib.request

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from unitree_hg.msg import LowState  # noqa: E402

from g1d_sim.chassis_sim import ChassisSim  # noqa: E402

WORLDS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "worlds")


@pytest.fixture
def sim():
    rclpy.init()
    ex = SingleThreadedExecutor()
    node = ChassisSim(parameter_overrides=[
        Parameter("world", value=os.path.join(WORLDS, "wall.yaml")),
        Parameter("http_port", value=0), Parameter("scan_noise", value=0.0)])
    h = rclpy.create_node("test_helper")
    ex.add_node(node)
    ex.add_node(h)
    yield ex, node, h
    ex.shutdown()
    node.destroy_node()
    h.destroy_node()
    rclpy.shutdown()


def spin_for(ex, seconds, every=None):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if every:
            every()
        ex.spin_once(timeout_sec=0.01)


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def test_topics_lagged_stamps_and_scan(sim):
    ex, node, h = sim
    got = {}
    h.create_subscription(Odometry, "/slamware_ros_sdk_server_node/odom", lambda m: got.__setitem__("odom", m), 10)
    h.create_subscription(LaserScan, "/slamware_ros_sdk_server_node/scan", lambda m: got.__setitem__("scan", m), 10)
    h.create_subscription(LowState, "/lowstate", lambda m: got.__setitem__("low", m), 10)
    spin_for(ex, 0.6)
    assert {"odom", "scan", "low"} <= set(got)
    now = stamp(h.get_clock().now().to_msg())
    assert now - stamp(got["odom"].header.stamp) == pytest.approx(181.0, abs=0.5)
    r = got["scan"].ranges
    front = r[len(r) // 2]                         # angle 0 is the middle beam (angle_min = -pi)
    assert front == pytest.approx(3.0 - 0.1871, abs=0.02)


def test_drive_timeout_and_http_events(sim):
    ex, node, h = sim
    pub = h.create_publisher(Twist, "/cmd_vel_no_limit", 10)
    gt = []
    h.create_subscription(Odometry, "/sim/ground_truth", gt.append, 50)
    cmd = Twist()
    cmd.linear.x = 0.4
    spin_for(ex, 1.0, every=lambda: pub.publish(cmd))
    assert gt[-1].twist.twist.linear.x == pytest.approx(0.4, abs=0.01)
    t_last = time.monotonic()
    while gt[-1].twist.twist.linear.x > 0.0 and time.monotonic() - t_last < 3.0:
        spin_for(ex, 0.02)
    # chassis timeout 0.5 s + 0.4 / 2.0 s braking
    assert time.monotonic() - t_last == pytest.approx(0.7, abs=0.15)
    assert gt[-1].pose.pose.position.x > 0.3 and not node.collided

    url = f"http://127.0.0.1:{node._http.server_address[1]}"
    h.create_publisher(String, "/sim/trigger_event", 10).publish(String(data="BUMPER_TRIGGERED"))
    spin_for(ex, 0.2)
    with urllib.request.urlopen(url + "/api/platform/v1/events", timeout=2) as resp:
        events = json.loads(resp.read())
    assert events and events[-1]["type"] == "BUMPER_TRIGGERED"
    assert math.isfinite(node.min_gap)
