import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from g1d_msgs.msg import GuardState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from g1d_guard.guard_node import GuardNode  # noqa: E402

BASE = [Parameter("require_cloud", value=False), Parameter("discovery_wait", value=0.3),
        Parameter("chassis_http", value="")]


@pytest.fixture
def ros():
    rclpy.init()
    ex = SingleThreadedExecutor()
    h = rclpy.create_node("test_helper")
    ex.add_node(h)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = "base_link", "chassis_laser"
    tf.transform.translation.x = 0.1871
    tf.transform.rotation.w = 1.0
    StaticTransformBroadcaster(h).sendTransform(tf)
    yield ex, h
    ex.shutdown()
    h.destroy_node()
    rclpy.shutdown()


def wall_scan(h, dist_from_laser):
    """360 x 1 deg scan with a wall perpendicular to x at `dist_from_laser` ahead."""
    m = LaserScan()
    m.header.stamp = h.get_clock().now().to_msg()
    m.header.frame_id = "chassis_laser"
    m.angle_min, m.angle_increment = -math.pi, 2 * math.pi / 360
    m.angle_max = m.angle_min + m.angle_increment * 359
    m.range_min, m.range_max = 0.05, 25.0
    rng = []
    for i in range(360):
        a = m.angle_min + i * m.angle_increment
        c = math.cos(a)
        rng.append(dist_from_laser / c if c > 0.2 else float("inf"))
    m.ranges = rng
    return m


class Harness:
    def __init__(self, ex, h, wall=20.0, params=()):
        self.ex, self.h = ex, h
        self.node = GuardNode(parameter_overrides=BASE + list(params))
        ex.add_node(self.node)
        self.out, self.state = [], []
        h.create_subscription(Twist, "/cmd_vel_no_limit", self.out.append, 50)
        h.create_subscription(GuardState, "/g1d/guard_state", self.state.append, 50)
        self.pub_cmd = h.create_publisher(Twist, "/nav/cmd_vel", 10)
        self.pub_scan = h.create_publisher(LaserScan, "/chassis/scan", 10)
        self.pub_estop = h.create_publisher(Bool, "/g1d/estop", 10)
        self.wall = wall
        self.cmd = None

    def run(self, seconds):
        t_end = time.monotonic() + seconds
        next_in = 0.0
        while time.monotonic() < t_end:
            if time.monotonic() >= next_in:
                next_in = time.monotonic() + 0.05
                self.pub_scan.publish(wall_scan(self.h, self.wall))
                if self.cmd is not None:
                    c = Twist()
                    c.linear.x, c.angular.z = self.cmd
                    self.pub_cmd.publish(c)
            self.ex.spin_once(timeout_sec=0.005)

    def close(self):
        self.node.destroy_node()


def test_silent_without_commands_then_follows_nav(ros):
    hn = Harness(*ros)
    hn.run(0.8)
    assert hn.out == []
    hn.cmd = (0.3, 0.0)
    hn.run(1.5)
    assert hn.out and hn.out[-1].linear.x == pytest.approx(0.3, abs=1e-3)
    assert hn.state[-1].source == "nav" and hn.state[-1].publishing
    hn.close()


def test_front_wall_blocks(ros):
    hn = Harness(*ros, wall=0.6)
    hn.cmd = (0.3, 0.2)
    hn.run(1.0)
    assert all(abs(m.linear.x) < 1e-6 and abs(m.angular.z) < 1e-6 for m in hn.out)
    assert hn.state[-1].blocked == "front"
    hn.close()


def test_estop_latches_and_releases(ros):
    hn = Harness(*ros)
    hn.cmd = (0.3, 0.0)
    hn.run(1.2)
    hn.pub_estop.publish(Bool(data=True))
    hn.run(0.3)
    assert hn.out[-1].linear.x == 0.0 and hn.state[-1].estop
    hn.run(0.5)
    assert hn.state[-1].estop                         # latched while nav keeps sending
    hn.pub_estop.publish(Bool(data=False))
    hn.run(1.2)
    assert hn.out[-1].linear.x > 0.1 and not hn.state[-1].estop
    hn.close()


def test_runtime_params_whitelist(ros):
    hn = Harness(*ros)
    ok = hn.node.set_parameters([Parameter("stop_m", value=0.5)])[0]
    assert ok.successful and hn.node.core.p.stop_m == 0.5
    assert not hn.node.set_parameters([Parameter("rate_hz", value=10.0)])[0].successful
    assert not hn.node.set_parameters([Parameter("slow_m", value=0.3)])[0].successful
    hn.close()


def test_refuses_when_another_writer_exists(ros):
    ex, h = ros
    h.create_publisher(Twist, "/cmd_vel_no_limit", 1)
    with pytest.raises(RuntimeError):
        GuardNode(parameter_overrides=BASE + [Parameter("discovery_wait", value=1.0)])
