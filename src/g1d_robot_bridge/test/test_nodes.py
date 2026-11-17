import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import Point32  # noqa: E402
from g1d_msgs.msg import BodyLock  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState, LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from unitree_hg.msg import LowState, MotorState  # noqa: E402

from g1d_robot_bridge.body_state import BodyState  # noqa: E402
from g1d_robot_bridge.chassis_events import ChassisEvents  # noqa: E402
from g1d_robot_bridge.chassis_restamp import ChassisRestamp, stamp_sec  # noqa: E402


@pytest.fixture
def ros():
    rclpy.init()
    ex = SingleThreadedExecutor()
    helper = rclpy.create_node("test_helper")
    ex.add_node(helper)
    yield ex, helper
    ex.shutdown()
    helper.destroy_node()
    rclpy.shutdown()


def spin_until(ex, cond, timeout=5.0, every=None):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if every:
            every()
        ex.spin_once(timeout_sec=0.02)
        if cond():
            return True
    return False


def test_restamp_odom_and_scan(ros):
    ex, h = ros
    node = ChassisRestamp(parameter_overrides=[Parameter("log_offset_every", value=0.0)])
    ex.add_node(node)
    got = {}
    h.create_subscription(Odometry, "/chassis/odom", lambda m: got.setdefault("odom", m), 10)
    h.create_subscription(LaserScan, "/chassis/scan", lambda m: got.setdefault("scan", m), 10)
    pub_o = h.create_publisher(Odometry, "/slamware_ros_sdk_server_node/odom", 10)
    pub_s = h.create_publisher(LaserScan, "/slamware_ros_sdk_server_node/scan", 10)

    def send():
        lagged = (h.get_clock().now() - rclpy.duration.Duration(seconds=181.0)).to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = lagged, "odom", "laser"
        s = LaserScan()
        s.header.stamp, s.header.frame_id = lagged, "laser"
        s.ranges = [1.0] * 360
        pub_o.publish(o)
        pub_s.publish(s)

    assert spin_until(ex, lambda: "odom" in got and "scan" in got, every=send)
    now = stamp_sec(h.get_clock().now().to_msg())
    assert abs(now - stamp_sec(got["odom"].header.stamp)) < 0.5
    assert abs(now - stamp_sec(got["scan"].header.stamp)) < 0.5
    assert got["odom"].header.frame_id == "wheel_odom" and got["odom"].child_frame_id == "base_link"
    assert got["scan"].header.frame_id == "chassis_laser"
    assert node.offsets["odom"] == pytest.approx(181.0, abs=0.5)
    node.destroy_node()


def test_body_state_joint_states_and_lock(ros):
    ex, h = ros
    node = BodyState()
    ex.add_node(node)
    js, locks = [], []
    h.create_subscription(JointState, "/joint_states", js.append, 10)
    h.create_subscription(BodyLock, "/g1d/body_lock", locks.append, 10)
    pub_l = h.create_publisher(Point32, "/hispeed_state", 10)
    pub_s = h.create_publisher(LowState, "/lowstate", 10)
    pub_w = h.create_publisher(MotorState, "/waistpitchstate", 10)
    waist_yaw = [0.0]

    def send():
        pub_l.publish(Point32(x=0.0, y=0.2, z=0.0))
        ls = LowState()
        ls.motor_state[12].q = waist_yaw[0]
        pub_s.publish(ls)
        pub_w.publish(MotorState())

    assert not spin_until(ex, lambda: js, timeout=0.3)          # nothing before feedback
    assert spin_until(ex, lambda: js and locks and locks[-1].locked, every=send)
    pos = dict(zip(js[-1].name, js[-1].position))
    assert pos["LZ_mt_Joint"] == pytest.approx(0.1) and pos["LZ_it_Joint"] == pytest.approx(0.1)
    assert pos["torso_Joint"] == 0.0 and len(pos) == 18
    waist_yaw[0] = 0.1
    assert spin_until(ex, lambda: not locks[-1].locked and "waist_yaw" in locks[-1].reasons,
                      every=send)
    node.destroy_node()


@pytest.fixture
def http_events():
    events = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):  # noqa: N802
            body = json.dumps(list(events)).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield events, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def test_chassis_events_reports_new_events(ros, http_events):
    ex, h = ros
    events, url = http_events
    events.append({"type": "BUMPER_TRIGGERED", "timestamp": 1})     # history: never reported
    node = ChassisEvents(parameter_overrides=[Parameter("chassis_http", value=url),
                                              Parameter("period", value=0.05)])
    ex.add_node(node)
    got = []
    h.create_subscription(String, "/g1d/chassis_event", lambda m: got.append(m.data), 10)
    spin_until(ex, lambda: False, timeout=0.4)
    assert got == []
    events.append({"type": "CLIFF_DETECTED", "timestamp": 2})
    assert spin_until(ex, lambda: got == ["CLIFF_DETECTED"])
    node.destroy_node()
