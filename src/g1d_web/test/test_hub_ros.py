"""ROS side of g1d_web: the hub collects guard / status / clouds and publishes goals and e-stops."""
import json
import math
import os
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from g1d_msgs.msg import GuardState  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from g1d_web.hub import Hub  # noqa: E402

os.environ["ROS_DOMAIN_ID"] = "57"


def wait(cond, timeout=5.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if cond():
            return True
        time.sleep(0.02)
    return False


def test_hub_inputs_and_outputs():
    rclpy.init()
    hub = Hub(display_frame="map")
    hub.start()
    h = rclpy.create_node("hub_tester")
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(h)
    try:
        tfs = []
        for parent, child, x in (("map", "odom", 1.0), ("odom", "base_link", 0.5), ("base_link", "chassis_laser", 0.2)):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x = x
            t.transform.rotation.w = 1.0
            tfs.append(t)
        StaticTransformBroadcaster(h).sendTransform(tfs)
        goals, estops = [], []
        h.create_subscription(String, "/nav/goal", lambda m: goals.append(json.loads(m.data)), 10)
        h.create_subscription(Bool, "/g1d/estop", lambda m: estops.append(m.data), 10)
        pub_g = h.create_publisher(GuardState, "/g1d/guard_state", 10)
        pub_s = h.create_publisher(String, "/nav/status", 10)
        pub_scan = h.create_publisher(LaserScan, "/chassis/scan", qos_profile_sensor_data)

        def pump():
            g = GuardState(blocked="front", front_clear=math.inf)
            pub_g.publish(g)
            pub_s.publish(String(data=json.dumps({"goal_id": "x", "state": "NAVIGATING"})))
            s = LaserScan()
            s.header.frame_id = "chassis_laser"
            s.angle_min, s.angle_increment, s.range_min, s.range_max = 0.0, 0.1, 0.05, 10.0
            s.ranges = [1.0] * 10
            pub_scan.publish(s)
            ex.spin_once(timeout_sec=0.05)
            return hub.json.get("guard", {}).get("blocked") == "front" and "chassis" in hub.clouds \
                and hub.pose_nav is not None

        assert wait(pump, 10.0)
        assert hub.json["guard"]["front_clear"] is None
        assert hub.json["status"]["state"] == "NAVIGATING"
        assert hub.pose_nav["x"] == pytest.approx(1.5)              # map <- base through odom
        pts = hub.clouds["chassis"][1]
        assert pts[0, 0] == pytest.approx(1.2, abs=1e-5)            # laser at x 0.2 + range 1.0
        _, t, valid = hub.T_odom_lio()
        assert valid and t[0] == pytest.approx(1.0)
        hub.send_goal({"id": "g1", "mode": "navigate", "x": 1.0, "y": 0.0, "frame": "map"})
        hub.send_estop(True)
        assert wait(lambda: (ex.spin_once(timeout_sec=0.05), goals and estops)[1], 5.0)
        assert goals[-1]["id"] == "g1" and estops[-1] is True
    finally:
        h.destroy_node()
        hub.node.destroy_node()
        rclpy.shutdown()
