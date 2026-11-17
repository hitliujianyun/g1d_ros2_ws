import math
import time

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from g1d_core.kinematics import rpy_to_matrix  # noqa: E402
from g1d_core.lio import inv, matrix_to_quat  # noqa: E402
from g1d_msgs.msg import LioHealth  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rclpy.time import Time  # noqa: E402
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener  # noqa: E402

from g1d_localization.lio_bridge import LioBridge  # noqa: E402

XYZ, RPY = [0.1166, -0.0231, 1.374], [math.pi, -0.04, 0.0]


def tf(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_to_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def odom_msg(stamp, m4, frame="lio_world", child="livox_imu"):
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, frame, child
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(v) for v in m4[:3, 3])
    q = matrix_to_quat(m4[:3, :3])
    o.pose.pose.orientation.x, o.pose.pose.orientation.y = q[0], q[1]
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
    return o


def test_bridge_publishes_base_pose_and_health():
    rclpy.init()
    ex = SingleThreadedExecutor()
    h = rclpy.create_node("test_helper")
    st = TransformStamped()
    st.header.frame_id, st.child_frame_id = "base_link", "livox_imu"
    st.transform.translation.x, st.transform.translation.y, st.transform.translation.z = XYZ
    q = matrix_to_quat(rpy_to_matrix(*RPY))
    r = st.transform.rotation
    r.x, r.y, r.z, r.w = q
    StaticTransformBroadcaster(h).sendTransform(st)
    node = LioBridge()
    ex.add_node(h)
    ex.add_node(node)
    buf = Buffer()
    TransformListener(buf, h)
    health = []
    h.create_subscription(LioHealth, "/g1d/lio_health", health.append, 10)
    pub_lio = h.create_publisher(Odometry, "/Odometry", 10)
    pub_wheel = h.create_publisher(Odometry, "/chassis/odom", qos_profile_sensor_data)
    t_base_imu = tf(XYZ, RPY)
    lio_T_odom = tf([0.2, 0.1, 0.0], [0.0, 0.0, 0.7]) @ inv(t_base_imu)   # arbitrary LIO world
    try:
        t0 = time.monotonic()
        x = 0.0
        while time.monotonic() - t0 < 3.0:
            now = h.get_clock().now()
            dt = time.monotonic() - t0
            x = 0.2 * dt                                   # base drives at 0.2 m/s along odom x
            w = odom_msg(now.to_msg(), tf([x, 0, 0], [0, 0, 0]), "wheel_odom", "base_link")
            pub_wheel.publish(w)
            pub_lio.publish(odom_msg(now.to_msg(), lio_T_odom @ tf([x, 0, 0], [0, 0, 0]) @ t_base_imu))
            t_end = time.monotonic() + 0.1
            while time.monotonic() < t_end:
                ex.spin_once(timeout_sec=0.01)
        t = buf.lookup_transform("odom", "base_link", Time())
        # odom origin = base at the first LIO pose (x ~ 0), so the TF shows the distance driven
        assert t.transform.translation.x == pytest.approx(x, abs=0.05)
        assert abs(t.transform.translation.y) < 1e-3
        assert health and health[-1].ok, list(health[-1].errors) if health else "no health"
        assert abs(health[-1].z) < 1e-3
    finally:
        node.destroy_node()
        h.destroy_node()
        rclpy.shutdown()
