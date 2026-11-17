"""FAST_LIO -> odom -> base_link (design doc 5.2).

  /Odometry (FAST_LIO: IMU pose in its world frame), TF base_link <- livox_imu (URDF + joints)
  /chassis/odom (restamped wheel odometry)
  -> TF odom -> base_link at `rate_hz` (LIO pose + wheel increment), /odometry, /g1d/lio_health
No TF is published while LIO is missing or older than `max_extrapolation_s`: downstream users
(costmaps, controller, guard) then fail safe instead of driving on a frozen pose.
"""
import math

import numpy as np
import rclpy
from g1d_core.lio import BridgeState, HealthParams, LioHealth, make_tf, matrix_to_quat
from g1d_msgs.msg import LioHealth as LioHealthMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

BEST_EFFORT = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_sec(s) -> float:
    return s.sec + s.nanosec * 1e-9


def yaw_of_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LioBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("lio_bridge", **kwargs)
        d = self.declare_parameter
        self.odom_frame = d("odom_frame", "odom").value
        self.base_frame = d("base_frame", "base_link").value
        self.imu_frame = d("imu_frame", "livox_imu").value
        rate = float(d("rate_hz", 50.0).value)
        hp = HealthParams(
            lio_timeout_s=float(d("lio_timeout_s", 0.5).value),
            max_jump_m=float(d("max_jump_m", 0.3).value),
            max_jump_rad=math.radians(float(d("max_jump_deg", 15.0).value)),
            max_z_drift_m=float(d("max_z_drift_m", 0.3).value),
            max_tilt_rad=math.radians(float(d("max_tilt_deg", 10.0).value)),
            window_s=float(d("consistency_window_s", 2.0).value),
            max_wheel_dpos_m=float(d("max_wheel_dpos_m", 0.3).value),
            max_wheel_dyaw_rad=math.radians(float(d("max_wheel_dyaw_deg", 10.0).value)))
        self.st = BridgeState(health=LioHealth(hp),
                              max_extrapolation_s=float(d("max_extrapolation_s", 0.5).value))
        self.twist = None
        self.tf_missing = False
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.br = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(Odometry, "/odometry", 10)
        self.pub_health = self.create_publisher(LioHealthMsg, "/g1d/lio_health", 10)
        self.pub_anchor = self.create_publisher(TransformStamped, "/g1d/lio_anchor", LATCHED)
        self.create_subscription(Odometry, d("lio_topic", "/Odometry").value, self.on_lio, 20)
        self.create_subscription(Odometry, d("wheel_topic", "/chassis/odom").value, self.on_wheel,
                                 BEST_EFFORT)
        # odom-frame copy of the de-skewed scan for the relocalization (small_gicp)
        self.registered_max_points = int(d("registered_max_points", 20000).value)
        self.pub_registered = self.create_publisher(PointCloud2, "/g1d/registered_scan", 10)
        self.create_subscription(PointCloud2, d("scan_topic", "/g1d/obstacle_cloud").value,
                                 self.on_scan, BEST_EFFORT)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(0.2, self.publish_health)

    def on_wheel(self, m: Odometry) -> None:
        p = m.pose.pose
        self.st.wheel.add(stamp_sec(m.header.stamp), (p.position.x, p.position.y, yaw_of_quat(p.orientation)))
        self.twist = m.twist.twist

    def on_lio(self, m: Odometry) -> None:
        stamp = Time.from_msg(m.header.stamp)
        try:
            t = self.tf.lookup_transform(self.base_frame, self.imu_frame, stamp)
        except TransformException:
            try:
                t = self.tf.lookup_transform(self.base_frame, self.imu_frame, Time())
            except TransformException as e:
                if not self.tf_missing:
                    self.get_logger().warn(f"no TF {self.base_frame} <- {self.imu_frame}: {e}")
                self.tf_missing = True
                return
        self.tf_missing = False
        tr, q = t.transform.translation, t.transform.rotation
        t_base_imu = make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        p = m.pose.pose
        t_lio_imu = make_tf((p.position.x, p.position.y, p.position.z),
                            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        anchored = self.st.anchor.anchored
        self.st.on_lio(stamp_sec(m.header.stamp), t_lio_imu, t_base_imu)
        if not anchored:
            self.get_logger().info("odom anchored at the first LIO pose")
            self.publish_anchor(m.header.stamp, m.header.frame_id)

    def publish_anchor(self, stamp, lio_frame: str) -> None:
        """T_odom_lio (latched): converts FAST_LIO / pose-graph map coordinates to odom."""
        a = self.st.anchor.t_odom_lio
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = lio_frame or "lio_world"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in a[:3, 3])
        q = matrix_to_quat(a[:3, :3])
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.pub_anchor.publish(t)

    def on_scan(self, m: PointCloud2) -> None:
        if self.pub_registered.get_subscription_count() == 0:
            return
        try:
            t = self.tf.lookup_transform(self.odom_frame, m.header.frame_id, Time.from_msg(m.header.stamp))
        except TransformException:
            return
        tr, q = t.transform.translation, t.transform.rotation
        T = make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        xyz = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        if len(xyz) > self.registered_max_points:
            xyz = xyz[:: int(math.ceil(len(xyz) / self.registered_max_points))]
        pts = (xyz.astype(np.float64) @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
        hdr = m.header
        hdr.frame_id = self.odom_frame
        self.pub_registered.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

    def tick(self) -> None:
        now = self.get_clock().now()
        pose = self.st.pose_at(stamp_sec(now.to_msg()))
        if pose is None:
            return
        x, y, yaw = pose
        q = (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = x, y
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = q
        self.br.sendTransform(t)
        o = Odometry()
        o.header = t.header
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = x, y
        o.pose.pose.orientation = r
        if self.twist is not None:
            o.twist.twist = self.twist
        self.pub_odom.publish(o)

    def publish_health(self) -> None:
        now = self.get_clock().now()
        ok, errors = self.st.health.status(stamp_sec(now.to_msg()))
        if self.tf_missing:
            ok, errors = False, sorted(errors + ["no_tf"])
        m = LioHealthMsg()
        m.header.stamp = now.to_msg()
        m.ok = ok
        m.errors = errors
        last = self.st.last
        m.age = -1.0 if last is None else float(stamp_sec(now.to_msg()) - last.t)
        if last is not None:
            m.z, m.roll, m.pitch = float(last.z), float(last.roll), float(last.pitch)
        m.wheel_dpos = float(self.st.health.dpos)
        m.wheel_dyaw = float(self.st.health.dyaw)
        self.pub_health.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = LioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
