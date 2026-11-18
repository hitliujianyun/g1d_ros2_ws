"""g1d_guard: the ONLY writer of /cmd_vel_no_limit (design doc 5.6).

  /nav/cmd_vel, /g1d/teleop_cmd (Twist)       -> GuardCore -> /cmd_vel_no_limit (Twist, 30 Hz)
  /chassis/scan (LaserScan), /g1d/obstacle_cloud (PointCloud2) -> reflex layer (TF to base_link)
  /g1d/estop (Bool: true latches, false releases), /g1d/chassis_event (String), /wirelesscontroller
  /g1d/guard_state (GuardState, 10 Hz)
Refuses to start when /cmd_vel_no_limit already has a publisher (other_writers: refuse).
"""
import math
import time
from dataclasses import fields

import numpy as np
import rclpy
from g1d_core.chassis_http import ChassisHttp
from g1d_core.guard import TUNABLE, GuardCore, GuardParams, scan_to_points
from g1d_core.shutdown import context_closing
from g1d_msgs.msg import GuardState
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_go.msg import WirelessController

CMD_OUT = "/cmd_vel_no_limit"
BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
RELIABLE_OUT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def other_writers(node: Node, topic: str):
    return [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in node.get_publishers_info_by_topic(topic)]


class GuardNode(Node):
    def __init__(self, **kwargs):
        super().__init__("g1d_guard", **kwargs)
        defaults = GuardParams()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(GuardParams)}
        self.p = GuardParams.from_dict(values)
        d = self.declare_parameter
        self.base_frame = d("base_frame", "base_link").value
        self.cloud_z = (float(d("cloud_z_min", 0.08).value), float(d("cloud_z_max", 1.85).value))
        self.cloud_range = float(d("cloud_range", 3.0).value)
        self.cloud_max_points = int(d("cloud_max_points", 6000).value)
        policy = d("other_writers", "refuse").value
        wait = float(d("discovery_wait", 1.5).value)
        chassis_http = d("chassis_http", "").value
        abort = bool(d("abort_slamware_on_start", True).value)

        time.sleep(wait)                           # let DDS discovery find existing writers
        others = other_writers(self, CMD_OUT)
        if others:
            msg = f"other publishers on {CMD_OUT}: {others}"
            if policy == "refuse":
                raise RuntimeError(msg + " (stop them, or set other_writers:=warn)")
            self.get_logger().error(msg)

        self.core = GuardCore(self.p, time.monotonic())
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.scan_tf = {}                           # frame -> (x, y, z, yaw), static
        self.pub = self.create_publisher(Twist, CMD_OUT, RELIABLE_OUT)
        self.pub_state = self.create_publisher(GuardState, "/g1d/guard_state", 10)
        self.create_subscription(Twist, "/nav/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Twist, "/g1d/teleop_cmd", self.on_teleop, 10)
        self.create_subscription(LaserScan, "/chassis/scan", self.on_scan, BEST_EFFORT)
        self.create_subscription(PointCloud2, "/g1d/obstacle_cloud", self.on_cloud, BEST_EFFORT)
        self.create_subscription(Bool, "/g1d/estop", self.on_estop, 10)
        self.create_subscription(String, "/g1d/chassis_event", self.on_event, 10)
        self.create_subscription(WirelessController, "/wirelesscontroller", self.on_joy, BEST_EFFORT)
        self.add_on_set_parameters_callback(self.on_params)
        if chassis_http and abort:
            ChassisHttp(chassis_http, timeout=1.0).abort_current_action()
        self.dt = 1.0 / self.p.rate_hz
        self.k = 0
        self.state_every = max(1, int(round(self.p.rate_hz / 10.0)))
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"guard running at {self.p.rate_hz:.0f} Hz, single writer of {CMD_OUT}")

    # ---- inputs -----------------------------------------------------------------------------
    def on_cmd(self, m: Twist) -> None:
        self.core.on_cmd(m.linear.x, m.angular.z, time.monotonic())

    def on_teleop(self, m: Twist) -> None:
        self.core.on_teleop(m.linear.x, m.angular.z, time.monotonic())

    def on_estop(self, m: Bool) -> None:
        self.core.on_estop(bool(m.data), "topic")
        self.get_logger().warn(f"estop {'ON (latched)' if m.data else 'released'}")

    def on_event(self, m: String) -> None:
        self.core.on_event(m.data)

    def on_joy(self, m: WirelessController) -> None:
        self.core.on_joystick(m.lx, m.ly, m.rx, m.ry, time.monotonic())

    def on_scan(self, m: LaserScan) -> None:
        pose = self.scan_tf.get(m.header.frame_id)
        if pose is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, m.header.frame_id, Time())
            except TransformException:
                return                              # stays stale -> guard stops the base
            tr, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            pose = self.scan_tf[m.header.frame_id] = (tr.x, tr.y, tr.z, yaw)
        rmax = float(m.range_max) if m.range_max > 0 else 30.0
        pts = scan_to_points(m.angle_min, m.angle_increment, m.ranges, pose[:3], pose[3],
                             max(0.05, float(m.range_min)), rmax)
        self.core.on_scan_points(pts, time.monotonic())

    def on_cloud(self, m: PointCloud2) -> None:
        try:
            t = self.tf.lookup_transform(self.base_frame, m.header.frame_id, Time.from_msg(m.header.stamp))
        except TransformException:
            try:
                t = self.tf.lookup_transform(self.base_frame, m.header.frame_id, Time())
            except TransformException:
                return
        xyz = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        if xyz.shape[0] > self.cloud_max_points:
            xyz = xyz[:: int(math.ceil(xyz.shape[0] / self.cloud_max_points))]
        tr = t.transform.translation
        pts = xyz.astype(np.float64) @ quat_matrix(t.transform.rotation).T + [tr.x, tr.y, tr.z]
        keep = ((pts[:, 2] >= self.cloud_z[0]) & (pts[:, 2] <= self.cloud_z[1])
                & (np.hypot(pts[:, 0], pts[:, 1]) <= self.cloud_range))
        self.core.on_cloud_points(pts[keep].astype(np.float32), time.monotonic())

    def on_params(self, params):
        for prm in params:
            if prm.name not in TUNABLE:
                return SetParametersResult(successful=False, reason=f"{prm.name} is read-only at runtime")
            err = self.core.set_param(prm.name, prm.value)
            if err:
                return SetParametersResult(successful=False, reason=err)
            self.get_logger().info(f"param {prm.name} = {prm.value}")
        return SetParametersResult(successful=True)

    # ---- loop -------------------------------------------------------------------------------
    def tick(self) -> None:
        now = time.monotonic()
        out = self.core.step(now, self.dt)
        if out.publish:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = float(out.v), float(out.w)
            self.pub.publish(cmd)
        self.k += 1
        if self.k % self.state_every == 0:
            self.pub_state.publish(self.state_msg(now))

    def state_msg(self, now: float) -> GuardState:
        s = self.core.state(now)
        m = GuardState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        for k in ("v", "w", "v_cmd", "w_cmd", "v_target", "w_target"):
            setattr(m, k, float(s[k]))
        for k in ("blocked", "estop_reason", "manual_source", "source"):
            setattr(m, k, str(s[k]))
        for k in ("estop", "manual", "publishing", "rotate_ok"):
            setattr(m, k, bool(s[k]))
        m.front_clear = math.inf if s["front_clear"] is None else float(s["front_clear"])
        m.rear_clear = math.inf if s["rear_clear"] is None else float(s["rear_clear"])
        m.cmd_age = -1.0 if s["cmd_age"] is None else float(s["cmd_age"])
        m.scan_age = -1.0 if s["scan_age"] is None else float(s["scan_age"])
        m.cloud_age = -1.0 if s["cloud_age"] is None else float(s["cloud_age"])
        return m

    def stop_base(self) -> None:
        """On shutdown: if the base was moving, write zeros a few times before going silent."""
        if self.core.v != 0.0 or self.core.w != 0.0:
            for _ in range(3):
                self.pub.publish(Twist())
                time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GuardNode()
    except RuntimeError as e:
        rclpy.logging.get_logger("g1d_guard").fatal(str(e))
        rclpy.try_shutdown()
        raise SystemExit(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TypeError):
        if not context_closing(rclpy.ok):   # taken while the context shuts down: harmless
            raise
    finally:
        node.stop_base()
        node.destroy_node()
        rclpy.try_shutdown()
