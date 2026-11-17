"""Simulated G1-D chassis + body on the same ROS 2 / DDS topics as the real robot (no hardware).

  /cmd_vel_no_limit (Twist)                 -> diff-drive base (command timeout like the chassis)
  /slamware_ros_sdk_server_node/odom, /scan <- stamps lag the host clock by `stamp_lag` (~181 s)
  /hispeed_state, /lowstate, /waistpitchstate, /wirelesscontroller <- body / remote controller
  chassis HTTP API (events, abort action) on `http_port` (disabled if < 0)
Test hooks: /sim/ground_truth (Odometry, frame "world"), /sim/min_gap (Float32, chassis footprint
vs boxes overlapping 0..`robot_height`), /sim/collided (Bool), /sim/trigger_event (String),
/sim/body (JointState: lift_y, waist_pitch, waist_yaw, arm_0..arm_13), /sim/joystick
(Float32MultiArray [lx, ly, rx, ry]).
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from g1d_core.sim2d import (SLAMWARE_STAMP_LAG_S, DiffDrive, DiffDriveParams, boxes_at_height,
                            boxes_in_band, footprint_gap, load_world, raycast_boxes)
from geometry_msgs.msg import Point32, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from unitree_go.msg import WirelessController
from unitree_hg.msg import LowState, MotorState

BEST_EFFORT = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw_quat(yaw: float):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def resolve_world(name: str) -> str:
    if os.path.isfile(name):
        return name
    return os.path.join(get_package_share_directory("g1d_sim"), "worlds", f"{name}.yaml")


class ChassisSim(Node):
    def __init__(self, **kwargs):
        super().__init__("chassis_sim", **kwargs)
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.boxes = load_world(resolve_world(p("world", "empty")))
        start = [float(v) for v in p("start", [0.0, 0.0, 0.0])]
        self.rate = float(p("rate_hz", 50.0))
        self.odom_every = max(1, round(self.rate / float(p("odom_hz", 20.0))))
        self.scan_every = max(1, round(self.rate / float(p("scan_hz", 10.0))))
        self.joy_every = max(1, round(self.rate / float(p("joy_hz", 20.0))))
        self.dd = DiffDrive(DiffDriveParams(
            cmd_timeout=float(p("cmd_timeout", 0.5)), max_acc=float(p("max_acc", 1.0)),
            max_dec=float(p("max_dec", 2.0)), max_ang_acc=float(p("max_ang_acc", 2.0)),
            v_limit=float(p("v_limit", 1.0)), w_limit=float(p("w_limit", 1.0))), start)
        self.laser_xyz = [float(v) for v in p("laser_xyz", [0.1871, -0.016137, 0.20085])]
        self.scan_beams = int(p("scan_beams", 360))
        self.scan_range = float(p("scan_range", 25.0))
        self.scan_noise = float(p("scan_noise", 0.005))
        self.stamp_lag = Duration(seconds=float(p("stamp_lag", SLAMWARE_STAMP_LAG_S)))
        self.robot_height = float(p("robot_height", 1.75))
        self.body = {"lift_y": float(p("lift_y", 0.0)), "waist_pitch": float(p("waist_pitch", 0.0)),
                     "waist_yaw": float(p("waist_yaw", 0.0))}
        self.arm_q = [float(v) for v in p("arm_q", [0.0] * 14)]
        self.joy = [0.0, 0.0, 0.0, 0.0]
        self.rng = np.random.default_rng(int(p("seed", 3)))
        self.scan_boxes = boxes_at_height(self.boxes, self.laser_xyz[2])
        self.body_boxes = boxes_in_band(self.boxes, 0.0, self.robot_height)
        self.min_gap = math.inf
        self.collided = False
        self.events = []
        self.aborted_actions = 0
        self.lock = threading.Lock()
        self.k = 0
        self._last_tick = None

        self.pub_odom = self.create_publisher(Odometry, "/slamware_ros_sdk_server_node/odom", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/slamware_ros_sdk_server_node/scan", 10)
        self.pub_lift = self.create_publisher(Point32, "/hispeed_state", 10)
        self.pub_low = self.create_publisher(LowState, "/lowstate", 10)
        self.pub_waist = self.create_publisher(MotorState, "/waistpitchstate", 10)
        self.pub_joy = self.create_publisher(WirelessController, "/wirelesscontroller", 10)
        self.pub_gt = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.pub_gap = self.create_publisher(Float32, "/sim/min_gap", 10)
        self.pub_col = self.create_publisher(Bool, "/sim/collided", 10)
        self.create_subscription(Twist, "/cmd_vel_no_limit", self.on_cmd, BEST_EFFORT)
        self.create_subscription(String, "/sim/trigger_event", lambda m: self.trigger_event(m.data), 10)
        self.create_subscription(JointState, "/sim/body", self.on_body, 10)
        self.create_subscription(Float32MultiArray, "/sim/joystick", self.on_joystick, 10)
        self._http = None
        port = int(p("http_port", -1))
        if port >= 0:
            self._start_http(port)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"chassis sim: {len(self.boxes)} boxes, start {start}")

    # ---- inputs -----------------------------------------------------------------------------
    def on_cmd(self, m: Twist) -> None:
        with self.lock:
            self.dd.on_cmd(m.linear.x, m.angular.z, time.monotonic())

    def on_body(self, m: JointState) -> None:
        for name, q in zip(m.name, m.position):
            if name in self.body:
                self.body[name] = float(q)
            elif name.startswith("arm_") and name[4:].isdigit() and int(name[4:]) < 14:
                self.arm_q[int(name[4:])] = float(q)

    def on_joystick(self, m: Float32MultiArray) -> None:
        self.joy = [float(v) for v in list(m.data)[:4]] + [0.0] * max(0, 4 - len(m.data))

    def trigger_event(self, event_type: str) -> None:
        with self.lock:
            self.events.append({"type": event_type, "timestamp": int(time.time() * 1000),
                                "id": len(self.events)})

    # ---- HTTP -------------------------------------------------------------------------------
    def _start_http(self, port: int) -> None:
        sim = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _reply(self, obj):
                body = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802
                if self.path.startswith("/api/platform/v1/events"):
                    with sim.lock:
                        self._reply(list(sim.events))
                else:
                    self.send_error(404)

            def do_DELETE(self):  # noqa: N802
                if self.path.startswith("/api/core/motion/v1/actions/:current"):
                    sim.aborted_actions += 1
                    self._reply({})
                else:
                    self.send_error(404)

        self._http = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        threading.Thread(target=self._http.serve_forever, name="sim_http", daemon=True).start()
        self.get_logger().info(f"chassis HTTP on http://127.0.0.1:{self._http.server_address[1]}")

    # ---- loop -------------------------------------------------------------------------------
    def tick(self) -> None:
        self.k += 1
        now_mono = time.monotonic()
        # integrate over the real elapsed time so ground-truth poses match their stamps under load
        dt = 1.0 / self.rate if self._last_tick is None else min(max(now_mono - self._last_tick, 0.0), 0.1)
        self._last_tick = now_mono
        with self.lock:
            self.dd.step(now_mono, dt)
            x, y, th = self.dd.pose
            v, w = self.dd.v, self.dd.w
        gap = footprint_gap((x, y, th), self.body_boxes) if self.body_boxes else math.inf
        self.min_gap = min(self.min_gap, gap)
        self.collided = self.collided or gap <= 0.0
        now = self.get_clock().now()
        self.pub_gt.publish(self.odom_msg(now, x, y, th, v, w, "world"))
        self.pub_gap.publish(Float32(data=float(gap)))
        self.pub_col.publish(Bool(data=self.collided))
        self.publish_body()
        if self.k % self.odom_every == 0:
            self.pub_odom.publish(self.odom_msg(now - self.stamp_lag, x, y, th, v, w, "odom"))
            self.pub_lift.publish(Point32(x=0.0, y=self.body["lift_y"], z=0.0))
        if self.k % self.scan_every == 0:
            self.pub_scan.publish(self.scan_msg(now - self.stamp_lag, x, y, th))
        if self.k % self.joy_every == 0:
            j = self.joy
            self.pub_joy.publish(WirelessController(lx=j[0], ly=j[1], rx=j[2], ry=j[3], keys=0))

    def publish_body(self) -> None:
        ls = LowState()
        ls.motor_state[12].q = self.body["waist_yaw"]
        for i, q in enumerate(self.arm_q):
            ls.motor_state[15 + i].q = q
        self.pub_low.publish(ls)
        ms = MotorState()
        ms.q = self.body["waist_pitch"]
        self.pub_waist.publish(ms)

    @staticmethod
    def odom_msg(stamp, x, y, th, v, w, frame):
        m = Odometry()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = frame
        m.child_frame_id = "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        q = yaw_quat(th)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        m.twist.twist.linear.x, m.twist.twist.angular.z = v, w
        return m

    def scan_msg(self, stamp, x, y, th):
        lx, ly, _ = self.laser_xyz
        ox, oy = x + math.cos(th) * lx - math.sin(th) * ly, y + math.sin(th) * lx + math.cos(th) * ly
        inc = 2.0 * math.pi / self.scan_beams
        a_min = -math.pi
        r = raycast_boxes((ox, oy), th + a_min + inc * np.arange(self.scan_beams), self.scan_boxes,
                          self.scan_range)
        r = r + self.rng.normal(0.0, self.scan_noise, r.shape)
        m = LaserScan()
        m.header.stamp = stamp.to_msg()
        m.header.frame_id = "laser"
        m.angle_min, m.angle_increment = a_min, inc
        m.angle_max = a_min + inc * (self.scan_beams - 1)
        m.scan_time = self.scan_every / self.rate
        m.range_min, m.range_max = 0.05, self.scan_range
        m.ranges = [float(v) for v in r]
        return m

    def destroy_node(self):
        if self._http is not None:
            self._http.shutdown()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
