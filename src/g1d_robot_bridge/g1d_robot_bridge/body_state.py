"""Unitree body feedback -> /joint_states (for robot_state_publisher) and /g1d/body_lock.

  /hispeed_state (geometry_msgs/Point32: y lift height, z lift speed)
  /lowstate (unitree_hg/LowState: motor_state[12] waist yaw, [15..28] arms)
  /waistpitchstate (unitree_hg/MotorState: waist pitch)
Joint states are only published while every required source is fresh: TF then goes stale instead
of silently freezing the head pose.
"""
import math
import time
from dataclasses import fields

import rclpy
from g1d_core.body import (BodyConfig, BodySample, LockChecker, arms_from_lowstate,
                           joint_positions, lift_height)
from g1d_msgs.msg import BodyLock
from geometry_msgs.msg import Point32
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from unitree_hg.msg import LowState, MotorState

from .qos import ROBOT_IN


class BodyState(Node):
    def __init__(self, **kwargs):
        super().__init__("body_state", **kwargs)
        defaults = BodyConfig()
        cfg = {}
        for f in fields(BodyConfig):
            cfg[f.name] = self.declare_parameter(f.name, getattr(defaults, f.name)).value
        self.cfg = BodyConfig.from_dict(cfg)
        rate = float(self.declare_parameter("rate_hz", 50.0).value)
        self.sample = BodySample(waist_pitch=self.cfg.nav_waist_pitch)
        self.lock = LockChecker(self.cfg)
        self._warned = 0.0
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_lock = self.create_publisher(BodyLock, "/g1d/body_lock", 10)
        self.create_subscription(Point32, self.declare_parameter("hispeed_topic", "/hispeed_state").value,
                                 self.on_lift, ROBOT_IN)
        self.create_subscription(LowState, self.declare_parameter("lowstate_topic", "/lowstate").value,
                                 self.on_lowstate, ROBOT_IN)
        self.create_subscription(MotorState,
                                 self.declare_parameter("waist_pitch_topic", "/waistpitchstate").value,
                                 self.on_waist_pitch, ROBOT_IN)
        self.create_service(Trigger, "/g1d/body_lock/reset", self.on_reset)
        self.create_timer(1.0 / rate, self.tick)

    def on_lift(self, m: Point32) -> None:
        self.sample.lift_y, self.sample.lift_speed = float(m.y), float(m.z)
        self.sample.t_lift = time.monotonic()

    def on_lowstate(self, m: LowState) -> None:
        q = [ms.q for ms in m.motor_state]
        self.sample.waist_yaw = float(q[self.cfg.waist_yaw_index])
        self.sample.arm_q = arms_from_lowstate(q, self.cfg)
        self.sample.t_lowstate = time.monotonic()

    def on_waist_pitch(self, m: MotorState) -> None:
        self.sample.waist_pitch = float(m.q)
        self.sample.t_waist = time.monotonic()

    def on_reset(self, _req, resp):
        self.lock.reset_reference(self.sample)
        resp.success = True
        resp.message = f"reference lift height {self.lock.ref_height:.3f} m"
        return resp

    def tick(self) -> None:
        now = time.monotonic()
        s, c = self.sample, self.cfg
        stale = [name for name, t, req in (("lift", s.t_lift, True), ("lowstate", s.t_lowstate, True),
                                           ("waist_pitch", s.t_waist, c.require_waist_pitch))
                 if req and now - t > c.stale_s]
        stamp = self.get_clock().now().to_msg()
        if not stale:
            pitch = s.waist_pitch if c.require_waist_pitch else c.nav_waist_pitch
            pos = joint_positions(s.lift_y, pitch, s.waist_yaw, s.arm_q, c)
            js = JointState()
            js.header.stamp = stamp
            js.name = list(pos.keys())
            js.position = list(pos.values())
            self.pub_js.publish(js)
        elif now - self._warned > 5.0:
            self._warned = now
            self.get_logger().warn(f"no joint states: stale {', '.join(stale)}")
        locked, reasons = self.lock.check(s, now)
        msg = BodyLock()
        msg.header.stamp = stamp
        msg.locked = locked
        msg.reasons = reasons
        msg.lift_height = float(lift_height(s.lift_y, c))
        msg.lift_speed = float(s.lift_speed)
        msg.waist_pitch = float(s.waist_pitch)
        msg.waist_yaw = float(s.waist_yaw)
        self.pub_lock.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BodyState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():                 # a message taken while the context shuts down is harmless
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
