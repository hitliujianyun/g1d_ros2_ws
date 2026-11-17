"""g1d_nav_manager: NavGPT-style goals on top of Nav2 (design doc 5.5).

Action /g1d/navigate_relative (g1d_msgs/NavigateRelative):
  frame base_at_stamp: goal relative to base_link at `stamp` (image time) -> TF history compensates
  the inference delay; base / odom / map frames are also accepted.
  navigate: goal sanitizing on the global costmap (partial / adjusted / reasons) -> NavigateToPose
  rotate:   Nav2 Spin; simple: Spin to the bearing, then DriveOnHeading
  stop:     cancels the running goal
Pre-conditions / abort conditions: body lock, LIO health, localization (map frame), guard e-stop /
manual override, timeout. Failures carry `reason` and `free_sectors` (local costmap).
JSON compatibility with g1d_ws: /nav/goal (String) -> action, /nav/status (String, 10 Hz).
"""
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from g1d_core.lio import make_tf, wrap, yaw_of
from g1d_core.navgoal import (SECTOR_ANGLES, CostGrid, GoalParams, free_sectors, nav2_reason,
                              relative_to_global, sanitize_goal)
from g1d_msgs.action import NavigateRelative
from g1d_msgs.msg import BodyLock, GuardState, LioHealth, LocalizationStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RDuration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def tf_matrix(t):
    tr, q = t.transform.translation, t.transform.rotation
    return make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))


def grid_from_msg(m: OccupancyGrid) -> CostGrid:
    i = m.info
    return CostGrid.from_msg_fields(i.width, i.height, i.resolution, i.origin.position.x,
                                    i.origin.position.y, m.data)


class Abort(Exception):
    def __init__(self, reason, state="FAILED"):
        super().__init__(reason)
        self.reason, self.state = reason, state


class NavManager(Node):
    def __init__(self, **kwargs):
        super().__init__("nav_manager", **kwargs)
        d = self.declare_parameter
        self.global_frame = d("global_frame", "map").value
        self.odom_frame = d("odom_frame", "odom").value
        self.base_frame = d("base_frame", "base_link").value
        self.default_timeout = float(d("default_timeout_s", 120.0).value)
        self.goal_params = GoalParams(max_distance=float(d("max_goal_distance", 9.0).value),
                                      cost_threshold=int(d("goal_cost_threshold", 50).value),
                                      search_radius=float(d("goal_search_radius", 0.5).value))
        self.max_stamp_age = float(d("max_stamp_age_s", 5.0).value)
        self.require_body_lock = bool(d("require_body_lock", True).value)
        self.require_localization = bool(d("require_localization", self.global_frame == "map").value)
        self.sector_range = float(d("free_sector_range", 1.5).value)
        self.yaw_tol = float(d("yaw_goal_tolerance", 0.25).value)
        self.yaw_factor = float(d("yaw_not_reached_factor", 1.5).value)
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer(cache_time=RDuration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf, self)
        self.global_grid = self.local_grid = None
        self.body = self.lio = self.loc = self.guard = None
        self.active = None                       # id of the running goal
        self.status = {"goal_id": "", "state": "IDLE", "reason": ""}
        self.lock = threading.Lock()
        sub = lambda t, ty, cb, q=10: self.create_subscription(ty, t, cb, q, callback_group=self.cb)  # noqa: E731
        sub("/global_costmap/costmap", OccupancyGrid, lambda m: setattr(self, "global_grid", grid_from_msg(m)), LATCHED)
        sub("/local_costmap/costmap", OccupancyGrid, lambda m: setattr(self, "local_grid", grid_from_msg(m)), LATCHED)
        sub("/g1d/body_lock", BodyLock, lambda m: setattr(self, "body", m))
        sub("/g1d/lio_health", LioHealth, lambda m: setattr(self, "lio", m))
        sub("/g1d/localization_status", LocalizationStatus, lambda m: setattr(self, "loc", m))
        sub("/g1d/guard_state", GuardState, lambda m: setattr(self, "guard", m))
        sub("/nav/goal", String, self.on_json_goal)
        self.pub_speed = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.pub_status = self.create_publisher(String, "/nav/status", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb)
        self.spin_ac = ActionClient(self, Spin, "spin", callback_group=self.cb)
        self.drive_ac = ActionClient(self, DriveOnHeading, "drive_on_heading", callback_group=self.cb)
        self.self_client = ActionClient(self, NavigateRelative, "/g1d/navigate_relative", callback_group=self.cb)
        self.server = ActionServer(self, NavigateRelative, "/g1d/navigate_relative", self.execute,
                                   goal_callback=lambda _: GoalResponse.ACCEPT,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.cb)
        self.create_timer(0.1, self.publish_status, callback_group=self.cb)

    # ---- helpers ----------------------------------------------------------------------------
    def lookup(self, target, source, stamp=None, timeout=0.2):
        t = self.tf.lookup_transform(target, source, stamp if stamp is not None else Time(),
                                     timeout=RDuration(seconds=timeout))
        return tf_matrix(t)

    def robot_pose(self):
        try:
            t = self.lookup(self.global_frame, self.base_frame, timeout=0.0)
            return t[0, 3], t[1, 3], yaw_of(t)
        except TransformException:
            return None

    def sectors(self):
        pose = None
        try:
            t = self.lookup(self.odom_frame, self.base_frame, timeout=0.0)
            pose = (t[0, 3], t[1, 3], yaw_of(t))
        except TransformException:
            pass
        if self.local_grid is None or pose is None:
            return {}
        return free_sectors(self.local_grid, pose, SECTOR_ANGLES, self.sector_range)

    def check_conditions(self, starting: bool):
        if self.require_body_lock and (self.body is None or not self.body.locked):
            raise Abort("body_not_locked" if starting else "body_moved")
        if self.lio is not None and not self.lio.ok:
            raise Abort("lio_lost")
        if self.require_localization and (self.loc is None or not self.loc.localized):
            raise Abort("localization_lost")
        if self.guard is not None:
            if self.guard.estop:
                raise Abort("guard_estop")
            if self.guard.manual:
                raise Abort("manual_override", "CANCELED")

    def wait_future(self, fut, timeout):
        ev = threading.Event()
        fut.add_done_callback(lambda _: ev.set())
        if not ev.wait(timeout):
            raise Abort("timeout")
        return fut.result()

    def global_goal(self, g):
        """Goal (x, y, yaw) in the global frame."""
        if g.frame in ("", "base_at_stamp", "base"):
            stamp = Time.from_msg(g.stamp)
            if g.frame == "base" or stamp.nanoseconds == 0:
                stamp = Time()
            else:
                age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
                if age > self.max_stamp_age or age < -1.0:
                    raise Abort("goal_stamp_unavailable")
            try:
                t = self.lookup(self.global_frame, self.base_frame, stamp)
            except TransformException:
                raise Abort("goal_stamp_unavailable")
            return relative_to_global(t, g.x, g.y, g.yaw)
        if g.frame in (self.global_frame, self.odom_frame, "map", "odom"):
            try:
                t = self.lookup(self.global_frame, g.frame)
            except TransformException:
                raise Abort("tf_error")
            return relative_to_global(t, g.x, g.y, g.yaw)
        raise Abort("invalid_goal")

    # ---- execution --------------------------------------------------------------------------
    def execute(self, gh):
        g = gh.request
        my_id = object()
        with self.lock:
            self.active = my_id
        res = NavigateRelative.Result()
        self.set_status(g.id, "PLANNING", "")
        nav_handle = [None]
        try:
            if g.mode == "stop":
                raise Abort("stop", "CANCELED")
            self.check_conditions(starting=True)
            gx, gy, gyaw = self.global_goal(g)
            robot = self.robot_pose()
            if robot is None:
                raise Abort("tf_error")
            timeout = g.timeout_s if g.timeout_s > 0 else self.default_timeout
            self.pub_speed.publish(SpeedLimit(percentage=False, speed_limit=float(max(0.0, g.max_speed))))
            if g.mode in ("", "navigate"):
                if self.global_grid is not None:
                    chk = sanitize_goal(self.global_grid, robot[:2], (gx, gy), self.goal_params)
                    res.partial, res.adjusted = chk.partial, chk.adjusted
                    if not chk.ok:
                        raise Abort(chk.reason)
                    gx, gy = chk.xy
                if not g.use_yaw:
                    gyaw = math.atan2(gy - robot[1], gx - robot[0])
                res.goal.x, res.goal.y, res.goal.theta = gx, gy, gyaw
                self.navigate(gh, g, my_id, (gx, gy, gyaw), timeout, nav_handle)
                if g.use_yaw:
                    p = self.robot_pose()
                    if p is not None and abs(wrap(p[2] - gyaw)) > self.yaw_tol * self.yaw_factor:
                        res.reason = "yaw_not_reached"
            elif g.mode == "rotate":
                target = gyaw
                res.goal.x, res.goal.y, res.goal.theta = robot[0], robot[1], target
                self.spin(gh, g, my_id, wrap(target - robot[2]), timeout)
            elif g.mode == "simple":
                bearing = math.atan2(gy - robot[1], gx - robot[0])
                dist = math.hypot(gx - robot[0], gy - robot[1])
                res.goal.x, res.goal.y, res.goal.theta = gx, gy, bearing
                self.spin(gh, g, my_id, wrap(bearing - robot[2]), timeout)
                self.drive(gh, g, my_id, dist, g.max_speed if g.max_speed > 0 else 0.25, timeout)
            else:
                raise Abort("invalid_goal")
            res.state = "SUCCEEDED"
        except Abort as a:
            res.state, res.reason = a.state, a.reason
            if a.reason != "preempted" and nav_handle[0] is not None:
                nav_handle[0].cancel_goal_async()
        p = self.robot_pose()
        if p is not None:
            res.final_pose.x, res.final_pose.y, res.final_pose.theta = p
        if res.state != "SUCCEEDED":
            s = self.sectors()
            res.free_sector_angles = list(s.keys())
            res.free_sectors = list(s.values())
        self.set_status(g.id, res.state, res.reason, res=res)
        if res.state == "SUCCEEDED":
            gh.succeed()
        elif res.state == "CANCELED" and gh.is_cancel_requested:
            gh.canceled()
        else:
            gh.abort()
        return res

    def monitor(self, gh, my_id, fut_result, t_end, feedback_state, goal=None, nav_handle=None):
        """Waits for a Nav2 action result while checking abort conditions."""
        ev = threading.Event()
        fut_result.add_done_callback(lambda _: ev.set())
        while not ev.wait(0.1):
            if self.active is not my_id:
                raise Abort("preempted", "CANCELED")
            if gh.is_cancel_requested:
                raise Abort("stop", "CANCELED")
            if time.monotonic() > t_end:
                raise Abort("timeout")
            self.check_conditions(starting=False)
            fb = NavigateRelative.Feedback(state=feedback_state)
            p = self.robot_pose()
            if p is not None:
                fb.pose.x, fb.pose.y, fb.pose.theta = p
                if goal is not None:
                    fb.distance_remaining = float(math.hypot(goal[0] - p[0], goal[1] - p[1]))
                    fb.yaw_error = float(abs(wrap(goal[2] - p[2])))
            if self.guard is not None:
                fb.guard_blocked = self.guard.blocked
            gh.publish_feedback(fb)
            self.status.update({"distance_remaining": fb.distance_remaining, "yaw_error": fb.yaw_error})
        return fut_result.result()

    def send(self, client, goal_msg, timeout=5.0):
        if not client.wait_for_server(timeout_sec=timeout):
            raise Abort("nav2_unavailable")
        handle = self.wait_future(client.send_goal_async(goal_msg), timeout)
        if not handle.accepted:
            raise Abort("nav2_rejected")
        return handle

    def navigate(self, gh, g, my_id, goal, timeout, nav_handle):
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.global_frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(goal[0]), float(goal[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(0.5 * goal[2]), math.cos(0.5 * goal[2])
        handle = nav_handle[0] = self.send(self.nav, msg)
        self.set_status(g.id, "NAVIGATING", "")
        r = self.monitor(gh, my_id, handle.get_result_async(), time.monotonic() + timeout, "NAVIGATING",
                         goal, nav_handle)
        nav_handle[0] = None
        if r.status != GoalStatus.STATUS_SUCCEEDED:
            raise Abort(nav2_reason(int(getattr(r.result, "error_code", 0)), str(getattr(r.result, "error_msg", ""))))

    def spin(self, gh, g, my_id, angle, timeout):
        if abs(angle) < 0.02:
            return
        msg = Spin.Goal(target_yaw=float(angle), time_allowance=Duration(sec=int(min(timeout, 60))))
        handle = self.send(self.spin_ac, msg)
        self.set_status(g.id, "ROTATING", "")
        r = self.monitor(gh, my_id, handle.get_result_async(), time.monotonic() + timeout, "ROTATING")
        if r.status != GoalStatus.STATUS_SUCCEEDED:
            raise Abort("blocked")

    def drive(self, gh, g, my_id, dist, speed, timeout):
        if dist < 0.02:
            return
        msg = DriveOnHeading.Goal(speed=float(min(speed, 0.3)),
                                  time_allowance=Duration(sec=int(min(timeout, dist / min(speed, 0.3) * 2 + 5))))
        msg.target.x = float(dist)
        handle = self.send(self.drive_ac, msg)
        self.set_status(g.id, "DRIVING", "")
        r = self.monitor(gh, my_id, handle.get_result_async(), time.monotonic() + timeout, "DRIVING")
        if r.status != GoalStatus.STATUS_SUCCEEDED:
            raise Abort("blocked")

    # ---- status / JSON compatibility --------------------------------------------------------
    def set_status(self, gid, state, reason, res=None):
        with self.lock:
            self.status = {"goal_id": gid, "state": state, "reason": reason}
            if res is not None:
                self.status.update({
                    "partial": res.partial, "adjusted": res.adjusted,
                    "free_sectors": {str(a): bool(f) for a, f in zip(res.free_sector_angles, res.free_sectors)}})

    def publish_status(self) -> None:
        with self.lock:
            st = dict(self.status)
        p = self.robot_pose()
        if p is not None:
            st["pose"] = {"x": round(p[0], 3), "y": round(p[1], 3), "yaw": round(p[2], 4)}
        st["lio_ok"] = bool(self.lio.ok) if self.lio is not None else None
        st["localized"] = bool(self.loc.localized) if self.loc is not None else None
        st["body_locked"] = bool(self.body.locked) if self.body is not None else None
        if self.guard is not None:
            st["guard"] = {"blocked": self.guard.blocked, "estop": self.guard.estop, "manual": self.guard.manual}
        st["stamp"] = self.get_clock().now().nanoseconds * 1e-9
        self.pub_status.publish(String(data=json.dumps(st)))

    def on_json_goal(self, m: String) -> None:
        """g1d_ws rt/nav/goal JSON -> /g1d/navigate_relative."""
        try:
            j = json.loads(m.data)
        except ValueError:
            self.get_logger().warn("invalid /nav/goal JSON")
            return
        g = NavigateRelative.Goal()
        g.id, g.mode = str(j.get("id", "")), str(j.get("mode", "navigate"))
        g.frame = str(j.get("frame", "base_at_stamp"))
        st = float(j.get("stamp", 0.0))
        g.stamp.sec, g.stamp.nanosec = int(st), int((st - int(st)) * 1e9)
        g.x, g.y = float(j.get("x", 0.0)), float(j.get("y", 0.0))
        g.use_yaw = "yaw" in j and bool(j.get("use_yaw", True))
        g.yaw = float(j.get("yaw", 0.0))
        g.max_speed, g.timeout_s = float(j.get("max_speed", 0.0)), float(j.get("timeout_s", 0.0))
        if not self.self_client.wait_for_server(timeout_sec=1.0):
            return
        self.self_client.send_goal_async(g)


def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
