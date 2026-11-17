"""DDS side of g1d_web: latest value of every topic (thread-safe), topic rates, publishers.

Callbacks run on DDS reader threads; the asyncio app polls `version` counters, so nothing here
touches the event loop.
"""
from __future__ import annotations

import collections
import json
import math
import threading
import time
from typing import Any, Callable, Deque, Dict, Optional, Tuple

import numpy as np

from g1d_nav_py import msgs

TOPICS = {
    "status": "rt/nav/status", "guard": "rt/nav/guard_state", "lio": "rt/nav/lio_health",
    "debug": "rt/nav/debug", "param_state": "rt/nav/param_state", "base_odom": "rt/nav/base_odom",
    "wheel_odom": "rt/slamware_ros_sdk_server_node/odom", "cloud": "rt/nav/registered_cloud",
    "obstacles": "rt/nav/obstacle_points", "chassis": "rt/nav/chassis_scan",
    "local_costmap": "rt/nav/local_costmap", "global_costmap": "rt/nav/global_costmap",
    "lio_odom": "rt/nav/lio_odom", "cmd_vel": "rt/nav/cmd_vel", "cmd_out": "rt/cmd_vel_no_limit",
}
POSE_FRESH_S = 1.0


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def base_to_odom(pts: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.empty_like(pts, dtype=np.float32)
    out[:, 0] = x + c * pts[:, 0] - s * pts[:, 1]
    out[:, 1] = y + s * pts[:, 0] + c * pts[:, 1]
    out[:, 2] = pts[:, 2]
    return out


class RateMeter:
    def __init__(self, window: float = 3.0):
        self.window = window
        self.t: Deque[float] = collections.deque()

    def tick(self, now: float) -> None:
        self.t.append(now)
        while self.t and now - self.t[0] > self.window:
            self.t.popleft()

    def rate(self, now: float) -> float:
        while self.t and now - self.t[0] > self.window:
            self.t.popleft()
        if len(self.t) < 2:
            return 0.0
        return (len(self.t) - 1) / max(1e-3, self.t[-1] - self.t[0]) if now - self.t[-1] < self.window else 0.0


class Hub:
    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.version: Dict[str, int] = collections.defaultdict(int)
        self.t_recv: Dict[str, float] = {}
        self.rates = {k: RateMeter() for k in TOPICS}
        self.json: Dict[str, Dict[str, Any]] = {}
        self.params: Dict[str, Dict[str, Any]] = {}
        self.param_acks: Deque[Dict[str, Any]] = collections.deque(maxlen=20)
        self.pose_nav: Optional[Dict[str, float]] = None
        self.pose_wheel: Optional[Dict[str, float]] = None
        self.clouds: Dict[str, Tuple[float, np.ndarray]] = {}
        self.grids: Dict[str, Dict[str, Any]] = {}
        self.cmd: Dict[str, Tuple[float, float]] = {}
        self.cloud_listeners = []
        self._subs = []
        self._pubs: Dict[str, Any] = {}

    # ---- DDS --------------------------------------------------------------------------------
    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Twist_
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import OccupancyGrid_, Odometry_
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

        def sub(key, typ, cb, q=4):
            s = ChannelSubscriber(TOPICS[key], typ)
            s.Init(lambda m, key=key: self._wrap(key, cb, m), q)
            self._subs.append(s)

        for key in ("status", "guard", "lio", "debug"):
            sub(key, String_, self._on_json)
        sub("param_state", String_, self._on_param_state, 8)
        sub("base_odom", Odometry_, self._on_odom, 8)
        sub("wheel_odom", Odometry_, self._on_odom, 8)
        sub("lio_odom", Odometry_, lambda k, m: None, 8)
        for key in ("cloud", "obstacles", "chassis"):
            sub(key, PointCloud2_, self._on_cloud, 2)
        for key in ("local_costmap", "global_costmap"):
            sub(key, OccupancyGrid_, self._on_grid, 1)
        for key in ("cmd_vel", "cmd_out"):
            sub(key, Twist_, self._on_twist, 8)
        for key, topic, typ in (("goal", "rt/nav/goal", String_), ("estop", "rt/nav/estop", String_),
                                ("teleop", "rt/nav/teleop_cmd", Twist_),
                                ("param_set", "rt/nav/param_set", String_)):
            p = ChannelPublisher(topic, typ)
            p.Init()
            self._pubs[key] = p

    def _wrap(self, key: str, cb, m) -> None:
        now = self.clock()
        try:
            cb(key, m)
        except Exception:  # noqa: BLE001 - one bad message must not kill the reader thread
            return
        with self.lock:
            self.version[key] += 1
            self.t_recv[key] = now
            self.rates[key].tick(now)

    def _on_json(self, key: str, m) -> None:
        d = json.loads(m.data)
        if isinstance(d, dict):
            self.json[key] = d

    def _on_param_state(self, key: str, m) -> None:
        d = json.loads(m.data)
        target = d.get("target")
        if target in ("guard", "nav"):
            self.params[target] = d.get("params", {})
            if d.get("ack"):
                self.param_acks.append(dict(d["ack"], target=target))

    def _on_odom(self, key: str, m) -> None:
        x, y, yaw, v, w = msgs.odom_pose(m)
        rec = {"x": x, "y": y, "yaw": yaw, "v": v, "w": w, "t": self.clock()}
        if key == "base_odom":
            self.pose_nav = rec
        else:
            self.pose_wheel = rec

    def _on_cloud(self, key: str, m) -> None:
        pts = msgs.read_cloud_xyz(m)
        self.clouds[key] = (msgs.to_sec(m.header.stamp), pts)
        if key == "cloud":
            for cb in list(self.cloud_listeners):
                cb(pts)

    def _on_grid(self, key: str, m) -> None:
        info = m.info
        self.grids[key] = {"width": int(info.width), "height": int(info.height),
                           "resolution": float(info.resolution),
                           "origin": [float(info.origin.position.x), float(info.origin.position.y)],
                           "data": bytes(m.data)}

    def _on_twist(self, key: str, m) -> None:
        self.cmd[key] = (float(m.linear.x), float(m.angular.z))

    # ---- queries ----------------------------------------------------------------------------
    def fresh(self, key: str, max_age: float) -> bool:
        t = self.t_recv.get(key)
        return t is not None and self.clock() - t < max_age

    def pose(self) -> Tuple[Optional[Dict[str, float]], str]:
        """Navigation pose (odom, LIO-based) if fresh, else the SLAMWARE wheel pose."""
        now = self.clock()
        p = self.pose_nav
        if p and now - p["t"] < POSE_FRESH_S:
            return p, "nav"
        w = self.pose_wheel
        if w and now - w["t"] < POSE_FRESH_S:
            return w, "wheel"
        return None, "none"

    def velocity(self) -> Tuple[float, float]:
        """Measured base velocity: wheel odometry (also without g1d_nav), else nav estimate."""
        now = self.clock()
        for p in (self.pose_wheel, self.pose_nav):
            if p and now - p["t"] < POSE_FRESH_S:
                return p["v"], p["w"]
        return 0.0, 0.0

    def T_odom_lio(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        d = (self.json.get("debug") or {}).get("T_odom_lio") or {}
        if d.get("valid"):
            return quat_to_matrix(d["q"]), np.asarray(d["t"], np.float64), True
        return np.eye(3), np.zeros(3), False

    def cloud_in_odom(self, pts: np.ndarray) -> np.ndarray:
        r, t, _ = self.T_odom_lio()
        return (pts.astype(np.float64) @ r.T + t).astype(np.float32)

    def rate_table(self) -> Dict[str, Dict[str, Any]]:
        now = self.clock()
        with self.lock:
            return {k: {"topic": TOPICS[k], "hz": round(self.rates[k].rate(now), 1),
                        "age": (round(now - self.t_recv[k], 2) if k in self.t_recv else None)}
                    for k in TOPICS}

    # ---- publishers -------------------------------------------------------------------------
    def send_goal(self, goal: Dict[str, Any]) -> None:
        self._pubs["goal"].Write(msgs.json_msg(goal))

    def send_estop(self, on: bool) -> None:
        self._pubs["estop"].Write(msgs.string("on" if on else "off"))

    def send_teleop(self, v: float, w: float) -> None:
        self._pubs["teleop"].Write(msgs.twist(v, w))

    def send_param(self, target: str, name: str, value: Any, req_id: str) -> None:
        self._pubs["param_set"].Write(msgs.json_msg({"target": target, "name": name, "value": value,
                                                     "id": req_id}))
