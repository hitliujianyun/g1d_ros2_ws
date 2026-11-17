"""ROS 2 side of g1d_web (replaces the g1d_ws DDS hub; same interface for app.py).

An rclpy node spun in a background thread keeps the latest value of every input (thread-safe);
the asyncio app polls `version` counters. Everything is expressed in the display frame (the global
frame: map, which equals odom without a prior map); odom-frame grids are resampled into it.
"""
from __future__ import annotations

import collections
import json
import math
import threading
import time
from typing import Any, Callable, Deque, Dict, Optional, Tuple

import numpy as np

TOPICS = {
    "status": "/nav/status", "guard": "/g1d/guard_state", "lio": "/g1d/lio_health",
    "loc": "/g1d/localization_status", "debug": "/plan", "base_odom": "/odometry",
    "wheel_odom": "/chassis/odom", "cloud": "/g1d/registered_scan", "obstacles": "/g1d/obstacle_cloud",
    "chassis": "/chassis/scan", "local_costmap": "/local_costmap/costmap",
    "global_costmap": "/global_costmap/costmap", "cmd_vel": "/nav/cmd_vel",
    "cmd_out": "/cmd_vel_no_limit", "param_state": "(parameters)",
}
POSE_FRESH_S = 1.0
GUARD_PARAMS = ("vx_max", "vx_min", "wz_max", "acc_v", "brake_v", "acc_w", "stop_m", "slow_m", "creep_v",
                "rear_clear_m", "rotate_radius", "zero_hold", "cmd_timeout", "teleop_vx_max", "teleop_wz_max",
                "require_cloud")


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


def resample_grid(g: Dict[str, Any], t: np.ndarray) -> Dict[str, Any]:
    """Grid (dict: width, height, resolution, origin, data bytes) in frame A -> axis-aligned grid in
    frame B, t = T_B_A (4x4); nearest neighbour, unknown (255) outside."""
    yaw = math.atan2(t[1, 0], t[0, 0])
    if abs(yaw) < 1e-3 and np.allclose(t[:2, 3], 0.0, atol=1e-4):
        return g
    w, h, res = g["width"], g["height"], g["resolution"]
    ox, oy = g["origin"]
    src = np.frombuffer(g["data"], np.uint8).reshape(h, w)
    corners = np.array([[ox, oy], [ox + w * res, oy], [ox, oy + h * res], [ox + w * res, oy + h * res]])
    c, s = math.cos(yaw), math.sin(yaw)
    cb = corners @ np.array([[c, s], [-s, c]]) + t[:2, 3]
    lo, hi = cb.min(axis=0), cb.max(axis=0)
    nw, nh = int(math.ceil((hi[0] - lo[0]) / res - 1e-6)), int(math.ceil((hi[1] - lo[1]) / res - 1e-6))
    xs = lo[0] + (np.arange(nw) + 0.5) * res
    ys = lo[1] + (np.arange(nh) + 0.5) * res
    X, Y = np.meshgrid(xs - t[0, 3], ys - t[1, 3])
    ax, ay = c * X + s * Y, -s * X + c * Y                   # back into frame A
    col = np.floor((ax - ox) / res).astype(np.int64)
    row = np.floor((ay - oy) / res).astype(np.int64)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    out = np.full((nh, nw), 255, np.uint8)
    out[ok] = src[row[ok], col[ok]]
    return {"width": nw, "height": nh, "resolution": res, "origin": [float(lo[0]), float(lo[1])],
            "data": out.tobytes()}


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
    def __init__(self, clock: Callable[[], float] = time.monotonic, display_frame: str = "map"):
        self.clock = clock
        self.display_frame = display_frame
        self.lock = threading.Lock()
        self.version: Dict[str, int] = collections.defaultdict(int)
        self.t_recv: Dict[str, float] = {}
        self.rates = {k: RateMeter() for k in TOPICS}
        self.json: Dict[str, Dict[str, Any]] = {}
        self.params: Dict[str, Dict[str, Any]] = {}
        self.param_acks: Deque[Dict[str, Any]] = collections.deque(maxlen=20)
        self.pose_nav: Optional[Dict[str, float]] = None
        self.pose_wheel: Optional[Dict[str, float]] = None
        self.pose_odom: Optional[Dict[str, float]] = None
        self.clouds: Dict[str, Tuple[float, np.ndarray]] = {}
        self.grids: Dict[str, Dict[str, Any]] = {}
        self.cmd: Dict[str, Tuple[float, float]] = {}
        self.cloud_listeners = []
        self.node = None
        self._twist = (0.0, 0.0)
        self._paths = {"global_path": [], "local_path": []}

    # ---- ROS --------------------------------------------------------------------------------
    def start(self) -> None:
        import rclpy
        from g1d_msgs.msg import GuardState, LioHealth, LocalizationStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener

        if not rclpy.ok():
            rclpy.init()
        n = self.node = rclpy.create_node("g1d_web")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, n)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def sub(key, typ, cb, qos=10):
            n.create_subscription(typ, TOPICS[key], lambda m, key=key: self._wrap(key, cb, m), qos)

        sub("status", String, lambda k, m: self.json.__setitem__(k, json.loads(m.data)))
        sub("guard", GuardState, self._on_guard)
        sub("lio", LioHealth, self._on_lio)
        sub("loc", LocalizationStatus, self._on_loc)
        sub("base_odom", Odometry, self._on_odom)
        sub("wheel_odom", Odometry, self._on_odom, qos_profile_sensor_data)
        for key in ("cloud", "obstacles"):
            sub(key, PointCloud2, self._on_cloud, qos_profile_sensor_data)
        sub("chassis", LaserScan, self._on_scan, qos_profile_sensor_data)
        for key in ("local_costmap", "global_costmap"):
            sub(key, OccupancyGrid, self._on_grid, latched)
        sub("debug", Path, lambda k, m: self._on_path("global_path", m))
        n.create_subscription(Path, "/local_plan", lambda m: self._on_path("local_path", m), 10)
        for key in ("cmd_vel", "cmd_out"):
            sub(key, Twist, self._on_twist, qos_profile_sensor_data)
        self._pubs = {
            "goal": n.create_publisher(String, "/nav/goal", 10),
            "estop": n.create_publisher(Bool, "/g1d/estop", 10),
            "teleop": n.create_publisher(Twist, "/g1d/teleop_cmd", 10),
            "initialpose": n.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10),
        }
        self._msg = {"String": String, "Bool": Bool, "Twist": Twist, "Pose": PoseWithCovarianceStamped}
        self._guard_params = AsyncParameterClient(n, "g1d_guard")
        n.create_timer(0.1, self._poll_pose)
        n.create_timer(1.0, self._poll_params)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(n)
        threading.Thread(target=self._executor.spin, name="g1d_web_ros", daemon=True).start()

    def _wrap(self, key: str, cb, m) -> None:
        now = self.clock()
        try:
            cb(key, m)
        except Exception:  # noqa: BLE001 - one bad message must not stop the hub
            return
        with self.lock:
            self.version[key] += 1
            self.t_recv[key] = now
            self.rates[key].tick(now)

    def _lookup(self, target: str, source: str) -> Optional[np.ndarray]:
        from rclpy.time import Time
        try:
            t = self.tf.lookup_transform(target, source, Time())
        except Exception:  # noqa: BLE001
            return None
        tr, q = t.transform.translation, t.transform.rotation
        m = np.eye(4)
        m[:3, :3] = quat_to_matrix((q.x, q.y, q.z, q.w))
        m[:3, 3] = (tr.x, tr.y, tr.z)
        return m

    def _on_guard(self, key, m) -> None:
        d = {f: getattr(m, f) for f in ("v", "w", "v_cmd", "w_cmd", "blocked", "estop", "estop_reason", "manual",
                                        "manual_source", "source", "publishing", "rotate_ok")}
        for f in ("front_clear", "rear_clear"):
            v = getattr(m, f)
            d[f] = None if math.isinf(v) else round(float(v), 3)
        self.json[key] = d

    def _on_lio(self, key, m) -> None:
        self.json[key] = {"ok": m.ok, "errors": list(m.errors), "age": m.age, "z": m.z,
                          "wheel_dpos": m.wheel_dpos}

    def _on_loc(self, key, m) -> None:
        self.json[key] = {"mode": m.mode, "localized": m.localized, "reasons": list(m.reasons),
                          "inlier_ratio": m.inlier_ratio, "last_ok_age": m.last_ok_age}

    def _on_odom(self, key, m) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        rec = {"x": p.x, "y": p.y, "yaw": yaw, "v": m.twist.twist.linear.x, "w": m.twist.twist.angular.z,
               "t": self.clock()}
        if key == "base_odom":
            self._twist = (rec["v"], rec["w"])
            self.pose_odom = rec                              # odom frame: LIO drift tests (T3 / T4)
        else:
            self.pose_wheel = rec

    def _poll_pose(self) -> None:
        t = self._lookup(self.display_frame, "base_link")
        if t is not None:
            self.pose_nav = {"x": float(t[0, 3]), "y": float(t[1, 3]), "yaw": math.atan2(t[1, 0], t[0, 0]),
                             "v": self._twist[0], "w": self._twist[1], "t": self.clock()}
            with self.lock:
                self.version["base_odom"] += 1
                self.t_recv["base_odom"] = self.clock()

    def _on_cloud(self, key, m) -> None:
        from sensor_msgs_py import point_cloud2
        pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True).astype(np.float32)
        if key == "obstacles":                                   # -> base_link (app adds the pose)
            t = self._lookup("base_link", m.header.frame_id)
            if t is None:
                return
            pts = (pts @ t[:3, :3].T + t[:3, 3]).astype(np.float32)
        self.clouds[key] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, pts)
        if key == "cloud":
            for cb in list(self.cloud_listeners):
                cb(pts)

    def _on_scan(self, key, m) -> None:
        from g1d_core.guard import scan_to_points
        t = self._lookup("base_link", m.header.frame_id)
        if t is None:
            return
        pts = scan_to_points(m.angle_min, m.angle_increment, m.ranges, t[:3, 3], math.atan2(t[1, 0], t[0, 0]),
                             max(0.05, m.range_min), m.range_max if m.range_max > 0 else 30.0)
        self.clouds[key] = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, pts)

    def _on_grid(self, key, m) -> None:
        info = m.info
        data = np.asarray(m.data, dtype=np.int16)
        g = {"width": int(info.width), "height": int(info.height), "resolution": float(info.resolution),
             "origin": [float(info.origin.position.x), float(info.origin.position.y)],
             "data": np.where(data < 0, 255, data).astype(np.uint8).tobytes()}
        if m.header.frame_id and m.header.frame_id != self.display_frame:
            t = self._lookup(self.display_frame, m.header.frame_id)
            if t is None:
                return
            g = resample_grid(g, t)
        self.grids[key] = g

    def _on_path(self, name, m) -> None:
        t = self._lookup(self.display_frame, m.header.frame_id) if m.header.frame_id else None
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses[:: max(1, len(m.poses) // 300)]]
        if t is not None and pts:
            a = np.array(pts) @ t[:2, :2].T + t[:2, 3]
            pts = a.tolist()
        self._paths[name] = [[round(x, 3), round(y, 3)] for x, y in pts]
        st = self.json.get("status") or {}
        self.json["debug"] = dict(self._paths, goal=st.get("goal"), mppi_traj=[],
                                  T_odom_lio={"valid": True})

    def _on_twist(self, key, m) -> None:
        self.cmd[key] = (float(m.linear.x), float(m.angular.z))

    def _poll_params(self) -> None:
        if not self._guard_params.services_are_ready():
            return
        fut = self._guard_params.get_parameters(list(GUARD_PARAMS))

        def done(f):
            try:
                vals = f.result().values
            except Exception:  # noqa: BLE001
                return
            from rclpy.parameter import parameter_value_to_python
            self.params["guard"] = {k: parameter_value_to_python(v) for k, v in zip(GUARD_PARAMS, vals)}
            with self.lock:
                self.version["param_state"] += 1
                self.t_recv["param_state"] = self.clock()
        fut.add_done_callback(done)

    # ---- queries ----------------------------------------------------------------------------
    def fresh(self, key: str, max_age: float) -> bool:
        t = self.t_recv.get(key)
        return t is not None and self.clock() - t < max_age

    def pose(self) -> Tuple[Optional[Dict[str, float]], str]:
        now = self.clock()
        p = self.pose_nav
        if p and now - p["t"] < POSE_FRESH_S:
            return p, "nav"
        w = self.pose_wheel
        if w and now - w["t"] < POSE_FRESH_S:
            return w, "wheel"
        return None, "none"

    def velocity(self) -> Tuple[float, float]:
        now = self.clock()
        for p in (self.pose_wheel, self.pose_nav):
            if p and now - p["t"] < POSE_FRESH_S:
                return p["v"], p["w"]
        return 0.0, 0.0

    def T_odom_lio(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        """Transform of the accumulated scans (/g1d/registered_scan, odom frame) into the display frame."""
        t = self._lookup(self.display_frame, "odom") if self.node is not None else None
        if t is None:
            return np.eye(3), np.zeros(3), False
        return t[:3, :3], t[:3, 3], True

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
        self._pubs["goal"].publish(self._msg["String"](data=json.dumps(goal)))

    def send_estop(self, on: bool) -> None:
        self._pubs["estop"].publish(self._msg["Bool"](data=bool(on)))

    def send_teleop(self, v: float, w: float) -> None:
        t = self._msg["Twist"]()
        t.linear.x, t.angular.z = float(v), float(w)
        self._pubs["teleop"].publish(t)

    def send_initial_pose(self, x: float, y: float, yaw: float) -> None:
        m = self._msg["Pose"]()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = self.display_frame
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        self._pubs["initialpose"].publish(m)

    def call_service(self, name: str, srv_type, request, timeout: float = 60.0) -> Any:
        """Blocking service call from the app thread (the hub executor serves the response)."""
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                raise RuntimeError(f"{name} not available")
            fut = cli.call_async(request)
            ev = threading.Event()
            fut.add_done_callback(lambda _: ev.set())
            if not ev.wait(timeout):
                raise RuntimeError(f"{name} timed out")
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def send_param(self, target: str, name: str, value: Any, req_id: str) -> None:
        from rclpy.parameter import Parameter
        if target != "guard":
            self.param_acks.append({"id": req_id, "name": name, "ok": False, "target": target,
                                    "error": "only guard parameters are tunable at runtime (nav: config/nav2.yaml)"})
            return
        cur = (self.params.get("guard") or {}).get(name)
        val = bool(value) if isinstance(cur, bool) else float(value)
        fut = self._guard_params.set_parameters([Parameter(name, value=val)])

        def done(f):
            try:
                r = f.result().results[0]
                ack = {"id": req_id, "name": name, "ok": r.successful, "error": r.reason, "target": target}
            except Exception as e:  # noqa: BLE001
                ack = {"id": req_id, "name": name, "ok": False, "error": str(e), "target": target}
            self.param_acks.append(ack)
            with self.lock:
                self.version["param_state"] += 1
        fut.add_done_callback(done)
