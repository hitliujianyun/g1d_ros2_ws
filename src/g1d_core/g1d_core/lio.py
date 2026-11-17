"""LIO -> base_link odometry (design doc 5.2; logic of g1d_ws StateEstimator, without ROS).

FAST_LIO estimates the IMU pose in its own world frame (the IMU frame at start-up). The odom frame
is anchored at the first LIO pose so that its origin is base_link at start-up (z = floor):
    T_odom_lio = T_base_imu(q_0) * T_lio_imu(t_0)^-1
    T_odom_base(t) = T_odom_lio * T_lio_imu(t) * T_imu_base(q_t)
q_t are the body joints (lift, waist) at time t, so the head lidar may sit anywhere on the body.
The published pose is planar (x, y, yaw); z / roll / pitch only feed the health monitor. Between
LIO updates (10 Hz) the pose is extrapolated with the wheel odometry increment.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

Planar = Tuple[float, float, float]     # x, y, yaw


def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(r: np.ndarray) -> Tuple[float, float, float, float]:
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q[0], q[1], q[2], q[3]


def make_tf(translation: Sequence[float], quat: Sequence[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(*quat)
    m[:3, 3] = translation
    return m


def inv(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(t: np.ndarray) -> float:
    return math.atan2(t[1, 0], t[0, 0])


def tilt_of(t: np.ndarray) -> Tuple[float, float]:
    """roll, pitch of a pose (ZYX convention)."""
    r = t[:3, :3]
    return math.atan2(r[2, 1], r[2, 2]), math.atan2(-r[2, 0], math.hypot(r[2, 1], r[2, 2]))


def planar(t: np.ndarray) -> Planar:
    return float(t[0, 3]), float(t[1, 3]), yaw_of(t)


def compose2(a: Planar, b: Planar) -> Planar:
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def between2(a: Planar, b: Planar) -> Planar:
    """a^-1 * b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])


class LioAnchor:
    def __init__(self) -> None:
        self.t_odom_lio: Optional[np.ndarray] = None

    @property
    def anchored(self) -> bool:
        return self.t_odom_lio is not None

    def anchor(self, t_lio_imu: np.ndarray, t_base_imu: np.ndarray,
               t_odom_base: Optional[np.ndarray] = None) -> None:
        """T_odom_lio such that the current LIO pose maps to t_odom_base (default: the odom origin)."""
        self.t_odom_lio = (np.eye(4) if t_odom_base is None else t_odom_base) @ t_base_imu @ inv(t_lio_imu)

    def base_pose(self, t_lio_imu: np.ndarray, t_base_imu: np.ndarray) -> np.ndarray:
        """T_odom_base (4x4); anchors on the first call."""
        if self.t_odom_lio is None:
            self.anchor(t_lio_imu, t_base_imu)
        return self.t_odom_lio @ t_lio_imu @ inv(t_base_imu)


class WheelOdomBuffer:
    """Wheel odometry poses (any fixed frame) for increments between two times."""

    def __init__(self, horizon_s: float = 5.0) -> None:
        self.horizon = horizon_s
        self.t: List[float] = []
        self.p: List[Planar] = []

    def add(self, t: float, pose: Planar) -> None:
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t)
        self.p.append(pose)
        while self.t and self.t[0] < t - self.horizon:
            self.t.pop(0)
            self.p.pop(0)

    def at(self, t: float) -> Optional[Planar]:
        """Interpolated pose; None outside the buffered interval."""
        if not self.t or t < self.t[0] or t > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t or i == 0:
            return self.p[i]
        t0, t1 = self.t[i - 1], self.t[i]
        a, b = self.p[i - 1], self.p[i]
        u = (t - t0) / (t1 - t0)
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), wrap(a[2] + u * wrap(b[2] - a[2])))

    def latest_time(self) -> Optional[float]:
        return self.t[-1] if self.t else None

    def delta(self, t_from: float, t_to: float) -> Optional[Planar]:
        """Motion of the base between t_from and min(t_to, latest sample), in the base frame."""
        last = self.latest_time()
        if last is None:
            return None
        a = self.at(t_from)
        b = self.at(min(t_to, last))
        if a is None or b is None:
            return None
        return between2(a, b)


@dataclass
class HealthParams:
    lio_timeout_s: float = 0.5
    max_jump_m: float = 0.3
    max_jump_rad: float = math.radians(15.0)
    max_z_drift_m: float = 0.3
    max_tilt_rad: float = math.radians(10.0)
    window_s: float = 2.0
    max_wheel_dpos_m: float = 0.3
    max_wheel_dyaw_rad: float = math.radians(10.0)
    bad_hold_s: float = 1.0           # an error stays reported at least this long


@dataclass
class LioSample:
    t: float
    pose: Planar
    z: float
    roll: float
    pitch: float


class LioHealth:
    def __init__(self, params: Optional[HealthParams] = None) -> None:
        self.p = params or HealthParams()
        self.hist: List[LioSample] = []
        self.latched: dict = {}           # error -> time it was last seen
        self.dpos = 0.0
        self.dyaw = 0.0

    def update(self, s: LioSample, wheel: Optional[WheelOdomBuffer] = None) -> None:
        p = self.p
        errs = []
        if self.hist:
            prev = self.hist[-1]
            d = between2(prev.pose, s.pose)
            if math.hypot(d[0], d[1]) > p.max_jump_m or abs(d[2]) > p.max_jump_rad:
                errs.append("jump")
        if abs(s.z) > p.max_z_drift_m:
            errs.append("z_drift")
        if abs(s.roll) > p.max_tilt_rad or abs(s.pitch) > p.max_tilt_rad:
            errs.append("tilt")
        self.hist.append(s)
        while self.hist and self.hist[0].t < s.t - p.window_s:
            self.hist.pop(0)
        if wheel is not None and len(self.hist) > 1 and s.t - self.hist[0].t > 0.5 * p.window_s:
            w = wheel.delta(self.hist[0].t, s.t)
            if w is not None and (wheel.latest_time() or 0.0) >= s.t - 0.2:
                lio = between2(self.hist[0].pose, s.pose)
                self.dpos = math.hypot(lio[0] - w[0], lio[1] - w[1])
                self.dyaw = abs(wrap(lio[2] - w[2]))
                if self.dpos > p.max_wheel_dpos_m or self.dyaw > p.max_wheel_dyaw_rad:
                    errs.append("wheel_inconsistent")
        for e in errs:
            self.latched[e] = s.t

    def status(self, now: float) -> Tuple[bool, List[str]]:
        errs = [e for e, t in self.latched.items() if now - t <= self.p.bad_hold_s]
        if not self.hist:
            errs.append("no_lio")
        elif now - self.hist[-1].t > self.p.lio_timeout_s:
            errs.append("lio_timeout")
        return not errs, sorted(errs)


def planar_matrix(p: Planar) -> np.ndarray:
    m = np.eye(4)
    c, s = math.cos(p[2]), math.sin(p[2])
    m[:2, :2] = [[c, -s], [s, c]]
    m[:2, 3] = p[:2]
    return m


@dataclass
class BridgeState:
    """Glue used by the ROS node: last LIO base pose + wheel extrapolation.

    A LIO gap longer than reanchor_gap_s (FAST_LIO restarted with a new world frame, or stalled)
    re-anchors odom so the first new pose continues from the last one plus the wheel motion.
    """
    anchor: LioAnchor = field(default_factory=LioAnchor)
    wheel: WheelOdomBuffer = field(default_factory=lambda: WheelOdomBuffer(horizon_s=60.0))
    health: LioHealth = field(default_factory=LioHealth)
    last: Optional[LioSample] = None
    max_extrapolation_s: float = 0.5
    reanchor_gap_s: float = 1.0
    reanchors: int = 0
    reanchored: bool = False          # the last on_lio() re-anchored

    def on_lio(self, t: float, t_lio_imu: np.ndarray, t_base_imu: np.ndarray) -> LioSample:
        self.reanchored = False
        if self.last is not None and t - self.last.t > self.reanchor_gap_s:
            d = self.wheel.delta(self.last.t, t)
            expected = self.last.pose if d is None else compose2(self.last.pose, d)
            self.anchor.anchor(t_lio_imu, t_base_imu, planar_matrix(expected))
            self.health.hist.clear()
            self.reanchors += 1
            self.reanchored = True
        t_odom_base = self.anchor.base_pose(t_lio_imu, t_base_imu)
        roll, pitch = tilt_of(t_odom_base)
        s = LioSample(t, planar(t_odom_base), float(t_odom_base[2, 3]), roll, pitch)
        self.health.update(s, self.wheel)
        self.last = s
        return s

    def pose_at(self, now: float) -> Optional[Planar]:
        """LIO pose extrapolated to `now` with the wheel increment; None if LIO is too old."""
        if self.last is None or now - self.last.t > self.max_extrapolation_s:
            return None
        d = self.wheel.delta(self.last.t, now)
        return self.last.pose if d is None else compose2(self.last.pose, d)
