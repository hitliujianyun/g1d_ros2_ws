"""Pure base-guard logic (no DDS), design doc §6.8.

Every control period: select the nav command (or zero), clamp it, check sensor freshness, apply
the reflex layer (swept-path clearance ahead / behind, rotation circle), the e-stop latch and the
manual-override hold, rate-limit, and decide whether rt/cmd_vel_no_limit is written at all
(the chassis is released once the zero command has been held for `zero_hold`).

The swept-path check follows the idea of LidarGuard.path_min in unitree_robotskills
(g1d_r1d_navigation/services/lidar.py) but sweeps the whole chassis rectangle along the
commanded arc instead of a fixed sector.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Dict, Optional

import numpy as np

ESTOP_EVENTS = ("BUMPER_TRIGGERED", "CLIFF_DETECTED")
STRAIGHT_CURVATURE = 1e-3       # |w / v| below this counts as a straight line (1/m)
MAX_SWEEP_RAD = math.pi / 2     # look a quarter turn ahead at most
SWEEP_STEP_RAD = 0.05
IN_PLACE_V = 0.05               # |v| below this the rotation circle check applies


@dataclass
class GuardParams:
    rate_hz: float = 30.0
    vx_max: float = 0.50
    vx_min: float = -0.10
    wz_max: float = 0.60
    acc_v: float = 0.5
    brake_v: float = 1.5
    acc_w: float = 1.0
    cmd_timeout: float = 0.3
    sensor_timeout: float = 0.5
    zero_hold: float = 1.0
    front_x: float = 0.28           # chassis front edge (base_link x)
    rear_x: float = 0.30            # chassis rear edge (positive distance behind base_link)
    half_width: float = 0.30        # swept rectangle half width
    body_x: float = 0.30            # self-return box |x| < body_x and |y| < body_y is ignored
    body_y: float = 0.28
    rotate_radius: float = 0.38
    stop_m: float = 0.45
    slow_m: float = 1.20
    creep_v: float = 0.05           # minimum forward speed inside the slow zone
    rear_clear_m: float = 0.50
    require_cloud: bool = True
    path_step_m: float = 0.05
    manual_deadzone: float = 0.15
    manual_hold_s: float = 5.0
    teleop_timeout: float = 0.3     # web teleop deadman: commands must arrive faster than this
    teleop_vx_max: float = 0.30
    teleop_wz_max: float = 0.50

    @classmethod
    def from_config(cls, cfg: Dict[str, Any]) -> "GuardParams":
        g = (cfg or {}).get("guard", {}) or {}
        p = cls()
        for f in fields(cls):
            if f.name in g:
                setattr(p, f.name, type(getattr(p, f.name))(g[f.name]))
        return p


# Parameters that may change at runtime (rt/nav/param_set) with their allowed ranges.
TUNABLE = {
    "vx_max": (0.0, 1.0), "vx_min": (-0.3, 0.0), "wz_max": (0.0, 1.5),
    "acc_v": (0.05, 3.0), "brake_v": (0.1, 5.0), "acc_w": (0.05, 5.0),
    "stop_m": (0.1, 2.0), "slow_m": (0.2, 4.0), "creep_v": (0.0, 0.2), "rear_clear_m": (0.1, 2.0),
    "rotate_radius": (0.2, 1.0), "zero_hold": (0.0, 5.0), "cmd_timeout": (0.1, 1.0),
    "teleop_vx_max": (0.0, 0.6), "teleop_wz_max": (0.0, 1.0), "require_cloud": (0.0, 1.0),
}


@dataclass
class GuardOutput:
    v: float
    w: float
    publish: bool
    blocked: str
    v_target: float
    w_target: float


def scan_to_points(angle_min: float, angle_inc: float, ranges, laser_xyz=(0.0, 0.0, 0.0),
                   laser_yaw: float = 0.0, range_min: float = 0.05,
                   range_max: float = 30.0) -> np.ndarray:
    """LaserScan ranges -> (N, 3) float32 points in base_link (invalid / inf ranges dropped)."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_inc * np.arange(r.size) + laser_yaw
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok], a[ok]
    out = np.empty((r.size, 3), dtype=np.float32)
    out[:, 0] = laser_xyz[0] + r * np.cos(a)
    out[:, 1] = laser_xyz[1] + r * np.sin(a)
    out[:, 2] = laser_xyz[2]
    return out


def swept_clearance(xy: np.ndarray, v: float, w: float, p: GuardParams, max_dist: float) -> float:
    """Distance (m) the chassis rectangle can travel along the (v, w) arc before touching a point
    (inf if nothing within max_dist / a quarter turn). v != 0; points are base_link xy."""
    if xy.shape[0] == 0 or v == 0.0:
        return math.inf
    sgn = 1.0 if v > 0 else -1.0
    edge = p.front_x if v > 0 else p.rear_x
    k = w / abs(v)
    if abs(k) < STRAIGHT_CURVATURE:
        lon = sgn * xy[:, 0]
        hit = (np.abs(xy[:, 1]) <= p.half_width) & (lon > edge - 1e-6)
        if not hit.any():
            return math.inf
        d = float((lon[hit] - edge).min())
        return max(d, 0.0) if d <= max_dist else math.inf
    ds = min(p.path_step_m, SWEEP_STEP_RAD / abs(k))
    s_max = min(max_dist, MAX_SWEEP_RAD / abs(k))
    n = int(math.ceil(s_max / ds))
    s = ds * np.arange(1, n + 1)
    th = k * s
    px = sgn * np.sin(th) / k
    py = sgn * (1.0 - np.cos(th)) / k
    c, sn = np.cos(th), np.sin(th)
    dx = xy[None, :, 0] - px[:, None]
    dy = xy[None, :, 1] - py[:, None]
    lx = c[:, None] * dx + sn[:, None] * dy
    ly = -sn[:, None] * dx + c[:, None] * dy
    inside = (lx <= p.front_x) & (lx >= -p.rear_x) & (np.abs(ly) <= p.half_width)
    any_hit = inside.any(axis=1)
    if not any_hit.any():
        return math.inf
    i = int(np.argmax(any_hit))
    return float(s[i - 1]) if i > 0 else 0.0


class GuardCore:
    def __init__(self, params: GuardParams, now: float = 0.0):
        self.p = params
        self.cmd = (0.0, 0.0)
        self.t_cmd = -math.inf
        self.scan_xy = np.zeros((0, 2), np.float32)
        self.t_scan = -math.inf
        self.cloud_xy = np.zeros((0, 2), np.float32)
        self.t_cloud = -math.inf
        self.v = 0.0
        self.w = 0.0
        self.t_zero = -math.inf         # nothing to hold at start-up: stay silent
        self.estop = False
        self.estop_reason = ""
        self.manual_until = -math.inf
        self.teleop = (0.0, 0.0)
        self.t_teleop = -math.inf
        self.source = "none"            # nav | teleop | none (what the last target came from)
        self.blocked = ""
        self.front_clear = math.inf
        self.rear_clear = math.inf
        self.rotate_ok = True
        self.publishing = False
        self.last_target = (0.0, 0.0)
        self.n_cmd = 0
        self.n_scan = 0
        self.n_cloud = 0

    # ---- inputs -----------------------------------------------------------------------------
    def on_cmd(self, v: float, w: float, now: float) -> None:
        self.cmd = (float(v), float(w))
        self.t_cmd = now
        self.n_cmd += 1

    def on_scan_points(self, pts: np.ndarray, now: float) -> None:
        self.scan_xy = np.ascontiguousarray(pts[:, :2], dtype=np.float32)
        self.t_scan = now
        self.n_scan += 1

    def on_cloud_points(self, pts: np.ndarray, now: float) -> None:
        self.cloud_xy = np.ascontiguousarray(pts[:, :2], dtype=np.float32)
        self.t_cloud = now
        self.n_cloud += 1

    def on_estop(self, on: bool, reason: str = "topic") -> None:
        """"on" latches; "off" is the manual release (operator has checked the scene)."""
        if on:
            if not self.estop:
                self.estop_reason = reason
            self.estop = True
        else:
            self.estop = False
            self.estop_reason = ""

    def on_event(self, event_type: str) -> None:
        if event_type in ESTOP_EVENTS:
            self.on_estop(True, event_type.lower())

    def on_joystick(self, lx: float, ly: float, rx: float, ry: float, now: float) -> None:
        if max(abs(lx), abs(ly), abs(rx), abs(ry)) > self.p.manual_deadzone:
            self.manual_until = now + self.p.manual_hold_s

    def on_teleop(self, v: float, w: float, now: float) -> None:
        """Web teleop (rt/nav/teleop_cmd): overrides nav while fresh, same limits and reflexes."""
        self.teleop = (float(v), float(w))
        self.t_teleop = now

    def set_param(self, name: str, value: Any) -> Optional[str]:
        """Applies a runtime parameter; returns an error message or None."""
        if name not in TUNABLE:
            return f"unknown or read-only parameter '{name}'"
        try:
            v = float(value)
        except (TypeError, ValueError):
            return f"'{name}' needs a number"
        lo, hi = TUNABLE[name]
        if not (lo <= v <= hi) or math.isnan(v):
            return f"'{name}' must be in [{lo}, {hi}]"
        stop = v if name == "stop_m" else self.p.stop_m
        slow = v if name == "slow_m" else self.p.slow_m
        if slow <= stop:
            return "slow_m must be greater than stop_m"
        setattr(self.p, name, bool(v) if name == "require_cloud" else v)
        return None

    def params(self) -> Dict[str, Any]:
        return {k: getattr(self.p, k) for k in TUNABLE}

    # ---- control ----------------------------------------------------------------------------
    def manual(self, now: float) -> bool:
        return now < self.manual_until

    def teleop_active(self, now: float) -> bool:
        return now - self.t_teleop < self.p.teleop_timeout

    def obstacles(self, now: float) -> Optional[np.ndarray]:
        """Fresh obstacle xy (base_link) outside the self box; None if a required source is stale."""
        if now - self.t_scan > self.p.sensor_timeout:
            return None
        parts = [self.scan_xy]
        cloud_fresh = now - self.t_cloud <= self.p.sensor_timeout
        if cloud_fresh:
            parts.append(self.cloud_xy)
        elif self.p.require_cloud:
            return None
        xy = np.vstack(parts) if len(parts) > 1 else parts[0]
        body = (np.abs(xy[:, 0]) < self.p.body_x) & (np.abs(xy[:, 1]) < self.p.body_y)
        return xy[~body]

    def _stale_reason(self, now: float) -> str:
        if now - self.t_scan > self.p.sensor_timeout:
            return "scan_stale"
        return "cloud_stale"

    def step(self, now: float, dt: float) -> GuardOutput:
        p = self.p
        manual = self.manual(now)
        fresh_cmd = now - self.t_cmd < p.cmd_timeout
        if self.estop or manual:
            vt, wt, self.source = 0.0, 0.0, "none"
        elif self.teleop_active(now):
            vt = min(max(self.teleop[0], -p.teleop_vx_max), p.teleop_vx_max)
            wt = min(max(self.teleop[1], -p.teleop_wz_max), p.teleop_wz_max)
            self.source = "teleop"
        elif fresh_cmd:
            vt, wt = self.cmd
            self.source = "nav"
        else:
            vt, wt, self.source = 0.0, 0.0, "none"
        vt = min(max(vt, p.vx_min), p.vx_max)
        wt = min(max(wt, -p.wz_max), p.wz_max)
        self.last_target = (vt, wt)
        blocked = ""
        self.front_clear = self.rear_clear = math.inf
        self.rotate_ok = True

        xy = self.obstacles(now)
        if xy is None:
            if vt != 0.0 or wt != 0.0:
                blocked = self._stale_reason(now)
            vt = wt = 0.0
        else:
            if vt > 0.0:
                d = swept_clearance(xy, vt, wt, p, p.slow_m)
                self.front_clear = d
                if d < p.stop_m:
                    vt, wt, blocked = 0.0, 0.0, "front"
                elif d < p.slow_m:
                    # The creep floor makes the base actually reach the stop line instead of
                    # approaching it asymptotically.
                    vt = max(vt * (d - p.stop_m) / (p.slow_m - p.stop_m), min(vt, p.creep_v))
            elif vt < 0.0:
                d = swept_clearance(xy, vt, wt, p, p.rear_clear_m + 0.5)
                self.rear_clear = d
                if d < p.rear_clear_m:
                    vt, blocked = 0.0, "rear"
            if wt != 0.0 and abs(vt) < IN_PLACE_V:
                in_circle = np.hypot(xy[:, 0], xy[:, 1]) < p.rotate_radius
                if in_circle.any():
                    self.rotate_ok = False
                    wt = 0.0
                    blocked = blocked or "rotate"

        if self.estop or manual:
            # Hard stop: no ramp on an e-stop; under manual control the remote owns the base.
            self.v = self.w = 0.0
        else:
            reversing = vt * self.v < 0.0
            a = p.brake_v if (abs(vt) < abs(self.v) or reversing) else p.acc_v
            self.v += max(-a * dt, min(a * dt, vt - self.v))
            self.w += max(-p.acc_w * dt, min(p.acc_w * dt, wt - self.w))
            if abs(self.v) < 1e-4 and vt == 0.0:
                self.v = 0.0
            if abs(self.w) < 1e-4 and wt == 0.0:
                self.w = 0.0

        active = self.v != 0.0 or self.w != 0.0 or vt != 0.0 or wt != 0.0
        if active:
            self.t_zero = math.nan
        elif math.isnan(self.t_zero):
            self.t_zero = now
        publish = not manual and (active or now - self.t_zero < p.zero_hold)
        self.blocked = blocked
        self.publishing = publish
        return GuardOutput(self.v, self.w, publish, blocked, vt, wt)

    def state(self, now: float) -> Dict[str, Any]:
        def age(t: float) -> Optional[float]:
            return None if math.isinf(t) else round(now - t, 3)

        def dist(d: float) -> Optional[float]:
            return None if math.isinf(d) else round(d, 3)

        return {
            "stamp": now,
            "v": round(self.v, 4), "w": round(self.w, 4),
            "v_cmd": round(self.cmd[0], 4), "w_cmd": round(self.cmd[1], 4),
            "v_target": round(self.last_target[0], 4), "w_target": round(self.last_target[1], 4),
            "blocked": self.blocked,
            "estop": self.estop, "estop_reason": self.estop_reason,
            # g1d_nav cancels its goal while "manual" is true (remote stick or web teleop).
            "manual": self.manual(now) or self.teleop_active(now),
            "manual_source": ("remote" if self.manual(now) else
                              "web" if self.teleop_active(now) else ""),
            "source": self.source,
            "publishing": self.publishing,
            "front_clear": dist(self.front_clear), "rear_clear": dist(self.rear_clear),
            "rotate_ok": self.rotate_ok,
            "cmd_age": age(self.t_cmd), "scan_age": age(self.t_scan), "cloud_age": age(self.t_cloud),
            "n_cmd": self.n_cmd, "n_scan": self.n_scan, "n_cloud": self.n_cloud,
        }
