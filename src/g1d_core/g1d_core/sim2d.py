"""Planar simulation utilities (migrated from g1d_ws@068e6dd mock_robot.py): box worlds, 2D ray
casting for the chassis lidar, chassis footprint clearance and a diff-drive base with a command
timeout. Worlds are 3D boxes so the lidar simulator (M3) can share them.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np
import yaml

Box2 = Tuple[float, float, float, float]               # x0, y0, x1, y1
Box3 = Tuple[float, float, float, float, float, float]  # x0, y0, z0, x1, y1, z1

SLAMWARE_STAMP_LAG_S = 181.0
CHASSIS_HALF = (0.2785, 0.2625)        # half length / width of the 557 x 525 mm chassis


def load_world(path: str) -> List[Box3]:
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    boxes = []
    for b in doc.get("boxes", []):
        if len(b) != 6:
            raise ValueError(f"box needs 6 values [x0, y0, z0, x1, y1, z1]: {b}")
        x0, y0, z0, x1, y1, z1 = (float(v) for v in b)
        boxes.append((min(x0, x1), min(y0, y1), min(z0, z1), max(x0, x1), max(y0, y1), max(z0, z1)))
    return boxes


def boxes_at_height(boxes: Iterable[Box3], z: float) -> List[Box2]:
    return [(b[0], b[1], b[3], b[4]) for b in boxes if b[2] <= z <= b[5]]


def boxes_in_band(boxes: Iterable[Box3], z_min: float, z_max: float) -> List[Box2]:
    """Footprints of the boxes overlapping the height band (collision check of the whole robot)."""
    return [(b[0], b[1], b[3], b[4]) for b in boxes if b[2] <= z_max and b[5] >= z_min]


def raycast_boxes(origin: Sequence[float], angles: np.ndarray, boxes: Sequence[Box2],
                  max_range: float) -> np.ndarray:
    """Distance to the first box along each angle (inf if none within max_range)."""
    ox, oy = float(origin[0]), float(origin[1])
    dx, dy = np.cos(angles), np.sin(angles)
    best = np.full(angles.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for x0, y0, x1, y1 in boxes:
            tx0, tx1 = (x0 - ox) / dx, (x1 - ox) / dx
            ty0, ty1 = (y0 - oy) / dy, (y1 - oy) / dy
            tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
            tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))
            hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
            t = np.where(tmin > 0.0, tmin, tmax)
            best = np.where(hit & (t > 0) & (t < best), t, best)
    best[best > max_range] = np.inf
    return best


def footprint_gap(pose: Tuple[float, float, float], boxes: Sequence[Box2],
                  half: Tuple[float, float] = CHASSIS_HALF, n: int = 12) -> float:
    """Smallest distance from the chassis rectangle outline to any box (<= 0: collision)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    u = np.linspace(-1.0, 1.0, n)
    edge = np.concatenate([np.c_[u * half[0], np.full(n, half[1])], np.c_[u * half[0], np.full(n, -half[1])],
                           np.c_[np.full(n, half[0]), u * half[1]], np.c_[np.full(n, -half[0]), u * half[1]]])
    wx = x + c * edge[:, 0] - s * edge[:, 1]
    wy = y + s * edge[:, 0] + c * edge[:, 1]
    gap = math.inf
    for x0, y0, x1, y1 in boxes:
        ddx = np.maximum(np.maximum(x0 - wx, 0.0), wx - x1)
        ddy = np.maximum(np.maximum(y0 - wy, 0.0), wy - y1)
        inside = (wx > x0) & (wx < x1) & (wy > y0) & (wy < y1)
        d = np.where(inside, -1.0, np.hypot(ddx, ddy))
        gap = min(gap, float(d.min()))
    return gap


@dataclass
class DiffDriveParams:
    cmd_timeout: float = 0.5          # unitree_sim_isaaclab stops after 0.5 s without commands
    max_acc: float = 1.0
    max_dec: float = 2.0
    max_ang_acc: float = 2.0
    v_limit: float = 1.0
    w_limit: float = 1.0


class DiffDrive:
    def __init__(self, params: DiffDriveParams, pose=(0.0, 0.0, 0.0)):
        self.p = params
        self.pose = [float(v) for v in pose]
        self.v = self.w = 0.0
        self.cmd = (0.0, 0.0)
        self.t_cmd = -math.inf

    def on_cmd(self, v: float, w: float, now: float) -> None:
        self.cmd = (float(v), float(w))
        self.t_cmd = now

    def step(self, now: float, dt: float) -> None:
        p = self.p
        vt, wt = self.cmd if now - self.t_cmd < p.cmd_timeout else (0.0, 0.0)
        vt = max(-p.v_limit, min(p.v_limit, vt))
        wt = max(-p.w_limit, min(p.w_limit, wt))
        a = p.max_dec if (abs(vt) < abs(self.v) or vt * self.v < 0.0) else p.max_acc
        self.v += max(-a * dt, min(a * dt, vt - self.v))
        self.w += max(-p.max_ang_acc * dt, min(p.max_ang_acc * dt, wt - self.w))
        x, y, th = self.pose
        th_mid = th + 0.5 * self.w * dt
        self.pose = [x + self.v * dt * math.cos(th_mid), y + self.v * dt * math.sin(th_mid),
                     math.atan2(math.sin(th + self.w * dt), math.cos(th + self.w * dt))]
