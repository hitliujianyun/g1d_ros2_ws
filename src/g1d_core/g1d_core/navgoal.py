"""Goal handling for the navigation manager (design doc 5.5; logic of g1d_ws sanitizeGoal).

Costs follow nav_msgs/OccupancyGrid as published by Nav2 costmaps: 0..100 (100 lethal, 99 inscribed),
-1 unknown.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class CostGrid:
    data: np.ndarray                 # (H, W) int, row 0 = lowest y
    origin: Tuple[float, float]
    resolution: float

    @classmethod
    def from_msg_fields(cls, width: int, height: int, resolution: float, ox: float, oy: float,
                        data: Sequence[int]) -> "CostGrid":
        return cls(np.asarray(data, dtype=np.int16).reshape(height, width), (ox, oy), resolution)

    def inside(self, x: float, y: float) -> bool:
        c = (x - self.origin[0]) / self.resolution
        r = (y - self.origin[1]) / self.resolution
        return 0 <= c < self.data.shape[1] and 0 <= r < self.data.shape[0]

    def cost(self, x: float, y: float) -> int:
        """Cost at (x, y); -2 outside the grid."""
        if not self.inside(x, y):
            return -2
        return int(self.data[int((y - self.origin[1]) / self.resolution),
                             int((x - self.origin[0]) / self.resolution)])


@dataclass
class GoalParams:
    max_distance: float = 9.0
    cost_threshold: int = 50          # >= : not acceptable as a goal (Nav2 128/254)
    search_radius: float = 0.5
    allow_unknown: bool = True


@dataclass
class GoalCheck:
    ok: bool
    xy: Tuple[float, float]
    partial: bool = False
    adjusted: bool = False
    reason: str = ""


def _acceptable(grid: CostGrid, x: float, y: float, p: GoalParams) -> bool:
    c = grid.cost(x, y)
    if c == -2:
        return False
    if c == -1:
        return p.allow_unknown
    return c < p.cost_threshold


def sanitize_goal(grid: CostGrid, robot: Tuple[float, float], goal: Tuple[float, float],
                  p: Optional[GoalParams] = None) -> GoalCheck:
    p = p or GoalParams()
    rx, ry = robot
    gx, gy = goal
    partial = False
    d = math.hypot(gx - rx, gy - ry)
    if d > p.max_distance:
        k = p.max_distance / d
        gx, gy = rx + k * (gx - rx), ry + k * (gy - ry)
        partial = True
    if not grid.inside(gx, gy):
        return GoalCheck(False, (gx, gy), partial, False, "goal_out_of_range")
    if _acceptable(grid, gx, gy, p):
        return GoalCheck(True, (gx, gy), partial, False, "")
    # 1) back along goal -> robot, at most search_radius
    d = math.hypot(gx - rx, gy - ry)
    step = grid.resolution
    if d > 1e-6:
        ux, uy = (rx - gx) / d, (ry - gy) / d
        s = step
        while s <= min(p.search_radius, d) + 1e-9:
            x, y = gx + ux * s, gy + uy * s
            if _acceptable(grid, x, y, p):
                return GoalCheck(True, (x, y), partial, True, "")
            s += step
    # 2) nearest acceptable cell within search_radius
    best, best_d = None, math.inf
    n = int(math.ceil(p.search_radius / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            dd = math.hypot(i * step, j * step)
            if dd > p.search_radius or dd >= best_d:
                continue
            x, y = gx + i * step, gy + j * step
            if _acceptable(grid, x, y, p):
                best, best_d = (x, y), dd
    if best is not None:
        return GoalCheck(True, best, partial, True, "")
    return GoalCheck(False, (gx, gy), partial, False, "goal_in_obstacle")


SECTOR_ANGLES = (-90, -60, -30, 0, 30, 60, 90)


def free_sectors(grid: CostGrid, pose: Tuple[float, float, float], angles_deg: Sequence[int] = SECTOR_ANGLES,
                 range_m: float = 1.5, half_width: float = 0.30, threshold: int = 99,
                 start: float = 0.35) -> Dict[int, bool]:
    """Whether the corridor of `half_width` towards each direction (relative to the heading) is free
    for `range_m`; unknown cells count as free (rolling local costmap)."""
    x, y, th = pose
    out = {}
    step = grid.resolution
    for a in angles_deg:
        ang = th + math.radians(a)
        c, s = math.cos(ang), math.sin(ang)
        free = True
        r = start
        while r <= range_m + 1e-9 and free:
            for lat in (-half_width, 0.0, half_width):
                px, py = x + c * r - s * lat, y + s * r + c * lat
                v = grid.cost(px, py)
                if v >= threshold:
                    free = False
                    break
            r += step
        out[int(a)] = free
    return out


# Nav2 error codes (Jazzy): FollowPath 1xx, ComputePathToPose 2xx.
NAV2_REASONS = {
    104: "blocked", 105: "blocked", 106: "blocked", 107: "timeout",
    102: "tf_error", 202: "tf_error",
    203: "start_out_of_range", 204: "goal_out_of_range", 205: "blocked", 206: "goal_in_obstacle",
    207: "no_path", 208: "no_path",
}


def nav2_reason(code: int, msg: str = "") -> str:
    if code in NAV2_REASONS:
        return NAV2_REASONS[code]
    if 100 <= code < 200:
        return "blocked"
    if 200 <= code < 300:
        return "no_path"
    return f"nav2_error_{code}" if code else (msg or "nav2_failed")


def relative_to_global(t_global_base: np.ndarray, x: float, y: float, yaw: float) -> Tuple[float, float, float]:
    """(x, y, yaw) in a base frame (4x4 global <- base) -> global (x, y, yaw)."""
    c, s = t_global_base[0, 0], t_global_base[1, 0]
    th = math.atan2(s, c)
    gx = t_global_base[0, 3] + math.cos(th) * x - math.sin(th) * y
    gy = t_global_base[1, 3] + math.sin(th) * x + math.cos(th) * y
    return gx, gy, math.atan2(math.sin(th + yaw), math.cos(th + yaw))
