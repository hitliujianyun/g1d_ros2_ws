"""Global relocalization without an initial guess (design doc 5.4, `auto` mode).

The current scan (base_link frame, obstacle points) is scored against the prior map for candidate
base poses (x, y, yaw): score = fraction of scan points that fall into occupied (dilated) map
voxels. Candidates are seeded at the mapping trajectory (the robot is normally somewhere it has
been driven before), searched coarse-to-fine; the best pose is handed to small_gicp
(/initialpose) for the precise alignment.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

Pose2 = Tuple[float, float, float]


class VoxelMap:
    """Dense boolean occupancy of map points, dilated by `dilate` voxels."""

    def __init__(self, points: np.ndarray, voxel: float = 0.2, dilate: int = 1,
                 z_range: Tuple[float, float] = (0.08, 2.0)):
        pts = np.asarray(points, dtype=np.float64)
        pts = pts[(pts[:, 2] >= z_range[0]) & (pts[:, 2] <= z_range[1])]
        if len(pts) == 0:
            raise ValueError("no map points in the height range")
        self.voxel = voxel
        self.origin = pts.min(axis=0) - (dilate + 1) * voxel
        idx = np.floor((pts - self.origin) / voxel).astype(np.int64)
        self.shape = tuple(int(v) for v in idx.max(axis=0) + dilate + 2)
        occ = np.zeros(self.shape, bool)
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        for _ in range(dilate):
            g = occ.copy()
            for ax in range(3):
                g |= np.roll(occ, 1, axis=ax) | np.roll(occ, -1, axis=ax)
            occ = g
        self.occ = occ

    def hits(self, pts: np.ndarray) -> np.ndarray:
        """Boolean hit per point; pts (..., 3) in the map frame."""
        idx = np.floor((pts - self.origin) / self.voxel).astype(np.int64)
        ok = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=-1)
        out = np.zeros(idx.shape[:-1], bool)
        sel = idx[ok]
        out[ok] = self.occ[sel[:, 0], sel[:, 1], sel[:, 2]]
        return out


def transform_scan(scan: np.ndarray, poses: np.ndarray) -> np.ndarray:
    """scan (N, 3) base frame, poses (K, 3) [x, y, yaw] -> (K, N, 3) map frame."""
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    x = c[:, None] * scan[None, :, 0] - s[:, None] * scan[None, :, 1] + poses[:, 0, None]
    y = s[:, None] * scan[None, :, 0] + c[:, None] * scan[None, :, 1] + poses[:, 1, None]
    z = np.broadcast_to(scan[None, :, 2], x.shape)
    return np.stack([x, y, z], axis=-1)


def score(vmap: VoxelMap, scan: np.ndarray, poses: np.ndarray, batch: int = 256) -> np.ndarray:
    out = np.empty(len(poses))
    for i in range(0, len(poses), batch):
        out[i:i + batch] = vmap.hits(transform_scan(scan, poses[i:i + batch])).mean(axis=1)
    return out


@dataclass
class SearchParams:
    seed_spacing: float = 0.5       # m between trajectory seeds
    xy_offsets: float = 0.25        # +- offset around each seed (3 x 3 grid incl. 0)
    yaw_step_deg: float = 10.0
    refine_steps: Sequence[Tuple[float, float]] = ((0.1, 3.0), (0.05, 1.0))   # (m, deg)
    top_k: int = 5
    max_scan_points: int = 1500
    min_score: float = 0.6          # below this the result is reported as not found


@dataclass
class SearchResult:
    found: bool
    pose: Pose2
    score: float
    second_score: float             # best score of a clearly different pose (ambiguity check)
    candidates: int


def subsample_seeds(traj: np.ndarray, spacing: float) -> np.ndarray:
    seeds, last = [], None
    for x, y in np.asarray(traj)[:, :2]:
        if last is None or math.hypot(x - last[0], y - last[1]) >= spacing:
            seeds.append((x, y))
            last = (x, y)
    return np.array(seeds).reshape(-1, 2)


def global_search(vmap: VoxelMap, scan: np.ndarray, seeds_xy: np.ndarray,
                  p: Optional[SearchParams] = None, rng: Optional[np.random.Generator] = None) -> SearchResult:
    p = p or SearchParams()
    rng = rng or np.random.default_rng(0)
    scan = np.asarray(scan, dtype=np.float64)
    if len(scan) > p.max_scan_points:
        scan = scan[rng.choice(len(scan), p.max_scan_points, replace=False)]
    if len(scan) == 0 or len(seeds_xy) == 0:
        return SearchResult(False, (0.0, 0.0, 0.0), 0.0, 0.0, 0)
    offs = np.array([(dx, dy) for dx in (-p.xy_offsets, 0.0, p.xy_offsets)
                     for dy in (-p.xy_offsets, 0.0, p.xy_offsets)])
    xy = (seeds_xy[:, None, :] + offs[None, :, :]).reshape(-1, 2)
    yaws = np.deg2rad(np.arange(0.0, 360.0, p.yaw_step_deg))
    cand = np.column_stack([np.repeat(xy, len(yaws), axis=0), np.tile(yaws, len(xy))])
    s = score(vmap, scan, cand)
    n_total = len(cand)
    order = np.argsort(-s)
    best = []
    for i in order:                                  # top-k distinct starts
        if all(math.hypot(cand[i, 0] - b[0], cand[i, 1] - b[1]) > 0.5 or
               abs(math.remainder(cand[i, 2] - b[2], 2 * math.pi)) > math.radians(20) for b in best):
            best.append(cand[i].copy())
        if len(best) >= p.top_k:
            break
    refined = []
    for b in best:
        cur, cur_s = b, None
        for step_m, step_deg in p.refine_steps:
            d = np.array([(dx, dy, dth) for dx in (-step_m, 0, step_m) for dy in (-step_m, 0, step_m)
                          for dth in np.deg2rad([-step_deg, 0, step_deg])])
            for _ in range(10):                      # hill climb
                local = cur[None, :] + d
                ls = score(vmap, scan, local)
                n_total += len(local)
                j = int(np.argmax(ls))
                if cur_s is not None and ls[j] <= cur_s + 1e-9:
                    break
                cur, cur_s = local[j], ls[j]
        refined.append((cur_s if cur_s is not None else 0.0, cur))
    refined.sort(key=lambda t: -t[0])
    top_s, top = refined[0]
    second = next((rs for rs, r in refined[1:]
                   if math.hypot(r[0] - top[0], r[1] - top[1]) > 0.5 or
                   abs(math.remainder(r[2] - top[2], 2 * math.pi)) > math.radians(20)), 0.0)
    pose = (float(top[0]), float(top[1]), float(math.remainder(top[2], 2 * math.pi)))
    return SearchResult(bool(top_s >= p.min_score), pose, float(top_s), float(second), n_total)
