"""Map quality metrics without scipy: voxel-hash nearest neighbours, point-to-point ICP,
map-to-map consistency (repeat mapping of the same area from the same start marker)."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .pcd import voxel_downsample


class NearestGrid:
    """Approximate nearest neighbour within `cell` (exact if the true neighbour is closer than cell)."""

    def __init__(self, points: np.ndarray, cell: float):
        self.pts = np.asarray(points, dtype=np.float64)
        self.cell = cell
        keys = np.floor(self.pts / cell).astype(np.int64)
        self.buckets: Dict[Tuple[int, int, int], np.ndarray] = {}
        order = np.lexsort(keys.T[::-1])
        sk = keys[order]
        brk = np.flatnonzero(np.any(np.diff(sk, axis=0) != 0, axis=1)) + 1
        for chunk in np.split(order, brk):
            self.buckets[tuple(keys[chunk[0]])] = chunk

    def query(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """(distance, index) per query point; inf / -1 when nothing within ~cell."""
        q = np.asarray(q, dtype=np.float64)
        dist = np.full(len(q), np.inf)
        idx = np.full(len(q), -1, np.int64)
        keys = np.floor(q / self.cell).astype(np.int64)
        offs = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)]
        for n, (key, p) in enumerate(zip(map(tuple, keys), q)):
            cand = [self.buckets[(key[0] + a, key[1] + b, key[2] + c)] for a, b, c in offs
                    if (key[0] + a, key[1] + b, key[2] + c) in self.buckets]
            if not cand:
                continue
            c = np.concatenate(cand)
            d = np.linalg.norm(self.pts[c] - p, axis=1)
            m = int(np.argmin(d))
            dist[n], idx[n] = d[m], c[m]
        return dist, idx


def best_rigid(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """T (4x4) minimising |T a - b| (Kabsch)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[2] *= -1
        r = vt.T @ u.T
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = cb - r @ ca
    return t


@dataclass
class Consistency:
    t_ab: np.ndarray            # aligns map b onto map a
    rmse: float
    p90: float
    inlier_ratio: float
    offset_m: float             # translation of t_ab
    offset_deg: float           # yaw of t_ab


def map_consistency(a: np.ndarray, b: np.ndarray, voxel: float = 0.1, max_corr: float = 0.5,
                    iters: int = 20, max_points: int = 15000, seed: int = 0) -> Consistency:
    rng = np.random.default_rng(seed)
    a_d = voxel_downsample(np.asarray(a, dtype=np.float64), 0.5 * voxel)   # denser target
    b_d = voxel_downsample(np.asarray(b, dtype=np.float64), voxel)
    if len(b_d) > max_points:
        b_d = b_d[rng.choice(len(b_d), max_points, replace=False)]
    grid = NearestGrid(a_d, max_corr)
    t = np.eye(4)
    for corr in (max_corr, 0.4 * max_corr):                 # coarse, then tight correspondences
        for _ in range(iters):
            bt = b_d @ t[:3, :3].T + t[:3, 3]
            d, i = grid.query(bt)
            ok = d < corr
            if ok.sum() < 10:
                break
            step = best_rigid(bt[ok], a_d[i[ok]])
            t = step @ t
            if np.linalg.norm(step[:3, 3]) < 1e-4 and abs(math.atan2(step[1, 0], step[0, 0])) < 1e-5:
                break
    bt = b_d @ t[:3, :3].T + t[:3, 3]
    d, _ = grid.query(bt)
    ok = d < max_corr
    dd = d[ok] if ok.any() else np.array([np.inf])
    return Consistency(t, float(np.sqrt(np.mean(dd ** 2))), float(np.percentile(dd, 90)), float(ok.mean()),
                       float(np.linalg.norm(t[:2, 3])), float(math.degrees(math.atan2(t[1, 0], t[0, 0]))))
