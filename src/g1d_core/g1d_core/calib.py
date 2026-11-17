"""Mid-360S extrinsic helpers: floor-plane fit (roll, pitch, height) and yaw against the chassis
scan. Conventions match src/common geometry.hpp (rpy = ZYX, R = Rz(yaw) Ry(pitch) Rx(roll))."""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def matrix_to_rpy(r: np.ndarray) -> Tuple[float, float, float]:
    return (math.atan2(r[2, 1], r[2, 2]), math.asin(-max(-1.0, min(1.0, r[2, 0]))),
            math.atan2(r[1, 0], r[0, 0]))


def align_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Minimal rotation Q with Q a = b (unit vectors)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -1.0 + 1e-9:
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def fit_floor(pts: np.ndarray, up_hint: np.ndarray, *, max_tilt_deg: float = 25.0,
              tol: float = 0.02, iters: int = 400, min_height: float = 0.3,
              seed: int = 0) -> Optional[Tuple[np.ndarray, float, int]]:
    """RANSAC floor plane in the sensor frame: (n, h, inliers) with n the floor normal pointing
    up (towards the sensor) and h the sensor height above the floor (n . p + h = 0 on the floor).
    Only planes within max_tilt_deg of up_hint and at least min_height below the sensor count
    (rejects walls and the ceiling)."""
    pts = np.asarray(pts, dtype=np.float64)
    if pts.shape[0] < 50:
        return None
    up_hint = up_hint / np.linalg.norm(up_hint)
    cos_tilt = math.cos(math.radians(max_tilt_deg))
    rng = np.random.default_rng(seed)
    best, best_count = None, 0
    for _ in range(iters):
        a, b, c = pts[rng.choice(pts.shape[0], 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if np.dot(n, up_hint) < 0:
            n = -n
        if np.dot(n, up_hint) < cos_tilt:
            continue
        h = -float(np.dot(n, a))
        if h < min_height:
            continue
        count = int((np.abs(pts @ n + h) < tol).sum())
        if count > best_count:
            best, best_count = (n, h), count
    if best is None:
        return None
    n, h = best
    inl = pts[np.abs(pts @ n + h) < tol]
    centroid = inl.mean(axis=0)
    _, _, vt = np.linalg.svd(inl - centroid, full_matrices=False)
    n = vt[2] if np.dot(vt[2], up_hint) > 0 else -vt[2]
    h = -float(np.dot(n, centroid))
    return n, h, int(inl.shape[0])


def level_rotation(r_cfg: np.ndarray, up_measured: np.ndarray) -> np.ndarray:
    """Base->sensor rotation closest to r_cfg (same yaw) whose floor-up direction in the sensor
    frame equals up_measured."""
    up_cfg = r_cfg.T @ np.array([0.0, 0.0, 1.0])
    q = align_vectors(up_cfg, up_measured)
    return r_cfg @ q.T


def estimate_yaw(band_xy: np.ndarray, scan_xy: np.ndarray, pivot_xy: np.ndarray,
                 search_deg: float = 10.0, step_deg: float = 0.1, cap: float = 0.15,
                 max_points: int = 1500, seed: int = 0) -> Tuple[float, float]:
    """Yaw correction (rad) rotating LiDAR points (base frame, a horizontal slice at the chassis
    LiDAR height) about pivot_xy onto the chassis scan points; returns (yaw, mean residual m)."""
    rng = np.random.default_rng(seed)
    if band_xy.shape[0] > max_points:
        band_xy = band_xy[rng.choice(band_xy.shape[0], max_points, replace=False)]
    rel = band_xy - pivot_xy
    best = (0.0, math.inf)
    for deg in np.arange(-search_deg, search_deg + 1e-9, step_deg):
        a = math.radians(deg)
        c, s = math.cos(a), math.sin(a)
        p = np.c_[c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]] + pivot_xy
        d2 = ((p[:, None, :] - scan_xy[None, :, :]) ** 2).sum(axis=2).min(axis=1)
        score = float(np.minimum(np.sqrt(d2), cap).mean())
        if score < best[1]:
            best = (a, score)
    return best
