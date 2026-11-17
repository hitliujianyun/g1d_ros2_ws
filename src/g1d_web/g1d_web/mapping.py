"""Accumulated LIO map of the current run (voxel filtered, odom frame), trajectory and exports.

Exports (maps/<time>/): map.pcd (binary float32 xyz), grid.png + grid.yaml (ROS map_server format,
occupied = map points between grid_z_min and grid_z_max), trajectory.csv, costmap.png (last global
costmap, if any), meta.json.
"""
from __future__ import annotations

import json
import math
import os
import struct
import threading
import time
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1


def voxel_keys(points: np.ndarray, voxel: float) -> np.ndarray:
    k = np.floor(points / voxel).astype(np.int64) + _OFF
    k &= _MASK
    return (k[:, 0] << (2 * _BITS)) | (k[:, 1] << _BITS) | k[:, 2]


class MapBuilder:
    """Thread-safe voxel map: add_cloud() returns only the points of newly occupied voxels."""

    def __init__(self, voxel: float = 0.05, max_points: int = 3_000_000):
        self.lock = threading.Lock()
        self.max_points = int(max_points)
        self.reset(voxel)

    def reset(self, voxel: Optional[float] = None) -> None:
        with self.lock:
            if voxel is not None:
                if not (0.01 <= voxel <= 1.0):
                    raise ValueError("voxel must be in [0.01, 1.0] m")
                self.voxel = float(voxel)
            self._keys: set = set()
            self._chunks: List[np.ndarray] = []
            self._cache: Optional[np.ndarray] = None
            self.n_points = 0
            self.frames = 0
            self.full = False
            self.trajectory: List[Tuple[float, float, float, float]] = []
            self.started = time.time()

    def add_cloud(self, points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        with self.lock:
            self.frames += 1
            if pts.shape[0] == 0 or self.full:
                return np.zeros((0, 3), np.float32)
            keys = voxel_keys(pts, self.voxel)
            uniq, idx = np.unique(keys, return_index=True)
            known = self._keys
            new_idx = [i for k, i in zip(uniq.tolist(), idx.tolist()) if k not in known]
            if not new_idx:
                return np.zeros((0, 3), np.float32)
            room = self.max_points - self.n_points
            if len(new_idx) > room:
                new_idx = new_idx[:room]
                self.full = True
            sel = np.asarray(new_idx, dtype=np.int64)
            known.update(keys[sel].tolist())
            new_pts = np.ascontiguousarray(pts[sel])
            self._chunks.append(new_pts)
            self._cache = None
            self.n_points += new_pts.shape[0]
            return new_pts

    def points(self) -> np.ndarray:
        with self.lock:
            if self._cache is None:
                self._cache = (np.concatenate(self._chunks) if self._chunks
                               else np.zeros((0, 3), np.float32))
                self._chunks = [self._cache] if self._chunks else []
            return self._cache

    def add_pose(self, t: float, x: float, y: float, yaw: float, min_step: float = 0.05) -> bool:
        with self.lock:
            if self.trajectory:
                lt, lx, ly, lyaw = self.trajectory[-1]
                if (math.hypot(x - lx, y - ly) < min_step and abs(yaw - lyaw) < 0.1
                        and t - lt < 5.0):
                    return False
            self.trajectory.append((t, x, y, yaw))
            return True

    def stats(self) -> Dict:
        with self.lock:
            return {"points": self.n_points, "frames": self.frames, "voxel": self.voxel,
                    "full": self.full, "trajectory": len(self.trajectory),
                    "started": self.started}

    def occupancy(self, resolution: float = 0.05, z_min: float = 0.10, z_max: float = 1.80,
                  floor_z: float = 0.08, margin: float = 0.5):
        """2D grid from the map: returns (img uint8 [h, w] with row 0 = +y edge, origin_xy) using
        map_server values occupied 0, free 254, unknown 205; None if the map is empty."""
        pts = self.points()
        with self.lock:
            traj = np.array([(x, y) for _, x, y, _ in self.trajectory], np.float64).reshape(-1, 2)
        if pts.shape[0] == 0 and traj.shape[0] == 0:
            return None
        xy = np.vstack([pts[:, :2].astype(np.float64), traj]) if traj.size else pts[:, :2]
        x0, y0 = xy.min(axis=0) - margin
        x1, y1 = xy.max(axis=0) + margin
        w = int(math.ceil((x1 - x0) / resolution))
        h = int(math.ceil((y1 - y0) / resolution))
        grid = np.full((h, w), 205, np.uint8)

        def cells(p):
            ix = np.clip(((p[:, 0] - x0) / resolution).astype(np.int64), 0, w - 1)
            iy = np.clip(((p[:, 1] - y0) / resolution).astype(np.int64), 0, h - 1)
            return iy, ix

        z = pts[:, 2]
        floor = pts[np.abs(z) < floor_z]
        if floor.shape[0]:
            grid[cells(floor)] = 254
        if traj.shape[0]:
            grid[cells(traj)] = 254
        # Sparse floor returns leave single unknown cells inside free space: one dilation pass.
        free = grid == 254
        grown = free.copy()
        grown[1:, :] |= free[:-1, :]
        grown[:-1, :] |= free[1:, :]
        grown[:, 1:] |= free[:, :-1]
        grown[:, :-1] |= free[:, 1:]
        grid[grown] = 254
        occ = pts[(z >= z_min) & (z <= z_max)]
        if occ.shape[0]:
            grid[cells(occ)] = 0
        return np.flipud(grid), (float(x0), float(y0))


# ------------------------------------------------------------------------------------- writers
def write_pcd(path: str, pts: np.ndarray) -> None:
    pts = np.ascontiguousarray(pts, dtype="<f4").reshape(-1, 3)
    n = pts.shape[0]
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
              f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {n}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(pts.tobytes())


def read_pcd_binary(path: str) -> np.ndarray:
    with open(path, "rb") as f:
        data = f.read()
    end = data.index(b"DATA binary\n") + len(b"DATA binary\n")
    n = int([ln for ln in data[:end].decode().splitlines() if ln.startswith("POINTS")][0].split()[1])
    return np.frombuffer(data[end:end + 12 * n], dtype="<f4").reshape(n, 3)


def png_gray(img: np.ndarray) -> bytes:
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape
    raw = b"".join(b"\x00" + img[r].tobytes() for r in range(h))

    def chunk(tag: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + tag + body
                + struct.pack(">I", zlib.crc32(tag + body) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def write_png_gray(path: str, img: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(png_gray(img))


def write_map_yaml(path: str, image: str, resolution: float, origin: Sequence[float]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"image: {image}\nmode: trinary\nresolution: {resolution}\n"
                f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\nnegate: 0\n"
                "occupied_thresh: 0.65\nfree_thresh: 0.25\n")


def costmap_image(width: int, height: int, data: np.ndarray) -> np.ndarray:
    """ROS OccupancyGrid values (0..100, 255 unknown) -> grey image, row 0 = +y edge."""
    d = np.asarray(data, dtype=np.uint8).reshape(height, width)
    img = np.where(d == 255, 205, 254 - (d.astype(np.int32) * 254) // 100).astype(np.uint8)
    return np.flipud(img)


def export_map(builder: MapBuilder, root: str, *, resolution: float = 0.05,
               z_min: float = 0.10, z_max: float = 1.80, costmap: Optional[Dict] = None,
               extra_meta: Optional[Dict] = None) -> Dict:
    name = time.strftime("%Y%m%d-%H%M%S")
    out = os.path.join(root, name)
    os.makedirs(out, exist_ok=True)
    pts = builder.points()
    files = []
    write_pcd(os.path.join(out, "map.pcd"), pts)
    files.append("map.pcd")
    occ = builder.occupancy(resolution, z_min, z_max)
    meta = {"name": name, "created": time.time(), **builder.stats(),
            "grid_resolution": resolution, "grid_z": [z_min, z_max], "frame": "odom"}
    if occ is not None:
        img, origin = occ
        write_png_gray(os.path.join(out, "grid.png"), img)
        write_map_yaml(os.path.join(out, "grid.yaml"), "grid.png", resolution, origin)
        files += ["grid.png", "grid.yaml"]
        meta["grid_origin"] = origin
        meta["grid_size"] = [int(img.shape[1]), int(img.shape[0])]
    with builder.lock:
        traj = list(builder.trajectory)
    with open(os.path.join(out, "trajectory.csv"), "w", encoding="utf-8") as f:
        f.write("t,x,y,yaw\n")
        for t, x, y, yaw in traj:
            f.write(f"{t:.3f},{x:.4f},{y:.4f},{yaw:.4f}\n")
    files.append("trajectory.csv")
    if costmap:
        write_png_gray(os.path.join(out, "costmap.png"),
                       costmap_image(costmap["width"], costmap["height"], costmap["data"]))
        write_map_yaml(os.path.join(out, "costmap.yaml"), "costmap.png", costmap["resolution"],
                       costmap["origin"])
        files += ["costmap.png", "costmap.yaml"]
    meta.update(extra_meta or {})
    meta["files"] = files + ["meta.json"]
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    return meta


def list_exports(root: str) -> List[Dict]:
    out = []
    if not os.path.isdir(root):
        return out
    for name in sorted(os.listdir(root), reverse=True):
        p = os.path.join(root, name, "meta.json")
        if os.path.isfile(p):
            try:
                with open(p, encoding="utf-8") as f:
                    m = json.load(f)
                out.append({"name": name, "points": m.get("points"), "files": m.get("files", []),
                            "created": m.get("created")})
            except (OSError, ValueError):
                continue
    return out
