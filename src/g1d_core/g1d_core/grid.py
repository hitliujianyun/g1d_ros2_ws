"""2D occupancy grid from a 3D map (design doc 5.3): height-band projection.

Map frame: z up, floor at z = 0 (the mapping session's odom frame, anchored at base_link on the floor).
  occupied: >= min_hits points with z in [z_min, z_max] (overhangs such as table tops included)
  free:     floor points (z in ground_band) and not occupied; then grown by `free_grow` cells into
            unknown cells (Mid-360 floor returns are sparse far away)
  unknown:  everything else
Output follows the Nav2 map_server format (trinary PGM + YAML).
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


@dataclass
class GridParams:
    resolution: float = 0.05
    z_min: float = 0.08
    z_max: float = 1.85
    ground_min: float = -0.10
    ground_max: float = 0.05
    min_hits: int = 2
    free_grow: int = 1
    margin: float = 0.5


@dataclass
class Grid:
    data: np.ndarray            # (H, W) int8, row 0 = lowest y
    origin: Tuple[float, float]  # world xy of cell (0, 0) corner
    resolution: float

    def cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(np.floor((y - self.origin[1]) / self.resolution)),
                int(np.floor((x - self.origin[0]) / self.resolution)))

    def value_at(self, x: float, y: float) -> int:
        r, c = self.cell(x, y)
        if 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]:
            return int(self.data[r, c])
        return UNKNOWN


def _grow(mask: np.ndarray, n: int) -> np.ndarray:
    out = mask.copy()
    for _ in range(n):
        g = out.copy()
        g[1:, :] |= out[:-1, :]
        g[:-1, :] |= out[1:, :]
        g[:, 1:] |= out[:, :-1]
        g[:, :-1] |= out[:, 1:]
        out = g
    return out


def build_grid(points: np.ndarray, p: Optional[GridParams] = None,
               bounds: Optional[Tuple[float, float, float, float]] = None) -> Grid:
    p = p or GridParams()
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if bounds is None:
        if len(pts) == 0:
            raise ValueError("empty map")
        lo, hi = pts[:, :2].min(axis=0) - p.margin, pts[:, :2].max(axis=0) + p.margin
        bounds = (lo[0], lo[1], hi[0], hi[1])
    x0, y0, x1, y1 = bounds
    w = int(np.ceil((x1 - x0) / p.resolution))
    h = int(np.ceil((y1 - y0) / p.resolution))
    col = np.floor((pts[:, 0] - x0) / p.resolution).astype(np.int64)
    row = np.floor((pts[:, 1] - y0) / p.resolution).astype(np.int64)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    z = pts[:, 2]
    occ_hits = np.zeros((h, w), np.int32)
    band = inside & (z >= p.z_min) & (z <= p.z_max)
    np.add.at(occ_hits, (row[band], col[band]), 1)
    ground = np.zeros((h, w), bool)
    g = inside & (z >= p.ground_min) & (z <= p.ground_max)
    ground[row[g], col[g]] = True
    occupied = occ_hits >= p.min_hits
    free = ground & ~occupied
    if p.free_grow > 0:
        free = _grow(free, p.free_grow) & ~occupied
    data = np.full((h, w), UNKNOWN, np.int8)
    data[free] = FREE
    data[occupied] = OCCUPIED
    return Grid(data, (x0, y0), p.resolution)


def save_map(grid: Grid, directory: str, name: str = "map") -> str:
    """Writes <name>.pgm + <name>.yaml (map_server trinary); returns the yaml path."""
    os.makedirs(directory, exist_ok=True)
    img = np.full(grid.data.shape, 205, np.uint8)          # unknown
    img[grid.data == FREE] = 254
    img[grid.data == OCCUPIED] = 0
    img = img[::-1, :]                                      # PGM row 0 = top = highest y
    pgm = os.path.join(directory, f"{name}.pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii"))
        f.write(img.tobytes())
    meta = {"image": f"{name}.pgm", "mode": "trinary", "resolution": float(grid.resolution),
            "origin": [float(grid.origin[0]), float(grid.origin[1]), 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.196}   # 205 (unknown) -> 0.19608: not free
    path = os.path.join(directory, f"{name}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return path


def load_map(yaml_path: str) -> Grid:
    with open(yaml_path, encoding="utf-8") as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb") as f:
        raw = f.read()
    tokens, idx = [], 0
    while len(tokens) < 4:                                  # P5 w h maxval (comments skipped)
        while raw[idx:idx + 1].isspace():
            idx += 1
        if raw[idx:idx + 1] == b"#":
            idx = raw.index(b"\n", idx) + 1
            continue
        end = idx
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[idx:end])
        idx = end
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(raw[idx + 1: idx + 1 + w * h], np.uint8).reshape(h, w)[::-1, :]
    occ = (255 - img) / 255.0
    data = np.full((h, w), UNKNOWN, np.int8)
    data[occ > meta["occupied_thresh"]] = OCCUPIED
    data[occ < meta["free_thresh"]] = FREE
    return Grid(data, (meta["origin"][0], meta["origin"][1]), meta["resolution"])


def rasterize_boxes(boxes: Sequence[Sequence[float]], like: Grid, z_min: float = 0.08,
                    z_max: float = 1.85) -> np.ndarray:
    """Ground-truth occupancy (bool) of 3D boxes [x0, y0, z0, x1, y1, z1] overlapping the band."""
    h, w = like.data.shape
    occ = np.zeros((h, w), bool)
    res, (ox, oy) = like.resolution, like.origin
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    for b in boxes:
        if b[2] > z_max or b[5] < z_min:
            continue
        cx = (xs >= b[0]) & (xs <= b[3])
        cy = (ys >= b[1]) & (ys <= b[4])
        occ |= np.outer(cy, cx)
    return occ


def occupancy_scores(grid: Grid, truth: np.ndarray, tol_cells: int = 1) -> Tuple[float, float]:
    """(precision, recall) of occupied cells with a tolerance of `tol_cells`; recall only counts
    truth cells on box outlines that could be observed (cells next to a non-truth cell)."""
    occ = grid.data == OCCUPIED
    truth_near = _grow(truth, tol_cells)
    precision = float((occ & truth_near).sum()) / max(1, int(occ.sum()))
    outline = truth & _grow(~truth, 1)
    occ_near = _grow(occ, tol_cells)
    recall = float((outline & occ_near).sum()) / max(1, int(outline.sum()))
    return precision, recall
