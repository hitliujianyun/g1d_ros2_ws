"""Minimal PCD reader / writer (ascii and binary; numpy only) for map files written by PCL."""
from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

_TYPES = {("F", 4): np.float32, ("F", 8): np.float64, ("U", 1): np.uint8, ("U", 2): np.uint16,
          ("U", 4): np.uint32, ("I", 1): np.int8, ("I", 2): np.int16, ("I", 4): np.int32}


def read_pcd(path: str) -> Dict[str, np.ndarray]:
    """Fields of a PCD file as 1-D arrays (count > 1 fields are skipped)."""
    with open(path, "rb") as f:
        header: Dict[str, list] = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path}: truncated header")
            parts = line.decode("ascii", "replace").strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            header[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                break
        body = f.read()
    names = header["FIELDS"]
    sizes = [int(s) for s in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(c) for c in header.get("COUNT", ["1"] * len(names))]
    n = int(header["POINTS"][0])
    mode = header["DATA"][0].lower()
    dtype = np.dtype([(f"{nm}_{i}" if c > 1 else nm, _TYPES[(t, s)], (c,) if c > 1 else ())
                      for i, (nm, s, t, c) in enumerate(zip(names, sizes, types, counts))])
    if mode == "binary":
        arr = np.frombuffer(body[: n * dtype.itemsize], dtype=dtype, count=n)
    elif mode == "ascii":
        rows = np.loadtxt(body.decode("ascii").splitlines(), ndmin=2) if n else np.zeros((0, len(names)))
        arr = np.zeros(n, dtype=dtype)
        col = 0
        for nm, c in zip(dtype.names, counts):
            arr[nm] = rows[:, col] if c == 1 else rows[:, col:col + c]
            col += c
    else:
        raise ValueError(f"{path}: unsupported PCD DATA mode {mode!r}")
    return {nm: np.array(arr[nm]) for nm, c in zip(dtype.names, counts) if c == 1 and nm != "_"}


def xyz(fields: Dict[str, np.ndarray]) -> np.ndarray:
    return np.stack([fields["x"], fields["y"], fields["z"]], axis=1).astype(np.float64)


def write_pcd(path: str, points: np.ndarray, intensity: np.ndarray = None) -> None:
    """Binary PCD with float32 x y z [intensity]."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    names = ["x", "y", "z"] + (["intensity"] if intensity is not None else [])
    data = np.zeros(len(pts), dtype=[(nm, np.float32) for nm in names])
    data["x"], data["y"], data["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
    if intensity is not None:
        data["intensity"] = np.asarray(intensity, dtype=np.float32)
    k = len(names)
    header = (f"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS {' '.join(names)}\n"
              f"SIZE {' '.join(['4'] * k)}\nTYPE {' '.join(['F'] * k)}\nCOUNT {' '.join(['1'] * k)}\n"
              f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data.tobytes())


def voxel_downsample(points: np.ndarray, leaf: float) -> np.ndarray:
    """Centroid per occupied voxel."""
    if len(points) == 0 or leaf <= 0:
        return np.asarray(points, dtype=np.float64)
    keys = np.floor(points / leaf).astype(np.int64)
    _, inv_idx, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv_idx = inv_idx.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inv_idx, points)
    return sums / counts[:, None]


def transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.asarray(points, dtype=np.float64) @ t[:3, :3].T + t[:3, 3]


def read_poses_txt(path: str) -> Tuple[list, np.ndarray]:
    """FASTLIO2_ROS2 pgo poses.txt: `name x y z qw qx qy qz` per line -> (names, (N, 7) x y z qx qy qz qw)."""
    names, rows = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            p = line.split()
            if len(p) != 8:
                continue
            names.append(p[0])
            x, y, z, qw, qx, qy, qz = (float(v) for v in p[1:])
            rows.append((x, y, z, qx, qy, qz, qw))
    return names, np.array(rows).reshape(-1, 7)
