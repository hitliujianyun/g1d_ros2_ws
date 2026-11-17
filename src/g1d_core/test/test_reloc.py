import math
import os

import numpy as np
import pytest

from g1d_core.reloc import SearchParams, VoxelMap, global_search, subsample_seeds, transform_scan
from g1d_core.sim2d import load_world


def surface_points(boxes, step=0.05):
    pts = []
    for x0, y0, z0, x1, y1, z1 in boxes:
        for z in np.arange(max(z0, 0.0), z1 + 1e-9, step):
            xs, ys = np.arange(x0, x1 + 1e-9, step), np.arange(y0, y1 + 1e-9, step)
            pts += [(x, y, z) for x in xs for y in (y0, y1)] + [(x, y, z) for y in ys for x in (x0, x1)]
    return np.array(pts)


def visible_scan(boxes, pose, max_range=6.0, n=4000, seed=1):
    """Obstacle points around a base pose (crude visibility: nearest return per azimuth ray)."""
    rng = np.random.default_rng(seed)
    pts = surface_points(boxes, 0.04)
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    d = pts[:, :2] - [x, y]
    base = np.column_stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], pts[:, 2]])
    r = np.hypot(base[:, 0], base[:, 1])
    az = np.round(np.degrees(np.arctan2(base[:, 1], base[:, 0])) * 2).astype(int)
    keep = r < max_range
    base, r, az = base[keep], r[keep], az[keep]
    first = {}
    for i in np.argsort(r):
        first.setdefault(az[i], r[i])
    vis = np.array([r[i] <= first[az[i]] + 0.15 for i in range(len(r))])
    base = base[vis]
    return base[rng.choice(len(base), min(n, len(base)), replace=False)] + rng.normal(0, 0.01, (min(n, len(base)), 3))


@pytest.fixture(scope="module")
def room(worlds_dir):
    boxes = load_world(os.path.join(worlds_dir, "room.yaml"))
    return boxes, VoxelMap(surface_points(boxes), voxel=0.2, dilate=1)


def test_transform_scan():
    out = transform_scan(np.array([[1.0, 0.0, 0.5]]), np.array([[1.0, 2.0, math.pi / 2]]))
    np.testing.assert_allclose(out[0, 0], [1.0, 3.0, 0.5], atol=1e-12)


def test_subsample_seeds():
    traj = np.column_stack([np.linspace(0, 5, 101), np.zeros(101)])
    assert len(subsample_seeds(traj, 0.5)) == 11


@pytest.mark.parametrize("truth", [(2.0, -1.0, 0.6), (6.5, 2.5, -2.0), (0.3, 3.0, 3.0)])
def test_global_search_finds_pose(room, truth):
    boxes, vmap = room
    scan = visible_scan(boxes, truth)
    seeds = np.array([(x, y) for x in np.arange(-2.0, 8.5, 1.0) for y in np.arange(-3.5, 4.0, 1.0)])
    r = global_search(vmap, scan, seeds, SearchParams(seed_spacing=1.0, xy_offsets=0.33))
    assert r.found, r
    assert math.hypot(r.pose[0] - truth[0], r.pose[1] - truth[1]) < 0.2, r
    assert abs(math.remainder(r.pose[2] - truth[2], 2 * math.pi)) < math.radians(5), r
    assert r.score > r.second_score


def test_global_search_reports_not_found_for_foreign_scan(room):
    _, vmap = room
    rng = np.random.default_rng(3)
    scan = np.column_stack([rng.uniform(-3, 3, 800), rng.uniform(-3, 3, 800), rng.uniform(0.2, 1.5, 800)])
    r = global_search(vmap, scan, np.array([[0.0, 0.0]]), SearchParams(min_score=0.6))
    assert not r.found
