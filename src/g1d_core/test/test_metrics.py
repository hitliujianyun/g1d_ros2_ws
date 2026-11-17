import math

import numpy as np
import pytest

from g1d_core.metrics import NearestGrid, best_rigid, map_consistency


def room_points(step=0.05):
    pts = []
    for z in np.arange(0.0, 2.0, step):
        for x in np.arange(-3, 3, step):
            pts += [(x, -2.0, z), (x, 2.0, z)]
        for y in np.arange(-2, 2, step):
            pts += [(-3.0, y, z), (3.0, y, z)]
        for y in np.arange(-0.5, 0.5, step):           # asymmetric box: constrains yaw / x
            pts += [(1.0, y, z)] if z < 1.0 else []
    return np.array(pts)


def test_nearest_grid():
    pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    g = NearestGrid(pts, 0.5)
    d, i = g.query(np.array([[0.1, 0.0, 0.0], [0.9, 0.1, 0.0], [5.0, 0.0, 0.0]]))
    assert i.tolist() == [0, 1, -1] and d[0] == pytest.approx(0.1) and math.isinf(d[2])


def test_best_rigid_recovers_transform():
    rng = np.random.default_rng(0)
    a = rng.uniform(-1, 1, (100, 3))
    t = np.eye(4)
    c, s = math.cos(0.3), math.sin(0.3)
    t[:2, :2] = [[c, -s], [s, c]]
    t[:3, 3] = [0.5, -0.2, 0.1]
    b = a @ t[:3, :3].T + t[:3, 3]
    np.testing.assert_allclose(best_rigid(a, b), t, atol=1e-9)


def test_map_consistency_finds_offset():
    a = room_points()
    yaw = math.radians(2.0)
    c, s = math.cos(yaw), math.sin(yaw)
    rng = np.random.default_rng(1)
    b = a @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T + [0.08, -0.05, 0.0]   # map b shifted
    b = b + rng.normal(0, 0.01, b.shape)
    r = map_consistency(a, b)
    assert r.inlier_ratio > 0.95 and r.rmse < 0.04          # sampling differences dominate
    assert r.offset_m == pytest.approx(math.hypot(0.08, 0.05), abs=0.03)
    assert r.offset_deg == pytest.approx(-2.0, abs=0.5)
