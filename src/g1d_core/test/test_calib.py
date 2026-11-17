import math

import numpy as np
import pytest

from g1d_nav_py.calib import (align_vectors, estimate_yaw, fit_floor, level_rotation,
                              matrix_to_rpy, rpy_to_matrix)


def test_rpy_roundtrip_matches_zyx():
    r = rpy_to_matrix(0.1, -0.2, 0.3)
    assert matrix_to_rpy(r) == pytest.approx((0.1, -0.2, 0.3))
    q = align_vectors(np.array([0, 0, 1.0]), np.array([0, 0, -1.0]))
    np.testing.assert_allclose(q @ [0, 0, 1.0], [0, 0, -1.0], atol=1e-9)


def synthetic_room(rng):
    """Floor + two walls + ceiling in the base frame."""
    floor = np.c_[rng.uniform(-1, 4, 4000), rng.uniform(-3, 3, 4000), rng.normal(0, 0.005, 4000)]
    wall = np.c_[np.full(1500, 4.0), rng.uniform(-3, 3, 1500), rng.uniform(0, 2.5, 1500)]
    side = np.c_[rng.uniform(-1, 4, 1500), np.full(1500, 3.0), rng.uniform(0, 2.5, 1500)]
    ceil = np.c_[rng.uniform(-1, 4, 3000), rng.uniform(-3, 3, 3000), np.full(3000, 2.6)]
    return np.vstack([floor, wall, side, ceil])


def test_floor_fit_recovers_roll_pitch_height():
    rng = np.random.default_rng(1)
    world = synthetic_room(rng)
    r_true = rpy_to_matrix(math.pi + 0.02, -0.07, 0.01)       # inverted, slightly tilted
    t_true = np.array([0.11, 0.0, 1.25])
    lidar = (world - t_true) @ r_true                          # base -> lidar frame
    r_cfg = rpy_to_matrix(math.pi, -0.04, 0.01)                # nominal config
    n, h, inl = fit_floor(lidar, r_cfg.T @ [0, 0, 1.0])
    assert h == pytest.approx(1.25, abs=0.005)
    assert inl > 3500
    r_new = level_rotation(r_cfg, n)
    up_err = np.degrees(np.arccos(np.clip((r_new.T @ [0, 0, 1.0]) @ (r_true.T @ [0, 0, 1.0]), -1, 1)))
    assert up_err < 0.2
    roll, pitch, _ = matrix_to_rpy(r_new)
    assert math.cos(roll - (math.pi + 0.02)) > math.cos(math.radians(0.3))
    assert pitch == pytest.approx(-0.07, abs=0.005)


def test_yaw_from_scan_slice():
    rng = np.random.default_rng(2)
    xs = np.linspace(-1, 4, 300)
    room = np.vstack([np.c_[xs, np.full(300, 2.0)], np.c_[xs, np.full(300, -1.5)],
                      np.c_[np.full(200, 4.0), np.linspace(-1.5, 2.0, 200)]])
    pivot = np.array([0.1, 0.0])
    err = math.radians(2.5)
    c, s = math.cos(-err), math.sin(-err)
    rel = room - pivot
    band = np.c_[c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]] + pivot
    band += rng.normal(0, 0.005, band.shape)
    yaw, resid = estimate_yaw(band, room[::2], pivot)
    assert math.degrees(yaw) == pytest.approx(2.5, abs=0.15)
    assert resid < 0.02
