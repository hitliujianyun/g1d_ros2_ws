import math

import numpy as np
import pytest

from g1d_core.sim2d import (DiffDrive, DiffDriveParams, boxes_at_height, boxes_in_band,
                            footprint_gap, load_world, raycast_boxes)


def test_raycast_exact_distances():
    boxes = [(2.0, -1.0, 2.2, 1.0)]
    a = np.array([0.0, math.pi, math.atan2(0.5, 2.0)])
    r = raycast_boxes((0.0, 0.0), a, boxes, 10.0)
    assert r[0] == pytest.approx(2.0)
    assert math.isinf(r[1])
    assert r[2] == pytest.approx(math.hypot(2.0, 0.5))
    assert math.isinf(raycast_boxes((0.0, 0.0), a[:1], boxes, 1.5)[0])


def test_footprint_gap_sign():
    boxes = [(1.0, -0.5, 1.2, 0.5)]
    assert footprint_gap((0.0, 0.0, 0.0), boxes) == pytest.approx(1.0 - 0.2785)
    assert footprint_gap((0.8, 0.0, 0.0), boxes) < 0.0


def test_world_height_filters(tmp_path):
    p = tmp_path / "w.yaml"
    p.write_text("boxes:\n  - [0, 0, 0, 1, 1, 0.3]\n  - [2, 0, 0.7, 3, 1, 0.75]\n")
    boxes = load_world(str(p))
    assert boxes_at_height(boxes, 0.2) == [(0.0, 0.0, 1.0, 1.0)]
    assert boxes_in_band(boxes, 0.0, 1.75) == [(0.0, 0.0, 1.0, 1.0), (2.0, 0.0, 3.0, 1.0)]


def test_diff_drive_timeout_and_deceleration():
    dd = DiffDrive(DiffDriveParams(cmd_timeout=0.5, max_acc=1.0, max_dec=2.0))
    t, dt = 0.0, 0.01
    while t < 1.0:
        dd.on_cmd(0.5, 0.0, t)
        dd.step(t, dt)
        t += dt
    assert dd.v == pytest.approx(0.5)
    t_last = t - dt
    while dd.v > 0.0:
        dd.step(t, dt)
        t += dt
    # holds the last command until the timeout, then brakes at max_dec
    assert t - t_last == pytest.approx(0.5 + 0.25, abs=0.03)
    assert dd.pose[0] > 0.0 and dd.pose[1] == pytest.approx(0.0)


def test_diff_drive_limits():
    dd = DiffDrive(DiffDriveParams(v_limit=0.3, w_limit=0.2, max_acc=10.0, max_ang_acc=10.0))
    for i in range(100):
        dd.on_cmd(5.0, -5.0, i * 0.01)
        dd.step(i * 0.01, 0.01)
    assert dd.v == pytest.approx(0.3) and dd.w == pytest.approx(-0.2)
