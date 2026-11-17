import math

import numpy as np
import pytest

from g1d_core.navgoal import (CostGrid, GoalParams, free_sectors, nav2_reason, relative_to_global,
                              sanitize_goal)


def grid_with_box(box=(2.0, -0.3, 2.6, 0.3), size=(10.0, 6.0), res=0.05, inflate=0.2):
    w, h = int(size[0] / res), int(size[1] / res)
    data = np.zeros((h, w), np.int16)
    xs = -1.0 + (np.arange(w) + 0.5) * res
    ys = -3.0 + (np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    dx = np.maximum(np.maximum(box[0] - X, X - box[2]), 0)
    dy = np.maximum(np.maximum(box[1] - Y, Y - box[3]), 0)
    d = np.hypot(dx, dy)
    data[d <= inflate] = 70
    data[d == 0] = 100
    return CostGrid(data, (-1.0, -3.0), res)


def test_free_goal_unchanged_and_partial_clip():
    g = grid_with_box()
    r = sanitize_goal(g, (0.0, 0.0), (1.0, 1.0))
    assert r.ok and not r.adjusted and not r.partial and r.xy == pytest.approx((1.0, 1.0))
    r = sanitize_goal(g, (0.0, 0.0), (20.0, 0.0), GoalParams(max_distance=8.0))
    assert r.ok and r.partial and r.xy == pytest.approx((8.0, 0.0))


def test_goal_in_obstacle_moves_back_towards_robot():
    g = grid_with_box()
    r = sanitize_goal(g, (0.0, 0.0), (2.2, 0.0))
    assert r.ok and r.adjusted
    assert r.xy[0] < 2.0 - 0.2 + 1e-6 and r.xy[0] > 1.6        # just before the inflated box
    assert g.cost(*r.xy) < 50


def test_goal_in_obstacle_prefers_the_robot_side():
    """The way back towards the robot is blocked beyond the search radius (a wall behind the box):
    the nearest free cell on the far side is as close, but the robot side is preferred."""
    g = grid_with_box(box=(3.0, -0.4, 3.4, 0.4), res=0.1, inflate=0.5)
    h, w = g.data.shape
    X, Y = np.meshgrid(-1.0 + (np.arange(w) + 0.5) * 0.1, -3.0 + (np.arange(h) + 0.5) * 0.1)
    g.data[(np.abs(Y) < 0.45) & (X > 3.4) & (X < 4.6)] = 70     # high-cost strip along the way back
    r = sanitize_goal(g, (6.0, 0.1), (3.2, 0.0), GoalParams(search_radius=1.0))
    assert r.ok and r.adjusted and g.cost(*r.xy) < 50
    assert r.xy[0] > 3.4, r.xy                                    # not behind the box (x < 3.0)


def test_goal_deep_in_obstacle_fails_and_out_of_range():
    g = grid_with_box(box=(1.0, -3.0, 9.0, 3.0))                   # huge obstacle
    r = sanitize_goal(g, (0.0, 0.0), (5.0, 0.0))
    assert not r.ok and r.reason == "goal_in_obstacle"
    r = sanitize_goal(grid_with_box(), (0.0, 0.0), (0.0, 5.0))
    assert not r.ok and r.reason == "goal_out_of_range"


def test_free_sectors():
    g = grid_with_box(box=(0.8, -0.2, 1.2, 0.2))
    s = free_sectors(g, (0.0, 0.0, 0.0))
    assert s[0] is False and s[90] is True and s[-90] is True
    s = free_sectors(g, (0.0, 0.0, math.pi / 2))                     # facing +y: box is to the right
    assert s[0] is True and s[-90] is False


def test_nav2_reason_and_relative():
    assert nav2_reason(206) == "goal_in_obstacle"
    assert nav2_reason(105) == "blocked" and nav2_reason(150) == "blocked"
    assert nav2_reason(208) == "no_path" and nav2_reason(0, "x") == "x"
    t = np.eye(4)
    t[:2, :2] = [[0, -1], [1, 0]]
    t[:3, 3] = [1, 2, 0]
    gx, gy, gyaw = relative_to_global(t, 1.0, 0.0, 0.5)
    assert (gx, gy) == pytest.approx((1.0, 3.0)) and gyaw == pytest.approx(math.pi / 2 + 0.5)
