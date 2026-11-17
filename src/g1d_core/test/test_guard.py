import math

import numpy as np
import pytest

from g1d_nav_py.config import load_config
from g1d_nav_py.guard_core import GuardCore, GuardParams, scan_to_points, swept_clearance

DT = 1.0 / 30.0


def wall_x(x, y0=-2.0, y1=2.0, n=200):
    ys = np.linspace(y0, y1, n)
    return np.c_[np.full(n, x), ys, np.zeros(n)].astype(np.float32)


def fed(p=None, scan=None, cloud=None, now=0.0):
    g = GuardCore(p or GuardParams(), now)
    g.on_scan_points(np.zeros((0, 3), np.float32) if scan is None else scan, now)
    g.on_cloud_points(np.zeros((0, 3), np.float32) if cloud is None else cloud, now)
    return g


def run(g, t0, t1, cmd=None, refresh=True):
    """Steps the guard from t0 to t1; returns the list of outputs."""
    outs, t = [], t0
    while t < t1 - 1e-9:
        if cmd is not None:
            g.on_cmd(cmd[0], cmd[1], t)
        if refresh:
            g.t_scan = g.t_cloud = t
        outs.append(g.step(t, DT))
        t += DT
    return outs


def test_params_from_project_config():
    p = GuardParams.from_config(load_config())
    assert p.stop_m == pytest.approx(0.45)
    assert p.vx_max == pytest.approx(0.50)
    assert p.require_cloud is True


def test_straight_clearance_is_exact():
    p = GuardParams()
    xy = wall_x(1.0)[:, :2]
    assert swept_clearance(xy, 0.3, 0.0, p, 2.0) == pytest.approx(1.0 - p.front_x, abs=1e-6)
    assert swept_clearance(xy, -0.1, 0.0, p, 2.0) == math.inf
    assert swept_clearance(-xy, -0.1, 0.0, p, 2.0) == pytest.approx(1.0 - p.rear_x, abs=1e-6)
    # points beside the chassis (outside the corridor) do not block straight motion
    side = np.array([[0.0, 0.5], [1.0, -0.45]], np.float32)
    assert swept_clearance(side, 0.3, 0.0, p, 2.0) == math.inf


def test_arc_clearance_turning_away_and_towards():
    p = GuardParams()
    box = np.array([[0.9, y] for y in np.linspace(-0.6, -0.05, 20)], np.float32)  # front-right
    straight = swept_clearance(box, 0.3, 0.0, p, 1.2)
    left = swept_clearance(box, 0.3, 0.6, p, 1.2)    # turning left, away from the box
    right = swept_clearance(box, 0.3, -0.15, p, 1.2)  # gentle right turn, into the box
    tight = swept_clearance(box, 0.3, -0.6, p, 1.2)   # R = 0.5 m: curves inside the box
    assert straight == pytest.approx(0.9 - p.front_x, abs=1e-6)
    assert left > straight
    assert right <= straight + p.path_step_m
    assert tight == math.inf


def test_front_stop_slow_and_no_rotation_when_blocked():
    p = GuardParams()
    g = fed(p, scan=wall_x(p.front_x + 0.8))
    out = run(g, 0.0, 3.0, cmd=(0.4, 0.0))[-1]
    k = (0.8 - p.stop_m) / (p.slow_m - p.stop_m)
    assert out.v == pytest.approx(0.4 * k, abs=1e-3)
    assert out.blocked == ""
    g = fed(p, scan=wall_x(p.front_x + p.stop_m + 0.02))
    out = run(g, 0.0, 2.0, cmd=(0.4, 0.0))[-1]
    assert out.v == pytest.approx(p.creep_v, abs=1e-3)   # creeps up to the stop line
    g = fed(p, scan=wall_x(p.front_x + 0.3))
    out = run(g, 0.0, 1.0, cmd=(0.4, 0.3))[-1]
    assert out.v == 0.0 and out.w == 0.0 and out.blocked == "front"
    assert not out.publish                            # never moved: nothing to hold


def test_rotation_circle_blocks_in_place_turn_only():
    p = GuardParams()
    pole = np.array([[0.32, 0.20, 0.0]], np.float32)   # inside r=0.38, outside the self box
    g = fed(p, scan=pole)
    out = run(g, 0.0, 0.5, cmd=(0.0, 0.5))[-1]
    assert out.w == 0.0 and out.blocked == "rotate"
    far = np.array([[0.8, 0.0, 0.0]], np.float32)
    g = fed(p, scan=far)
    out = run(g, 0.0, 1.0, cmd=(0.0, 0.5))[-1]
    assert out.w == pytest.approx(0.5) and out.blocked == ""


def test_self_box_points_are_ignored():
    p = GuardParams()
    self_pts = np.array([[0.1, 0.1, 0.0], [-0.25, -0.2, 0.0]], np.float32)
    g = fed(p, scan=self_pts)
    out = run(g, 0.0, 2.0, cmd=(0.3, 0.0))[-1]
    assert out.v == pytest.approx(0.3) and out.blocked == ""


def test_reverse_needs_clear_rear():
    p = GuardParams()
    g = fed(p, scan=-wall_x(p.rear_x + 0.3))
    out = run(g, 0.0, 1.0, cmd=(-0.1, 0.0))[-1]
    assert out.v == 0.0 and out.blocked == "rear"
    g = fed(p, scan=-wall_x(p.rear_x + 1.5))
    out = run(g, 0.0, 1.0, cmd=(-0.1, 0.0))[-1]
    assert out.v == pytest.approx(-0.1)


def test_limits_and_acceleration():
    p = GuardParams()
    g = fed(p)
    outs = run(g, 0.0, 2.0, cmd=(2.0, -3.0))
    assert outs[0].v == pytest.approx(p.acc_v * DT)
    assert outs[0].w == pytest.approx(-p.acc_w * DT)
    assert outs[-1].v == pytest.approx(p.vx_max)
    assert outs[-1].w == pytest.approx(-p.wz_max)
    dv = np.diff([o.v for o in outs])
    assert dv.max() <= p.acc_v * DT + 1e-9


def test_command_timeout_ramps_down_then_releases_chassis():
    p = GuardParams()
    g = fed(p)
    assert not run(g, 0.0, 0.5)[-1].publish          # silent at start-up
    run(g, 0.0, 2.0, cmd=(0.3, 0.0))
    outs = run(g, 2.0, 5.0)                           # nav stops sending
    t = 2.0 + DT * np.arange(len(outs))
    v = np.array([o.v for o in outs])
    pub = np.array([o.publish for o in outs])
    t_stop = t[np.argmax(v == 0.0)]
    assert t_stop - 2.0 < p.cmd_timeout + 0.3 / p.brake_v + 2 * DT
    assert pub[t < t_stop + p.zero_hold - DT].all()   # zero held for zero_hold
    assert not pub[t > t_stop + p.zero_hold + DT].any()


def test_stale_sensors_stop_the_base():
    p = GuardParams()
    g = fed(p)
    run(g, 0.0, 1.0, cmd=(0.3, 0.0))
    g.t_scan = 0.3                                    # scan stops at t = 0.3
    out = run(g, 1.0, 1.2, cmd=(0.3, 0.0), refresh=False)[-1]
    assert out.blocked == "scan_stale" and out.v < 0.3
    g = fed(p)
    g.t_cloud = -10.0
    g.on_cmd(0.3, 0.0, 0.0)
    assert g.step(0.0, DT).blocked == "cloud_stale"
    p2 = GuardParams(require_cloud=False)
    g = fed(p2)
    g.t_cloud = -10.0
    g.on_cmd(0.3, 0.0, 0.0)
    assert g.step(0.0, DT).blocked == ""


def test_estop_latch_hard_stop_and_release():
    p = GuardParams()
    g = fed(p)
    run(g, 0.0, 2.0, cmd=(0.3, 0.2))
    g.on_event("BUMPER_TRIGGERED")
    out = run(g, 2.0, 2.1, cmd=(0.3, 0.2))[0]
    assert out.v == 0.0 and out.w == 0.0 and out.publish
    assert g.state(2.1)["estop"] and g.state(2.1)["estop_reason"] == "bumper_triggered"
    g.on_event("LOW_BATTERY")                         # not an e-stop event
    g.on_estop(False)
    out = run(g, 2.1, 3.0, cmd=(0.3, 0.0))[-1]
    assert out.v > 0.0


def test_manual_override_silences_the_guard():
    p = GuardParams()
    g = fed(p)
    run(g, 0.0, 1.0, cmd=(0.3, 0.0))
    g.on_joystick(0.0, 0.8, 0.0, 0.0, 1.0)
    outs = run(g, 1.0, 1.0 + p.manual_hold_s - 0.1, cmd=(0.3, 0.0))
    assert not any(o.publish for o in outs)
    assert g.state(2.0)["manual"]
    out = run(g, 1.0 + p.manual_hold_s + 0.05, 1.0 + p.manual_hold_s + 0.5, cmd=(0.3, 0.0))[-1]
    assert out.publish and out.v > 0.0


def test_scan_to_points_frame_and_filtering():
    r = [1.0, float("inf"), 0.01, 2.0]
    pts = scan_to_points(0.0, math.pi / 2, r, laser_xyz=(0.2, -0.1, 0.1))
    assert pts.shape == (2, 3)
    np.testing.assert_allclose(pts[0], [1.2, -0.1, 0.1], atol=1e-6)
    np.testing.assert_allclose(pts[1], [0.2, -0.1 - 2.0, 0.1], atol=1e-6)   # beam 3 at -90 deg


def test_web_teleop_overrides_nav_with_deadman():
    p = GuardParams()
    g = fed(p)
    outs, t = [], 0.0
    while t < 2.0:
        g.on_cmd(0.4, 0.0, t)              # nav keeps streaming
        g.on_teleop(0.5, -0.2, t)          # web teleop above its own limits
        g.t_scan = g.t_cloud = t
        outs.append(g.step(t, DT))
        t += DT
    assert outs[-1].v == pytest.approx(p.teleop_vx_max)
    assert outs[-1].w == pytest.approx(-0.2)
    st = g.state(t)
    assert st["manual"] and st["manual_source"] == "web" and st["source"] == "teleop"
    # deadman: teleop stops -> nav command again after teleop_timeout
    out = run(g, t, t + 1.5, cmd=(0.4, 0.0))[-1]
    assert out.v == pytest.approx(0.4) and g.state(t + 1.5)["source"] == "nav"
    assert not g.state(t + 1.5)["manual"]


def test_teleop_still_gets_the_reflex_layer():
    p = GuardParams()
    g = fed(p, scan=wall_x(p.front_x + 0.3))
    t = 0.0
    while t < 1.0:
        g.on_teleop(0.3, 0.0, t)
        g.t_scan = g.t_cloud = t
        out = g.step(t, DT)
        t += DT
    assert out.v == 0.0 and out.blocked == "front"


def test_runtime_params_whitelist_and_validation():
    g = fed()
    assert g.set_param("stop_m", 0.5) is None and g.p.stop_m == 0.5
    assert "unknown" in g.set_param("rate_hz", 10)
    assert g.set_param("vx_max", 3.0) is not None
    assert g.set_param("slow_m", 0.4) == "slow_m must be greater than stop_m"
    assert g.set_param("require_cloud", 0) is None and g.p.require_cloud is False
    assert g.set_param("wz_max", "fast") is not None
    assert g.params()["stop_m"] == 0.5


def test_state_json_is_serialisable():
    import json
    g = fed()
    g.step(0.0, DT)
    s = json.dumps(g.state(0.0))
    assert '"blocked": ""' in s
