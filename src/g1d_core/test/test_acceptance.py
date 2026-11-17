"""Acceptance procedures (migrated from g1d_ws test_web_units.py, plus T7)."""
import math

import pytest

from g1d_core import acceptance


def test_t1_stop_distance_trial():
    t = acceptance.T1StopDistance({"speed": 0.3})
    acts = t.start(0.0, {"pose": (0, 0, 0)})
    assert acts == [("teleop", 0.3, 0.0)]
    now = 0.0
    for _ in range(20):
        now += 0.05
        assert t.tick(now, {"pose": (now * 0.3, 0, 0), "v": 0.3, "guard": {"blocked": ""}})[0][1] == 0.3
    out = []
    for _ in range(20):
        now += 0.05
        out = t.tick(now, {"pose": (1.5, 0, 0), "v": 0.0, "guard": {"blocked": "front", "front_clear": 0.43}})
        if t.state == "ready":
            break
    assert out == [("teleop", 0.0, 0.0)]
    assert t.trials[-1]["ok"] and t.trials[-1]["front_clear_m"] == 0.43
    assert t.passed() is None


def test_t2_stop_time():
    t = acceptance.T2StreamLoss({"drive_s": 1.0})
    t.start(0.0, {})
    now = 0.0
    while now < 1.0:
        now += 0.05
        t.tick(now, {"v": 0.3})
    t.tick(now + 0.05, {"v": 0.3})
    t.tick(now + 0.3, {"v": 0.1})
    t.tick(now + 0.45, {"v": 0.0})
    assert t.trials[-1]["ok"] and t.trials[-1]["stop_time_s"] == pytest.approx(0.45, abs=0.06)


def test_t3_and_t4():
    t3 = acceptance.T3LoopClosure()
    t3.start(0.0, {"pose": (0.0, 0.0, 0.0)})
    t3.tick(1.0, {"pose": (5.0, 0.0, 0.0)})
    t3.mark(2.0, {"pose": (0.05, -0.03, math.radians(1.0))})
    assert t3.trials[0]["ok"] and t3.trials[0]["path_m"] == 5.0
    t4 = acceptance.T4RotationDrift({"turns": 2})
    acts = t4.start(0.0, {"pose": (1.0, 1.0, 0.0)})
    assert acts[0][0] == "goal" and acts[0][1]["mode"] == "rotate"
    gid = acts[0][1]["id"]
    acts = t4.tick(1.0, {"pose": (1.0, 1.0, 1.0), "status": {"goal_id": gid, "state": "SUCCEEDED"}})
    gid2 = acts[0][1]["id"]
    t4.tick(2.0, {"pose": (1.01, 1.0, 3.1), "status": {"goal_id": gid2, "state": "SUCCEEDED"}})
    assert t4.state == "done" and t4.trials[0]["ok"] and t4.trials[0]["drift_m"] == pytest.approx(0.01)


def test_t5_t6_trial_recording():
    t5 = acceptance.T5Corridor()
    t5.start(0.0, {})
    for i in range(10):
        gid = f"g{i}"
        t5.tick(i + 0.1, {"status": {"goal_id": gid, "state": "NAVIGATING"}, "guard": {"front_clear": 0.8}})
        end = "SUCCEEDED" if i else "FAILED"
        t5.tick(i + 0.5, {"status": {"goal_id": gid, "state": end, "reason": "blocked"}, "guard": {}})
    assert len(t5.trials) == 10 and t5.passed() is True
    t6 = acceptance.T6Overhang()
    t6.start(0.0, {})
    t6.tick(0.1, {"status": {"goal_id": "a", "state": "PLANNING"}, "guard": {}})
    t6.tick(0.2, {"status": {"goal_id": "a", "state": "FAILED", "reason": "no_path"}, "guard": {}})
    t6.tick(0.3, {"status": {"goal_id": "b", "state": "NAVIGATING"}, "guard": {"estop": True}})
    t6.tick(0.4, {"status": {"goal_id": "b", "state": "FAILED", "reason": "guard_estop"}, "guard": {}})
    assert [x["ok"] for x in t6.trials] == [True, False]


def test_t7_relocalization_spread():
    t = acceptance.T7Relocalization()
    t.start(0.0, {})
    assert t.mark(1.0, {"map_pose": None}) == [] and not t.trials
    for i, (dx, dyaw) in enumerate([(0.0, 0.0), (0.01, 0.003), (-0.01, -0.003), (0.02, 0.0), (0.0, 0.005)]):
        t.mark(i, {"map_pose": (3.0 + dx, 1.0, 0.5 + dyaw)})
    s = t.spread()
    assert s["max_pos_dev_m"] < 0.02 and s["max_yaw_dev_deg"] < 0.5
    assert t.passed() is True
    assert "T7" in acceptance.TESTS and len(acceptance.catalog()) == 7
