import math

import pytest

from g1d_core.body import (ARM_JOINTS, BodyConfig, BodySample, LockChecker, arms_from_lowstate,
                           joint_positions)


def test_arm_joint_names_follow_unitree_order():
    assert len(ARM_JOINTS) == 14
    assert ARM_JOINTS[0] == "left_shoulder_pitch_joint"
    assert ARM_JOINTS[6] == "left_wrist_yaw_joint"
    assert ARM_JOINTS[7] == "right_shoulder_pitch_joint"


def test_joint_positions_split_lift_and_clamp():
    cfg = BodyConfig(lift_scale=2.0, lift_offset=-0.1)
    j = joint_positions(0.2, 0.1, -0.05, None, cfg)
    assert j["LZ_mt_Joint"] == pytest.approx(0.15) and j["LZ_it_Joint"] == pytest.approx(0.15)
    assert j["Yaw_Joint"] == pytest.approx(0.1) and j["torso_Joint"] == pytest.approx(-0.05)
    assert j["left_elbow_joint"] == 0.0                     # arm_tuck default
    assert joint_positions(5.0, 0, 0, None, cfg)["LZ_mt_Joint"] == pytest.approx(0.21)
    assert joint_positions(-1.0, 0, 0, None, cfg)["LZ_it_Joint"] == 0.0
    arms = [0.01 * i for i in range(14)]
    j = joint_positions(0.0, 0, 0, arms, cfg)
    assert j["right_wrist_yaw_joint"] == pytest.approx(0.13)


def test_arms_from_lowstate_indices():
    q = [float(i) for i in range(35)]
    assert arms_from_lowstate(q, BodyConfig()) == [float(i) for i in range(15, 29)]


def good(now=10.0, **kw):
    s = BodySample(lift_y=0.1, lift_speed=0.0, waist_pitch=0.0, waist_yaw=0.0, arm_q=[0.0] * 14,
                   t_lift=now, t_lowstate=now, t_waist=now)
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def test_lock_ok_and_each_reason():
    cfg = BodyConfig()
    lc = LockChecker(cfg)
    assert lc.check(good(), 10.0) == (True, [])
    assert "waist_yaw" in lc.check(good(waist_yaw=math.radians(3)), 10.0)[1]
    assert "waist_pitch" in lc.check(good(waist_pitch=math.radians(3)), 10.0)[1]
    assert "lift_moving" in lc.check(good(lift_speed=0.01), 10.0)[1]
    assert "lift_changed" in lc.check(good(lift_y=0.12), 10.0)[1]
    arms = [0.0] * 14
    arms[3] = math.radians(6)
    assert "arms" in lc.check(good(arm_q=arms), 10.0)[1]
    assert "stale_lowstate" in lc.check(good(t_lowstate=9.0), 10.0)[1]
    assert "arms" in lc.check(good(arm_q=None), 10.0)[1]


def test_lock_reference_reset_and_optional_sources():
    lc = LockChecker(BodyConfig())
    lc.check(good(lift_y=0.1), 10.0)
    lc.reset_reference(good(lift_y=0.3))
    assert lc.check(good(lift_y=0.3), 10.0) == (True, [])
    cfg = BodyConfig(require_waist_pitch=False, require_arms=False)
    lc = LockChecker(cfg)
    ok, reasons = lc.check(good(t_waist=-math.inf, waist_pitch=1.0, arm_q=None), 10.0)
    assert ok, reasons


def test_config_from_dict():
    cfg = BodyConfig.from_dict({"lift_scale": 0.5, "arm_tuck": [0.1] * 14, "require_arms": False})
    assert cfg.lift_scale == 0.5 and cfg.arm_tuck[0] == 0.1 and cfg.require_arms is False
    with pytest.raises(ValueError):
        BodyConfig.from_dict({"arm_tuck": [0.0] * 3})
