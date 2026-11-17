import math

import numpy as np
import pytest

from g1d_core.kinematics import Urdf, axis_angle, matrix_to_rpy, rpy_to_matrix

HEAD_ZERO = np.array([0.057335, 0.000168, 0.679509])   # AGV_link -> head_link, joints at zero


@pytest.fixture(scope="module")
def urdf(g1d_urdf_path):
    return Urdf.from_file(g1d_urdf_path)


def test_rpy_roundtrip_and_convention():
    r = rpy_to_matrix(0.3, -0.2, 1.1)
    np.testing.assert_allclose(matrix_to_rpy(r), [0.3, -0.2, 1.1], atol=1e-12)
    # yaw-only rotation maps x to (cos, sin)
    np.testing.assert_allclose(rpy_to_matrix(0, 0, math.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    # URDF order: roll first (about x), then pitch, then yaw (all fixed axes)
    expect = axis_angle([0, 0, 1], 1.1) @ axis_angle([0, 1, 0], -0.2) @ axis_angle([1, 0, 0], 0.3)
    np.testing.assert_allclose(r, expect, atol=1e-12)


def test_head_at_zero(urdf):
    m = urdf.fk({}, "AGV_link", "head_link")
    np.testing.assert_allclose(m[:3, 3], HEAD_ZERO, atol=1e-5)
    np.testing.assert_allclose(m[:3, :3], np.eye(3), atol=1e-12)


def test_lift_moves_head_up(urdf):
    m = urdf.fk({"LZ_mt_Joint": 0.21, "LZ_it_Joint": 0.21}, "AGV_link", "head_link")
    np.testing.assert_allclose(m[:3, 3], HEAD_ZERO + [0, 0, 0.42], atol=1e-5)


def test_waist_pitch_rotates_about_yaw_link_y(urdf):
    q = 0.5
    base_to_yaw = urdf.fk({}, "AGV_link", "Yaw_Link")
    yaw_to_head = urdf.fk({}, "Yaw_Link", "head_link")[:3, 3]
    m = urdf.fk({"Yaw_Joint": q}, "AGV_link", "head_link")
    expect = base_to_yaw[:3, 3] + axis_angle([0, 1, 0], q) @ yaw_to_head
    np.testing.assert_allclose(m[:3, 3], expect, atol=1e-9)
    assert m[0, 3] > HEAD_ZERO[0]                     # positive pitch leans the head forward


def test_waist_yaw_turns_head_offset(urdf):
    torso = urdf.fk({}, "AGV_link", "torso_link")[:3, 3]
    off = urdf.fk({}, "torso_link", "head_link")[:3, 3]
    m = urdf.fk({"torso_Joint": math.pi / 2}, "AGV_link", "head_link")
    np.testing.assert_allclose(m[:3, 3], torso + [-off[1], off[0], off[2]], atol=1e-9)
    np.testing.assert_allclose(matrix_to_rpy(m[:3, :3])[2], math.pi / 2, atol=1e-9)


def test_chassis_lidar(urdf):
    m = urdf.fk({}, "AGV_link", "Lidar_Link")
    np.testing.assert_allclose(m[:3, 3], [0.1871, -0.016137, 0.10085], atol=1e-9)


def test_missing_chain_raises(urdf):
    with pytest.raises(KeyError):
        urdf.fk({}, "head_link", "AGV_link")
