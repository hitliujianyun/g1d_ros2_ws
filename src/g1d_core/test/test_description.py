import math
import os

import numpy as np
import pytest
import yaml

from g1d_core.description import xacro_args
from g1d_core.kinematics import Urdf, rpy_to_matrix

xacro = pytest.importorskip("xacro")


@pytest.fixture(scope="module")
def robot(description_dir):
    with open(os.path.join(description_dir, "config", "extrinsics.yaml"), encoding="utf-8") as f:
        extr = yaml.safe_load(f)
    doc = xacro.process_file(os.path.join(description_dir, "urdf", "g1d.urdf.xacro"),
                             mappings=xacro_args(extr))
    return Urdf.from_string(doc.toxml()), extr


def test_xacro_args_format():
    a = xacro_args({"base_to_agv_z": 0.1, "head_to_livox_xyz": [0.05, 0, 0.55], "other": 1})
    assert a == {"base_to_agv_z": "0.1", "head_to_livox_xyz": "0.05 0.0 0.55"}


def test_livox_frame_at_zero_joints(robot):
    urdf, extr = robot
    assert extr["base_to_agv_z"] == pytest.approx(0.10)
    m = urdf.fk({}, "base_link", "livox_frame")
    np.testing.assert_allclose(m[:3, 3], [0.107335, 0.000168, 1.329509], atol=1e-5)
    np.testing.assert_allclose(m[:3, :3], rpy_to_matrix(math.pi, -0.04, 0.0), atol=1e-6)


def test_imu_and_chassis_laser(robot):
    urdf, _ = robot
    m = urdf.fk({}, "livox_frame", "livox_imu")
    np.testing.assert_allclose(m[:3, 3], [0.011, 0.02329, -0.04412], atol=1e-9)
    m = urdf.fk({}, "base_link", "chassis_laser")
    np.testing.assert_allclose(m[:3, 3], [0.1871, -0.016137, 0.20085], atol=1e-9)


def test_lift_raises_lidar(robot):
    urdf, _ = robot
    z0 = urdf.fk({}, "base_link", "livox_frame")[2, 3]
    z1 = urdf.fk({"LZ_mt_Joint": 0.1, "LZ_it_Joint": 0.1}, "base_link", "livox_frame")[2, 3]
    assert z1 - z0 == pytest.approx(0.2, abs=1e-9)
