import math

import numpy as np
import pytest

from g1d_core.kinematics import rpy_to_matrix
from g1d_core.lio import (BridgeState, HealthParams, LioAnchor, LioHealth, LioSample,
                          WheelOdomBuffer, between2, compose2, inv, make_tf, matrix_to_quat,
                          planar, quat_to_matrix, tilt_of)


def tf(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_to_matrix(*rpy)
    m[:3, 3] = xyz
    return m


T_BASE_IMU = tf([0.1166, -0.0231, 1.374], [math.pi, -0.04, 0.0])   # upside-down head lidar IMU


def test_quaternion_roundtrip():
    r = rpy_to_matrix(0.3, -1.2, 2.5)
    np.testing.assert_allclose(quat_to_matrix(*matrix_to_quat(r)), r, atol=1e-12)
    r = rpy_to_matrix(math.pi, 0.0, 0.0)
    np.testing.assert_allclose(quat_to_matrix(*matrix_to_quat(r)), r, atol=1e-12)
    m = make_tf([1, 2, 3], matrix_to_quat(r))
    np.testing.assert_allclose(inv(m) @ m, np.eye(4), atol=1e-12)


def test_anchor_maps_lio_motion_to_base_motion():
    """Ground-truth base motion -> IMU poses in FAST_LIO's world frame -> recovered base pose."""
    t_lio_world0 = tf([0.3, -0.2, 0.5], [0.4, -0.3, 1.0])   # arbitrary FAST_LIO world frame
    lio_T_odom = t_lio_world0 @ inv(T_BASE_IMU)              # lio <- odom, fixed
    a = LioAnchor()
    for x, y, yaw in [(0, 0, 0), (1.0, 0.5, 0.3), (-2.0, 3.0, -2.5)]:
        t_odom_base = tf([x, y, 0], [0, 0, yaw])
        t_lio_imu = lio_T_odom @ t_odom_base @ T_BASE_IMU
        p = planar(a.base_pose(t_lio_imu, T_BASE_IMU))
        assert p == pytest.approx((x, y, yaw), abs=1e-9)


def test_lift_change_keeps_base_pose():
    """The lidar moves up with the lift; the base did not move -> same base pose."""
    a = LioAnchor()
    t_lio_imu0 = tf([0, 0, 0], [0, 0, 0])
    a.anchor(t_lio_imu0, T_BASE_IMU)
    lifted = T_BASE_IMU.copy()
    lifted[2, 3] += 0.2
    # FAST_LIO sees the IMU move up by 0.2 m (in odom z, which is -z of the inverted IMU frame)
    t_lio_imu = a.t_odom_lio.copy()
    t_lio_imu = inv(a.t_odom_lio) @ lifted
    t = a.base_pose(t_lio_imu, lifted)
    assert planar(t) == pytest.approx((0, 0, 0), abs=1e-9) and t[2, 3] == pytest.approx(0, abs=1e-9)


def test_planar_compose_between():
    a, b = (1.0, 2.0, 0.5), (0.3, -0.2, 0.4)
    c = compose2(a, b)
    assert between2(a, c) == pytest.approx(b)


def test_wheel_buffer_interpolation_and_delta():
    w = WheelOdomBuffer()
    for i in range(11):
        w.add(i * 0.1, (0.05 * i, 0.0, 0.0))          # 0.5 m/s straight
    assert w.at(0.25) == pytest.approx((0.125, 0.0, 0.0))
    assert w.delta(0.2, 0.7) == pytest.approx((0.25, 0.0, 0.0))
    assert w.delta(0.5, 5.0) == pytest.approx((0.25, 0.0, 0.0))  # clipped at the latest sample
    assert w.at(-1.0) is None and w.delta(-1.0, 0.5) is None


def test_bridge_extrapolates_with_wheel_and_times_out():
    st = BridgeState(max_extrapolation_s=0.5)
    for i in range(30):
        st.wheel.add(i * 0.05, (0.02 * i, 0.0, 0.0))  # 0.4 m/s
    lio_T_odom = inv(T_BASE_IMU)
    st.on_lio(0.0, lio_T_odom @ T_BASE_IMU, T_BASE_IMU)                 # anchor: base at origin
    st.on_lio(1.0, lio_T_odom @ tf([0.4, 0, 0], [0, 0, 0]) @ T_BASE_IMU, T_BASE_IMU)
    assert st.pose_at(1.2) == pytest.approx((0.48, 0.0, 0.0), abs=1e-6)
    assert st.pose_at(1.6) is None


def test_bridge_reanchors_after_lio_restart():
    """FAST_LIO restarts (new world frame at the current IMU pose) after the base drove 1 m on wheels."""
    st = BridgeState()
    for i in range(101):
        st.wheel.add(i * 0.05, (0.2 * i * 0.05, 0.0, 0.0))             # 0.2 m/s along x for 5 s
    lio_T_odom = inv(T_BASE_IMU)
    st.on_lio(0.0, lio_T_odom @ T_BASE_IMU, T_BASE_IMU)
    st.on_lio(0.1, lio_T_odom @ tf([0.02, 0, 0], [0, 0, 0]) @ T_BASE_IMU, T_BASE_IMU)
    assert st.reanchors == 0
    # crash at 0.1 s, new process publishes from 5.0 s on; its world frame is the IMU at 5.0 s
    new_world = tf([0.0, 0.0, 0.0], [0.1, -0.05, 0.7])                  # arbitrary (not gravity aligned)
    t_imu_5 = tf([1.0, 0, 0], [0, 0, 0]) @ T_BASE_IMU                    # IMU pose in odom at 5.0 s
    lio_T_odom2 = new_world @ inv(t_imu_5)
    s = st.on_lio(5.0, lio_T_odom2 @ t_imu_5, T_BASE_IMU)
    assert st.reanchored and st.reanchors == 1
    assert s.pose == pytest.approx((1.0, 0.0, 0.0), abs=1e-6)
    s = st.on_lio(5.1, lio_T_odom2 @ tf([1.0, 0.1, 0], [0, 0, 0.2]) @ T_BASE_IMU, T_BASE_IMU)
    assert not st.reanchored
    assert s.pose == pytest.approx((1.0, 0.1, 0.2), abs=1e-6)
    assert "jump" not in st.health.status(5.1)[1]


def test_health_jump_drift_timeout_and_wheel_consistency():
    h = LioHealth(HealthParams())
    w = WheelOdomBuffer()
    for i in range(41):
        w.add(i * 0.05, (0.0, 0.0, 0.0))              # wheels say: standing still
    h.update(LioSample(0.0, (0, 0, 0), 0.0, 0.0, 0.0), w)
    assert h.status(0.05) == (True, [])
    h.update(LioSample(0.1, (0.5, 0, 0), 0.0, 0.0, 0.0), w)
    assert "jump" in h.status(0.15)[1]
    assert h.status(1.2)[1] == ["lio_timeout"]          # jump released after bad_hold_s
    h = LioHealth(HealthParams())
    for i in range(21):                                 # LIO drifts 0.5 m in 2 s, wheels static
        h.update(LioSample(i * 0.1, (0.025 * i, 0, 0), 0.0, 0.0, 0.0), w)
    ok, errs = h.status(2.0)
    assert not ok and "wheel_inconsistent" in errs and h.dpos == pytest.approx(0.5, abs=0.01)
    h = LioHealth()
    h.update(LioSample(0.0, (0, 0, 0), 0.4, 0.0, 0.2), None)
    assert set(h.status(0.0)[1]) == {"z_drift", "tilt"}
    assert LioHealth().status(0.0) == (False, ["no_lio"])


def test_tilt_of():
    r, p = tilt_of(tf([0, 0, 0], [0.1, -0.2, 1.0]))
    assert (r, p) == pytest.approx((0.1, -0.2))
