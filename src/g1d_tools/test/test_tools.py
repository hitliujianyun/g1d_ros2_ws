import threading

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from g1d_sim.chassis_sim import ChassisSim  # noqa: E402
from g1d_tools import check_interfaces, cmd_vel_probe  # noqa: E402


def spin_in_background(ex, stop):
    while not stop.is_set():
        ex.spin_once(timeout_sec=0.01)


def test_check_interfaces_against_sim():
    rclpy.init()
    sim = ChassisSim(parameter_overrides=[Parameter("world", value="empty")])
    ex = SingleThreadedExecutor()
    ex.add_node(sim)
    stop = threading.Event()
    t = threading.Thread(target=spin_in_background, args=(ex, stop), daemon=True)
    t.start()
    node = rclpy.create_node("check_interfaces_test")
    try:
        report = check_interfaces.collect(node, 2.0)
    finally:
        stop.set()
        t.join(timeout=2.0)
        node.destroy_node()
        sim.destroy_node()
        rclpy.shutdown()
    odom = report["topics"]["odom"]
    assert odom["count"] > 20 and odom["rate_hz"] == pytest.approx(20.0, rel=0.25)
    assert odom["stamp_offset_s"] == pytest.approx(181.0, abs=0.5)
    assert report["topics"]["lowstate"]["sample"]["waist_yaw_q12"] == 0.0
    assert report["topics"]["scan"]["sample"]["beams"] == 360
    assert report["cmd_vel_no_limit_publishers"] == []
    assert "/slamware_ros_sdk_server_node/odom" in report["all_topics"]


def test_cmd_vel_probe_requires_confirmation():
    with pytest.raises(SystemExit):
        cmd_vel_probe.main(["--tests", "silence"])
