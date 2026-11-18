"""The field-tool launch files run as shipped: record.launch.py (MCAP bag finalised on shutdown),
web.launch.py (browser UI served) and viz.launch.py with foxglove_bridge (RViz needs a display),
on top of the simulated robot (sim.launch.py).
"""
import glob
import os
import socket
import tempfile
import time
import unittest
import urllib.request

import launch
import launch_testing
import launch_testing.actions
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

WORK = tempfile.mkdtemp(prefix="g1d_tools_")
BAGS = os.path.join(WORK, "bags")
WEB_PORT, FOXGLOVE_PORT = 18097, 18765


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(get_package_share_directory("g1d_bringup"), "launch")
    inc = lambda name, args: IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(os.path.join(bringup, name)), launch_arguments=args.items())
    return launch.LaunchDescription([
        inc("sim.launch.py", {"world": "room", "http_port": "18096"}),
        inc("record.launch.py", {"preset": "nav", "out": BAGS}),
        inc("web.launch.py", {"port": str(WEB_PORT), "root": os.path.join(WORK, "web")}),
        inc("viz.launch.py", {"rviz": "false", "foxglove": "true", "foxglove_port": str(FOXGLOVE_PORT)}),
        launch_testing.actions.ReadyToTest(),
    ])


def wait_for(fn, timeout):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            if fn():
                return True
        except OSError:
            pass
        time.sleep(0.5)
    return False


class TestToolsLaunch(unittest.TestCase):
    def test_web_ui_served(self):
        def page():
            with urllib.request.urlopen(f"http://127.0.0.1:{WEB_PORT}/", timeout=2) as r:
                return r.status == 200 and b"<html" in r.read(4096).lower()
        self.assertTrue(wait_for(page, 60.0), "g1d_web did not serve /")

    def test_foxglove_bridge_listening(self):
        def connect():
            with socket.create_connection(("127.0.0.1", FOXGLOVE_PORT), timeout=2):
                return True
        self.assertTrue(wait_for(connect, 60.0), "foxglove_bridge port closed")

    def test_bag_is_recording(self):
        self.assertTrue(wait_for(lambda: glob.glob(os.path.join(BAGS, "nav_*", "*.mcap")), 60.0))
        time.sleep(5.0)                                       # some messages before the shutdown


@launch_testing.post_shutdown_test()
class TestBagFinalised(unittest.TestCase):
    def test_metadata_has_messages(self):
        meta = glob.glob(os.path.join(BAGS, "nav_*", "metadata.yaml"))
        self.assertEqual(len(meta), 1, meta)
        with open(meta[0], encoding="utf-8") as f:
            info = yaml.safe_load(f)["rosbag2_bagfile_information"]
        counts = {t["topic_metadata"]["name"]: t["message_count"] for t in info["topics_with_message_count"]}
        self.assertEqual(info["storage_identifier"], "mcap")
        for topic in ("/chassis/odom", "/tf", "/g1d/guard_state"):
            self.assertGreater(counts.get(topic, 0), 0, counts)
