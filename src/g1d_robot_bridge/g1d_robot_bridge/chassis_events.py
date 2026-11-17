"""Chassis HTTP events (SLAMWARE) -> /g1d/chassis_event (std_msgs/String, event type).

Only events that appear after the first poll are reported (history must not latch an e-stop).
"""
import threading

import rclpy
from g1d_core.chassis_http import ChassisHttp, EventTracker
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .qos import EVENTS


class ChassisEvents(Node):
    def __init__(self, **kwargs):
        super().__init__("chassis_events", **kwargs)
        url = self.declare_parameter("chassis_http", "http://192.168.123.163:1448").value
        self.period = float(self.declare_parameter("period", 0.5).value)
        self.http = ChassisHttp(url, timeout=float(self.declare_parameter("timeout", 0.5).value))
        self.tracker = EventTracker()
        self.pub = self.create_publisher(String, "/g1d/chassis_event", EVENTS)
        self.pub_ok = self.create_publisher(Bool, "/g1d/chassis_http_ok", 1)
        self._stop = threading.Event()
        self._ok = None
        self._thread = threading.Thread(target=self._loop, name="chassis_events", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                for ev in self.tracker.update(self.http.events()):
                    self.get_logger().warn(f"chassis event {ev}")
                    self.pub.publish(String(data=ev))
                ok = True
            except Exception as e:  # noqa: BLE001 - chassis may be unreachable
                ok = False
                if self._ok is not False:
                    self.get_logger().warn(f"chassis HTTP unreachable: {e}")
            self._ok = ok
            self.pub_ok.publish(Bool(data=ok))
            self._stop.wait(self.period)

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisEvents()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
