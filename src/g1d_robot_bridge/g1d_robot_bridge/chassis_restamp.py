"""SLAMWARE odom / scan -> /chassis/odom, /chassis/scan stamped with the host receive time.

SLAMWARE header stamps lag the host clock by ~181 s and drift by seconds per day, so TF lookups and
costmaps would drop the data. The original offset is logged so drifts stay visible.
"""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .qos import ROBOT_IN, SENSOR_OUT


def stamp_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class ChassisRestamp(Node):
    def __init__(self, **kwargs):
        super().__init__("chassis_restamp", **kwargs)
        p = self.declare_parameter
        odom_in = p("odom_in", "/slamware_ros_sdk_server_node/odom").value
        scan_in = p("scan_in", "/slamware_ros_sdk_server_node/scan").value
        self.odom_frame = p("odom_frame", "wheel_odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.scan_frame = p("scan_frame", "chassis_laser").value
        self.odom_latency = Duration(seconds=float(p("odom_latency", 0.0).value))
        self.scan_latency = Duration(seconds=float(p("scan_latency", 0.0).value))
        self.log_every = float(p("log_offset_every", 30.0).value)
        self.offsets = {"odom": None, "scan": None}
        self.pub_odom = self.create_publisher(Odometry, p("odom_out", "/chassis/odom").value, SENSOR_OUT)
        self.pub_scan = self.create_publisher(LaserScan, p("scan_out", "/chassis/scan").value, SENSOR_OUT)
        self.create_subscription(Odometry, odom_in, self.on_odom, ROBOT_IN)
        self.create_subscription(LaserScan, scan_in, self.on_scan, ROBOT_IN)
        if self.log_every > 0:
            self.create_timer(self.log_every, self.log_offsets)

    def on_odom(self, m: Odometry) -> None:
        now = self.get_clock().now()
        self.offsets["odom"] = stamp_sec(now.to_msg()) - stamp_sec(m.header.stamp)
        m.header.stamp = (now - self.odom_latency).to_msg()
        m.header.frame_id = self.odom_frame
        m.child_frame_id = self.base_frame
        self.pub_odom.publish(m)

    def on_scan(self, m: LaserScan) -> None:
        now = self.get_clock().now()
        self.offsets["scan"] = stamp_sec(now.to_msg()) - stamp_sec(m.header.stamp)
        m.header.stamp = (now - self.scan_latency).to_msg()
        m.header.frame_id = self.scan_frame
        self.pub_scan.publish(m)

    def log_offsets(self) -> None:
        parts = [f"{k} {v:+.3f} s" for k, v in self.offsets.items() if v is not None]
        if parts:
            self.get_logger().info("SLAMWARE stamp offset (host - stamp): " + ", ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = ChassisRestamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TypeError):
        if rclpy.ok():                 # a message taken while the context shuts down is harmless
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
