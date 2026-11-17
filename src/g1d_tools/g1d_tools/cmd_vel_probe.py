"""Measures /cmd_vel_no_limit behaviour from the wheel odometry (design doc 10, Q1).

!! Moves the robot. Needs >= 3 m of free space ahead, someone at the e-stop, waist / arms locked,
!! and g1d_guard STOPPED (this tool writes /cmd_vel_no_limit directly).

Tests (each: accelerate to --speed for --drive-s, then):
  silence : stop publishing      -> time / distance until the base stops (chassis timeout).
            This is the release gate: autonomous navigation needs <= 0.6 s.
  zero    : publish zeros        -> braking time / distance
  clamp   : command 2x --speed and --wz -> reached speeds (silent clamping?)
  hold    : publish zeros for 5 s -> manual check: can the base be pushed?

    ros2 run g1d_tools cmd_vel_probe --tests silence,zero,clamp --i-know-it-moves
"""
import argparse
import json
import math
import os
import threading
import time

import rclpy
from g1d_core.probe import stop_gate, stop_stats
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

RATE = 30.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe:
    def __init__(self, node):
        self.node = node
        self.rows, self.lock = [], threading.Lock()
        node.create_subscription(Odometry, "/slamware_ros_sdk_server_node/odom", self.on_odom,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = node.create_publisher(Twist, "/cmd_vel_no_limit",
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def on_odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        with self.lock:
            self.rows.append((time.monotonic(), p.position.x, p.position.y, yaw_of(p.orientation),
                              t.linear.x, t.angular.z))

    def since(self, t0):
        with self.lock:
            return [r for r in self.rows if r[0] >= t0]

    def drive(self, v, w, seconds):
        t_end = time.monotonic() + seconds
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        while time.monotonic() < t_end:
            self.pub.publish(cmd)
            time.sleep(1.0 / RATE)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tests", default="silence,zero,clamp")
    ap.add_argument("--speed", type=float, default=0.2)
    ap.add_argument("--wz", type=float, default=0.4)
    ap.add_argument("--drive-s", type=float, default=2.0)
    ap.add_argument("--settle-s", type=float, default=3.0)
    ap.add_argument("--out", default="logs/cmd_vel_probe.json")
    ap.add_argument("--yes", action="store_true", help="do not ask before each test")
    ap.add_argument("--i-know-it-moves", action="store_true")
    args, ros_args = ap.parse_known_args(argv)
    if not args.i_know_it_moves:
        ap.error("this tool moves the robot: pass --i-know-it-moves")
    rclpy.init(args=ros_args)
    node = rclpy.create_node("cmd_vel_probe")
    time.sleep(1.5)                                        # discovery
    others = node.get_publishers_info_by_topic("/cmd_vel_no_limit")
    if others:
        names = [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in others]
        raise SystemExit(f"other publishers on /cmd_vel_no_limit ({names}): stop g1d_guard first")
    probe = Probe(node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(1.0)
    if not probe.since(0.0):
        raise SystemExit("no wheel odometry on /slamware_ros_sdk_server_node/odom")

    results = {"speed": args.speed, "wz": args.wz}
    for test in [t.strip() for t in args.tests.split(",") if t.strip()]:
        if not args.yes:
            input(f"\n[{test}] clear space ahead? press Enter to run (Ctrl-C aborts) ")
        if test == "silence":
            probe.drive(args.speed, 0.0, args.drive_s)
            t0 = time.monotonic()
            time.sleep(args.settle_s)
            results[test] = stop_stats(probe.since(t0 - 0.2), t0)
        elif test == "zero":
            probe.drive(args.speed, 0.0, args.drive_s)
            t0 = time.monotonic()
            probe.drive(0.0, 0.0, args.settle_s)
            results[test] = stop_stats(probe.since(t0 - 0.2), t0)
        elif test == "clamp":
            t0 = time.monotonic()
            probe.drive(2.0 * args.speed, 0.0, args.drive_s)
            v_max = max((abs(r[4]) for r in probe.since(t0)), default=None)
            probe.drive(0.0, 0.0, 1.5)
            t0 = time.monotonic()
            probe.drive(0.0, 2.0 * args.wz, args.drive_s)
            w_max = max((abs(r[5]) for r in probe.since(t0)), default=None)
            probe.drive(0.0, 0.0, 1.5)
            results[test] = {"v_cmd": 2.0 * args.speed, "v_reached": v_max,
                             "w_cmd": 2.0 * args.wz, "w_reached": w_max}
        elif test == "hold":
            print("publishing zeros for 5 s: try to push the robot by hand now")
            probe.drive(0.0, 0.0, 5.0)
            print("stopped publishing: try to push again")
            locked = input("was the base locked while zeros were published? [y/n] ").strip().lower()
            results[test] = {"locked_while_zero": locked.startswith("y")}
        else:
            print(f"unknown test {test!r}")
            continue
        print(json.dumps(results[test], indent=2))

    if "silence" in results:
        results["release_gate"] = stop_gate(results["silence"])
        t = results["silence"].get("stop_time_s")
        if t:
            results["suggested_guard"] = {"zero_hold": max(1.0, round(t + 0.3, 1)),
                                          "note": "zero_hold should exceed the chassis' own stop time"}
        print(f"\nrelease gate (stop after silence <= 0.6 s): {results['release_gate']}")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)
    print(f"report: {args.out}")
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
