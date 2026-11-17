"""Command line goals / e-stop / status.

    ros2 run g1d_tools nav_cli go 2.0 0.5 [--yaw 0.3] [--frame base|odom|map]   # relative to base by default
    ros2 run g1d_tools nav_cli rotate 90
    ros2 run g1d_tools nav_cli stop | estop on|off | status
    ros2 run g1d_tools nav_cli relocalize auto | pose X Y YAW | last
"""
import argparse
import json
import math

import rclpy
from g1d_msgs.srv import Relocalize
from std_msgs.msg import Bool, String

from g1d_nav_manager.client import NavClient, prompt_feedback


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["go", "rotate", "stop", "estop", "status", "relocalize"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--yaw", type=float, default=None, help="final heading (rad)")
    ap.add_argument("--frame", default="base")
    ap.add_argument("--mode", default="navigate", help="navigate | simple")
    ap.add_argument("--max-speed", type=float, default=0.0)
    ap.add_argument("--timeout", type=float, default=150.0)
    a, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = rclpy.create_node("nav_cli")
    try:
        if a.cmd in ("go", "rotate", "stop"):
            nav = NavClient(node)
            if a.cmd == "go":
                gid = nav.send(a.mode, float(a.args[0]), float(a.args[1]), a.yaw or 0.0, a.frame,
                               use_yaw=a.yaw is not None, max_speed=a.max_speed)
            elif a.cmd == "rotate":
                gid = nav.send("rotate", yaw=math.radians(float(a.args[0])), frame="base")
            else:
                gid = nav.send("stop")
            r = nav.wait(gid, a.timeout)
            print(json.dumps(r, indent=2, ensure_ascii=False))
            fb = prompt_feedback(r)
            if fb:
                print(fb)
        elif a.cmd == "estop":
            pub = node.create_publisher(Bool, "/g1d/estop", 10)
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Bool(data=(a.args[:1] or ["on"])[0] == "on"))
            rclpy.spin_once(node, timeout_sec=0.2)
        elif a.cmd == "status":
            got = []
            node.create_subscription(String, "/nav/status", lambda m: got.append(m.data), 10)
            while not got:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(json.dumps(json.loads(got[0]), indent=2, ensure_ascii=False))
        elif a.cmd == "relocalize":
            cli = node.create_client(Relocalize, "/g1d/relocalize")
            if not cli.wait_for_service(timeout_sec=5.0):
                raise SystemExit("/g1d/relocalize not available")
            mode = (a.args[:1] or ["auto"])[0]
            req = Relocalize.Request(mode=mode)
            if mode == "pose":
                req.x, req.y, req.yaw = (float(v) for v in a.args[1:4])
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
            r = fut.result()
            print(f"{r.success}: {r.message} pose=({r.x:.2f}, {r.y:.2f}, {math.degrees(r.yaw):.1f} deg) score={r.score:.2f}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
