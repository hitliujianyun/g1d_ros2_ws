"""Interactive acceptance tests T1-T7 (g1d_core.acceptance) on the robot or in simulation.

    ros2 run g1d_tools acceptance T1 [--speed 0.3] [--trials 10]
    ros2 run g1d_tools acceptance T3          # Enter at the end marker
    ros2 run g1d_tools acceptance --list
Observations: /odometry (pose, speed), /g1d/localization_status (map pose), /g1d/guard_state,
/nav/status (JSON). Actions: /g1d/teleop_cmd (deadman stream), /nav/goal (JSON).
Report: logs/acceptance/<test>-<time>.json and .md
"""
import argparse
import json
import math
import os
import select
import sys
import threading
import time

import rclpy
from g1d_core.acceptance import TESTS, catalog
from g1d_msgs.msg import GuardState, LocalizationStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

GUARD_FIELDS = ("blocked", "estop", "manual", "front_clear", "rear_clear", "v", "w")


class Runner:
    def __init__(self, node, test):
        self.node, self.test = node, test
        self.obs = {"pose": None, "map_pose": None, "v": 0.0, "w": 0.0, "guard": {}, "status": {}}
        self.teleop = None
        self.lock = threading.Lock()
        node.create_subscription(Odometry, "/odometry", self.on_odom, 10)
        node.create_subscription(LocalizationStatus, "/g1d/localization_status", self.on_loc, 10)
        node.create_subscription(GuardState, "/g1d/guard_state", self.on_guard, 10)
        node.create_subscription(String, "/nav/status", self.on_status, 10)
        self.pub_teleop = node.create_publisher(Twist, "/g1d/teleop_cmd", 10)
        self.pub_goal = node.create_publisher(String, "/nav/goal", 10)
        node.create_timer(0.05, self.tick)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.obs.update(pose=(p.x, p.y, yaw), v=m.twist.twist.linear.x, w=m.twist.twist.angular.z)

    def on_loc(self, m):
        self.obs["map_pose"] = (m.pose.x, m.pose.y, m.pose.theta) if m.localized else None

    def on_guard(self, m):
        g = {k: getattr(m, k) for k in GUARD_FIELDS}
        for k in ("front_clear", "rear_clear"):
            if math.isinf(g[k]):
                g[k] = None
        self.obs["guard"] = g

    def on_status(self, m):
        try:
            self.obs["status"] = json.loads(m.data)
        except ValueError:
            pass

    def apply(self, actions):
        for a in actions or []:
            if a[0] == "teleop":
                self.teleop = None if (a[1] == 0.0 and a[2] == 0.0) else (a[1], a[2])
                if self.teleop is None:
                    self.pub_teleop.publish(Twist())
            elif a[0] == "goal":
                self.pub_goal.publish(String(data=json.dumps(a[1])))
            elif a[0] == "stop_nav":
                self.pub_goal.publish(String(data=json.dumps({"id": "acceptance-stop", "mode": "stop"})))

    def tick(self):
        with self.lock:
            self.apply(self.test.tick(time.monotonic(), self.obs))
            if self.teleop is not None:
                t = Twist()
                t.linear.x, t.angular.z = self.teleop
                self.pub_teleop.publish(t)


def key_pressed():
    return select.select([sys.stdin], [], [], 0.0)[0] if hasattr(select, "select") else False


def write_report(summary, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, f"{summary['id']}-{time.strftime('%Y%m%d-%H%M%S')}")
    with open(base + ".json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False, default=str)
    lines = [f"# {summary['title']}", "", f"- 结果：{ {True: '通过', False: '未通过', None: '未完成'}[summary['passed']] }",
             f"- 试验次数：{len(summary['trials'])} / {summary['target_trials']}", "", "| # | 记录 |", "|---|---|"]
    lines += [f"| {i + 1} | `{json.dumps(t, ensure_ascii=False)}` |" for i, t in enumerate(summary["trials"])]
    if summary.get("spread"):
        lines += ["", f"离散程度：`{summary['spread']}`"]
    with open(base + ".md", "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return base


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", nargs="?", help="T1 .. T7")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--trials", type=int, default=None)
    ap.add_argument("--out", default="logs/acceptance")
    args, ros_args = ap.parse_known_args(argv)
    if args.list or not args.test:
        for c in catalog():
            print(f"{c['id']}: {c['title']}\n    {c['instructions']}")
        return
    cls = TESTS[args.test.upper()]
    targs = {k: v for k, v in {"speed": args.speed}.items() if v is not None}
    test = cls(targs)
    if args.trials:
        test.target_trials = args.trials
    rclpy.init(args=ros_args)
    node = rclpy.create_node("acceptance")
    runner = Runner(node, test)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f"{cls.title}\n{cls.instructions}\n")
    try:
        if test.state in ("ready", "idle"):
            input("Enter = 开始（Ctrl-C 结束）")
            with runner.lock:
                runner.apply(test.start(time.monotonic(), runner.obs))
        while test.state not in ("done", "aborted"):
            time.sleep(0.2)
            if test.state == "ready":
                print(test.message)
                if len(test.trials) >= test.target_trials:
                    break
                input("Enter = 下一次试验（Ctrl-C 结束）")
                with runner.lock:
                    runner.apply(test.start(time.monotonic(), runner.obs))
            elif test.state == "running" and key_pressed():
                sys.stdin.readline()
                with runner.lock:
                    runner.apply(test.mark(time.monotonic(), runner.obs))
                print(test.message)
                if len(test.trials) >= test.target_trials and test.id == "T7":
                    break
    except KeyboardInterrupt:
        pass
    with runner.lock:
        runner.apply(test.stop(time.monotonic()) if test.state != "aborted" else [])
    summary = test.summary()
    path = write_report(summary, args.out)
    print(f"\n{summary['title']}: {summary['passed']}  ->  {path}.md")
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
