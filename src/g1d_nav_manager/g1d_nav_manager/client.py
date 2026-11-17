"""Python client for NavGPT-style goals (keeps the g1d_ws NavClient call pattern).

    from g1d_nav_manager.client import NavClient
    nav = NavClient(node)                                   # any rclpy node, spun elsewhere or by wait()
    gid = nav.send_relative(dx, dy, dyaw, stamp=image_stamp)   # stamp: builtin_interfaces/Time or float s
    result = nav.wait(gid, timeout=90.0)                    # dict: state, reason, free_sectors, ...
"""
import math
import threading
import time
import uuid
from typing import Dict, Optional, Union

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from g1d_msgs.action import NavigateRelative
from rclpy.action import ActionClient
from rclpy.node import Node


def to_stamp(t: Union[None, float, TimeMsg]) -> TimeMsg:
    if t is None:
        return TimeMsg()
    if isinstance(t, TimeMsg):
        return t
    return TimeMsg(sec=int(t), nanosec=int((t - int(t)) * 1e9))


class NavClient:
    def __init__(self, node: Node, action: str = "/g1d/navigate_relative"):
        self.node = node
        self.ac = ActionClient(node, NavigateRelative, action)
        self.goals: Dict[str, dict] = {}
        self.lock = threading.Lock()

    def send(self, mode: str, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, frame: str = "base_at_stamp",
             stamp=None, use_yaw: bool = True, max_speed: float = 0.0, timeout_s: float = 0.0,
             gid: Optional[str] = None) -> str:
        gid = gid or f"navgpt-{uuid.uuid4().hex[:8]}"
        g = NavigateRelative.Goal(id=gid, mode=mode, frame=frame, stamp=to_stamp(stamp), x=float(x), y=float(y),
                                  yaw=float(yaw), use_yaw=use_yaw, max_speed=float(max_speed),
                                  timeout_s=float(timeout_s))
        entry = {"done": threading.Event(), "result": None, "handle": None}
        with self.lock:
            self.goals[gid] = entry
        if not self.ac.wait_for_server(timeout_sec=5.0):
            entry["result"] = {"goal_id": gid, "state": "FAILED", "reason": "nav_manager_unavailable"}
            entry["done"].set()
            return gid

        def on_goal(fut):
            h = fut.result()
            entry["handle"] = h
            if not h.accepted:
                entry["result"] = {"goal_id": gid, "state": "FAILED", "reason": "rejected"}
                entry["done"].set()
                return
            h.get_result_async().add_done_callback(on_result)

        def on_result(fut):
            r = fut.result().result
            entry["result"] = {
                "goal_id": gid, "state": r.state, "reason": r.reason, "partial": r.partial,
                "adjusted": r.adjusted, "goal": {"x": r.goal.x, "y": r.goal.y, "yaw": r.goal.theta},
                "final_pose": {"x": r.final_pose.x, "y": r.final_pose.y, "yaw": r.final_pose.theta},
                "free_sectors": {str(a): bool(f) for a, f in zip(r.free_sector_angles, r.free_sectors)}}
            entry["done"].set()

        self.ac.send_goal_async(g).add_done_callback(on_goal)
        return gid

    def send_relative(self, dx: float, dy: float, dyaw: float = 0.0, stamp=None, use_yaw: bool = True,
                      max_speed: float = 0.0, timeout_s: float = 0.0) -> str:
        return self.send("navigate", dx, dy, dyaw, "base_at_stamp", stamp, use_yaw, max_speed, timeout_s)

    def rotate(self, dyaw: float, stamp=None) -> str:
        return self.send("rotate", 0.0, 0.0, dyaw, "base_at_stamp", stamp)

    def cancel(self, gid: str) -> None:
        h = self.goals.get(gid, {}).get("handle")
        if h is not None:
            h.cancel_goal_async()

    def wait(self, gid: str, timeout: float = 90.0, spin: bool = True) -> dict:
        entry = self.goals[gid]
        t_end = time.monotonic() + timeout
        while not entry["done"].is_set() and time.monotonic() < t_end:
            if spin:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            else:
                entry["done"].wait(0.05)
        return entry["result"] or {"goal_id": gid, "state": "FAILED", "reason": "client_timeout"}


def prompt_feedback(result: dict) -> str:
    """Failure text for the next NavGPT prompt (g1d_ws navgpt_bridge convention)."""
    if result.get("state") == "SUCCEEDED":
        return "" if not result.get("partial") else "[导航] 已到达截断后的目标，请继续给出下一个航点"
    free = [a for a, f in result.get("free_sectors", {}).items() if f]
    return f"[导航失败] 原因={result.get('reason')} 可通行方向(度)={','.join(free) or '无'}"
