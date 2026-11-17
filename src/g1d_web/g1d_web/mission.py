"""Multi-waypoint missions on top of rt/nav/goal + rt/nav/status (pure logic, clock injected).

Mission JSON (missions/<name>.json):
  {"name": "lab_loop", "loops": 1, "on_fail": "stop" | "skip" | "retry", "retries": 1,
   "waypoints": [{"x": 2.0, "y": 0.5, "yaw": 1.57, "mode": "navigate", "wait_s": 2.0,
                  "max_speed": 0.3, "xy_tolerance": 0.2}, ...]}
Waypoints are in the odom frame. loops = 0 repeats until cancelled.
"""
from __future__ import annotations

import json
import os
import re
import uuid
from typing import Any, Callable, Dict, List, Optional

TERMINAL = ("SUCCEEDED", "FAILED", "CANCELED")
NO_ACK_S = 3.0          # no status for the goal within this time -> nav not running


def validate_mission(m: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(m, dict):
        raise ValueError("mission must be an object")
    name = str(m.get("name") or "mission").strip()
    if not re.match(r"^[\w\-. ]{1,64}$", name):
        raise ValueError("mission name: letters, digits, '-', '_', '.', ' ' (max 64)")
    wps = m.get("waypoints")
    if not isinstance(wps, list) or not wps:
        raise ValueError("mission needs at least one waypoint")
    out = []
    for i, w in enumerate(wps):
        try:
            wp = {"x": float(w["x"]), "y": float(w["y"])}
        except (KeyError, TypeError, ValueError):
            raise ValueError(f"waypoint {i + 1}: x / y required") from None
        if w.get("yaw") is not None:
            wp["yaw"] = float(w["yaw"])
        mode = w.get("mode", "navigate")
        if mode not in ("navigate", "simple", "rotate"):
            raise ValueError(f"waypoint {i + 1}: mode must be navigate / simple / rotate")
        wp["mode"] = mode
        wp["wait_s"] = max(0.0, float(w.get("wait_s", 0.0) or 0.0))
        for k in ("max_speed", "xy_tolerance", "yaw_tolerance", "timeout_s"):
            if w.get(k) not in (None, ""):
                wp[k] = float(w[k])
        out.append(wp)
    on_fail = m.get("on_fail", "stop")
    if on_fail not in ("stop", "skip", "retry"):
        raise ValueError("on_fail must be stop / skip / retry")
    return {"name": name, "waypoints": out, "loops": max(0, int(m.get("loops", 1))),
            "on_fail": on_fail, "retries": max(0, int(m.get("retries", 1)))}


class MissionRunner:
    def __init__(self, send_goal: Callable[[Dict[str, Any]], None], send_stop: Callable[[], None],
                 log: Optional[Callable[[str, str], None]] = None):
        self.send_goal = send_goal
        self.send_stop = send_stop
        self.log = log or (lambda level, text: None)
        self.mission: Optional[Dict[str, Any]] = None
        self.state = "idle"     # idle running waiting paused done failed canceled
        self.reason = ""
        self.index = 0
        self.loop = 0
        self.attempt = 0
        self.goal_id = ""
        self.t_sent = 0.0
        self.acked = False
        self.wait_until = 0.0
        self.results: List[Dict[str, Any]] = []

    # ---- control ----------------------------------------------------------------------------
    def start(self, mission: Dict[str, Any], now: float) -> None:
        self.mission = validate_mission(mission)
        self.index = self.loop = self.attempt = 0
        self.results = []
        self.reason = ""
        self.log("info", f"mission '{self.mission['name']}' started "
                         f"({len(self.mission['waypoints'])} waypoints)")
        self._send_current(now)

    def pause(self, now: float, reason: str = "paused by user") -> None:
        if self.state in ("running", "waiting"):
            if self.state == "running":
                self.send_stop()
            self.state = "paused"
            self.reason = reason
            self.log("warn", f"mission paused: {reason}")

    def resume(self, now: float) -> None:
        if self.state == "paused" and self.mission:
            self.reason = ""
            self.log("info", "mission resumed")
            self._send_current(now)

    def cancel(self, now: float) -> None:
        if self.state in ("running", "waiting", "paused"):
            if self.state == "running":
                self.send_stop()
            self._finish("canceled", "canceled by user")

    @property
    def active(self) -> bool:
        return self.state in ("running", "waiting", "paused")

    # ---- feedback ---------------------------------------------------------------------------
    def on_status(self, st: Dict[str, Any], now: float) -> None:
        if self.state != "running" or st.get("goal_id") != self.goal_id:
            return
        self.acked = True
        state = st.get("state")
        if state not in TERMINAL:
            return
        wp = self.mission["waypoints"][self.index]
        self.results.append({"loop": self.loop, "index": self.index, "attempt": self.attempt,
                             "goal_id": self.goal_id, "state": state, "reason": st.get("reason", ""),
                             "duration_s": round(now - self.t_sent, 2),
                             "x": wp["x"], "y": wp["y"]})
        if state == "SUCCEEDED":
            self.log("info", f"waypoint {self.index + 1} reached")
            if wp["wait_s"] > 0:
                self.state = "waiting"
                self.wait_until = now + wp["wait_s"]
            else:
                self._advance(now)
            return
        reason = st.get("reason", "")
        if reason == "manual_override":
            self.state = "paused"
            self.reason = "manual override (teleop / remote)"
            self.log("warn", "mission paused by manual override")
            return
        if reason in ("guard_estop", "lio_lost", "body_not_locked", "body_moved"):
            self._finish("failed", f"waypoint {self.index + 1}: {reason}")
            return
        self._handle_failure(now, reason or state.lower())

    def tick(self, now: float) -> None:
        if self.state == "waiting" and now >= self.wait_until:
            self._advance(now)
        elif self.state == "running" and not self.acked and now - self.t_sent > NO_ACK_S:
            self._finish("failed", "no status from g1d_nav (is it running?)")

    # ---- internals --------------------------------------------------------------------------
    def _handle_failure(self, now: float, reason: str) -> None:
        m = self.mission
        if m["on_fail"] == "retry" and self.attempt < m["retries"]:
            self.attempt += 1
            self.log("warn", f"waypoint {self.index + 1} failed ({reason}), retry {self.attempt}")
            self._send_current(now, keep_attempt=True)
        elif m["on_fail"] in ("skip", "retry"):
            self.log("warn", f"waypoint {self.index + 1} failed ({reason}), skipped")
            self._advance(now)
        else:
            self._finish("failed", f"waypoint {self.index + 1}: {reason}")

    def _advance(self, now: float) -> None:
        m = self.mission
        self.index += 1
        self.attempt = 0
        if self.index >= len(m["waypoints"]):
            self.loop += 1
            if m["loops"] and self.loop >= m["loops"]:
                failed = sum(1 for r in self.results if r["state"] != "SUCCEEDED")
                self._finish("done", f"{len(self.results) - failed}/{len(self.results)} goals succeeded")
                return
            self.index = 0
        self._send_current(now)

    def _send_current(self, now: float, keep_attempt: bool = False) -> None:
        wp = self.mission["waypoints"][self.index]
        self.goal_id = (f"m-{self.mission['name'][:16]}-{self.loop}-{self.index}-{self.attempt}-"
                        f"{uuid.uuid4().hex[:4]}")
        goal = {"id": self.goal_id, "mode": wp["mode"], "frame": "odom", "x": wp["x"], "y": wp["y"]}
        if "yaw" in wp:
            goal["yaw"] = wp["yaw"]
        for k in ("max_speed", "xy_tolerance", "yaw_tolerance", "timeout_s"):
            if k in wp:
                goal[k] = wp[k]
        self.send_goal(goal)
        self.state = "running"
        self.t_sent = now
        self.acked = False

    def _finish(self, state: str, reason: str) -> None:
        self.state = state
        self.reason = reason
        self.log("info" if state == "done" else "warn", f"mission {state}: {reason}")

    def snapshot(self) -> Dict[str, Any]:
        m = self.mission
        return {"state": self.state, "reason": self.reason, "name": m["name"] if m else "",
                "index": self.index, "total": len(m["waypoints"]) if m else 0, "loop": self.loop,
                "loops": m["loops"] if m else 0, "attempt": self.attempt, "goal_id": self.goal_id,
                "waypoints": m["waypoints"] if m else [], "results": self.results[-50:]}


# ------------------------------------------------------------------------------------ storage
def _path(root: str, name: str) -> str:
    if not re.match(r"^[\w\-. ]{1,64}$", name) or name.startswith("."):
        raise ValueError("bad mission name")
    return os.path.join(root, name + ".json")


def save_mission(root: str, mission: Dict[str, Any]) -> Dict[str, Any]:
    m = validate_mission(mission)
    os.makedirs(root, exist_ok=True)
    with open(_path(root, m["name"]), "w", encoding="utf-8") as f:
        json.dump(m, f, indent=2, ensure_ascii=False)
    return m


def load_mission(root: str, name: str) -> Dict[str, Any]:
    with open(_path(root, name), encoding="utf-8") as f:
        return validate_mission(json.load(f))


def delete_mission(root: str, name: str) -> None:
    os.remove(_path(root, name))


def list_missions(root: str) -> List[Dict[str, Any]]:
    out = []
    if os.path.isdir(root):
        for fn in sorted(os.listdir(root)):
            if fn.endswith(".json"):
                try:
                    m = load_mission(root, fn[:-5])
                    out.append({"name": m["name"], "waypoints": len(m["waypoints"]), "mission": m})
                except (OSError, ValueError):
                    continue
    return out
