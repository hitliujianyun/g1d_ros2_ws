"""Acceptance tests T1–T6 (design doc §12.4) as guided, self-measuring procedures.

Each test is pure logic: tick(now, obs) returns actions for the app to publish.
  obs:     {"pose": (x, y, yaw) | None, "v": float, "w": float, "guard": dict, "status": dict}
  actions: ("teleop", v, w) | ("goal", goal_dict) | ("stop_nav",)
"""
from __future__ import annotations

import math
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

STOPPED_V = 0.02


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class AcceptanceTest:
    id = ""
    title = ""
    instructions = ""
    target_trials = 1
    needs_control = True

    def __init__(self, args: Optional[Dict[str, Any]] = None):
        self.args = args or {}
        self.state = "idle"        # idle | running | ready (between trials) | done | aborted
        self.phase = ""
        self.message = ""
        self.trials: List[Dict[str, Any]] = []
        self.started = time.time()

    def start(self, now: float, obs: Dict[str, Any]) -> List[Tuple]:
        self.state = "running"
        return []

    def tick(self, now: float, obs: Dict[str, Any]) -> List[Tuple]:
        return []

    def mark(self, now: float, obs: Dict[str, Any]) -> List[Tuple]:
        return []

    def stop(self, now: float) -> List[Tuple]:
        self.state = "done"
        return [("teleop", 0.0, 0.0)]

    def abort(self, now: float, why: str = "aborted") -> List[Tuple]:
        self.state = "aborted"
        self.message = why
        return [("teleop", 0.0, 0.0), ("stop_nav",)]

    def passed(self) -> Optional[bool]:
        return None

    def summary(self) -> Dict[str, Any]:
        return {"id": self.id, "title": self.title, "instructions": self.instructions,
                "state": self.state, "phase": self.phase, "message": self.message,
                "trials": self.trials, "target_trials": self.target_trials,
                "passed": self.passed(), "args": self.args, "started": self.started}


class T1StopDistance(AcceptanceTest):
    id = "T1"
    title = "T1 停车距离（guard 反射层）"
    instructions = ("机器人正对墙 / 桌 / 人腿，距离 2–3 m。点「开始一次」：Web 以设定速度遥控前进，"
                    "直到 guard 拦停。自动记录停车后的前方净距。重复 10 次（可换不同障碍物）。")
    target_trials = 10

    def __init__(self, args=None):
        super().__init__(args)
        self.speed = float(self.args.get("speed", 0.3))
        self.max_time = float(self.args.get("max_time", 20.0))
        self.min_clear = float(self.args.get("min_clear", 0.40))
        self.state = "ready"

    def start(self, now, obs):
        self.state, self.phase = "running", "driving"
        self.t0, self.t_stopped, self.blocked_seen = now, None, False
        self.pose0 = obs.get("pose")
        self.message = f"以 {self.speed} m/s 前进中…"
        return [("teleop", self.speed, 0.0)]

    def tick(self, now, obs):
        if self.state != "running":
            return []
        g = obs.get("guard") or {}
        if g.get("blocked") == "front":
            self.blocked_seen = True
        if self.blocked_seen and abs(obs.get("v", 0.0)) < STOPPED_V:
            self.t_stopped = self.t_stopped or now
        else:
            self.t_stopped = None
        if self.t_stopped is not None and now - self.t_stopped > 0.5:
            clear = g.get("front_clear")
            travel = None
            if self.pose0 and obs.get("pose"):
                travel = math.hypot(obs["pose"][0] - self.pose0[0], obs["pose"][1] - self.pose0[1])
            ok = clear is not None and clear >= self.min_clear
            self.trials.append({"front_clear_m": clear, "travel_m": travel, "ok": ok,
                                "duration_s": round(now - self.t0, 2)})
            self.state, self.phase = "ready", ""
            self.message = f"第 {len(self.trials)} 次：净距 {clear} m {'通过' if ok else '未通过'}"
            return [("teleop", 0.0, 0.0)]
        if now - self.t0 > self.max_time:
            self.trials.append({"front_clear_m": None, "ok": False, "error": "未在超时内停车"})
            self.state, self.phase, self.message = "ready", "", "超时未停车（检查前方是否有障碍物）"
            return [("teleop", 0.0, 0.0)]
        return [("teleop", self.speed, 0.0)]

    def passed(self):
        if len(self.trials) < self.target_trials:
            return None
        return all(t["ok"] for t in self.trials)


class T2StreamLoss(AcceptanceTest):
    id = "T2"
    title = "T2 指令中断停车时间"
    instructions = ("前方 ≥ 3 m 空旷。点「开始一次」：以设定速度行驶 2 s 后突然停止发送指令（模拟上游进程卡死），"
                    "测量最后一条指令到底盘停止（轮速 < 0.02 m/s）的时间。")
    target_trials = 3

    def __init__(self, args=None):
        super().__init__(args)
        self.speed = float(self.args.get("speed", 0.3))
        self.drive_s = float(self.args.get("drive_s", 2.0))
        self.max_stop_s = float(self.args.get("max_stop_s", 0.6))
        self.state = "ready"

    def start(self, now, obs):
        self.state, self.phase, self.t0, self.t_last = "running", "driving", now, None
        self.message = "行驶中…"
        return [("teleop", self.speed, 0.0)]

    def tick(self, now, obs):
        if self.state != "running":
            return []
        if self.phase == "driving":
            if now - self.t0 < self.drive_s:
                self.t_last = now
                return [("teleop", self.speed, 0.0)]
            self.phase, self.v_cut = "coasting", obs.get("v", 0.0)
            self.message = "已停止发送指令，等待停车…"
            return []
        if abs(obs.get("v", 0.0)) < STOPPED_V:
            dt = now - self.t_last
            ok = dt <= self.max_stop_s
            self.trials.append({"stop_time_s": round(dt, 3), "v_at_cut": round(self.v_cut, 3), "ok": ok})
            self.state, self.phase = "ready", ""
            self.message = f"停车时间 {dt:.2f} s {'通过' if ok else '未通过'}"
        elif now - self.t_last > 5.0:
            self.trials.append({"stop_time_s": None, "ok": False, "error": "5 s 内未停车"})
            self.state, self.phase, self.message = "ready", "", "5 s 内未停车！"
        return []

    def passed(self):
        if len(self.trials) < self.target_trials:
            return None
        return all(t["ok"] for t in self.trials)


class T3LoopClosure(AcceptanceTest):
    id = "T3"
    title = "T3 矩形闭环误差（LIO）"
    instructions = ("在地面贴起点标记并让机器人对准。点「开始」记录起点位姿，然后用遥控或导航绕约 10 m 矩形，"
                    "回到标记处并摆正，点「标记终点」。计算起终点位姿差。")
    target_trials = 1

    def start(self, now, obs):
        if obs.get("pose") is None:
            return self.abort(now, "没有位姿（g1d_nav 未运行？）")
        self.state, self.phase, self.pose0 = "running", "driving", obs["pose"]
        self.path_len, self.last = 0.0, obs["pose"]
        self.message = "已记录起点，绕行后回到标记处点「标记终点」"
        return []

    def tick(self, now, obs):
        p = obs.get("pose")
        if self.state == "running" and p:
            self.path_len += math.hypot(p[0] - self.last[0], p[1] - self.last[1])
            self.last = p
        return []

    def mark(self, now, obs):
        p = obs.get("pose")
        if self.state != "running" or p is None:
            return []
        d = math.hypot(p[0] - self.pose0[0], p[1] - self.pose0[1])
        dyaw = abs(math.degrees(wrap(p[2] - self.pose0[2])))
        ok = d < float(self.args.get("max_err_m", 0.10)) and dyaw < float(self.args.get("max_err_deg", 2.0))
        self.trials.append({"error_m": round(d, 4), "error_deg": round(dyaw, 3),
                            "path_m": round(self.path_len, 2), "ok": ok})
        self.state, self.phase = "done", ""
        self.message = f"闭环误差 {d:.3f} m / {dyaw:.2f}°，{'通过' if ok else '未通过'}"
        return []

    def passed(self):
        return self.trials[-1]["ok"] if self.trials else None


class T4RotationDrift(AcceptanceTest):
    id = "T4"
    title = "T4 原地旋转漂移"
    instructions = "周围 0.5 m 内无障碍。点「开始」：自动执行 12 次 +90° 原地旋转（3 圈），比较起终点平移。"

    def start(self, now, obs):
        if obs.get("pose") is None:
            return self.abort(now, "没有位姿（g1d_nav 未运行？）")
        self.turns = int(self.args.get("turns", 12))
        self.state, self.phase, self.pose0, self.done_turns = "running", "rotating", obs["pose"], 0
        self.max_dev = 0.0
        return self._next(now)

    def _next(self, now):
        self.goal_id = f"t4-{self.done_turns}-{uuid.uuid4().hex[:4]}"
        self.t_sent = now
        self.message = f"旋转 {self.done_turns + 1}/{self.turns}"
        return [("goal", {"id": self.goal_id, "mode": "rotate", "frame": "base_at_stamp",
                          "yaw": math.pi / 2, "yaw_tolerance": 0.05})]

    def tick(self, now, obs):
        if self.state != "running":
            return []
        p = obs.get("pose")
        if p:
            self.max_dev = max(self.max_dev, math.hypot(p[0] - self.pose0[0], p[1] - self.pose0[1]))
        st = obs.get("status") or {}
        if st.get("goal_id") != self.goal_id:
            return self.abort(now, "旋转目标无响应") if now - self.t_sent > 5.0 else []
        if st.get("state") == "SUCCEEDED":
            self.done_turns += 1
            if self.done_turns < self.turns:
                return self._next(now)
            d = math.hypot(p[0] - self.pose0[0], p[1] - self.pose0[1]) if p else None
            ok = d is not None and d < float(self.args.get("max_drift_m", 0.05))
            self.trials.append({"drift_m": round(d, 4) if d is not None else None,
                                "max_deviation_m": round(self.max_dev, 4), "turns": self.turns, "ok": ok})
            self.state, self.phase = "done", ""
            self.message = f"平移漂移 {d:.3f} m，{'通过' if ok else '未通过'}"
        elif st.get("state") in ("FAILED", "CANCELED"):
            return self.abort(now, f"旋转失败：{st.get('reason')}")
        return []

    def passed(self):
        return self.trials[-1]["ok"] if self.trials else None


class NavTrials(AcceptanceTest):
    """T5 / T6: every navigation goal finished while the test runs counts as one trial."""
    target_trials = 10
    needs_control = False
    min_rate = 0.9

    def start(self, now, obs):
        self.state, self.phase = "running", "recording"
        self.current: Optional[str] = None
        self.cur_min_clear = math.inf
        self.cur_estop = False
        self.t_goal = now
        self.message = "记录中：从「导航」或「任务」页下发目标，每个结束的目标计为一次试验"
        return []

    def tick(self, now, obs):
        if self.state != "running":
            return []
        st = obs.get("status") or {}
        gid = st.get("goal_id") or ""
        g = obs.get("guard") or {}
        if gid and not gid.startswith("t4-") and st.get("state") in ("NAVIGATING", "RECOVERING", "ROTATING"):
            if gid != self.current:
                self.current, self.cur_min_clear, self.cur_estop, self.t_goal = gid, math.inf, False, now
            fc = g.get("front_clear")
            if fc is not None:
                self.cur_min_clear = min(self.cur_min_clear, fc)
            self.cur_estop = self.cur_estop or bool(g.get("estop"))
        if gid == self.current and st.get("state") in ("SUCCEEDED", "FAILED", "CANCELED"):
            ok = st.get("state") == "SUCCEEDED" and not self.cur_estop
            self.trials.append({"goal_id": gid, "state": st.get("state"), "reason": st.get("reason", ""),
                                "duration_s": round(now - self.t_goal, 1), "estop": self.cur_estop,
                                "min_front_clear_m": (None if math.isinf(self.cur_min_clear)
                                                      else round(self.cur_min_clear, 3)), "ok": ok})
            self.current = None
            n_ok = sum(t["ok"] for t in self.trials)
            self.message = f"{n_ok}/{len(self.trials)} 成功"
        return []

    def passed(self):
        if len(self.trials) < self.target_trials:
            return None
        return sum(t["ok"] for t in self.trials) / len(self.trials) >= self.min_rate


class T5Corridor(NavTrials):
    id = "T5"
    title = "T5 走廊绕障到达率"
    instructions = "走廊中间放纸箱、椅子。开始后下发 ≥ 10 个穿过障碍的目标（可用往返任务），统计成功率（≥ 9/10）。"
    min_rate = 0.9


class T6Overhang(NavTrials):
    id = "T6"
    title = "T6 悬空物（桌沿）不碰撞"
    instructions = "目标设在桌子另一侧，桌沿高于底盘、低于肩部。开始后下发 ≥ 10 个目标，要求 10/10 无碰撞完成或安全拒绝。"
    min_rate = 1.0

    def tick(self, now, obs):
        before = len(self.trials)
        out = super().tick(now, obs)
        if len(self.trials) > before:
            t = self.trials[-1]
            # A safe refusal (no path / blocked, no e-stop) also counts as a pass for T6.
            t["ok"] = not t["estop"] and (t["state"] == "SUCCEEDED" or
                                          t["reason"] in ("no_path", "blocked", "goal_in_obstacle"))
            self.message = f"{sum(x['ok'] for x in self.trials)}/{len(self.trials)} 无碰撞"
        return out


TESTS = {c.id: c for c in (T1StopDistance, T2StreamLoss, T3LoopClosure, T4RotationDrift,
                           T5Corridor, T6Overhang)}


def catalog() -> List[Dict[str, Any]]:
    return [{"id": c.id, "title": c.title, "instructions": c.instructions,
             "target_trials": c.target_trials, "needs_control": c.needs_control}
            for c in TESTS.values()]
