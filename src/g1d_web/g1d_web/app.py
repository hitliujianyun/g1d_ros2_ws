"""g1d_web application: browser clients, control lock, command dispatch and streaming.

One asyncio loop; DDS data comes from Hub (reader threads). Only the client holding the control
lock may command the robot; anyone may press the e-stop.
"""
from __future__ import annotations

import asyncio
import collections
import itertools
import json
import logging
import math
import os
import struct
import time
import uuid
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from g1d_nav_py.config import get

from . import acceptance, mapping, mission, params
from .hub import Hub, base_to_odom
from .wsproto import OP_TEXT, WebSocket, WebSocketClosed

log = logging.getLogger("g1d.web")

K_SCAN, K_MAP, K_MAP_RESET, K_OBST, K_CHASSIS, K_LOCAL, K_GLOBAL = 1, 2, 3, 4, 5, 6, 7
TICK_HZ = 20.0
MAX_STREAM_BACKLOG = 400
GOAL_KEYS = {"id", "mode", "frame", "stamp", "x", "y", "yaw", "use_yaw", "xy_tolerance",
             "yaw_tolerance", "max_speed", "timeout_s"}


def pack(kind: int, header: Dict[str, Any], payload: bytes = b"") -> bytes:
    h = json.dumps(header, separators=(",", ":")).encode()
    pad = (-(8 + len(h))) % 4
    return struct.pack("<B3xI", kind, len(h) + pad) + h + b" " * pad + payload


def clean_goal(g: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(g, dict):
        raise ValueError("goal must be an object")
    out = {k: v for k, v in g.items() if k in GOAL_KEYS and v is not None and v != ""}
    out.setdefault("mode", "navigate")
    out.setdefault("frame", "odom")
    if out["mode"] not in ("navigate", "rotate", "simple", "stop"):
        raise ValueError("bad mode")
    if out["frame"] not in ("odom", "base_at_stamp"):
        raise ValueError("bad frame")
    for k in ("x", "y", "yaw", "xy_tolerance", "yaw_tolerance", "max_speed", "timeout_s", "stamp"):
        if k in out:
            out[k] = float(out[k])
            if not math.isfinite(out[k]):
                raise ValueError(f"{k} must be finite")
    if out["mode"] in ("navigate", "simple") and not ("x" in out and "y" in out):
        raise ValueError("navigate / simple goals need x and y")
    if out["mode"] == "rotate" and "yaw" not in out:
        raise ValueError("rotate goals need yaw")
    out["id"] = str(out.get("id") or f"web-{uuid.uuid4().hex[:8]}")
    return out


class Client:
    _ids = itertools.count(1)

    def __init__(self, ws: WebSocket, addr: str):
        self.ws = ws
        self.id = f"c{next(self._ids)}"
        self.addr = addr
        self.name = addr
        self.text: Deque[str] = collections.deque()
        self.stream: Deque[bytes] = collections.deque()
        self.slots: Dict[int, bytes] = {}
        self.event = asyncio.Event()
        self.needs_full_map = True

    def push_text(self, obj: Dict[str, Any]) -> None:
        self.text.append(json.dumps(obj, separators=(",", ":"), default=float))
        self.event.set()

    def push_stream(self, data: bytes) -> None:
        if len(self.stream) > MAX_STREAM_BACKLOG:
            self.stream.clear()
            self.needs_full_map = True
            return
        self.stream.append(data)
        self.event.set()

    def push_slot(self, kind: int, data: bytes) -> None:
        self.slots[kind] = data
        self.event.set()


class WebApp:
    def __init__(self, cfg: Dict[str, Any], hub: Hub, config_path: str, root: str):
        self.cfg = cfg
        self.hub = hub
        self.config_path = config_path
        self.root = root
        w = cfg.get("web", {}) or {}
        self.maps_dir = os.path.join(root, w.get("maps_dir", "maps"))
        self.missions_dir = os.path.join(root, w.get("missions_dir", "missions"))
        self.tests_dir = os.path.join(root, w.get("tests_dir", "logs/tests"))
        self.scan_max = int(w.get("scan_max_points", 30000))
        self.grid_res = float(w.get("grid_resolution", 0.05))
        self.grid_z = (float(w.get("grid_z_min", 0.10)), float(w.get("grid_z_max", 1.80)))
        self.builder = mapping.MapBuilder(float(w.get("map_voxel", 0.05)),
                                          int(w.get("map_max_points", 3_000_000)))
        self.map_paused_reason = ""
        self.map_enabled = True
        self.new_map: Deque[np.ndarray] = collections.deque()
        self.latest_scan: Optional[np.ndarray] = None
        self.scan_version = 0
        self.sent_scan_version = 0
        self.clients: Dict[str, Client] = {}
        self.owner: Optional[str] = None
        self.events: Deque[Dict[str, Any]] = collections.deque(maxlen=200)
        self.mission = mission.MissionRunner(self._mission_goal, self._send_stop, self.log_event)
        self.test: Optional[acceptance.AcceptanceTest] = None
        self.test_file: Optional[str] = None
        self.seen: Dict[str, int] = collections.defaultdict(int)
        self.last_teleop = 0.0
        self.hub.cloud_listeners.append(self._on_cloud)

    # ---- helpers ----------------------------------------------------------------------------
    def now(self) -> float:
        return self.hub.clock()

    def log_event(self, level: str, text: str) -> None:
        ev = {"t": time.time(), "level": level, "text": text}
        self.events.append(ev)
        getattr(log, "warning" if level == "warn" else "info")(text)
        for c in self.clients.values():
            c.push_text({"type": "event", **ev})

    def broadcast(self, obj: Dict[str, Any]) -> None:
        for c in self.clients.values():
            c.push_text(obj)

    def hello(self, c: Client) -> Dict[str, Any]:
        cfg = self.cfg
        return {
            "type": "hello", "client_id": c.id,
            "config": {
                "footprint": get(cfg, "robot.footprint", []),
                "rotate_radius": get(cfg, "robot.rotate_radius", 0.38),
                "chassis_laser": get(cfg, "robot.chassis_laser.xyz", [0.19, 0, 0.1]),
                "guard": {k: get(cfg, f"guard.{k}") for k in
                          ("front_x", "rear_x", "half_width", "stop_m", "slow_m", "teleop_vx_max",
                           "teleop_wz_max")},
                "voxel": self.builder.voxel, "scan_max_points": self.scan_max,
                "mppi": {k: get(cfg, f"mppi.{k}") for k in ("vx_max", "wz_max")},
            },
            "tests": acceptance.catalog(),
            "events": list(self.events)[-50:],
        }

    # ---- DDS thread: LIO clouds -> map ------------------------------------------------------
    def _on_cloud(self, pts_lio: np.ndarray) -> None:
        r, t, valid = self.hub.T_odom_lio()
        pts = (pts_lio.astype(np.float64) @ r.T + t).astype(np.float32)
        if pts.shape[0] > self.scan_max:
            pts_view = pts[:: int(math.ceil(pts.shape[0] / self.scan_max))]
        else:
            pts_view = pts
        self.latest_scan = (pts_view, valid)
        self.scan_version += 1
        if not valid:
            self.map_paused_reason = "等待 g1d_nav 的 T_odom_lio（rt/nav/debug）"
            return
        self.map_paused_reason = "" if self.map_enabled else "已暂停"
        if self.map_enabled:
            new = self.builder.add_cloud(pts)
            if new.shape[0]:
                self.new_map.append(new)

    # ---- websocket session ------------------------------------------------------------------
    async def session(self, ws: WebSocket, addr: str) -> None:
        c = Client(ws, addr)
        self.clients[c.id] = c
        c.push_text(self.hello(c))
        c.push_text(self.params_msg())
        sender = asyncio.ensure_future(self._sender(c))
        self.log_event("info", f"客户端 {c.id} ({addr}) 已连接")
        try:
            while True:
                op, data = await ws.recv()
                if op != OP_TEXT:
                    continue
                try:
                    msg = json.loads(data.decode())
                    reply = self.handle(c, msg)
                except (ValueError, KeyError, TypeError) as e:
                    reply = {"type": "error", "text": str(e)}
                if reply:
                    c.push_text(reply)
        except WebSocketClosed:
            pass
        finally:
            sender.cancel()
            self.clients.pop(c.id, None)
            if self.owner == c.id:
                self.owner = None
                self.log_event("warn", f"控制者 {c.id} 断开，控制权已释放")
            await ws.close()

    async def _sender(self, c: Client) -> None:
        try:
            while not c.ws.closed:
                await c.event.wait()
                c.event.clear()
                if c.needs_full_map:
                    c.needs_full_map = False
                    await self._send_full_map(c)
                while c.text:
                    await c.ws.send_text(c.text.popleft())
                while c.stream:
                    await c.ws.send_bytes(c.stream.popleft())
                if c.ws.buffered() > 2_000_000:
                    await asyncio.sleep(0.05)
                    c.event.set()
                    continue
                for kind in list(c.slots):
                    data = c.slots.pop(kind, None)
                    if data is not None:
                        await c.ws.send_bytes(data)
        except (WebSocketClosed, asyncio.CancelledError):
            return

    async def _send_full_map(self, c: Client) -> None:
        c.stream.clear()
        await c.ws.send_bytes(pack(K_MAP_RESET, {"voxel": self.builder.voxel}))
        pts = self.builder.points()
        for i in range(0, pts.shape[0], 200_000):
            await c.ws.send_bytes(pack(K_MAP, {"n": int(min(200_000, pts.shape[0] - i)), "full": True},
                                       pts[i:i + 200_000].tobytes()))

    # ---- commands ---------------------------------------------------------------------------
    def _require_control(self, c: Client) -> None:
        if self.owner != c.id:
            raise ValueError("需要控制权（点击「获取控制权」）")

    def _send_stop(self) -> None:
        self.hub.send_goal({"id": f"stop-{uuid.uuid4().hex[:6]}", "mode": "stop"})

    def _mission_goal(self, goal: Dict[str, Any]) -> None:
        self.hub.send_goal(goal)

    def params_msg(self) -> Dict[str, Any]:
        return {"type": "params", "guard": self.hub.params.get("guard", {}),
                "nav": self.hub.params.get("nav", {}), "acks": list(self.hub.param_acks)[-10:]}

    def handle(self, c: Client, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        t = msg.get("type")
        now = self.now()
        if t == "ping":
            return {"type": "pong", "t": msg.get("t")}
        if t == "hello":
            c.name = str(msg.get("name") or c.addr)[:40]
            return None
        if t == "control":
            if msg.get("action") == "release":
                if self.owner == c.id:
                    self.owner = None
                    self.log_event("info", f"{c.id} 释放控制权")
            elif self.owner in (None, c.id) or msg.get("force"):
                prev = self.owner
                self.owner = c.id
                self.log_event("info", f"{c.id}（{c.name}）获得控制权" + (f"，接管自 {prev}" if prev and prev != c.id else ""))
            else:
                raise ValueError(f"控制权在 {self.owner} 手中（可选择强制接管）")
            return None
        if t == "estop":
            on = bool(msg.get("on"))
            if not on:
                self._require_control(c)
            self.hub.send_estop(on)
            if on:
                self.mission.pause(now, "急停")
                if self.test and self.test.state == "running":
                    self._apply(self.test.abort(now, "急停"))
            self.log_event("warn", f"{c.id} {'按下急停' if on else '解除急停'}")
            return None
        if t == "map" and msg.get("action") == "export":
            return self.export_map()
        if t == "map" and msg.get("action") == "list":
            return {"type": "exports", "items": mapping.list_exports(self.maps_dir)}
        if t == "mission" and msg.get("action") == "list":
            return {"type": "missions", "items": mission.list_missions(self.missions_dir)}
        if t == "tests" and msg.get("action") == "list":
            return {"type": "test_results", "items": self.list_test_results()}

        self._require_control(c)
        if t == "goal":
            goal = clean_goal(msg.get("goal") or {})
            if self.mission.active:
                self.mission.cancel(now)
            self.hub.send_goal(goal)
            self.log_event("info", f"下发目标 {goal['id']}：{goal['mode']} "
                                   f"({goal.get('x', 0):.2f}, {goal.get('y', 0):.2f}"
                                   + (f", {goal['yaw']:.2f}" if "yaw" in goal else "") + ")")
            return {"type": "goal_sent", "goal": goal}
        if t == "stop":
            self.mission.cancel(now)
            if self.test and self.test.state == "running" and self.test.needs_control:
                self._apply(self.test.abort(now, "取消"))
            self._send_stop()
            self.hub.send_teleop(0.0, 0.0)
            self.log_event("info", f"{c.id} 取消导航")
            return None
        if t == "teleop":
            v = max(-1.0, min(1.0, float(msg.get("v", 0.0))))
            w = max(-2.0, min(2.0, float(msg.get("w", 0.0))))
            self.hub.send_teleop(v, w)
            self.last_teleop = now
            return None
        if t == "mission":
            a = msg.get("action")
            if a == "start":
                self.mission.start(msg.get("mission") or {}, now)
            elif a == "pause":
                self.mission.pause(now)
            elif a == "resume":
                self.mission.resume(now)
            elif a == "cancel":
                self.mission.cancel(now)
            elif a == "save":
                m = mission.save_mission(self.missions_dir, msg.get("mission") or {})
                self.log_event("info", f"任务 '{m['name']}' 已保存")
                return {"type": "missions", "items": mission.list_missions(self.missions_dir)}
            elif a == "delete":
                mission.delete_mission(self.missions_dir, str(msg.get("name")))
                return {"type": "missions", "items": mission.list_missions(self.missions_dir)}
            else:
                raise ValueError("unknown mission action")
            return None
        if t == "map":
            a = msg.get("action")
            if a == "reset":
                self.builder.reset(float(msg["voxel"]) if msg.get("voxel") else None)
                self.new_map.clear()
                for cl in self.clients.values():
                    cl.needs_full_map = True
                    cl.event.set()
                self.log_event("info", f"地图已重置（体素 {self.builder.voxel} m）")
            elif a == "pause":
                self.map_enabled = not bool(msg.get("paused"))
            else:
                raise ValueError("unknown map action")
            return None
        if t == "param":
            target = msg.get("target")
            if target not in ("guard", "nav"):
                raise ValueError("target must be guard / nav")
            self.hub.send_param(target, str(msg["name"]), float(msg["value"]), uuid.uuid4().hex[:6])
            self.log_event("info", f"参数 {target}.{msg['name']} = {msg['value']}")
            return None
        if t == "param_save":
            target = msg.get("target")
            values = self.hub.params.get(target)
            if not values:
                raise ValueError(f"还没有收到 {target} 的参数")
            res = params.save_params(self.config_path, target, values)
            self.log_event("info", f"{target} 参数已写入配置（{len(res['changed'])} 项，备份 "
                                   f"{os.path.basename(res['backup'])}）")
            return {"type": "param_saved", **res}
        if t == "test":
            return self.handle_test(msg, now)
        raise ValueError(f"unknown message type {t!r}")

    # ---- acceptance tests -------------------------------------------------------------------
    def obs(self) -> Dict[str, Any]:
        p, _ = self.hub.pose()
        v, w = self.hub.velocity()
        return {"pose": (p["x"], p["y"], p["yaw"]) if p else None, "v": v, "w": w,
                "guard": self.hub.json.get("guard") or {}, "status": self.hub.json.get("status") or {}}

    def handle_test(self, msg: Dict[str, Any], now: float) -> Optional[Dict[str, Any]]:
        a = msg.get("action")
        if a == "select":
            tid = msg.get("test")
            if tid not in acceptance.TESTS:
                raise ValueError("unknown test")
            if self.test and self.test.state == "running":
                raise ValueError("当前测试仍在进行")
            self.test = acceptance.TESTS[tid](msg.get("args") or {})
            self.test_file = os.path.join(
                self.tests_dir, f"{tid}-{time.strftime('%Y%m%d-%H%M%S')}.json")
            return None
        if self.test is None:
            raise ValueError("先选择一个测试")
        if a == "start":
            self._apply(self.test.start(now, self.obs()))
        elif a == "mark":
            self._apply(self.test.mark(now, self.obs()))
        elif a == "stop":
            self._apply(self.test.stop(now))
        elif a == "abort":
            self._apply(self.test.abort(now))
        else:
            raise ValueError("unknown test action")
        self.save_test()
        return None

    def _apply(self, actions: List) -> None:
        for act in actions:
            if act[0] == "teleop":
                self.hub.send_teleop(act[1], act[2])
            elif act[0] == "goal":
                self.hub.send_goal(act[1])
            elif act[0] == "stop_nav":
                self._send_stop()

    def save_test(self) -> None:
        if not self.test or not self.test_file:
            return
        os.makedirs(self.tests_dir, exist_ok=True)
        with open(self.test_file, "w", encoding="utf-8") as f:
            json.dump(self.test.summary(), f, indent=2, ensure_ascii=False)

    def list_test_results(self) -> List[Dict[str, Any]]:
        out = []
        if os.path.isdir(self.tests_dir):
            for fn in sorted(os.listdir(self.tests_dir), reverse=True)[:100]:
                if fn.endswith(".json"):
                    try:
                        with open(os.path.join(self.tests_dir, fn), encoding="utf-8") as f:
                            s = json.load(f)
                        out.append({"file": fn, "id": s.get("id"), "passed": s.get("passed"),
                                    "trials": len(s.get("trials", [])), "state": s.get("state")})
                    except (OSError, ValueError):
                        continue
        return out

    # ---- map export -------------------------------------------------------------------------
    def export_map(self) -> Dict[str, Any]:
        cm = self.hub.grids.get("global_costmap")
        costmap = None
        if cm:
            costmap = {"width": cm["width"], "height": cm["height"], "resolution": cm["resolution"],
                       "origin": cm["origin"], "data": np.frombuffer(cm["data"], np.uint8)}
        _, _, valid = self.hub.T_odom_lio()
        meta = mapping.export_map(self.builder, self.maps_dir, resolution=self.grid_res,
                                  z_min=self.grid_z[0], z_max=self.grid_z[1], costmap=costmap,
                                  extra_meta={"T_odom_lio_valid": valid})
        self.log_event("info", f"地图已导出 maps/{meta['name']}（{meta['points']} 点）")
        return {"type": "exports", "items": mapping.list_exports(self.maps_dir), "new": meta["name"]}

    # ---- periodic loop ----------------------------------------------------------------------
    def _changed(self, key: str) -> bool:
        v = self.hub.version.get(key, 0)
        if v != self.seen[key]:
            self.seen[key] = v
            return True
        return False

    async def run(self) -> None:
        k = 0
        period = 1.0 / TICK_HZ
        while True:
            t0 = time.monotonic()
            try:
                self.step(k)
            except Exception:  # noqa: BLE001 - keep serving whatever happens
                log.exception("tick failed")
            k += 1
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def step(self, k: int) -> None:
        now = self.now()
        hub = self.hub
        if self._changed("status"):
            self.mission.on_status(hub.json.get("status") or {}, now)
        self.mission.tick(now)
        if self.test and self.test.state in ("running",):
            n_before = len(self.test.trials)
            state_before = self.test.state
            self._apply(self.test.tick(now, self.obs()))
            if len(self.test.trials) != n_before or self.test.state != state_before:
                self.save_test()
        p, source = hub.pose()
        if p and source == "nav" and k % 4 == 0:
            self.builder.add_pose(time.time(), p["x"], p["y"], p["yaw"])
        if not self.clients:
            return
        # streams
        if self.scan_version != self.sent_scan_version and self.latest_scan is not None:
            self.sent_scan_version = self.scan_version
            pts, valid = self.latest_scan
            data = pack(K_SCAN, {"n": int(pts.shape[0]), "frame": "odom" if valid else "odom_lio"},
                        pts.tobytes())
            for c in self.clients.values():
                c.push_slot(K_SCAN, data)
        if self.new_map:
            chunks = []
            while self.new_map:
                chunks.append(self.new_map.popleft())
            pts = np.concatenate(chunks)
            data = pack(K_MAP, {"n": int(pts.shape[0])}, pts.tobytes())
            for c in self.clients.values():
                if not c.needs_full_map:
                    c.push_stream(data)
        for key, kind in (("obstacles", K_OBST), ("chassis", K_CHASSIS)):
            if self._changed(key) and p:
                _, pts = hub.clouds[key]
                pts = pts[:: max(1, pts.shape[0] // 8000 + 1)]
                data = pack(kind, {"n": int(pts.shape[0])},
                            base_to_odom(pts, (p["x"], p["y"], p["yaw"])).tobytes())
                for c in self.clients.values():
                    c.push_slot(kind, data)
        for key, kind in (("local_costmap", K_LOCAL), ("global_costmap", K_GLOBAL)):
            if self._changed(key):
                g = hub.grids[key]
                data = pack(kind, {kk: g[kk] for kk in ("width", "height", "resolution", "origin")},
                            g["data"])
                for c in self.clients.values():
                    c.push_slot(kind, data)
        if self._changed("param_state"):
            self.broadcast(self.params_msg())
        if k % 2 == 0:
            self.broadcast_state(include_rates=(k % 20 == 0))

    def broadcast_state(self, include_rates: bool) -> None:
        hub = self.hub
        p, source = hub.pose()
        _, _, t_valid = hub.T_odom_lio()
        base = {
            "type": "state", "t": time.time(),
            "status": hub.json.get("status"), "guard": hub.json.get("guard"),
            "lio": hub.json.get("lio"), "debug": hub.json.get("debug"),
            "pose": dict(p, source=source) if p else None,
            "fresh": {k: hub.fresh(k, 1.0) for k in ("status", "guard", "lio", "debug", "cloud",
                                                      "chassis", "wheel_odom", "base_odom")},
            "cmd": {"nav": hub.cmd.get("cmd_vel"), "out": hub.cmd.get("cmd_out")},
            "mission": self.mission.snapshot(),
            "test": self.test.summary() if self.test else None,
            "map": dict(self.builder.stats(), paused=self.map_paused_reason, enabled=self.map_enabled,
                        T_valid=t_valid),
            "control": {"owner": self.owner,
                        "owner_name": self.clients[self.owner].name if self.owner in self.clients else None,
                        "clients": len(self.clients)},
        }
        if include_rates:
            base["rates"] = hub.rate_table()
        for c in self.clients.values():
            msg = dict(base)
            msg["control"] = dict(base["control"], you=(self.owner == c.id))
            c.push_text(msg)
