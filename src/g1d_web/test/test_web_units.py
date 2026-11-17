import asyncio
import json
import math
import os
import struct
import zlib

import numpy as np
import pytest

from g1d_web import acceptance, mapping, mission, params
from g1d_web.app import clean_goal, pack
from g1d_web.wsproto import (OP_BINARY, OP_TEXT, WebSocket, WebSocketClosed, accept_key,
                             encode_frame)


# ------------------------------------------------------------------------------------ wsproto
def test_accept_key_rfc6455_example():
    assert accept_key("dGhlIHNhbXBsZSBub25jZQ==") == "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="


class _Writer:
    def __init__(self):
        self.buf = b""
        self.transport = None

    def write(self, b):
        self.buf += b

    async def drain(self):
        pass

    def close(self):
        pass


def _ws_from(data: bytes):
    async def make():
        r = asyncio.StreamReader()
        r.feed_data(data)
        r.feed_eof()
        return WebSocket(r, _Writer())
    return make


def test_ws_masked_fragmented_and_ping():
    mask = b"\x11\x22\x33\x44"
    big = bytes(range(256)) * 300
    frames = (encode_frame(0x9, b"hi", mask)                                  # ping
              + struct.pack("!BB", 0x01, 0x80 | 5) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(b"hello"))
              + encode_frame(0x0, b" world", mask)                            # continuation (FIN)
              + encode_frame(OP_BINARY, big, mask))

    async def run():
        ws = await _ws_from(frames)()
        op1, d1 = await ws.recv()
        op2, d2 = await ws.recv()
        with pytest.raises(WebSocketClosed):
            await ws.recv()
        return ws, (op1, d1), (op2, d2)

    ws, m1, m2 = asyncio.run(run())
    assert m1 == (OP_TEXT, b"hello world")
    assert m2 == (OP_BINARY, big)
    assert ws.writer.buf.startswith(b"\x8a\x02hi")                            # pong echoed the ping


def test_pack_alignment():
    for n in range(0, 9):
        b = pack(2, {"n": n, "x": "a" * n}, np.zeros(3, np.float32).tobytes())
        hl = struct.unpack("<I", b[4:8])[0]
        assert (8 + hl) % 4 == 0
        assert json.loads(b[8:8 + hl].decode())["n"] == n
        assert len(b) - (8 + hl) == 12


# ------------------------------------------------------------------------------------ mapping
def test_map_builder_dedupes_voxels_and_caps():
    mb = mapping.MapBuilder(voxel=0.1, max_points=50)
    a = np.array([[0.01, 0.01, 0.0], [0.05, 0.02, 0.03], [0.5, 0.0, 0.0]], np.float32)
    new = mb.add_cloud(a)
    assert new.shape[0] == 2
    assert mb.add_cloud(a).shape[0] == 0
    many = np.c_[np.arange(100) * 0.2, np.zeros(100), np.zeros(100)].astype(np.float32)
    mb.add_cloud(many)
    assert mb.stats()["points"] == 50 and mb.stats()["full"]
    assert mb.points().shape == (50, 3)
    with pytest.raises(ValueError):
        mb.reset(5.0)


def test_occupancy_and_export(tmp_path):
    mb = mapping.MapBuilder(voxel=0.05)
    xs = np.arange(-1.0, 3.0, 0.02)
    floor = np.c_[np.repeat(xs, 50), np.tile(np.linspace(-1, 1, 50), xs.size), np.zeros(xs.size * 50)]
    wall = np.c_[np.full(400, 3.0), np.linspace(-1, 1, 400), np.full(400, 1.0)]
    mb.add_cloud(np.vstack([floor, wall]).astype(np.float32))
    mb.add_pose(0.0, 0.0, 0.0, 0.0)
    mb.add_pose(0.01, 0.01, 0.0, 0.0)          # below min_step: dropped
    mb.add_pose(1.0, 1.0, 0.0, 0.0)
    img, origin = mb.occupancy(0.05)
    assert (img == 0).any() and (img == 254).any() and (img == 205).any()
    # the wall at x = 3 is occupied in the image column for x = 3
    col = int((3.0 - origin[0]) / 0.05)
    assert (img[:, col] == 0).sum() > 30
    cm = {"width": 4, "height": 2, "resolution": 0.1, "origin": [0, 0],
          "data": np.array([0, 50, 100, 255, 0, 0, 0, 0], np.uint8)}
    meta = mapping.export_map(mb, str(tmp_path), costmap=cm)
    out = tmp_path / meta["name"]
    for f in ("map.pcd", "grid.png", "grid.yaml", "trajectory.csv", "costmap.png", "meta.json"):
        assert (out / f).is_file(), f
    pts = mapping.read_pcd_binary(str(out / "map.pcd"))
    assert pts.shape[0] == meta["points"]
    png = (out / "grid.png").read_bytes()
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    w, h = struct.unpack(">II", png[16:24])
    assert (w, h) == tuple(meta["grid_size"])
    idat = png[png.index(b"IDAT") + 4:png.index(b"IEND") - 8]
    assert len(zlib.decompress(idat)) == h * (w + 1)
    assert "origin: [" in (out / "grid.yaml").read_text()
    assert (out / "trajectory.csv").read_text().count("\n") == 3
    assert mapping.list_exports(str(tmp_path))[0]["name"] == meta["name"]


def test_costmap_image_values():
    img = mapping.costmap_image(3, 1, np.array([0, 100, 255], np.uint8))
    assert img.tolist() == [[254, 0, 205]]


# ------------------------------------------------------------------------------------- params
YAML = """manager:
  progress_window_s: 8.0
  approach_v_max: 0.2            # final approach speed
  max_speed: 0.0                   # default cap

guard:                             # guard comment
  stop_m: 0.45
  slow_m: 1.20                     # slow zone
  require_cloud: true              # P1

debug:
  publish_costmap: true
"""


def test_yaml_update_keeps_comments_and_inserts():
    text, changed = params.update_yaml_values(YAML, "guard", {"stop_m": 0.5, "slow_m": 1.35,
                                                              "require_cloud": False, "creep_v": 0.04})
    assert set(changed) == {"stop_m", "slow_m", "require_cloud", "creep_v"}
    assert "  stop_m: 0.5\n" in text
    assert "  slow_m: 1.35                     # slow zone\n" in text
    assert "  require_cloud: false             # P1\n" in text      # comment column kept
    assert "  creep_v: 0.04\n\ndebug:" in text
    assert "guard:                             # guard comment" in text
    assert text.count("approach_v_max: 0.2            # final approach speed") == 1


def test_save_params_writes_backup(tmp_path):
    p = tmp_path / "cfg.yaml"
    p.write_text(YAML, encoding="utf-8")
    res = params.save_params(str(p), "nav", {"approach_v_max": 0.15, "mppi_vx_max": 0.4, "max_speed": 0.3})
    assert "mppi_vx_max" not in res["changed"]
    assert os.path.isfile(res["backup"])
    t = p.read_text(encoding="utf-8")
    assert "approach_v_max: 0.15" in t and "  max_speed: 0.3 " in t
    import yaml
    assert yaml.safe_load(t)["manager"]["approach_v_max"] == 0.15


def test_format_value():
    assert params.format_value(1.0) == "1.0"
    assert params.format_value(0.45) == "0.45"
    assert params.format_value(True) == "true"


# ------------------------------------------------------------------------------------ mission
class Nav:
    def __init__(self):
        self.goals, self.stops = [], 0

    def goal(self, g):
        self.goals.append(g)

    def stop(self):
        self.stops += 1


def _finish(r, state, reason="", t=1.0):
    r.on_status({"goal_id": r.goal_id, "state": "NAVIGATING"}, t)
    r.on_status({"goal_id": r.goal_id, "state": state, "reason": reason}, t)


WPS = [{"x": 1, "y": 0}, {"x": 2, "y": 1, "yaw": 1.0, "wait_s": 2.0}, {"x": 0, "y": 0}]


def test_mission_sequence_wait_and_loops():
    nav = Nav()
    r = mission.MissionRunner(nav.goal, nav.stop)
    r.start({"name": "m1", "waypoints": WPS, "loops": 2}, 0.0)
    assert nav.goals[-1]["frame"] == "odom" and nav.goals[-1]["x"] == 1.0
    _finish(r, "SUCCEEDED")
    assert len(nav.goals) == 2 and nav.goals[-1]["yaw"] == 1.0
    _finish(r, "SUCCEEDED", t=2.0)
    assert r.state == "waiting"
    r.tick(3.0)
    assert len(nav.goals) == 2
    r.tick(4.1)
    assert len(nav.goals) == 3
    _finish(r, "SUCCEEDED", t=5.0)
    assert r.loop == 1 and r.index == 0 and len(nav.goals) == 4
    for t in (6, 7, 8):
        _finish(r, "SUCCEEDED", t=t)
        r.tick(t + 2.5)
    assert r.state == "done" and len(r.results) == 6


def test_mission_failure_policies_and_manual_override():
    nav = Nav()
    r = mission.MissionRunner(nav.goal, nav.stop)
    r.start({"name": "m2", "waypoints": WPS[:2], "on_fail": "retry", "retries": 1}, 0.0)
    _finish(r, "FAILED", "blocked")
    assert r.attempt == 1 and r.index == 0 and len(nav.goals) == 2
    _finish(r, "FAILED", "blocked")
    assert r.index == 1                       # retry exhausted -> skipped
    _finish(r, "CANCELED", "manual_override")
    assert r.state == "paused"
    r.resume(5.0)
    assert r.state == "running" and nav.goals[-1]["x"] == 2.0
    _finish(r, "FAILED", "guard_estop")
    assert r.state == "failed"

    r.start({"name": "m3", "waypoints": WPS, "on_fail": "stop"}, 10.0)
    _finish(r, "FAILED", "no_path")
    assert r.state == "failed" and "no_path" in r.reason


def test_mission_pause_cancel_and_no_ack(tmp_path):
    nav = Nav()
    r = mission.MissionRunner(nav.goal, nav.stop)
    r.start({"name": "m4", "waypoints": WPS}, 0.0)
    r.pause(0.5)
    assert r.state == "paused" and nav.stops == 1
    r.on_status({"goal_id": r.goal_id, "state": "CANCELED", "reason": "stop"}, 0.6)
    assert r.state == "paused"
    r.cancel(1.0)
    assert r.state == "canceled"
    r.start({"name": "m4", "waypoints": WPS}, 10.0)
    r.tick(14.0)
    assert r.state == "failed" and "no status" in r.reason
    with pytest.raises(ValueError):
        mission.validate_mission({"name": "../x", "waypoints": WPS})
    with pytest.raises(ValueError):
        mission.validate_mission({"name": "ok", "waypoints": []})
    mission.save_mission(str(tmp_path), {"name": "lab loop", "waypoints": WPS})
    assert mission.list_missions(str(tmp_path))[0]["waypoints"] == 3
    assert mission.load_mission(str(tmp_path), "lab loop")["waypoints"][1]["wait_s"] == 2.0
    mission.delete_mission(str(tmp_path), "lab loop")
    assert mission.list_missions(str(tmp_path)) == []


# --------------------------------------------------------------------------------- acceptance
def test_t1_stop_distance_trial():
    t = acceptance.T1StopDistance({"speed": 0.3})
    acts = t.start(0.0, {"pose": (0, 0, 0)})
    assert acts == [("teleop", 0.3, 0.0)]
    now = 0.0
    for _ in range(20):
        now += 0.05
        assert t.tick(now, {"pose": (now * 0.3, 0, 0), "v": 0.3, "guard": {"blocked": ""}})[0][1] == 0.3
    out = []
    for _ in range(20):
        now += 0.05
        out = t.tick(now, {"pose": (1.5, 0, 0), "v": 0.0, "guard": {"blocked": "front", "front_clear": 0.43}})
        if t.state == "ready":
            break
    assert out == [("teleop", 0.0, 0.0)]
    assert t.trials[-1]["ok"] and t.trials[-1]["front_clear_m"] == 0.43
    assert t.passed() is None                     # fewer than 10 trials


def test_t2_stop_time():
    t = acceptance.T2StreamLoss({"drive_s": 1.0})
    t.start(0.0, {})
    now = 0.0
    while now < 1.0:
        now += 0.05
        t.tick(now, {"v": 0.3})
    t.tick(now + 0.05, {"v": 0.3})
    t.tick(now + 0.3, {"v": 0.1})
    t.tick(now + 0.45, {"v": 0.0})
    assert t.trials[-1]["ok"] and t.trials[-1]["stop_time_s"] == pytest.approx(0.45, abs=0.06)


def test_t3_and_t4():
    t3 = acceptance.T3LoopClosure()
    t3.start(0.0, {"pose": (0.0, 0.0, 0.0)})
    t3.tick(1.0, {"pose": (5.0, 0.0, 0.0)})
    t3.mark(2.0, {"pose": (0.05, -0.03, math.radians(1.0))})
    assert t3.trials[0]["ok"] and t3.trials[0]["path_m"] == 5.0
    t4 = acceptance.T4RotationDrift({"turns": 2})
    acts = t4.start(0.0, {"pose": (1.0, 1.0, 0.0)})
    assert acts[0][0] == "goal" and acts[0][1]["mode"] == "rotate"
    gid = acts[0][1]["id"]
    acts = t4.tick(1.0, {"pose": (1.0, 1.0, 1.0), "status": {"goal_id": gid, "state": "SUCCEEDED"}})
    gid2 = acts[0][1]["id"]
    t4.tick(2.0, {"pose": (1.01, 1.0, 3.1), "status": {"goal_id": gid2, "state": "SUCCEEDED"}})
    assert t4.state == "done" and t4.trials[0]["ok"] and t4.trials[0]["drift_m"] == pytest.approx(0.01)


def test_t5_t6_trial_recording():
    t5 = acceptance.T5Corridor()
    t5.start(0.0, {})
    for i in range(10):
        gid = f"g{i}"
        t5.tick(i + 0.1, {"status": {"goal_id": gid, "state": "NAVIGATING"}, "guard": {"front_clear": 0.8}})
        end = "SUCCEEDED" if i else "FAILED"
        t5.tick(i + 0.5, {"status": {"goal_id": gid, "state": end, "reason": "blocked"}, "guard": {}})
    assert len(t5.trials) == 10 and t5.passed() is True
    t6 = acceptance.T6Overhang()
    t6.start(0.0, {})
    t6.tick(0.1, {"status": {"goal_id": "a", "state": "NAVIGATING"}, "guard": {}})
    t6.tick(0.2, {"status": {"goal_id": "a", "state": "FAILED", "reason": "no_path"}, "guard": {}})
    t6.tick(0.3, {"status": {"goal_id": "b", "state": "NAVIGATING"}, "guard": {"estop": True}})
    t6.tick(0.4, {"status": {"goal_id": "b", "state": "FAILED", "reason": "guard_estop"}, "guard": {}})
    assert [x["ok"] for x in t6.trials] == [True, False]


# -------------------------------------------------------------------------------------- goals
def test_clean_goal():
    g = clean_goal({"x": "1.5", "y": 2, "yaw": None, "evil": 1})
    assert g["frame"] == "odom" and g["x"] == 1.5 and "yaw" not in g and "evil" not in g
    assert g["id"].startswith("web-")
    with pytest.raises(ValueError):
        clean_goal({"mode": "rotate"})
    with pytest.raises(ValueError):
        clean_goal({"x": float("nan"), "y": 0})
    with pytest.raises(ValueError):
        clean_goal({"x": 1, "y": 1, "frame": "map"})


# ----------------------------------------------------------------------------------- fake nav
def test_fake_nav_planner_and_lidar():
    from g1d_nav_py.fake_nav import FakeNav, StaticCostmap, make_world3d, raycast3d
    boxes = make_world3d("lab")
    cm = StaticCostmap(boxes)
    path = cm.astar((0.0, 0.0), (4.0, 0.0))
    assert path is not None and path[-1] == (4.0, 0.0)
    assert all(cm.at(x, y) < 253 for x, y in path[1:-1])
    assert max(abs(y) for _, y in path) > 0.3                # detours around the box at x = 2..2.6
    assert cm.at(2.3, 0.0) == 254 and cm.at(5.0, 2.0) == 254  # box and table top are lethal
    dirs = np.array([[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    r = raycast3d(np.array([0.0, 0.0, 0.5]), dirs, boxes, 20.0)          # box is 0.8 m tall
    assert r.tolist() == pytest.approx([0.5, 2.0, 2.3])
    r = raycast3d(np.array([0.0, 0.0, 1.0]), dirs[1:2], boxes, 20.0)     # over the box -> wall
    assert r[0] == pytest.approx(10.0)
    nav = FakeNav(boxes, rays_per_frame=2000)
    nav.pose = (0.0, 0.0, 0.0)
    pts, pts_lio = nav.lidar_frame()
    back = pts_lio.astype(np.float64) @ nav.R_odom_lio.T + nav.t_odom_lio
    np.testing.assert_allclose(back, pts, atol=1e-4)
    assert (np.abs(pts[:, 2]) < 0.05).mean() > 0.3          # inverted Mid-360S sees mostly floor
