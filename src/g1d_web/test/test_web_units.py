import asyncio
import json
import math
import os
import struct
import zlib

import numpy as np
import pytest

from g1d_web import mapping, mission, params
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


GUARD_YAML = """# guard parameters
g1d_guard:
  ros__parameters:
    rate_hz: 30.0
    stop_m: 0.45            # stop distance
    slow_m: 1.20
"""


def test_save_params_writes_backup_ros_layout(tmp_path):
    p = tmp_path / "guard.yaml"
    p.write_text(GUARD_YAML, encoding="utf-8")
    res = params.save_params(str(p), "guard", {"stop_m": 0.5, "creep_v": 0.04})
    assert set(res["changed"]) == {"stop_m", "creep_v"} and os.path.isfile(res["backup"])
    t = p.read_text(encoding="utf-8")
    assert "    stop_m: 0.5             # stop distance\n" in t
    import yaml
    d = yaml.safe_load(t)["g1d_guard"]["ros__parameters"]
    assert d["stop_m"] == 0.5 and d["creep_v"] == 0.04 and d["rate_hz"] == 30.0
    with pytest.raises(ValueError):
        params.save_params(str(p), "nav", {"x": 1})


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


# -------------------------------------------------------------------------------------- goals
def test_clean_goal():
    g = clean_goal({"x": "1.5", "y": 2, "yaw": None, "evil": 1})
    assert g["frame"] == "map" and g["x"] == 1.5 and "yaw" not in g and "evil" not in g
    assert g["id"].startswith("web-")
    with pytest.raises(ValueError):
        clean_goal({"mode": "rotate"})
    with pytest.raises(ValueError):
        clean_goal({"x": float("nan"), "y": 0})
    with pytest.raises(ValueError):
        clean_goal({"x": 1, "y": 1, "frame": "lidar"})


def test_resample_grid_rotation():
    from g1d_web.hub import resample_grid
    data = np.zeros((10, 20), np.uint8)
    data[:, 15:] = 100                                       # occupied band at x in [1.5, 2.0)
    g = {"width": 20, "height": 10, "resolution": 0.1, "origin": [0.0, 0.0], "data": data.tobytes()}
    t = np.eye(4)
    t[:2, :2] = [[0.0, -1.0], [1.0, 0.0]]                    # rotate +90 deg
    out = resample_grid(g, t)
    assert (out["width"], out["height"]) == (10, 20) and out["origin"] == pytest.approx([-1.0, 0.0])
    img = np.frombuffer(out["data"], np.uint8).reshape(out["height"], out["width"])
    assert img[17, 5] == 100 and img[5, 5] == 0              # band now at y in [1.5, 2.0)
    assert resample_grid(g, np.eye(4)) is g
