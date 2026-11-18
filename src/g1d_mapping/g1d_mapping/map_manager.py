"""Map saving (design doc 5.3).

/g1d/save_map (g1d_msgs/SaveMap):
  1. /pgo/save_maps -> <dir>/raw/{map.pcd, poses.txt, patches/} (pose-graph optimised, FAST_LIO frame)
  2. convert to the map frame = odom of this mapping session (latched /g1d/lio_anchor T_odom_lio):
     map.pcd (voxel downsampled), trajectory.csv (base_link poses), keyframes/ (patches + poses)
  3. height-band 2D grid -> map.yaml / map.pgm (Nav2 map_server), meta.yaml
"""
import math
import os
import shutil
import threading
import time

import numpy as np
import rclpy
import yaml
from g1d_core.grid import GridParams, build_grid, save_map
from g1d_core.lio import inv, make_tf, matrix_to_quat, yaw_of
from g1d_core.pcd import read_pcd, read_poses_txt, transform, voxel_downsample, write_pcd, xyz
from g1d_msgs.srv import SaveMap
from geometry_msgs.msg import TransformStamped
from interface.srv import SaveMaps
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def tf_to_matrix(t: TransformStamped) -> np.ndarray:
    tr, q = t.transform.translation, t.transform.rotation
    return make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))


class MapManager(Node):
    def __init__(self, **kwargs):
        super().__init__("map_manager", **kwargs)
        d = self.declare_parameter
        self.maps_dir = os.path.expanduser(d("maps_dir", "~/g1d_maps").value)
        self.map_voxel = float(d("map_voxel", 0.05).value)
        self.base_frame = d("base_frame", "base_link").value
        self.imu_frame = d("imu_frame", "livox_imu").value
        self.gp = GridParams(
            resolution=float(d("grid.resolution", 0.05).value),
            z_min=float(d("grid.z_min", 0.08).value), z_max=float(d("grid.z_max", 1.85).value),
            ground_min=float(d("grid.ground_min", -0.10).value),
            ground_max=float(d("grid.ground_max", 0.05).value),
            min_hits=int(d("grid.min_hits", 2).value), free_grow=int(d("grid.free_grow", 1).value))
        self.anchor = None
        self.lock = threading.Lock()
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(TransformStamped, "/g1d/lio_anchor", self.on_anchor, LATCHED,
                                 callback_group=self.cb)
        self.pgo = self.create_client(SaveMaps, "/pgo/save_maps", callback_group=self.cb)
        self.create_service(SaveMap, "/g1d/save_map", self.on_save, callback_group=self.cb)
        self.get_logger().info(f"maps directory: {self.maps_dir}")

    def on_anchor(self, m: TransformStamped) -> None:
        self.anchor = tf_to_matrix(m)

    def on_save(self, req, resp):
        if not self.lock.acquire(blocking=False):
            resp.success, resp.message = False, "a save is already running"
            return resp
        try:
            return self.save(req, resp)
        except Exception as e:  # noqa: BLE001 - report every failure to the caller
            self.get_logger().error(f"save failed: {e}")
            resp.success, resp.message = False, str(e)
            return resp
        finally:
            self.lock.release()

    def save(self, req, resp):
        if self.anchor is None:
            raise RuntimeError("no /g1d/lio_anchor yet (is lio_bridge running and LIO initialised?)")
        name = req.name or time.strftime("map_%Y%m%d_%H%M%S")
        out = os.path.join(self.maps_dir, name)
        raw = os.path.join(out, "raw")
        if os.path.isdir(raw):
            shutil.rmtree(raw)
        os.makedirs(raw, exist_ok=True)
        if not self.pgo.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/pgo/save_maps not available (is pgo running?)")
        r = self.call(SaveMaps.Request(file_path=raw, save_patches=True), timeout=120.0)
        if not r.success:
            raise RuntimeError(f"pgo: {r.message}")
        t_map_lio = self.anchor
        try:
            t_base_imu = tf_to_matrix(self.tf.lookup_transform(self.base_frame, self.imu_frame, Time()))
        except TransformException as e:
            raise RuntimeError(f"no TF {self.base_frame} <- {self.imu_frame}: {e}") from e

        pts = transform(xyz(read_pcd(os.path.join(raw, "map.pcd"))), t_map_lio)
        pts = voxel_downsample(pts, self.map_voxel)
        write_pcd(os.path.join(out, "map.pcd"), pts)

        names, poses = read_poses_txt(os.path.join(raw, "poses.txt"))
        kf_dir = os.path.join(out, "keyframes")
        os.makedirs(kf_dir, exist_ok=True)
        rows = []
        with open(os.path.join(kf_dir, "poses.csv"), "w", encoding="utf-8") as fk:
            fk.write("file,x,y,z,qx,qy,qz,qw\n")
            for nm, p in zip(names, poses):
                t_map_imu = t_map_lio @ make_tf(p[:3], p[3:])
                t_map_base = t_map_imu @ inv(t_base_imu)
                rows.append((t_map_base[0, 3], t_map_base[1, 3], yaw_of(t_map_base)))
                q = matrix_to_quat(t_map_imu[:3, :3])
                fk.write(f"{nm},{t_map_imu[0, 3]:.4f},{t_map_imu[1, 3]:.4f},{t_map_imu[2, 3]:.4f},"
                         f"{q[0]:.6f},{q[1]:.6f},{q[2]:.6f},{q[3]:.6f}\n")
                shutil.copy(os.path.join(raw, "patches", nm), os.path.join(kf_dir, nm))
        with open(os.path.join(out, "trajectory.csv"), "w", encoding="utf-8") as ft:
            ft.write("x,y,yaw\n")
            for x, y, yaw in rows:
                ft.write(f"{x:.4f},{y:.4f},{yaw:.5f}\n")

        grid = build_grid(pts, self.gp)
        save_map(grid, out, "map")
        meta = {
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "frame": "map (= odom of the mapping session: base_link on the floor at start)",
            "points": int(len(pts)), "keyframes": len(names), "map_voxel": self.map_voxel,
            "grid": vars(self.gp),
            "T_map_lio": t_map_lio.round(6).tolist(),
            "T_base_imu": t_base_imu.round(6).tolist(),
            "path_length_m": float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(rows, rows[1:]))),
        }
        with open(os.path.join(out, "meta.yaml"), "w", encoding="utf-8") as fm:
            yaml.safe_dump(meta, fm, sort_keys=False)
        shutil.rmtree(raw, ignore_errors=True)
        resp.success, resp.path = True, out
        resp.keyframes, resp.points = len(names), int(len(pts))
        resp.message = f"saved {len(pts)} points, {len(names)} keyframes to {out}"
        self.get_logger().info(resp.message)
        return resp

    def call(self, req, timeout):
        fut = self.pgo.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise RuntimeError("/pgo/save_maps timed out")
        return fut.result()


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TypeError):
        if rclpy.ok():                 # a message taken while the context shuts down is harmless
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
