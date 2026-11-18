"""Localization supervision on the prior map (design doc 5.4).

  small_gicp_relocalization ~/status -> /g1d/localization_status (localized / lost, quality)
  /g1d/relocalize (g1d_msgs/Relocalize): pose | auto (global search, g1d_core.reloc) | last
      -> /initialpose for small_gicp_relocalization
  last pose saving (maps/<name>/last_pose.yaml) and optional use at start-up
In odom mode (no map) map -> odom is a static identity and the robot counts as localized.
"""
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from g1d_core.lio import inv, make_tf, yaw_of
from g1d_core.localization import LocalizationHealth, LocalizationParams
from g1d_core.pcd import read_pcd, xyz
from g1d_core.reloc import SearchParams, VoxelMap, global_search, subsample_seeds
from g1d_core.shutdown import context_closing
from g1d_msgs.msg import LocalizationStatus
from g1d_msgs.srv import Relocalize
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


def tf_matrix(t):
    tr, q = t.transform.translation, t.transform.rotation
    return make_tf((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))


class LocalizationMonitor(Node):
    def __init__(self, **kwargs):
        super().__init__("localization_monitor", **kwargs)
        d = self.declare_parameter
        self.mode = d("mode", "map").value
        self.map_dir = os.path.expanduser(d("map_dir", "").value)
        self.map_frame = d("map_frame", "map").value
        self.odom_frame = d("odom_frame", "odom").value
        self.base_frame = d("base_frame", "base_link").value
        self.health = LocalizationHealth(LocalizationParams(
            min_inlier_ratio=float(d("min_inlier_ratio", 0.5).value),
            max_failed=int(d("max_failed_registrations", 4).value),
            slow_s=float(d("registration_slow_s", 3.0).value),
            stale_s=float(d("registration_stale_s", 10.0).value)))
        self.save_every = float(d("save_last_pose_every_s", 5.0).value)
        self.startup_init = d("startup_init", "none").value      # none | last | auto
        self.scan_window = float(d("scan_window_s", 1.5).value)
        self.search = SearchParams(min_score=float(d("auto_min_score", 0.6).value))
        self.cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.prev_map_odom = None
        self.correction = 0.0
        self.scans = deque()
        self.vmap = None
        self.seeds = None
        self.lock = threading.Lock()
        self.pub_status = self.create_publisher(LocalizationStatus, "/g1d/localization_status", 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(Float64MultiArray, "/small_gicp_relocalization/status", self.on_status, 10,
                                 callback_group=self.cb)
        self.create_subscription(PointCloud2, "/g1d/registered_scan", self.on_scan, 5, callback_group=self.cb)
        self.create_service(Relocalize, "/g1d/relocalize", self.on_relocalize, callback_group=self.cb)
        self.create_timer(0.5, self.publish_status, callback_group=self.cb)
        if self.mode == "map" and self.save_every > 0:
            self.create_timer(self.save_every, self.save_last_pose, callback_group=self.cb)
        self._startup_done = self.mode != "map" or self.startup_init == "none"
        self._startup_tries = 0
        self._startup_lock = threading.Lock()
        if not self._startup_done:
            self.create_timer(1.0, self.try_startup_init, callback_group=self.cb)

    # ---- inputs -----------------------------------------------------------------------------
    def on_status(self, m: Float64MultiArray) -> None:
        if len(m.data) < 4:
            return
        converged, inliers, source, error = m.data[:4]
        self.health.update(time.monotonic(), bool(converged), inliers, source, error)
        mo = self.lookup(self.map_frame, self.odom_frame)
        if mo is not None:
            if self.prev_map_odom is not None:
                self.correction = float(np.linalg.norm(mo[:3, 3] - self.prev_map_odom[:3, 3]))
            self.prev_map_odom = mo

    def on_scan(self, m: PointCloud2) -> None:
        pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        now = time.monotonic()
        with self.lock:
            self.scans.append((now, pts.astype(np.float64)))
            while self.scans and self.scans[0][0] < now - self.scan_window:
                self.scans.popleft()

    def lookup(self, target, source):
        try:
            return tf_matrix(self.tf.lookup_transform(target, source, Time()))
        except TransformException:
            return None

    # ---- relocalization ---------------------------------------------------------------------
    def send_initial_pose(self, x, y, yaw) -> None:
        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25
        m.pose.covariance[35] = 0.07
        self.pub_init.publish(m)
        self.health.reset()                          # wait for small_gicp to confirm
        self.get_logger().info(f"initial pose x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg")

    def load_map(self) -> None:
        if self.vmap is not None:
            return
        pcd = os.path.join(self.map_dir, "map.pcd")
        self.vmap = VoxelMap(xyz(read_pcd(pcd)), voxel=0.2, dilate=1)
        traj = os.path.join(self.map_dir, "trajectory.csv")
        if os.path.isfile(traj):
            arr = np.loadtxt(traj, delimiter=",", skiprows=1, ndmin=2)
            self.seeds = subsample_seeds(arr[:, :2], self.search.seed_spacing)
        else:                                        # no trajectory: seed every 1 m over the map
            lo = self.vmap.origin[:2]
            hi = lo + np.array(self.vmap.shape[:2]) * self.vmap.voxel
            self.seeds = np.array([(x, y) for x in np.arange(lo[0], hi[0], 1.0) for y in np.arange(lo[1], hi[1], 1.0)])

    def auto(self):
        self.load_map()
        t_odom_base = self.lookup(self.odom_frame, self.base_frame)
        if t_odom_base is None:
            raise RuntimeError(f"no TF {self.odom_frame} <- {self.base_frame}")
        with self.lock:
            clouds = [c for _, c in self.scans]
        if not clouds:
            raise RuntimeError("no /g1d/registered_scan yet")
        pts = np.vstack(clouds)
        t = inv(t_odom_base)
        base = pts @ t[:3, :3].T + t[:3, 3]
        r = np.hypot(base[:, 0], base[:, 1])
        base = base[(base[:, 2] > 0.08) & (base[:, 2] < 2.0) & (r < 15.0) & (r > 0.4)]
        res = global_search(self.vmap, base, self.seeds, self.search)
        return res

    def on_relocalize(self, req, resp):
        try:
            if req.mode == "pose":
                pose, score = (req.x, req.y, req.yaw), 0.0
            elif req.mode == "last":
                pose, score = self.read_last_pose(), 0.0
            elif req.mode == "auto":
                res = self.auto()
                if not res.found:
                    raise RuntimeError(f"no confident match (best score {res.score:.2f}, second {res.second_score:.2f})")
                pose, score = res.pose, res.score
            else:
                raise RuntimeError(f"unknown mode {req.mode!r} (pose | auto | last)")
            self.send_initial_pose(*pose)
            resp.success, resp.message = True, f"initial pose sent ({req.mode})"
            resp.x, resp.y, resp.yaw, resp.score = float(pose[0]), float(pose[1]), float(pose[2]), float(score)
        except Exception as e:  # noqa: BLE001 - reported to the caller
            resp.success, resp.message = False, str(e)
            self.get_logger().warn(f"relocalize failed: {e}")
        return resp

    # ---- last pose --------------------------------------------------------------------------
    def last_pose_path(self):
        return os.path.join(self.map_dir, "last_pose.yaml")

    def read_last_pose(self):
        with open(self.last_pose_path(), encoding="utf-8") as f:
            p = yaml.safe_load(f)
        return float(p["x"]), float(p["y"]), float(p["yaw"])

    def try_startup_init(self) -> None:
        # the reentrant timer may fire again while a (slow) search runs: one attempt at a time
        if not self._startup_lock.acquire(blocking=False):
            return
        try:
            self._try_startup_init()
        finally:
            self._startup_lock.release()

    def _try_startup_init(self) -> None:
        if self._startup_done or self.lookup(self.odom_frame, self.base_frame) is None:
            return
        if self.startup_init == "last":
            self._startup_done = True
            if os.path.isfile(self.last_pose_path()):
                self.send_initial_pose(*self.read_last_pose())
            else:
                self.get_logger().warn("no last_pose.yaml: give the initial pose (RViz / web / /g1d/relocalize)")
            return
        with self.lock:                              # auto: wait for a full scan window
            ready = len(self.scans) > 0 and time.monotonic() - self.scans[0][0] >= 0.8 * self.scan_window
        if not ready:
            return
        self._startup_tries += 1
        try:
            res = self.auto()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"start-up auto relocalization: {e}")
            res = None
        if res is not None and res.found:
            self._startup_done = True
            self.send_initial_pose(*res.pose)
        elif self._startup_tries >= 5:
            self._startup_done = True
            self.get_logger().error("start-up auto relocalization failed 5 times: give the initial pose manually")

    def save_last_pose(self) -> None:
        if not self.map_dir or not self.localized()[0]:
            return
        mb = self.lookup(self.map_frame, self.base_frame)
        if mb is None:
            return
        try:
            with open(self.last_pose_path(), "w", encoding="utf-8") as f:
                yaml.safe_dump({"x": float(mb[0, 3]), "y": float(mb[1, 3]), "yaw": float(yaw_of(mb)),
                                "time": time.strftime("%Y-%m-%d %H:%M:%S")}, f)
        except OSError as e:
            self.get_logger().warn(f"cannot write {self.last_pose_path()}: {e}", throttle_duration_sec=60.0)

    # ---- status -----------------------------------------------------------------------------
    def localized(self):
        if self.mode == "odom":
            return True, []
        return self.health.status(time.monotonic())

    def publish_status(self) -> None:
        ok, reasons = self.localized()
        m = LocalizationStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.map_frame
        m.mode = self.mode
        m.localized = ok
        m.reasons = reasons
        h = self.health
        if h.last_result is not None:
            m.inlier_ratio, m.error = float(h.last_result[1]), float(h.last_result[2])
        m.last_ok_age = -1.0 if h.last_ok is None else float(time.monotonic() - h.last_ok)
        m.correction = float(self.correction)
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            t = None
        if t is not None:
            mb = tf_matrix(t)
            m.header.stamp = t.header.stamp
            m.pose.x, m.pose.y, m.pose.theta = float(mb[0, 3]), float(mb[1, 3]), float(yaw_of(mb))
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TypeError):
        if not context_closing(rclpy.ok):   # taken while the context shuts down: harmless
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
