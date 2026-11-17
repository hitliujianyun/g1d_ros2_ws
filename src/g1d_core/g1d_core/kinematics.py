"""Minimal URDF forward kinematics (fixed / revolute / continuous / prismatic joints).

Used by tests (check the xacro output against hand-computed chains) and by calibration tools that
need base_link <-> sensor transforms without a running robot_state_publisher.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_rpy(r: np.ndarray) -> np.ndarray:
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:     # gimbal lock: put everything into yaw
        return np.array([0.0, pitch, math.atan2(-r[0, 1], r[1, 1])])
    return np.array([math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])])


def axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def transform(r: np.ndarray, t: Sequence[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray          # 4x4
    axis: np.ndarray            # unit vector in the joint frame

    def motion(self, q: float) -> np.ndarray:
        if self.type in ("revolute", "continuous"):
            return transform(axis_angle(self.axis, q), (0.0, 0.0, 0.0))
        if self.type == "prismatic":
            return transform(np.eye(3), self.axis * q)
        return np.eye(4)


def _floats(text: Optional[str], n: int = 3) -> List[float]:
    if not text:
        return [0.0] * n
    return [float(v) for v in text.split()]


class Urdf:
    def __init__(self, joints: Sequence[Joint]):
        self.joints = {j.name: j for j in joints}
        self.by_child = {j.child: j for j in joints}

    @classmethod
    def from_string(cls, xml: str) -> "Urdf":
        root = ET.fromstring(xml)
        joints = []
        for el in root.findall("joint"):
            o = el.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None)
            rpy = _floats(o.get("rpy") if o is not None else None)
            ax = el.find("axis")
            axis = np.array(_floats(ax.get("xyz") if ax is not None else None) or [1.0, 0.0, 0.0])
            if not np.any(axis):
                axis = np.array([1.0, 0.0, 0.0])
            joints.append(Joint(el.get("name"), el.get("type"), el.find("parent").get("link"),
                                el.find("child").get("link"), transform(rpy_to_matrix(*rpy), xyz),
                                axis / np.linalg.norm(axis)))
        return cls(joints)

    @classmethod
    def from_file(cls, path: str) -> "Urdf":
        with open(path, encoding="utf-8") as f:
            return cls.from_string(f.read())

    def chain(self, root: str, tip: str) -> List[Joint]:
        out = []
        link = tip
        while link != root:
            j = self.by_child.get(link)
            if j is None:
                raise KeyError(f"no chain from {root!r} to {tip!r} (stopped at {link!r})")
            out.append(j)
            link = j.parent
        return out[::-1]

    def fk(self, joints: Optional[Dict[str, float]], root: str, tip: str) -> np.ndarray:
        """Pose of `tip` in `root` (4x4) for the given joint positions (missing joints = 0)."""
        joints = joints or {}
        m = np.eye(4)
        for j in self.chain(root, tip):
            m = m @ j.origin @ j.motion(float(joints.get(j.name, 0.0)))
        return m
