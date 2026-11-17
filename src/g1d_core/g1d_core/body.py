"""G1-D upper body: Unitree joint feedback -> URDF joint positions, and the navigation pose lock.

Sources: /hispeed_state (Point32: y = lift height, z = lift speed), /waistpitchstate (MotorState.q,
URDF Yaw_Joint), /lowstate motor_state[12].q (waist yaw, URDF torso_Joint) and [15..28] (arms).
While navigating, the head lidar must not move relative to the chassis: the lock checker reports
every deviation from the navigation pose (design doc 5.1).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List, Optional, Sequence, Tuple

ARM_JOINTS = tuple(
    f"{side}_{j}_joint"
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
              "wrist_roll", "wrist_pitch", "wrist_yaw"))
LIFT_JOINTS = ("LZ_mt_Joint", "LZ_it_Joint")
WAIST_PITCH_JOINT = "Yaw_Joint"     # URDF name; the joint actually pitches the waist
WAIST_YAW_JOINT = "torso_Joint"


@dataclass
class BodyConfig:
    lift_scale: float = 1.0
    lift_offset: float = 0.0
    lift_max: float = 0.42
    waist_yaw_index: int = 12
    arm_first_index: int = 15
    nav_waist_pitch: float = 0.0
    arm_tuck: List[float] = field(default_factory=lambda: [0.0] * 14)
    tol_waist_rad: float = math.radians(2.0)
    tol_arm_rad: float = math.radians(5.0)
    tol_lift_m: float = 0.01
    tol_lift_speed: float = 0.005
    stale_s: float = 0.5
    require_waist_pitch: bool = True
    require_arms: bool = True

    @classmethod
    def from_dict(cls, d: Optional[Dict[str, Any]]) -> "BodyConfig":
        c = cls()
        for f in fields(cls):
            if d and f.name in d:
                v = d[f.name]
                if f.name == "arm_tuck":
                    v = [float(x) for x in v]
                    if len(v) != 14:
                        raise ValueError("arm_tuck needs 14 values")
                else:
                    v = type(getattr(c, f.name))(v)
                setattr(c, f.name, v)
        return c


def lift_height(lift_y: float, cfg: BodyConfig) -> float:
    return min(max(cfg.lift_scale * lift_y + cfg.lift_offset, 0.0), cfg.lift_max)


def joint_positions(lift_y: float, waist_pitch: float, waist_yaw: float,
                    arm_q: Optional[Sequence[float]], cfg: BodyConfig) -> Dict[str, float]:
    """URDF joint positions; the two lift stages share the total travel equally."""
    h = lift_height(lift_y, cfg)
    out = {LIFT_JOINTS[0]: 0.5 * h, LIFT_JOINTS[1]: 0.5 * h,
           WAIST_PITCH_JOINT: float(waist_pitch), WAIST_YAW_JOINT: float(waist_yaw)}
    arms = cfg.arm_tuck if arm_q is None else arm_q
    out.update({name: float(q) for name, q in zip(ARM_JOINTS, arms)})
    return out


def arms_from_lowstate(motor_q: Sequence[float], cfg: BodyConfig) -> List[float]:
    i = cfg.arm_first_index
    return [float(q) for q in motor_q[i:i + 14]]


@dataclass
class BodySample:
    lift_y: float = 0.0
    lift_speed: float = 0.0
    waist_pitch: float = 0.0
    waist_yaw: float = 0.0
    arm_q: Optional[List[float]] = None
    t_lift: float = -math.inf
    t_lowstate: float = -math.inf
    t_waist: float = -math.inf


class LockChecker:
    def __init__(self, cfg: BodyConfig):
        self.cfg = cfg
        self.ref_height: Optional[float] = None

    def reset_reference(self, sample: BodySample) -> None:
        self.ref_height = lift_height(sample.lift_y, self.cfg)

    def check(self, s: BodySample, now: float) -> Tuple[bool, List[str]]:
        c = self.cfg
        reasons: List[str] = []
        if now - s.t_lift > c.stale_s:
            reasons.append("stale_lift")
        if now - s.t_lowstate > c.stale_s:
            reasons.append("stale_lowstate")
        if c.require_waist_pitch and now - s.t_waist > c.stale_s:
            reasons.append("stale_waist_pitch")
        if abs(s.waist_yaw) > c.tol_waist_rad:
            reasons.append("waist_yaw")
        if c.require_waist_pitch and abs(s.waist_pitch - c.nav_waist_pitch) > c.tol_waist_rad:
            reasons.append("waist_pitch")
        if abs(s.lift_speed) > c.tol_lift_speed:
            reasons.append("lift_moving")
        h = lift_height(s.lift_y, c)
        if self.ref_height is None and "stale_lift" not in reasons:
            self.ref_height = h
        if self.ref_height is not None and abs(h - self.ref_height) > c.tol_lift_m:
            reasons.append("lift_changed")
        if c.require_arms:
            if s.arm_q is None:
                reasons.append("arms")
            elif any(abs(q - t) > c.tol_arm_rad for q, t in zip(s.arm_q, c.arm_tuck)):
                reasons.append("arms")
        return not reasons, reasons
