"""Helpers shared by the g1d_description launch file, calibration tools and tests."""
from __future__ import annotations

from typing import Any, Dict

XACRO_KEYS = ("base_to_agv_z", "head_to_livox_xyz", "head_to_livox_rpy", "livox_to_imu_xyz")


def xacro_args(extrinsics: Dict[str, Any]) -> Dict[str, str]:
    """extrinsics.yaml content -> xacro argument mappings (lists become space separated)."""
    out = {}
    for k in XACRO_KEYS:
        if k not in extrinsics:
            continue
        v = extrinsics[k]
        out[k] = " ".join(repr(float(x)) for x in v) if isinstance(v, (list, tuple)) else repr(float(v))
    return out
