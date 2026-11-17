"""g1d_web configuration (config/web.yaml) and the guard parameter file used for write-back."""
from __future__ import annotations

import os
from typing import Any, Dict, Optional

import yaml


def load_config(path: str) -> Dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def get(cfg: Optional[Dict[str, Any]], dotted: str, default: Any = None) -> Any:
    cur: Any = cfg or {}
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def guard_params(guard_yaml: str) -> Dict[str, Any]:
    """ros__parameters of g1d_guard (for the UI limits)."""
    if not guard_yaml or not os.path.isfile(guard_yaml):
        return {}
    return (load_config(guard_yaml).get("g1d_guard") or {}).get("ros__parameters") or {}
