"""Runtime parameters: persisting guard / nav values back into config/g1d_nav.yaml.

The file is edited line by line so comments and layout survive; a timestamped backup is kept.
"""
from __future__ import annotations

import math
import os
import re
import shutil
import time
from typing import Any, Dict, List, Tuple

# Which YAML section holds each target's runtime parameters; read-only entries are skipped.
SECTIONS = {"guard": "guard", "nav": "manager"}
READ_ONLY = {"nav": {"mppi_vx_max", "mppi_wz_max"}}


def format_value(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if math.isfinite(v) and v == round(v) and abs(v) < 1e6:
            return f"{v:.1f}"
        return f"{v:.6g}"
    return str(v)


def update_yaml_values(text: str, section: str, values: Dict[str, Any]) -> Tuple[str, List[str]]:
    """Sets `section.key: value` for every key (indent 2). Returns (new_text, changed_keys)."""
    lines = text.splitlines(keepends=True)
    start = next((i for i, ln in enumerate(lines) if re.match(rf"^{re.escape(section)}:\s*(#.*)?$", ln)),
                 None)
    if start is None:
        block = [f"\n{section}:\n"] + [f"  {k}: {format_value(v)}\n" for k, v in values.items()]
        return "".join(lines) + "".join(block), list(values)
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].strip() and not lines[i].startswith((" ", "\t", "#")):
            end = i
            break
    changed, remaining = [], dict(values)
    for i in range(start + 1, end):
        m = re.match(r"^(  )([A-Za-z0-9_]+):(\s*)([^#\r\n]*?)(\s*)(#[^\r\n]*)?(\r?\n)?$", lines[i])
        if not m or m.group(2) not in remaining:
            continue
        key = m.group(2)
        new_val = format_value(remaining.pop(key))
        pad = m.group(5) or ""
        if m.group(6) and len(m.group(4)) != len(new_val):
            pad = " " * max(1, len(m.group(4)) + len(pad) - len(new_val))
        lines[i] = (f"{m.group(1)}{key}:{m.group(3) or ' '}{new_val}{pad}{m.group(6) or ''}"
                    f"{m.group(7) or ''}")
        changed.append(key)
    if remaining:
        insert_at = end
        while insert_at > start + 1 and not lines[insert_at - 1].strip():
            insert_at -= 1
        new_lines = [f"  {k}: {format_value(v)}\n" for k, v in remaining.items()]
        if insert_at > 0 and not lines[insert_at - 1].endswith("\n"):
            lines[insert_at - 1] += "\n"
        lines[insert_at:insert_at] = new_lines
        changed += list(remaining)
    return "".join(lines), changed


def save_params(config_path: str, target: str, params: Dict[str, Any]) -> Dict[str, Any]:
    if target not in SECTIONS:
        raise ValueError(f"unknown target {target!r}")
    values = {k: v for k, v in params.items() if k not in READ_ONLY.get(target, set())}
    with open(config_path, "r", encoding="utf-8", newline="") as f:
        text = f.read()
    new_text, changed = update_yaml_values(text, SECTIONS[target], values)
    backup = f"{config_path}.bak-{time.strftime('%Y%m%d-%H%M%S')}"
    shutil.copy2(config_path, backup)
    tmp = config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(new_text)
    os.replace(tmp, config_path)
    return {"target": target, "section": SECTIONS[target], "changed": changed, "backup": backup}
