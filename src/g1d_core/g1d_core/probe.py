"""Statistics for the phase-0 tools (check_interfaces, cmd_vel_probe); no ROS dependency."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

STOP_GATE_S = 0.6     # autonomous navigation is only released if the chassis stops this fast


class RateStats:
    """Receive times and header-stamp offsets (receive time - stamp) of one topic."""

    def __init__(self) -> None:
        self.t: List[float] = []
        self.offsets: List[float] = []

    def add(self, t_recv: float, stamp: Optional[float] = None) -> None:
        self.t.append(t_recv)
        if stamp is not None:
            self.offsets.append(t_recv - stamp)

    def report(self) -> Dict[str, Optional[float]]:
        n = len(self.t)
        rate = (n - 1) / (self.t[-1] - self.t[0]) if n > 2 and self.t[-1] > self.t[0] else None
        off = sorted(self.offsets)
        return {
            "count": n,
            "rate_hz": round(rate, 2) if rate else None,
            "stamp_offset_s": round(off[len(off) // 2], 4) if off else None,
            "stamp_offset_spread_s": round(off[-1] - off[0], 4) if len(off) > 1 else None,
        }


Row = Tuple[float, float, float, float, float, float]   # t, x, y, yaw, v, w


def stop_stats(rows: Sequence[Row], t0: float, v_eps: float = 0.02, w_eps: float = 0.03) -> Dict:
    """Time / distance from t0 (release) until the wheel odometry reports standstill."""
    rows = [r for r in rows if r[0] >= t0]
    if not rows:
        return {"error": "no odometry"}
    x0, y0 = rows[0][1], rows[0][2]
    out = {"v_at_release": rows[0][4], "w_at_release": rows[0][5]}
    stopped = next((r for r in rows if abs(r[4]) < v_eps and abs(r[5]) < w_eps), None)
    if stopped is None:
        out["stop_time_s"] = None
        return out
    out["stop_time_s"] = round(stopped[0] - t0, 3)
    out["stop_distance_m"] = round(math.hypot(stopped[1] - x0, stopped[2] - y0), 3)
    return out


def stop_gate(silence: Dict, limit: float = STOP_GATE_S) -> Dict:
    """Release gate (design doc 6.2): with the guard / host gone, the chassis must stop by itself."""
    t = silence.get("stop_time_s")
    return {"limit_s": limit, "stop_time_s": t, "pass": t is not None and t <= limit}
