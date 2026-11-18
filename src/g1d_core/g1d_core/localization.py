"""Localization health on a prior map (design doc 5.4), from small_gicp registration results.

Lost means the registration keeps failing, or stopped producing results for long; a registration
that is merely slow (CPU load) is reported but keeps the localization: while LIO is healthy, the
map -> odom correction drifts by centimetres over several seconds.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class LocalizationParams:
    min_inlier_ratio: float = 0.5
    max_failed: int = 4               # consecutive failed registrations -> lost (2 s at 2 Hz)
    slow_s: float = 3.0               # no result for this long -> "registration_slow" (still localized)
    stale_s: float = 10.0             # no result for this long -> "registration_stale" (lost)


class LocalizationHealth:
    def __init__(self, params: Optional[LocalizationParams] = None) -> None:
        self.p = params or LocalizationParams()
        self.last_result: Optional[Tuple[bool, float, float, float]] = None  # converged, ratio, error, t
        self.last_ok: Optional[float] = None
        self.failed = 0

    def update(self, t: float, converged: bool, inliers: float, source: float, error: float) -> bool:
        ratio = inliers / source if source > 0 else 0.0
        self.last_result = (bool(converged), ratio, error, t)
        ok = bool(converged) and ratio >= self.p.min_inlier_ratio
        if ok:
            self.last_ok, self.failed = t, 0
        else:
            self.failed += 1
        return ok

    def reset(self) -> None:
        """New initial pose: not localized until a registration confirms it; earlier failures are cleared."""
        self.last_ok, self.failed = None, 0

    def status(self, now: float) -> Tuple[bool, List[str]]:
        if self.last_result is None or self.last_ok is None:
            return False, ["no_registration" if self.last_result is None else "not_converged"]
        reasons = []
        conv, ratio, _, t = self.last_result
        if not conv:
            reasons.append("not_converged")
        elif ratio < self.p.min_inlier_ratio:
            reasons.append("low_inliers")
        lost = self.failed >= self.p.max_failed
        if lost:
            reasons.append("registration_failing")
        age = now - t
        if age > self.p.stale_s:
            reasons.append("registration_stale")
            lost = True
        elif age > self.p.slow_s:
            reasons.append("registration_slow")
        return not lost, reasons
