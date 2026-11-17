"""Minimal SLAMWARE chassis HTTP client (urllib, proxy-free): events, abort current action, AEB."""
from __future__ import annotations

import json
import logging
import urllib.request
from typing import Any, Iterable, List, Optional, Set, Tuple

log = logging.getLogger("g1d.chassis_http")

EVENTS = "/api/platform/v1/events"
CURRENT_ACTION = "/api/core/motion/v1/actions/:current"
AEB_CONTROL = "/api/core/system/v1/aeb/control"


class ChassisHttp:
    def __init__(self, base_url: str, timeout: float = 0.5):
        self.base = base_url.rstrip("/")
        self.timeout = timeout
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(self, method: str, path: str, body: Optional[Any] = None) -> Any:
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        with self._opener.open(req, timeout=self.timeout) as r:
            raw = r.read()
        return json.loads(raw) if raw else None

    def events(self) -> List[dict]:
        r = self.request("GET", EVENTS)
        if isinstance(r, dict):
            r = r.get("events", r.get("data", []))
        return [e for e in (r or []) if isinstance(e, dict)]

    def abort_current_action(self) -> bool:
        try:
            self.request("DELETE", CURRENT_ACTION)
            return True
        except Exception as e:  # noqa: BLE001 - best effort at start-up
            log.warning("abort current action failed: %s", e)
            return False

    def set_aeb(self, enabled: bool) -> bool:
        try:
            self.request("POST", AEB_CONTROL, {"enable": bool(enabled)})
            return True
        except Exception as e:  # noqa: BLE001
            log.warning("AEB control failed: %s", e)
            return False


def event_key(e: dict) -> Tuple[Any, Any]:
    return (e.get("type"), e.get("timestamp", e.get("time", e.get("id"))))


class EventTracker:
    """Reports only events that appear after the first poll (old events must not latch e-stop)."""

    def __init__(self) -> None:
        self.seen: Optional[Set[Tuple[Any, Any]]] = None

    def update(self, events: Iterable[dict]) -> List[str]:
        keys = {event_key(e) for e in events}
        if self.seen is None:
            self.seen = keys
            return []
        new = [k[0] for k in keys - self.seen if k[0]]
        self.seen |= keys
        return new
