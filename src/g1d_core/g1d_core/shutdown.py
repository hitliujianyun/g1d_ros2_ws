"""Shutdown race of rclpy nodes started by launch.

launch stops a node with SIGINT; a message taken from the middleware while rclpy is still closing the
context raises RuntimeError ("Unable to convert call argument ...") before rclpy.ok() turns False.
Such an exception is harmless; one raised while the context stays open is a real failure.
"""
from __future__ import annotations

import time
from typing import Callable


def context_closing(ok: Callable[[], bool], timeout: float = 1.0, poll: float = 0.05) -> bool:
    """True if ok() (e.g. rclpy.ok) becomes False within timeout seconds."""
    t_end = time.monotonic() + timeout
    while True:
        if not ok():
            return True
        if time.monotonic() >= t_end:
            return False
        time.sleep(poll)
