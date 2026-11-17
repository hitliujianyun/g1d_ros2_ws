"""g1d_web: debug / visualisation / control UI (ROS 2).

    ros2 run g1d_web g1d_web [--port 8080] [--display-frame map|odom] [--root ~/g1d_web]
    -> open http://<host>:8080 in Chrome / Edge
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os

from .app import WebApp
from .config import get, guard_params, load_config
from .hub import Hub
from .server import HttpServer


def share_file(pkg: str, *parts: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(pkg), *parts)
    except Exception:  # noqa: BLE001 - source tree fallback
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(here, *parts) if pkg == "g1d_web" else ""


async def serve(cfg, guard_yaml: str, root: str, host: str, port: int, display_frame: str) -> None:
    hub = Hub(display_frame=display_frame)
    hub.start()
    app = WebApp(cfg, hub, guard_yaml, root)
    server = HttpServer(app, host, port)
    await server.start()
    logging.getLogger("g1d.web").info("open http://<this-host>:%d (display frame %s)", server.port, display_frame)
    await app.run()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="web.yaml (default: package config)")
    ap.add_argument("--guard-config", default=None, help="g1d_guard parameter file (limits, write-back)")
    ap.add_argument("--root", default="~/g1d_web", help="directory for exports, missions, test results")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--display-frame", default="map", help="map (also without a prior map) | odom")
    args, _ = ap.parse_known_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = load_config(args.config or share_file("g1d_web", "config", "web.yaml"))
    guard_yaml = args.guard_config or share_file("g1d_guard", "config", "guard.yaml")
    cfg["guard"] = guard_params(guard_yaml)
    root = os.path.expanduser(args.root)
    os.makedirs(root, exist_ok=True)
    host = args.host or get(cfg, "web.host", "0.0.0.0")
    port = args.port if args.port is not None else int(get(cfg, "web.port", 8080))
    try:
        asyncio.run(serve(cfg, guard_yaml, root, host, port, args.display_frame))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
