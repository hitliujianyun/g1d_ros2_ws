"""g1d_web: debug / visualisation / control UI.

    python3 -m g1d_web [--config config/g1d_nav.yaml] [--port 8080] [--net-if eth0|auto] [--domain 0]
    -> open http://<robot-ip>:8080 in Chrome / Edge
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os

from g1d_nav_py.config import DEFAULT_CONFIG, REPO_ROOT, get, load_config, net_if

from .app import WebApp
from .hub import Hub
from .server import HttpServer


async def serve(cfg, config_path: str, host: str, port: int, domain: int, nif) -> None:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(domain, nif)
    hub = Hub()
    hub.start()
    app = WebApp(cfg, hub, config_path, REPO_ROOT)
    server = HttpServer(app, host, port)
    await server.start()
    logging.getLogger("g1d.web").info("open http://<this-host>:%d (domain %d, if %s)", server.port,
                                      domain, nif)
    await app.run()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--net-if", default=None)
    ap.add_argument("--domain", type=int, default=None)
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    config_path = os.path.abspath(args.config or os.environ.get("G1D_CONFIG") or DEFAULT_CONFIG)
    cfg = load_config(config_path)
    host = args.host or get(cfg, "web.host", "0.0.0.0")
    port = args.port if args.port is not None else int(get(cfg, "web.port", 8080))
    domain = args.domain if args.domain is not None else int(get(cfg, "system.dds_domain", 0))
    try:
        asyncio.run(serve(cfg, config_path, host, port, domain, net_if(cfg, args.net_if)))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
