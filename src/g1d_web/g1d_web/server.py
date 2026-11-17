"""Minimal asyncio HTTP server: static UI, small JSON API, file downloads, /ws upgrade."""
from __future__ import annotations

import asyncio
import json
import logging
import mimetypes
import os
import urllib.parse
from typing import Dict, Optional, Tuple

from .wsproto import WebSocket, handshake_response

log = logging.getLogger("g1d.web.http")
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
MIME = {".js": "text/javascript; charset=utf-8", ".css": "text/css; charset=utf-8",
        ".html": "text/html; charset=utf-8", ".json": "application/json", ".png": "image/png",
        ".yaml": "text/plain; charset=utf-8", ".csv": "text/csv; charset=utf-8",
        ".pcd": "application/octet-stream", ".svg": "image/svg+xml", ".md": "text/plain; charset=utf-8"}


def safe_join(root: str, rel: str) -> Optional[str]:
    root = os.path.realpath(root)
    p = os.path.realpath(os.path.join(root, rel.lstrip("/\\")))
    if p != root and not p.startswith(root + os.sep):
        return None
    return p


async def read_request(reader: asyncio.StreamReader) -> Optional[Tuple[str, str, Dict[str, str]]]:
    try:
        head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=10.0)
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, asyncio.TimeoutError,
            ConnectionError):
        return None
    lines = head.decode("latin-1").split("\r\n")
    try:
        method, target, _ = lines[0].split(" ", 2)
    except ValueError:
        return None
    headers = {}
    for ln in lines[1:]:
        if ":" in ln:
            k, v = ln.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    return method, target, headers


def response(status: str, body: bytes, ctype: str, extra: str = "") -> bytes:
    return (f"HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
            f"Cache-Control: no-cache\r\nConnection: close\r\n{extra}\r\n").encode() + body


class HttpServer:
    def __init__(self, app, host: str, port: int):
        self.app = app
        self.host = host
        self.port = port
        self.server = None

    async def start(self) -> None:
        self.server = await asyncio.start_server(self._handle, self.host, self.port)
        sock = self.server.sockets[0].getsockname()
        self.port = sock[1]
        log.info("g1d_web listening on http://%s:%d", self.host, self.port)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        addr = f"{peer[0]}:{peer[1]}" if peer else "?"
        req = await read_request(reader)
        if req is None:
            writer.close()
            return
        method, target, headers = req
        url = urllib.parse.urlsplit(target)
        path = urllib.parse.unquote(url.path)
        try:
            if path == "/ws" and headers.get("upgrade", "").lower() == "websocket":
                writer.write(handshake_response(headers.get("sec-websocket-key", "")))
                await writer.drain()
                await self.app.session(WebSocket(reader, writer), addr)
                return
            if method not in ("GET", "HEAD"):
                writer.write(response("405 Method Not Allowed", b"", "text/plain"))
            else:
                writer.write(self._route(path))
            await writer.drain()
        except (ConnectionError, asyncio.CancelledError):
            pass
        finally:
            try:
                writer.close()
            except Exception:  # noqa: BLE001
                pass

    def _route(self, path: str) -> bytes:
        app = self.app
        if path.startswith("/api/"):
            from . import mapping, mission
            items = {"/api/maps": lambda: mapping.list_exports(app.maps_dir),
                     "/api/missions": lambda: mission.list_missions(app.missions_dir),
                     "/api/tests": app.list_test_results}.get(path)
            if items is None:
                return response("404 Not Found", b"{}", "application/json")
            return response("200 OK", json.dumps(items()).encode(), "application/json")
        if path.startswith("/files/"):
            kind, _, rel = path[len("/files/"):].partition("/")
            root = {"maps": app.maps_dir, "tests": app.tests_dir, "missions": app.missions_dir}.get(kind)
            p = safe_join(root, rel) if root else None
            if not p or not os.path.isfile(p):
                return response("404 Not Found", b"not found", "text/plain")
            with open(p, "rb") as f:
                body = f.read()
            ext = os.path.splitext(p)[1].lower()
            disp = f'Content-Disposition: attachment; filename="{os.path.basename(p)}"\r\n'
            return response("200 OK", body, MIME.get(ext, "application/octet-stream"), disp)
        rel = "index.html" if path in ("", "/") else path
        p = safe_join(STATIC, rel)
        if not p or not os.path.isfile(p):
            return response("404 Not Found", b"not found", "text/plain")
        with open(p, "rb") as f:
            body = f.read()
        ext = os.path.splitext(p)[1].lower()
        return response("200 OK", body, MIME.get(ext) or mimetypes.guess_type(p)[0] or "application/octet-stream")
