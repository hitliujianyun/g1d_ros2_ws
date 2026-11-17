"""Tiny asyncio WebSocket client for tests (masked frames, as browsers send)."""
import asyncio
import base64
import json
import os
import struct

from g1d_web.wsproto import OP_BINARY, OP_TEXT, encode_frame


class WsClient:
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    @classmethod
    async def connect(cls, host: str, port: int, path: str = "/ws"):
        reader, writer = await asyncio.open_connection(host, port)
        key = base64.b64encode(os.urandom(16)).decode()
        writer.write((f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
                      f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
        await writer.drain()
        head = await reader.readuntil(b"\r\n\r\n")
        assert head.startswith(b"HTTP/1.1 101"), head
        return cls(reader, writer)

    async def send(self, obj) -> None:
        self.writer.write(encode_frame(OP_TEXT, json.dumps(obj).encode(), os.urandom(4)))
        await self.writer.drain()

    async def recv(self):
        h = await self.reader.readexactly(2)
        n = h[1] & 0x7F
        if n == 126:
            n = struct.unpack("!H", await self.reader.readexactly(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
        data = await self.reader.readexactly(n)
        op = h[0] & 0x0F
        if op == OP_TEXT:
            return json.loads(data.decode())
        if op == OP_BINARY:
            hl = struct.unpack("<I", data[4:8])[0]
            return {"type": "binary", "kind": data[0], "header": json.loads(data[8:8 + hl].decode()),
                    "payload": data[8 + hl:]}
        return {"type": "control", "op": op}

    def close(self):
        self.writer.close()


async def get(host: str, port: int, path: str) -> bytes:
    reader, writer = await asyncio.open_connection(host, port)
    writer.write(f"GET {path} HTTP/1.1\r\nHost: {host}\r\n\r\n".encode())
    await writer.drain()
    data = await reader.read()
    writer.close()
    return data
