"""Server side of RFC 6455 WebSocket over asyncio streams (no third-party dependency)."""
from __future__ import annotations

import asyncio
import base64
import hashlib
import struct
from typing import Optional, Tuple

import numpy as np

GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
OP_CONT, OP_TEXT, OP_BINARY, OP_CLOSE, OP_PING, OP_PONG = 0x0, 0x1, 0x2, 0x8, 0x9, 0xA
MAX_MESSAGE = 16 * 1024 * 1024


class WebSocketClosed(Exception):
    pass


def accept_key(key: str) -> str:
    return base64.b64encode(hashlib.sha1(key.strip().encode() + GUID).digest()).decode()


def handshake_response(key: str) -> bytes:
    return ("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept_key(key)}\r\n\r\n").encode()


def unmask(data: bytes, mask: bytes) -> bytes:
    if len(data) < 64:
        return bytes(b ^ mask[i & 3] for i, b in enumerate(data))
    n = len(data)
    reps = (n + 3) // 4
    m = np.frombuffer(mask * reps, dtype=np.uint8)[:n]
    return (np.frombuffer(data, dtype=np.uint8) ^ m).tobytes()


def encode_frame(opcode: int, payload: bytes, mask: Optional[bytes] = None) -> bytes:
    """One FIN frame. Servers send unmasked frames; `mask` is for tests acting as a client."""
    n = len(payload)
    b1 = (0x80 if mask else 0x00)
    if n < 126:
        head = struct.pack("!BB", 0x80 | opcode, b1 | n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x80 | opcode, b1 | 126, n)
    else:
        head = struct.pack("!BBQ", 0x80 | opcode, b1 | 127, n)
    if mask:
        return head + mask + unmask(payload, mask)
    return head + payload


class WebSocket:
    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self.reader = reader
        self.writer = writer
        self.closed = False
        self._send_lock = asyncio.Lock()

    async def _read_frame(self) -> Tuple[bool, int, bytes]:
        h = await self.reader.readexactly(2)
        fin, opcode = bool(h[0] & 0x80), h[0] & 0x0F
        masked, n = bool(h[1] & 0x80), h[1] & 0x7F
        if n == 126:
            n = struct.unpack("!H", await self.reader.readexactly(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", await self.reader.readexactly(8))[0]
        if n > MAX_MESSAGE:
            raise WebSocketClosed("frame too large")
        mask = await self.reader.readexactly(4) if masked else None
        data = await self.reader.readexactly(n) if n else b""
        return fin, opcode, unmask(data, mask) if mask else data

    async def recv(self) -> Tuple[int, bytes]:
        """Next text / binary message (opcode, payload). Raises WebSocketClosed."""
        buf, first_op = b"", None
        try:
            while True:
                fin, op, data = await self._read_frame()
                if op == OP_PING:
                    await self._send(OP_PONG, data)
                    continue
                if op == OP_PONG:
                    continue
                if op == OP_CLOSE:
                    await self.close()
                    raise WebSocketClosed("peer closed")
                if op in (OP_TEXT, OP_BINARY):
                    first_op, buf = op, data
                elif op == OP_CONT and first_op is not None:
                    buf += data
                if len(buf) > MAX_MESSAGE:
                    raise WebSocketClosed("message too large")
                if fin and first_op is not None:
                    return first_op, buf
        except (asyncio.IncompleteReadError, ConnectionError) as e:
            self.closed = True
            raise WebSocketClosed(str(e)) from e

    async def _send(self, opcode: int, payload: bytes) -> None:
        if self.closed:
            raise WebSocketClosed("closed")
        async with self._send_lock:
            try:
                self.writer.write(encode_frame(opcode, payload))
                await self.writer.drain()
            except (ConnectionError, RuntimeError) as e:
                self.closed = True
                raise WebSocketClosed(str(e)) from e

    async def send_text(self, text: str) -> None:
        await self._send(OP_TEXT, text.encode())

    async def send_bytes(self, data: bytes) -> None:
        await self._send(OP_BINARY, data)

    def buffered(self) -> int:
        t = self.writer.transport
        return t.get_write_buffer_size() if t is not None else 0

    async def close(self) -> None:
        if self.closed:
            return
        try:
            self.writer.write(encode_frame(OP_CLOSE, struct.pack("!H", 1000)))
            await self.writer.drain()
        except Exception:  # noqa: BLE001
            pass
        self.closed = True
        try:
            self.writer.close()
        except Exception:  # noqa: BLE001
            pass
