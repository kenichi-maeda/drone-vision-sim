"""Send Duckiematrix keyboard commands for the drive view.

The engine reads an INPUT resource named keyboard on the quadcopter.
W and S drive, A and D yaw, ascend and descend change height, and reset
returns the drone to its spawn pose. The WebSocket bridge on port 7505
accepts that packet.
"""

from __future__ import annotations

import base64
import hashlib
import os
import socket
import struct
from time import time
from urllib.parse import urlparse


KEYBOARD_KEY = "map_0/vehicle_0/keyboard"
COMMAND_FIELDS = (
    "forward",
    "backward",
    "left",
    "right",
    "ascend",
    "descend",
    "reset",
)


def empty_command() -> dict[str, bool]:
    return {name: False for name in COMMAND_FIELDS}


def encode_keyboard(command: dict) -> bytes:
    """CBOR field order of DriveKeyboardActionMessage."""
    flags = [bool(command.get(name)) for name in COMMAND_FIELDS]
    flags.append(False)  # camera-relative yaw is unused by this view
    payload = bytearray(0xF5 if flag else 0xF4 for flag in flags)
    payload.append(0xFB)
    payload.extend(struct.pack(">d", float(command.get("yaw_degrees", 0.0))))
    return bytes(payload)


def encode_input_frame(
    command: dict,
    *,
    session_id: int = 0,
    timestamp: float | None = None,
    key: str = KEYBOARD_KEY,
) -> bytes:
    """Binary frame the engine WebSocket bridge forwards to ZeroMQ."""
    if timestamp is None:
        timestamp = time()
    key_bytes = key.encode("utf-8")
    return b"".join(
        (
            bytes([5]),  # protocol: input
            struct.pack(">H", len(key_bytes)),
            key_bytes,
            struct.pack(">I", int(session_id)),
            struct.pack(">d", float(timestamp)),
            encode_keyboard(command),
        )
    )


class DriveLink:
    """One WebSocket to the Matrix data-in port."""

    def __init__(self, url: str) -> None:
        parsed = urlparse(url)
        if parsed.scheme not in ("ws", "http"):
            raise ValueError(f"drive link needs a ws:// URL, got {url}")
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or 7505
        self.path = parsed.path or "/"
        self._socket: socket.socket | None = None

    def connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))
        buffer = b""
        while b"\r\n\r\n" not in buffer:
            chunk = sock.recv(4096)
            if not chunk:
                sock.close()
                raise ConnectionError(f"no websocket handshake from {self.host}:{self.port}")
            buffer += chunk
            if len(buffer) > 8192:
                break
        status = buffer.split(b"\r\n", 1)[0]
        if b" 101 " not in status:
            sock.close()
            raise ConnectionError(status.decode("ascii", "replace"))
        accept_key = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()
        )
        if accept_key not in buffer:
            sock.close()
            raise ConnectionError("websocket accept key did not match")
        sock.settimeout(2)
        self._socket = sock

    def send(self, command: dict, *, session_id: int = 0) -> None:
        if self._socket is None:
            self.connect()
        payload = encode_input_frame(command, session_id=session_id)
        mask = os.urandom(4)
        header = bytearray([0x82])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header.extend(struct.pack(">H", length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack(">Q", length))
        header.extend(mask)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        assert self._socket is not None
        self._socket.sendall(bytes(header) + masked)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
