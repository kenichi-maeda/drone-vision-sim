"""Perception on another machine (perception.host: ws://HOST:PORT).

The remote side runs `python -m overwatch.perception.server`. Each request is
one binary websocket message: uint32 frame_seq | JPEG bytes. The reply is the
PerceptionResult as JSON. Requests are strictly one at a time.
"""

from __future__ import annotations

import json
import struct

from websockets.sync.client import connect

from .base import Perception, PerceptionResult


class RemotePerception(Perception):
    def __init__(self, url: str, timeout_s: float = 2.0) -> None:
        self.url = url
        self.timeout_s = timeout_s
        self._ws = None

    def describe(self) -> str:
        return f"remote {self.url}"

    def _connection(self):
        if self._ws is None:
            self._ws = connect(self.url, open_timeout=self.timeout_s, max_size=None)
        return self._ws

    def infer(self, jpeg: bytes, frame_seq: int) -> PerceptionResult:
        try:
            ws = self._connection()
            ws.send(struct.pack(">I", frame_seq) + jpeg)
            reply = json.loads(ws.recv(timeout=self.timeout_s))
        except Exception:
            self.close()
            raise
        if "error" in reply:
            raise RuntimeError(f"perception server: {reply['error']}")
        return PerceptionResult.from_dict(reply)

    def close(self) -> None:
        if self._ws is not None:
            try:
                self._ws.close()
            finally:
                self._ws = None
