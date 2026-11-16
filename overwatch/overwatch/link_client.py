"""Asyncio client for overwatch_link (shared by the link feed and link flight adapters)."""

from __future__ import annotations

import asyncio
import logging
import socket
from time import monotonic
from typing import Callable

from .protocol import encode_json, read_message

log = logging.getLogger("overwatch.link")

FrameListener = Callable[[dict, bytes, float], None]
MessageListener = Callable[[dict], None]

# overwatch_link sends state at 10 Hz; silence this long means the link is gone.
SILENCE_TIMEOUT_S = 3.0


class LinkClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.connected = False
        self.hello: dict | None = None
        self.state: dict | None = None
        self.state_at = 0.0
        self._writer: asyncio.StreamWriter | None = None
        self._seq = 0
        self._frame_listeners: list[FrameListener] = []
        self._message_listeners: list[MessageListener] = []
        self._task: asyncio.Task | None = None

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    def on_frame(self, listener: FrameListener) -> None:
        self._frame_listeners.append(listener)

    def on_message(self, listener: MessageListener) -> None:
        self._message_listeners.append(listener)

    def start(self) -> None:
        if self._task is None:
            self._task = asyncio.create_task(self._run(), name=f"link {self.address}")

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self._drop()

    def _drop(self) -> None:
        if self._writer is not None:
            self._writer.close()
        self._writer = None
        if self.connected:
            log.warning("Link %s lost", self.address)
        self.connected = False
        self.state = None

    async def _run(self) -> None:
        backoff = 0.5
        while True:
            try:
                reader, writer = await asyncio.wait_for(
                    asyncio.open_connection(self.host, self.port), timeout=3.0)
            except (OSError, asyncio.TimeoutError):
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 3.0)
                continue
            sock = writer.get_extra_info("socket")
            if sock is not None:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._writer = writer
            try:
                while True:
                    kind, obj, jpeg = await asyncio.wait_for(read_message(reader), SILENCE_TIMEOUT_S)
                    backoff = 0.5
                    now = monotonic()
                    if kind == "image":
                        for listener in self._frame_listeners:
                            listener(obj, jpeg, now)
                        continue
                    kind_name = obj.get("type")
                    if kind_name == "hello":
                        self.hello = obj
                        self.connected = True
                        log.info("Link %s connected: robot %s", self.address, obj.get("robot"))
                    elif kind_name == "state":
                        self.state, self.state_at = obj, now
                    for listener in self._message_listeners:
                        listener(obj)
            except (asyncio.IncompleteReadError, asyncio.TimeoutError, OSError, ValueError) as error:
                log.info("Link %s read ended: %s", self.address, type(error).__name__)
            finally:
                self._drop()
            await asyncio.sleep(backoff)

    async def send(self, obj: dict) -> bool:
        writer = self._writer
        if writer is None or not self.connected:
            return False
        try:
            writer.write(encode_json(obj))
            await asyncio.wait_for(writer.drain(), timeout=0.5)
            return True
        except (OSError, asyncio.TimeoutError):
            self._drop()
            return False

    async def command(self, cmd: str, **fields) -> int | None:
        """Send a flight command; returns its sequence number, or None if not sent."""
        self._seq += 1
        seq = self._seq
        ok = await self.send({"type": "cmd", "seq": seq, "cmd": cmd, **fields})
        return seq if ok else None


_clients: dict[tuple[str, int], LinkClient] = {}


def shared_client(host: str, port: int) -> LinkClient:
    """One connection per drone, shared by its feed and flight adapters."""
    key = (host, int(port))
    if key not in _clients:
        _clients[key] = LinkClient(host, int(port))
    return _clients[key]
