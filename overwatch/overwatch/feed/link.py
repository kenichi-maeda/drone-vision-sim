"""Camera frames from overwatch_link (SIM, and LIVE for an Ente/PX4 drone)."""

from __future__ import annotations

from ..link_client import LinkClient
from .base import FeedSource, Frame, FrameCallback


class LinkFeed(FeedSource):
    name = "link"

    def __init__(self, client: LinkClient) -> None:
        super().__init__()
        self.client = client
        client.on_frame(self._on_frame)

    def _on_frame(self, header: dict, jpeg: bytes, received_at: float) -> None:
        self._emit(Frame(seq=int(header.get("seq", 0)), jpeg=jpeg, received_at=received_at,
                         stamp=header.get("stamp"), source="link"))

    async def start(self, callback: FrameCallback) -> None:
        self._callback = callback
        self.client.start()

    async def stop(self) -> None:
        await self.client.stop()

    def connected(self) -> bool:
        return self.client.connected

    def describe(self) -> str:
        return f"link {self.client.address}"
