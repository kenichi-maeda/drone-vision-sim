"""Feed sources: where camera frames come from. Swapping SIM -> LIVE changes only this
and the flight adapter."""

from __future__ import annotations

from typing import Any

from ..config import resolve
from ..link_client import shared_client
from .base import FeedSource, Frame


def make_feed(source_cfg: dict[str, Any]) -> FeedSource:
    kind = source_cfg["feed"]
    if kind == "link":
        from .link import LinkFeed
        link = source_cfg["link"]
        return LinkFeed(shared_client(link["host"], link["port"]))
    if kind == "replay":
        from .replay import ReplayFeed
        replay = source_cfg["replay"]
        return ReplayFeed(resolve(replay["path"]), fps=float(replay.get("fps", 15)), loop=bool(replay.get("loop", True)))
    if kind == "mjpeg":
        from .mjpeg import MjpegFeed
        return MjpegFeed(source_cfg["mjpeg_url"])
    raise ValueError(f"Unknown feed type '{kind}'")


__all__ = ["FeedSource", "Frame", "make_feed"]
