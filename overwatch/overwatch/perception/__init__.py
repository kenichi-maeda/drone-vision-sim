"""Perception: detection, tracking and color. Where it runs is `perception.host`."""

from __future__ import annotations

from typing import Any

from .base import Detection, Perception, PerceptionResult


def make_perception(cfg: dict[str, Any]) -> Perception:
    host = str(cfg.get("host", "local"))
    if host == "local":
        from .yolo_world import YoloWorldPerception
        return YoloWorldPerception(cfg)
    if host.startswith(("ws://", "wss://")):
        from .remote import RemotePerception
        return RemotePerception(host)
    raise ValueError(f"perception.host must be 'local' or a ws:// URL, not '{host}'")


__all__ = ["Detection", "Perception", "PerceptionResult", "make_perception"]
