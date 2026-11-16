"""Feed source interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from time import monotonic
from typing import Callable


@dataclass(frozen=True)
class Frame:
    seq: int              # feed-assigned frame number
    jpeg: bytes           # the camera's own JPEG bytes, untouched
    received_at: float    # time.monotonic() on arrival
    stamp: float | None   # capture time from the drone, if known (seconds)
    source: str


FrameCallback = Callable[[Frame], None]


class FeedSource(ABC):
    """Delivers camera frames to one callback. Implementations must not queue:
    they hand over each frame as it arrives, and the consumer keeps only the newest."""

    name = "feed"

    def __init__(self) -> None:
        self._callback: FrameCallback | None = None
        self._arrivals: deque[float] = deque(maxlen=30)
        self.last_frame_at = 0.0

    def _emit(self, frame: Frame) -> None:
        self.last_frame_at = frame.received_at
        self._arrivals.append(frame.received_at)
        if self._callback is not None:
            self._callback(frame)

    def fps(self) -> float:
        times = self._arrivals
        if len(times) < 2 or monotonic() - times[-1] > 1.0:
            return 0.0
        return (len(times) - 1) / max(times[-1] - times[0], 1e-6)

    @abstractmethod
    async def start(self, callback: FrameCallback) -> None: ...

    @abstractmethod
    async def stop(self) -> None: ...

    @abstractmethod
    def connected(self) -> bool: ...

    def describe(self) -> str:
        return self.name
