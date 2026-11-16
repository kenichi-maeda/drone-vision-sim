"""Replay recorded JPEG frames (no vehicle). Used for development and tests."""

from __future__ import annotations

import asyncio
from pathlib import Path
from time import monotonic

from .base import FeedSource, Frame, FrameCallback


class ReplayFeed(FeedSource):
    name = "replay"

    def __init__(self, path: Path, fps: float = 15.0, loop: bool = True) -> None:
        super().__init__()
        self.path = Path(path)
        self.period = 1.0 / max(fps, 0.1)
        self.loop = loop
        self.paused = False  # tests use this to simulate a dropped feed
        self._task: asyncio.Task | None = None

    def _files(self) -> list[Path]:
        folder = self.path / "frames" if (self.path / "frames").is_dir() else self.path
        files = sorted(folder.glob("*.jpg"))
        if not files:
            raise FileNotFoundError(f"No .jpg frames in {folder}")
        return files

    async def start(self, callback: FrameCallback) -> None:
        self._callback = callback
        files = self._files()
        self._task = asyncio.create_task(self._run(files), name="replay feed")

    async def _run(self, files: list[Path]) -> None:
        seq = 0
        while True:
            for file in files:
                while self.paused:
                    await asyncio.sleep(0.05)
                seq += 1
                self._emit(Frame(seq=seq, jpeg=file.read_bytes(), received_at=monotonic(),
                                 stamp=None, source="replay"))
                await asyncio.sleep(self.period)
            if not self.loop:
                return

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()

    def connected(self) -> bool:
        return self._task is not None and not self._task.done() and not self.paused

    def describe(self) -> str:
        return f"replay {self.path.name}"
