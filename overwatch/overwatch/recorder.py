"""Record raw camera frames (the drone's own JPEGs) plus vehicle state, for the
recall eval, for fine-tuning, and for the replay feed.

Layout: recordings/<name>/frames/000001.jpg ... and recordings/<name>/frames.jsonl
(one line per saved frame: file, seq, time, tag, vehicle state). A symlink
recordings/latest points at the newest recording.
"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from time import time

from .feed.base import Frame


class Recorder:
    def __init__(self, root: Path, interval_s: float = 0.5) -> None:
        self.root = Path(root)
        self.interval_s = interval_s
        self.folder: Path | None = None
        self.tag = ""
        self.count = 0
        self._last_saved = 0.0

    @property
    def active(self) -> bool:
        return self.folder is not None

    def start(self, name: str | None = None, tag: str = "", interval_s: float | None = None) -> Path:
        name = name or datetime.now().strftime("rec-%Y%m%d-%H%M%S")
        self.folder = self.root / name
        (self.folder / "frames").mkdir(parents=True, exist_ok=True)
        self.tag = tag
        self.count = len(list((self.folder / "frames").glob("*.jpg")))
        if interval_s is not None:
            self.interval_s = interval_s
        latest = self.root / "latest"
        if latest.is_symlink() or not latest.exists():
            if latest.is_symlink():
                latest.unlink()
            os.symlink(self.folder.name, latest)
        return self.folder

    def set_tag(self, tag: str) -> None:
        self.tag = tag

    def stop(self) -> dict:
        summary = {"folder": str(self.folder) if self.folder else None, "frames": self.count}
        self.folder = None
        return summary

    def offer(self, frame: Frame, vehicle: dict) -> None:
        if self.folder is None or frame.received_at - self._last_saved < self.interval_s:
            return
        self._last_saved = frame.received_at
        self.count += 1
        name = f"{self.count:06d}.jpg"
        (self.folder / "frames" / name).write_bytes(frame.jpeg)
        with open(self.folder / "frames.jsonl", "a") as handle:
            handle.write(json.dumps({"file": name, "seq": frame.seq, "t": time(), "stamp": frame.stamp,
                                     "tag": self.tag, "vehicle": vehicle}) + "\n")
