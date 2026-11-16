"""Append-only JSONL event log (every KILL and LAND NOW, plus their acknowledgements)."""

from __future__ import annotations

import json
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from time import time


class EventLog:
    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.recent: deque[dict] = deque(maxlen=100)

    @property
    def path(self) -> Path:
        return self.directory / f"events-{datetime.now().strftime('%Y%m%d')}.jsonl"

    def record(self, event: str, **fields) -> dict:
        now = time()
        entry = {"t": now, "iso": datetime.fromtimestamp(now, timezone.utc).isoformat(), "event": event, **fields}
        with open(self.path, "a") as handle:
            handle.write(json.dumps(entry, default=str) + "\n")
        self.recent.append(entry)
        return entry
