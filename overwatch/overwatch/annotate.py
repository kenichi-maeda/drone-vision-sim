"""Draw detections and the target lock on a frame, server-side.

The browser draws its own overlay from the same packet. This copy is for
snapshots (/api/snapshot.jpg), the eval contact sheets, and test evidence.
"""

from __future__ import annotations

import cv2
import numpy as np

from .perception.base import PerceptionResult

BGR = {"red": (40, 40, 230), "orange": (20, 140, 255), "yellow": (0, 220, 240), "green": (60, 200, 60),
       "blue": (230, 120, 30), "purple": (200, 60, 170), "unknown": (200, 200, 200)}
LOCK = (0, 0, 255)


def annotate(jpeg: bytes, result: PerceptionResult, target: dict | None = None, cursor: int | None = None,
             banners: list[str] | None = None) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    locked = target.get("track_id") if target and target.get("status") == "TRACKING" else None
    for d in result.detections:
        x1, y1, x2, y2 = (int(round(v)) for v in d.box)
        is_lock = d.track_id is not None and d.track_id == locked
        color = LOCK if is_lock else BGR.get(d.color, BGR["unknown"])
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3 if is_lock else 2)
        if d.track_id is not None and d.track_id == cursor:
            cv2.rectangle(image, (x1 - 4, y1 - 4), (x2 + 4, y2 + 4), (255, 255, 255), 1)
        label = f"#{d.track_id if d.track_id is not None else '?'} {d.color} {d.class_name} {d.confidence:.2f}"
        cv2.putText(image, label, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3)
        cv2.putText(image, label, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        if is_lock:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            r = max(12, (x2 - x1) // 2 + 8)
            cv2.circle(image, (cx, cy), r, LOCK, 2)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                cv2.line(image, (cx + dx * (r - 6), cy + dy * (r - 6)), (cx + dx * (r + 8), cy + dy * (r + 8)), LOCK, 2)
            cv2.putText(image, "LOCKED", (x1, min(image.shape[0] - 4, y2 + 16)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, LOCK, 2)
    for i, text in enumerate(banners or []):
        cv2.putText(image, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4)
        cv2.putText(image, text, (10, 28 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
    return image
