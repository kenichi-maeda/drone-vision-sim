"""Target designation: one locked target, tracked by track ID across frames.

* A lock follows the tracker's track ID.
* If the locked track is not detected for `lost_after_s`, the status becomes LOST.
* While LOST and within `reacquire_window_s`, a single visible object with the
  same class and color is re-locked automatically (the tracker may give a
  returning object a new ID). If several match, the closest one to where the
  target was last seen is put under the keyboard cursor and the operator
  presses Enter to lock it.
* get_target_state() is the interface for a future pursuit controller.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import hypot

from ..perception.base import Detection, PerceptionResult


@dataclass(frozen=True)
class TargetState:
    status: str                       # NONE | TRACKING | LOST
    track_id: int | None = None
    class_name: str | None = None
    color: str | None = None
    confidence: float | None = None
    bbox_xyxy_px: tuple[float, float, float, float] | None = None  # last known box
    center_norm: tuple[float, float] | None = None  # 0..1 across the image (x right, y down)
    error_norm: tuple[float, float] | None = None   # -1..1 from the image center
    area_norm: float | None = None                  # box area / image area
    frame_seq: int | None = None                    # frame where it was last seen
    seconds_since_seen: float | None = None

    def to_dict(self) -> dict:
        data = asdict(self)
        for key in ("bbox_xyxy_px", "center_norm", "error_norm"):
            if data[key] is not None:
                data[key] = [round(v, 3) for v in data[key]]
        for key in ("confidence", "area_norm", "seconds_since_seen"):
            if data[key] is not None:
                data[key] = round(data[key], 3)
        return data


NO_TARGET = TargetState(status="NONE")


class TargetManager:
    def __init__(self, lost_after_s: float = 0.5, reacquire_window_s: float = 3.0) -> None:
        self.lost_after_s = lost_after_s
        self.reacquire_window_s = reacquire_window_s
        self.result: PerceptionResult | None = None
        self.track_id: int | None = None
        self.last: Detection | None = None
        self.last_frame_seq: int | None = None
        self.last_seen_at = 0.0
        self.lost_since: float | None = None
        self.cursor: int | None = None
        self.messages: list[tuple[str, str]] = []  # (level, text) for the UI, drained by the runtime

    # ---------------------------------------------------------------- queries
    @property
    def detections(self) -> list[Detection]:
        return self.result.detections if self.result else []

    def lockable(self) -> list[Detection]:
        """Tracked detections, left to right (the Tab order)."""
        return sorted((d for d in self.detections if d.track_id is not None), key=lambda d: d.center[0])

    def find(self, track_id: int | None) -> Detection | None:
        return next((d for d in self.detections if d.track_id == track_id and track_id is not None), None)

    @property
    def status(self) -> str:
        if self.track_id is None:
            return "NONE"
        return "LOST" if self.lost_since is not None else "TRACKING"

    def get_target_state(self, now: float) -> TargetState:
        if self.track_id is None or self.last is None or self.result is None:
            return NO_TARGET
        d = self.last
        width, height = self.result.width, self.result.height
        cx, cy = d.center
        return TargetState(
            status=self.status, track_id=self.track_id, class_name=d.class_name, color=d.color,
            confidence=d.confidence, bbox_xyxy_px=d.box,
            center_norm=(cx / width, cy / height),
            error_norm=((cx - width / 2) / (width / 2), (cy - height / 2) / (height / 2)),
            area_norm=d.area / float(width * height), frame_seq=self.last_frame_seq,
            seconds_since_seen=max(0.0, now - self.last_seen_at),
        )

    # --------------------------------------------------------------- updates
    def _say(self, level: str, text: str) -> None:
        self.messages.append((level, text))

    def update(self, result: PerceptionResult, now: float) -> None:
        self.result = result
        if self.cursor is not None and self.find(self.cursor) is None:
            self.cursor = None
        if self.track_id is None:
            return
        seen = self.find(self.track_id)
        if seen is not None:
            if self.lost_since is not None:
                self._say("info", f"Target #{self.track_id} back in view")
            self.last, self.last_frame_seq, self.last_seen_at = seen, result.frame_seq, now
            self.lost_since = None
            return
        if self.lost_since is None and now - self.last_seen_at > self.lost_after_s:
            self.lost_since = now
            self._say("warn", f"TARGET LOST: #{self.track_id} {self.describe(self.last)}")
        if self.lost_since is not None and now - self.lost_since <= self.reacquire_window_s:
            self._try_reacquire(now)

    def check(self, now: float) -> None:
        """Age the lock even when no new results arrive (feed lost, inference stalled)."""
        if self.track_id is not None and self.lost_since is None and now - self.last_seen_at > self.lost_after_s:
            self.lost_since = now
            self._say("warn", f"TARGET LOST: #{self.track_id} {self.describe(self.last)} (no fresh detections)")

    def _try_reacquire(self, now: float) -> None:
        old = self.last
        matches = [d for d in self.lockable()
                   if d.class_name == old.class_name and d.color == old.color and d.track_id != self.track_id]
        if len(matches) == 1:
            new = matches[0]
            previous = self.track_id
            self._set_lock(new, now)
            self._say("info", f"Reacquired {self.describe(new)} as #{new.track_id} (was #{previous})")
        elif len(matches) > 1 and self.cursor is None:
            ox, oy = old.center
            self.cursor = min(matches, key=lambda d: hypot(d.center[0] - ox, d.center[1] - oy)).track_id
            self._say("info", f"{len(matches)} possible matches for the lost target; Enter locks the highlighted one")

    def _set_lock(self, detection: Detection, now: float) -> None:
        self.track_id = detection.track_id
        self.last = detection
        self.last_frame_seq = self.result.frame_seq if self.result else None
        self.last_seen_at = now
        self.lost_since = None
        self.cursor = None

    # --------------------------------------------------------------- actions
    @staticmethod
    def describe(d: Detection | None) -> str:
        if d is None:
            return "target"
        color = "" if d.color == "unknown" else f"{d.color} "
        return f"{color}{d.class_name}"

    def lock(self, track_id: int, now: float) -> str:
        detection = self.find(track_id)
        if detection is None:
            return f"#{track_id} is not in view"
        self._set_lock(detection, now)
        return f"Locked #{track_id} {self.describe(detection)}"

    def lock_at(self, frame_result: PerceptionResult | None, x: float, y: float, now: float) -> str:
        """Lock the object under a click. `frame_result` is the result for the frame
        the operator clicked on, so the click is judged against what they saw."""
        if frame_result is None:
            return "That frame is too old; click again"
        hits = [d for d in frame_result.detections
                if d.box[0] <= x <= d.box[2] and d.box[1] <= y <= d.box[3]]
        if not hits:
            return "No object there"
        hit = min(hits, key=lambda d: d.area)  # the most specific box under the pointer
        if hit.track_id is None:
            return "That object is not tracked yet; click again in a moment"
        return self.lock(hit.track_id, now)

    def release(self) -> str:
        if self.track_id is None:
            return "No target locked"
        text = f"Released #{self.track_id}"
        self.track_id = None
        self.last = None
        self.lost_since = None
        return text

    def cycle(self, direction: int) -> str:
        order = self.lockable()
        if not order:
            self.cursor = None
            return "No tracked objects in view"
        ids = [d.track_id for d in order]
        if self.cursor in ids:
            index = (ids.index(self.cursor) + direction) % len(ids)
        elif self.track_id in ids:
            index = (ids.index(self.track_id) + direction) % len(ids)
        else:
            index = 0 if direction > 0 else len(ids) - 1
        self.cursor = ids[index]
        return f"Highlighted #{self.cursor} {self.describe(order[index])} ({index + 1}/{len(ids)}); Enter locks"

    def confirm(self, now: float) -> str:
        if self.cursor is None:
            return "Nothing highlighted; press Tab first"
        return self.lock(self.cursor, now)
