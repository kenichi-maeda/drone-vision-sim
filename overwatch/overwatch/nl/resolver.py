"""Resolve a TargetQuery against the current detections.

Image-position meanings (the camera is pitched down toward the ground):
  leftmost/rightmost   smallest/largest box center x
  topmost/bottommost   smallest/largest box center y
  closest/farthest     lowest/highest box bottom edge in the image
  largest/smallest     box area
  center               box center nearest the image center
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import hypot

from ..perception.base import Detection
from .schema import TargetQuery

PICKERS = {
    "leftmost": lambda d, w, h: d.center[0],
    "rightmost": lambda d, w, h: -d.center[0],
    "topmost": lambda d, w, h: d.center[1],
    "bottommost": lambda d, w, h: -d.center[1],
    "closest": lambda d, w, h: -d.box[3],
    "farthest": lambda d, w, h: d.box[3],
    "largest": lambda d, w, h: -d.area,
    "smallest": lambda d, w, h: d.area,
    "center": lambda d, w, h: hypot(d.center[0] - w / 2, d.center[1] - h / 2),
}


@dataclass
class Outcome:
    kind: str                     # lock | release | clarify | none
    message: str = ""
    track_id: int | None = None
    candidates: list[Detection] = field(default_factory=list)
    matched: list[int] = field(default_factory=list)  # track IDs that matched the description


def describe(d: Detection) -> str:
    color = "" if d.color == "unknown" else f"{d.color} "
    return f"{color}{d.class_name}"


def where(d: Detection, width: int, height: int) -> str:
    cx = d.center[0] / width
    horizontal = "left" if cx < 1 / 3 else "right" if cx > 2 / 3 else "center"
    bottom = d.box[3] / height
    depth = "near" if bottom > 0.7 else "far" if bottom < 0.4 else "mid"
    return f"{horizontal}, {depth}"


def summarize(detections: list[Detection]) -> str:
    if not detections:
        return "Nothing tracked in view."
    return "In view: " + ", ".join(f"#{d.track_id} {describe(d)}" for d in detections) + "."


def clarify(candidates: list[Detection], label: str, width: int, height: int) -> Outcome:
    ordered = sorted(candidates, key=lambda d: d.center[0])
    options = "  ".join(f"{i}) #{d.track_id} {describe(d)} ({where(d, width, height)})"
                        for i, d in enumerate(ordered, 1))
    return Outcome("clarify", f"{len(ordered)} match '{label}': {options}. Which one?",
                   candidates=ordered, matched=[d.track_id for d in ordered])


def _filter(query: TargetQuery, pool: list[Detection]) -> list[Detection]:
    return [d for d in pool
            if (query.class_name is None or d.class_name == query.class_name)
            and (query.color is None or d.color == query.color)]


def resolve(query: TargetQuery, detections: list[Detection], width: int, height: int,
            current: int | None = None, previous: list[int] | None = None) -> Outcome:
    """`detections` must be tracked (track_id set). `current` is the locked track,
    `previous` the track IDs matched by the last command (for "the other one")."""
    if query.action == "release":
        return Outcome("release")
    label = query.describe()
    if query.track_id is not None:
        hit = next((d for d in detections if d.track_id == query.track_id), None)
        if hit is None:
            return Outcome("none", f"#{query.track_id} is not in view. {summarize(detections)}")
        return Outcome("lock", track_id=hit.track_id, matched=[hit.track_id])

    if query.selector == "other":
        pool = [d for d in detections if previous and d.track_id in previous]
        if not pool and current is not None:
            locked = next((d for d in detections if d.track_id == current), None)
            if locked is not None:
                pool = [d for d in detections if d.class_name == locked.class_name and d.color == locked.color]
        pool = [d for d in _filter(query, pool) if d.track_id != current]
        if not pool:
            return Outcome("none", f"There is no other {label} in view. {summarize(detections)}")
        if len(pool) == 1:
            return Outcome("lock", track_id=pool[0].track_id, matched=[d.track_id for d in pool])
        return clarify(pool, f"other {label}", width, height)

    matches = _filter(query, detections)
    if not matches:
        return Outcome("none", f"No {label} in view. {summarize(detections)}")
    matched = [d.track_id for d in matches]
    if query.ordinal is not None:
        ordered = sorted(matches, key=lambda d: d.center[0], reverse=query.selector == "rightmost")
        if query.ordinal > len(ordered):
            return Outcome("none", f"Only {len(ordered)} {label} in view.")
        return Outcome("lock", track_id=ordered[query.ordinal - 1].track_id, matched=matched)
    if query.selector is not None:
        best = min(matches, key=lambda d: PICKERS[query.selector](d, width, height))
        return Outcome("lock", track_id=best.track_id, matched=matched)
    if len(matches) == 1:
        return Outcome("lock", track_id=matches[0].track_id, matched=matched)
    return clarify(matches, label, width, height)
