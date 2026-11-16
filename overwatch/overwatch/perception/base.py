"""Perception interface and result types."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field


@dataclass
class Detection:
    track_id: int | None          # stable ID from the tracker; None until the track is confirmed
    class_name: str
    confidence: float
    box: tuple[float, float, float, float]  # x1, y1, x2, y2 in image pixels
    color: str = "unknown"
    color_fraction: float = 0.0

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2.0, (y1 + y2) / 2.0

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.box
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)

    def to_dict(self) -> dict:
        data = asdict(self)
        data["box"] = [round(v, 1) for v in self.box]
        data["confidence"] = round(self.confidence, 3)
        data["color_fraction"] = round(self.color_fraction, 2)
        return data


@dataclass
class PerceptionResult:
    frame_seq: int
    width: int
    height: int
    detections: list[Detection] = field(default_factory=list)
    inference_ms: float = 0.0

    def to_dict(self) -> dict:
        return {
            "frame_seq": self.frame_seq, "width": self.width, "height": self.height,
            "inference_ms": round(self.inference_ms, 1),
            "detections": [d.to_dict() for d in self.detections],
        }

    @classmethod
    def from_dict(cls, data: dict) -> PerceptionResult:
        return cls(
            frame_seq=data["frame_seq"], width=data["width"], height=data["height"],
            inference_ms=data.get("inference_ms", 0.0),
            detections=[Detection(track_id=d["track_id"], class_name=d["class_name"], confidence=d["confidence"],
                                  box=tuple(d["box"]), color=d.get("color", "unknown"),
                                  color_fraction=d.get("color_fraction", 0.0)) for d in data["detections"]],
        )


class Perception(ABC):
    """Runs detection + tracking + color on one JPEG. Called from a worker thread,
    one frame at a time, in order."""

    @abstractmethod
    def infer(self, jpeg: bytes, frame_seq: int) -> PerceptionResult: ...

    def close(self) -> None:
        pass

    def describe(self) -> str:
        return type(self).__name__
