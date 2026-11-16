"""YOLO-World detection + ByteTrack tracking + HSV color, run in this process."""

from __future__ import annotations

import logging
from time import perf_counter
from typing import Any

import cv2
import numpy as np

from ..config import resolve
from .base import Detection, Perception, PerceptionResult
from .color import classify_color

log = logging.getLogger("overwatch.perception")


class YoloWorldPerception(Perception):
    def __init__(self, cfg: dict[str, Any]) -> None:
        from ultralytics import YOLO

        self.cfg = cfg
        weights = resolve(cfg["model"])
        if not weights.exists():
            raise FileNotFoundError(
                f"Model weights not found: {weights}. Download them with:\n"
                f"  cd overwatch && .venv/bin/python -c \"from ultralytics import YOLO; YOLO('{weights.name}')\" "
                f"&& mv {weights.name} {weights}")
        self.model = YOLO(str(weights))
        # One prompt index per (class, prompt); several prompts can map to one class.
        self.prompt_class: list[str] = []
        prompts: list[str] = []
        for entry in cfg["classes"]:
            for prompt in entry["prompts"]:
                prompts.append(prompt)
                self.prompt_class.append(entry["name"])
        if hasattr(self.model, "set_classes"):
            self.model.set_classes(prompts)
        self.tracker = str(resolve(cfg["tracker"]))
        self.color_cfg = cfg.get("color", {})
        # Warm up so the first live frame does not pay for model setup.
        self.model.predict(np.zeros((640, 480, 3), np.uint8), device=cfg["device"], verbose=False)
        log.info("YOLO-World ready: %s on %s, prompts %s", weights.name, cfg["device"], prompts)

    def describe(self) -> str:
        return f"YOLO-World {resolve(self.cfg['model']).stem} ({self.cfg['device']})"

    def infer(self, jpeg: bytes, frame_seq: int) -> PerceptionResult:
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("could not decode JPEG")
        started = perf_counter()
        result = self.model.track(
            image, persist=True, tracker=self.tracker, conf=float(self.cfg["conf"]),
            iou=float(self.cfg["iou"]), imgsz=int(self.cfg["imgsz"]), device=self.cfg["device"],
            verbose=False,
        )[0]
        detections: list[Detection] = []
        boxes = result.boxes
        if boxes is not None and len(boxes):
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
            ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else [None] * len(xyxy)
            for box, conf, cls, track_id in zip(xyxy, confs, classes, ids):
                box = tuple(float(v) for v in box)
                color, fraction = classify_color(image, box, **self.color_cfg)
                detections.append(Detection(
                    track_id=None if track_id is None else int(track_id),
                    class_name=self.prompt_class[cls] if cls < len(self.prompt_class) else str(cls),
                    confidence=float(conf), box=box, color=color, color_fraction=fraction,
                ))
        height, width = image.shape[:2]
        return PerceptionResult(frame_seq=frame_seq, width=width, height=height, detections=detections,
                                inference_ms=(perf_counter() - started) * 1000.0)
