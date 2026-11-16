import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from overwatch.perception.base import Detection, PerceptionResult  # noqa: E402

CLASSES = [
    {"name": "robot", "prompts": ["toy robot car"], "synonyms": ["robot", "duckiebot", "bot", "car", "rover", "vehicle"]},
    {"name": "duckie", "prompts": ["rubber duck"], "synonyms": ["duckie", "duck", "ducky", "rubber duck"]},
    {"name": "drone", "prompts": ["quadcopter drone"], "synonyms": ["drone", "quadcopter", "quad"]},
]


def det(track_id, class_name, color, x1, y1, x2, y2, conf=0.6):
    return Detection(track_id=track_id, class_name=class_name, confidence=conf, box=(x1, y1, x2, y2), color=color,
                     color_fraction=0.8)


def scene(seq=1):
    """Arena-like scene, 480x640: two red robots, a blue robot, a green robot, two duckies."""
    return PerceptionResult(frame_seq=seq, width=480, height=640, detections=[
        det(3, "robot", "red", 40, 300, 120, 360),       # left, mid
        det(8, "robot", "red", 330, 200, 400, 250),      # right, far
        det(5, "robot", "blue", 200, 380, 290, 450),     # center, near-ish
        det(6, "robot", "green", 380, 420, 470, 500),    # right, near
        det(11, "duckie", "yellow", 20, 520, 50, 550),   # left, nearest
        det(12, "duckie", "yellow", 300, 120, 315, 135),  # right, farthest
    ])


@pytest.fixture
def classes():
    return CLASSES
