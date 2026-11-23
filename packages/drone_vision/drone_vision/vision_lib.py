"""Color labels and fisheye removal for the simulated front camera.

The color ranges match the physical-drone viewer. The camera model is the
equidistant calibration in maps/tag_sandbox/cameras.yaml. Specified objects
are the props this map is built to show.
"""

from __future__ import annotations

import cv2
import numpy as np


# OpenCV hue is 0-179. Red wraps around 0. The spans are narrow so
# neighboring duck colors do not claim each other.
COLOR_RANGES = {
    "red": (((0, 130, 70), (6, 255, 255)), ((174, 130, 70), (179, 255, 255))),
    "orange": (((7, 140, 165), (18, 255, 255)),),
    "brown": (((8, 70, 30), (20, 220, 160)),),
    "yellow": (((19, 120, 80), (34, 255, 255)),),
    "lime": (((35, 100, 80), (48, 255, 255)),),
    "green": (((49, 80, 40), (84, 255, 255)),),
    "cyan": (((85, 80, 60), (102, 255, 255)),),
    "blue": (((103, 80, 40), (128, 255, 255)),),
    "purple": (((129, 70, 40), (155, 255, 255)),),
    "pink": (((156, 50, 80), (173, 255, 255)),),
}
COLOR_BGR = {
    "red": (32, 32, 224),
    "orange": (24, 120, 240),
    "brown": (32, 75, 138),
    "yellow": (32, 220, 240),
    "lime": (32, 224, 176),
    "green": (64, 192, 32),
    "cyan": (208, 208, 32),
    "blue": (240, 96, 32),
    "purple": (224, 32, 128),
    "pink": (160, 96, 240),
    "white": (242, 242, 242),
}

# Each flight target is a COCO class plus the color painted on that prop.
# Ten extra ducks sit around the yellow one. The stop sign stays red.
SPECIFIED_OBJECTS = {
    "duckie": {"name": "sports_ball", "label": "duckie", "color": "yellow", "class_id": 32},
    "red bot": {"name": "truck", "label": "red bot", "color": "red", "class_id": 7},
    "orange bot": {"name": "truck", "label": "orange bot", "color": "orange", "class_id": 7},
    "green bot": {"name": "truck", "label": "green bot", "color": "green", "class_id": 7},
    "cyan bot": {"name": "truck", "label": "cyan bot", "color": "cyan", "class_id": 7},
    "blue bot": {"name": "truck", "label": "blue bot", "color": "blue", "class_id": 7},
    "purple bot": {"name": "truck", "label": "purple bot", "color": "purple", "class_id": 7},
    "pink bot": {"name": "truck", "label": "pink bot", "color": "pink", "class_id": 7},
    "white bot": {"name": "truck", "label": "white bot", "color": "white", "class_id": 7},
    "brown bot": {"name": "truck", "label": "brown bot", "color": "brown", "class_id": 7},
    "lime bot": {"name": "truck", "label": "lime bot", "color": "lime", "class_id": 7},
    "stop sign": {"name": "stop_sign", "label": "stop sign", "color": "red", "class_id": 11},
}

# maps/tag_sandbox/cameras.yaml front_center, equidistant, 480x640.
CALIB_WIDTH = 480
CALIB_HEIGHT = 640
FISHEYE_K = np.array(
    [[311.2, 0.0, 240.0], [0.0, 311.07, 320.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
FISHEYE_D = np.array([-0.0285, 0.0145, -0.0169, 0.0031], dtype=np.float64)
# 0 crops the stop sign off the left edge. 0.7 keeps both props in frame
# and flattens the horizon, leaving thin black borders from the fisheye.
UNDISTORT_BALANCE = 0.7

_RECTIFY_MAPS: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}


def color_inside(frame: np.ndarray, x: int, y: int, w: int, h: int):
    """Strongest saturated color in the middle of a box.

    The middle of the box is used so grass around a small prop does not
    win. The confidence is the share of that crop painted in the color.
    """
    if w < 4 or h < 4:
        return None
    margin_x = int(round(w * 0.2))
    margin_y = int(round(h * 0.2))
    if w - 2 * margin_x >= 4 and h - 2 * margin_y >= 4:
        x += margin_x
        y += margin_y
        w -= 2 * margin_x
        h -= 2 * margin_y
    height, width = frame.shape[:2]
    x = min(max(int(x), 0), width - 1)
    y = min(max(int(y), 0), height - 1)
    x_max = min(max(x + int(w), x + 1), width)
    y_max = min(max(y + int(h), y + 1), height)
    crop = frame[y:y_max, x:x_max]
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    area = float(crop.shape[0] * crop.shape[1])
    best_share = 0.0
    best_name = None
    for name, spans in COLOR_RANGES.items():
        mask = None
        for low, high in spans:
            part = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        share = float(cv2.countNonZero(mask)) / area
        if share > best_share:
            best_share = share
            best_name = name
    white = cv2.inRange(hsv, np.array((0, 0, 185), dtype=np.uint8), np.array((179, 45, 255), dtype=np.uint8))
    white_share = float(cv2.countNonZero(white)) / area
    if white_share > best_share and white_share >= 0.45:
        best_share = white_share
        best_name = "white"
    if best_name is None or best_share < 0.08:
        return None
    return best_name, round(best_share, 2)


def match_specified(class_id: int, color: str | None) -> dict | None:
    """The prop with this COCO class and measured color, if the map has one."""
    if not color:
        return None
    hits = [
        spec for spec in SPECIFIED_OBJECTS.values()
        if int(spec["class_id"]) == int(class_id) and spec["color"] == color
    ]
    if len(hits) == 1:
        return hits[0]
    return None


def specified_for(class_id: int) -> dict | None:
    hits = [spec for spec in SPECIFIED_OBJECTS.values() if int(spec["class_id"]) == int(class_id)]
    if len(hits) == 1:
        return hits[0]
    return None


def resolve_specified(text: str) -> dict | None:
    """Match a typed name to one of the objects this map can fly to."""
    key = " ".join(str(text).strip().lower().replace("_", " ").split())
    if not key:
        return None
    if key in SPECIFIED_OBJECTS:
        return dict(SPECIFIED_OBJECTS[key])
    for spec in SPECIFIED_OBJECTS.values():
        names = {spec["name"].lower().replace("_", " "), str(spec["class_id"])}
        if key in names and specified_for(int(spec["class_id"])) is spec:
            return dict(spec)
    if key in {"sports ball", "32"}:
        return dict(SPECIFIED_OBJECTS["duckie"])
    if key == "11":
        return dict(SPECIFIED_OBJECTS["stop sign"])
    return None


def _scaled_camera(width: int, height: int) -> np.ndarray:
    camera = FISHEYE_K.copy()
    camera[0, 0] *= width / CALIB_WIDTH
    camera[0, 2] *= width / CALIB_WIDTH
    camera[1, 1] *= height / CALIB_HEIGHT
    camera[1, 2] *= height / CALIB_HEIGHT
    return camera


def rectify_maps(width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """Pixel maps that turn the equidistant frame into a pinhole frame."""
    cached = _RECTIFY_MAPS.get((width, height))
    if cached is not None:
        return cached
    camera = _scaled_camera(width, height)
    new_camera = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camera,
        FISHEYE_D.reshape(4, 1),
        (width, height),
        np.eye(3),
        balance=UNDISTORT_BALANCE,
        new_size=(width, height),
        fov_scale=1.0,
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        camera,
        FISHEYE_D.reshape(4, 1),
        np.eye(3),
        new_camera,
        (width, height),
        cv2.CV_16SC2,
    )
    _RECTIFY_MAPS[(width, height)] = (map1, map2)
    return map1, map2


def rectify(image: np.ndarray) -> np.ndarray:
    """Remove the front-camera fisheye. Same size as the input."""
    height, width = image.shape[:2]
    map1, map2 = rectify_maps(width, height)
    return cv2.remap(
        image,
        map1,
        map2,
        interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
