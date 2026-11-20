"""Color labels and fisheye removal for the simulated front camera.

The color ranges match the physical-drone viewer. The camera model is the
equidistant calibration in maps/tag_sandbox/cameras.yaml. Specified objects
are the props this map is built to show.
"""

from __future__ import annotations

import cv2
import numpy as np


# OpenCV hue is 0-179. Red wraps around 0.
COLOR_RANGES = {
    "red": (((0, 120, 50), (8, 255, 255)), ((170, 120, 50), (179, 255, 255))),
    "orange": (((9, 140, 50), (20, 255, 255)),),
    "yellow": (((21, 140, 70), (35, 255, 255)),),
    "green": (((36, 80, 40), (85, 255, 255)),),
    "blue": (((96, 80, 40), (130, 255, 255)),),
    "purple": (((131, 80, 40), (165, 255, 255)),),
}
COLOR_BGR = {
    "red": (40, 40, 220),
    "orange": (0, 140, 255),
    "yellow": (0, 220, 220),
    "green": (80, 200, 80),
    "blue": (220, 120, 40),
    "purple": (180, 60, 160),
}

# Props placed in front of the drone, and the COCO class plus color they must get.
# The Matrix traffic-light prefab does not draw a light on this grass map,
# so it is not one of the objects this view is required to name.
SPECIFIED_OBJECTS = {
    32: {"name": "sports_ball", "label": "duckie", "color": "yellow"},
    11: {"name": "stop_sign", "label": "stop sign", "color": "red"},
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
    if best_name is None or best_share < 0.08:
        return None
    return best_name, round(best_share, 2)


def specified_for(class_id: int) -> dict | None:
    return SPECIFIED_OBJECTS.get(int(class_id))


def resolve_specified(text: str) -> dict | None:
    """Match a typed name to one of the objects this map can fly to."""
    key = " ".join(str(text).strip().lower().replace("_", " ").split())
    if not key:
        return None
    for class_id, spec in SPECIFIED_OBJECTS.items():
        names = {
            spec["label"].lower(),
            spec["name"].lower().replace("_", " "),
            str(class_id),
        }
        if key in names:
            return {"class_id": int(class_id), **spec}
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
