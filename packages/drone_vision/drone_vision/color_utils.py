"""OpenCV color detection helpers shared by the color node, the YOLO node and
the offline test script. Pure NumPy/OpenCV: no ROS imports, so it runs anywhere.

Colors are classified in HSV, where OpenCV uses H in [0, 179], S and V in
[0, 255]. Hue says which color, saturation how vivid, value how bright. Tune
the ranges below if the simulator's lighting shifts a color into a neighbor.
"""

from __future__ import annotations

import cv2
import numpy as np

# name -> list of (low HSV, high HSV) ranges. Red wraps around hue 0, so it
# needs two ranges.
COLOR_RANGES: dict[str, list[tuple[tuple[int, int, int], tuple[int, int, int]]]] = {
    "red": [((0, 100, 70), (8, 255, 255)), ((170, 100, 70), (179, 255, 255))],
    "orange": [((9, 100, 70), (19, 255, 255))],
    "yellow": [((20, 100, 70), (34, 255, 255))],
    "green": [((35, 70, 50), (85, 255, 255))],
    "blue": [((86, 100, 50), (130, 255, 255))],
    "purple": [((131, 70, 50), (169, 255, 255))],
}

# BGR colors for drawing each label.
DRAW_COLORS: dict[str, tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "orange": (0, 140, 255),
    "yellow": (0, 230, 255),
    "green": (0, 200, 0),
    "blue": (255, 80, 0),
    "purple": (200, 0, 160),
    "white": (255, 255, 255),
    "black": (60, 60, 60),
    "gray": (160, 160, 160),
    "unknown": (160, 160, 160),
}

_KERNEL = np.ones((5, 5), np.uint8)


def to_hsv(bgr: np.ndarray) -> np.ndarray:
    """Lightly blur (to suppress JPEG speckle) and convert BGR to HSV."""
    return cv2.cvtColor(cv2.GaussianBlur(bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)


def color_mask(hsv: np.ndarray, color: str) -> np.ndarray:
    """Binary mask (0/255) of the pixels inside one named color's ranges."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in COLOR_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    return mask


def find_color_blobs(
    bgr: np.ndarray,
    colors: list[str] | None = None,
    min_area_fraction: float = 0.002,
) -> list[dict]:
    """Return the largest blob of each color that covers enough of the frame.

    Each result has: color, area_fraction (of the whole frame), bbox in pixels
    (x_min, y_min, x_max, y_max), center_norm (x, y in [0, 1]) and error_x_norm
    (-1 left edge, 0 center, +1 right edge). Sorted largest first.
    """
    height, width = bgr.shape[:2]
    frame_area = float(height * width)
    hsv = to_hsv(bgr)
    blobs = []
    for color in colors or list(COLOR_RANGES):
        mask = color_mask(hsv, color)
        # Opening removes isolated pixels; closing fills small holes in a blob.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        largest = max(contours, key=cv2.contourArea)
        area_fraction = cv2.contourArea(largest) / frame_area
        if area_fraction < min_area_fraction:
            continue
        x, y, w, h = cv2.boundingRect(largest)
        center_x, center_y = x + w / 2.0, y + h / 2.0
        blobs.append(
            {
                "color": color,
                "area_fraction": round(area_fraction, 4),
                "bbox": [x, y, x + w, y + h],
                "center_norm": [round(center_x / width, 3), round(center_y / height, 3)],
                "error_x_norm": round((center_x - width / 2.0) / (width / 2.0), 3),
            }
        )
    blobs.sort(key=lambda blob: blob["area_fraction"], reverse=True)
    return blobs


def dominant_color(
    bgr: np.ndarray,
    box: tuple[float, float, float, float] | None = None,
    inner_fraction: float = 0.6,
    min_fraction: float = 0.12,
) -> tuple[str, float]:
    """Name the main color inside a box (or the whole image) and its pixel share.

    Only the central `inner_fraction` of the box is used, because a detector's
    box also contains background around the object. A chromatic color wins if
    it covers at least `min_fraction` of those pixels; otherwise the region is
    called white, black or gray from its saturation and brightness.
    """
    if box is not None:
        x_min, y_min, x_max, y_max = box
        margin_x = (x_max - x_min) * (1.0 - inner_fraction) / 2.0
        margin_y = (y_max - y_min) * (1.0 - inner_fraction) / 2.0
        x0, x1 = int(x_min + margin_x), int(round(x_max - margin_x))
        y0, y1 = int(y_min + margin_y), int(round(y_max - margin_y))
        height, width = bgr.shape[:2]
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(max(x1, x0 + 1), width), min(max(y1, y0 + 1), height)
        bgr = bgr[y0:y1, x0:x1]
    if bgr.size == 0:
        return "unknown", 0.0

    hsv = to_hsv(bgr)
    pixels = float(hsv.shape[0] * hsv.shape[1])
    shares = {color: cv2.countNonZero(color_mask(hsv, color)) / pixels for color in COLOR_RANGES}
    best = max(shares, key=shares.get)
    if shares[best] >= min_fraction:
        return best, round(shares[best], 3)

    saturation = float(np.mean(hsv[:, :, 1]))
    value = float(np.mean(hsv[:, :, 2]))
    if value < 50:
        return "black", 1.0
    if saturation < 40 and value > 180:
        return "white", 1.0
    return "gray", 1.0


def draw_label(
    image: np.ndarray,
    text: str,
    top_left: tuple[int, int],
    color: tuple[int, int, int],
    below: tuple[int, int] | None = None,
) -> None:
    """Text on a dark strip, above a box (or at `below`, under it), readable on any background."""
    if below is not None:
        at = (below[0], min(below[1] + 16, image.shape[0] - 4))
    else:
        at = (top_left[0], max(top_left[1] - 6, 14))
    (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(
        image,
        (at[0] - 2, at[1] - text_height - 3),
        (at[0] + text_width + 2, at[1] + baseline),
        (0, 0, 0),
        -1,
    )
    cv2.putText(image, text, at, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def draw_blobs(image: np.ndarray, blobs: list[dict]) -> None:
    """Draw each color blob's box and name onto the image, in place.

    Color names go under the box, so they do not collide with YOLO labels,
    which go above.
    """
    for blob in blobs:
        x_min, y_min, x_max, y_max = blob["bbox"]
        color = DRAW_COLORS.get(blob["color"], (255, 255, 255))
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)
        draw_label(
            image, f"{blob['color']} {blob['area_fraction'] * 100:.1f}%", (x_min, y_min), color, below=(x_min, y_max)
        )
