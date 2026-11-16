"""Per-box color classification in HSV.

Only the inner part of the box is used (the outer `margin` on each side is
ignored) so background pixels at the box edges do not skew the result. Pixels
with low saturation or brightness (asphalt, black wheels, white, shadows) are
discarded. The dominant hue bin among the rest wins if it covers at least
`min_fraction` of them; otherwise the color is "unknown".
"""

from __future__ import annotations

import cv2
import numpy as np

COLORS = ("red", "orange", "yellow", "green", "blue", "purple")

# OpenCV hue runs 0-179 (degrees / 2). Red wraps around 0.
_HUE_BINS = (
    ("red", 0, 8), ("orange", 8, 19), ("yellow", 19, 34), ("green", 34, 85),
    ("blue", 85, 131), ("purple", 131, 166), ("red", 166, 180),
)


def classify_color(image_bgr: np.ndarray, box, margin: float = 0.2, min_saturation: float = 0.35,
                   min_value: float = 0.2, min_fraction: float = 0.4) -> tuple[str, float]:
    """Return (color name or "unknown", share of kept pixels in the winning bin)."""
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = box
    mx, my = (x2 - x1) * margin, (y2 - y1) * margin
    left, right = int(max(0, round(x1 + mx))), int(min(width, round(x2 - mx)))
    top, bottom = int(max(0, round(y1 + my))), int(min(height, round(y2 - my)))
    if right - left < 2 or bottom - top < 2:
        left, right = int(max(0, x1)), int(min(width, x2))
        top, bottom = int(max(0, y1)), int(min(height, y2))
        if right - left < 1 or bottom - top < 1:
            return "unknown", 0.0
    hsv = cv2.cvtColor(image_bgr[top:bottom, left:right], cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0].ravel(), hsv[..., 1].ravel(), hsv[..., 2].ravel()
    keep = (sat >= min_saturation * 255) & (val >= min_value * 255)
    kept = int(keep.sum())
    if kept == 0:
        return "unknown", 0.0
    hue = hue[keep]
    counts: dict[str, int] = {}
    for name, low, high in _HUE_BINS:
        counts[name] = counts.get(name, 0) + int(((hue >= low) & (hue < high)).sum())
    name, count = max(counts.items(), key=lambda item: item[1])
    fraction = count / kept
    return (name if fraction >= min_fraction else "unknown"), fraction
