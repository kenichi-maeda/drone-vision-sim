#!/usr/bin/env python3
"""Offline checks for color labels, fisheye removal, and drive packets."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages" / "drone_vision"))

from drone_vision.drive_link import encode_input_frame, encode_keyboard
from drone_vision.vision_lib import color_inside, rectify, rectify_maps, resolve_specified


def paint(color_bgr, width=80, height=60) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = color_bgr
    return image


def main() -> None:
    duckie = resolve_specified("duckie")
    stop = resolve_specified("Stop Sign")
    assert duckie is not None and duckie["class_id"] == 32 and duckie["color"] == "yellow"
    assert stop is not None and stop["class_id"] == 11 and stop["color"] == "red"
    assert resolve_specified("sports ball")["class_id"] == 32
    assert resolve_specified("11")["label"] == "stop sign"
    assert resolve_specified("truck") is None
    assert resolve_specified("  ") is None

    red = color_inside(paint((40, 40, 220)), 0, 0, 80, 60)
    yellow = color_inside(paint((0, 220, 220)), 0, 0, 80, 60)
    blue = color_inside(paint((220, 80, 40)), 0, 0, 80, 60)
    assert red is not None and red[0] == "red", red
    assert yellow is not None and yellow[0] == "yellow", yellow
    assert blue is not None and blue[0] == "blue", blue

    mixed = paint((40, 180, 40), 100, 80)
    mixed[16:64, 20:80] = (0, 220, 220)
    inner = color_inside(mixed, 0, 0, 100, 80)
    assert inner is not None and inner[0] == "yellow", inner

    image = np.zeros((640, 480, 3), dtype=np.uint8)
    image[:, :] = (30, 80, 30)
    cv2_circle = __import__("cv2")
    cv2_circle.circle(image, (240, 320), 40, (0, 220, 220), -1)
    straight = rectify(image)
    assert straight.shape == image.shape
    assert not np.array_equal(straight, image)
    map1, _map2 = rectify_maps(480, 640)
    corner = map1[10, 10]
    center = map1[320, 240]
    assert abs(int(corner[0]) - 10) + abs(int(corner[1]) - 10) > 2, corner
    assert abs(int(center[0]) - 240) + abs(int(center[1]) - 320) < 8, center

    packet = encode_keyboard({"forward": True, "ascend": True})
    assert packet[0] == 0xF5 and packet[1] == 0xF4 and packet[4] == 0xF5
    frame = encode_input_frame({"left": True}, session_id=0, timestamp=1.5)
    assert frame[0] == 5
    key_length = int.from_bytes(frame[1:3], "big")
    assert frame[3:3 + key_length] == b"map_0/vehicle_0/keyboard"
    print("vision_lib checks passed")
    print(f"  red={red} yellow={yellow} blue={blue} inner={inner}")
    print(f"  corner map {tuple(int(v) for v in corner)} center map {tuple(int(v) for v in center)}")


if __name__ == "__main__":
    main()
