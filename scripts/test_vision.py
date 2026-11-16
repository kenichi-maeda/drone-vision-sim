#!/usr/bin/env python3
"""Run YOLO + OpenCV color detection on image files, without the simulator.

For each image it prints what YOLO found (object + color) and which colors
OpenCV found, and saves an annotated copy. Needs only Python 3 with
opencv-python and numpy (no ROS), so it also runs on a laptop.

    python3 scripts/test_vision.py photo.jpg more/*.png
    python3 scripts/test_vision.py --out debug/vision frames/       # a folder
    python3 scripts/test_vision.py --no-yolo frame.jpg               # colors only

Sim camera frames to try it on: the camera probe keeps rolling samples in the
workbench container, e.g.
    docker exec dts-virtual-birdie docker cp \
        dts-run-drone-vision-sim:/tmp/duckiedrone_camera_sample_00.jpg /tmp/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "packages" / "drone_vision"))

from drone_vision.color_utils import COLOR_RANGES, DRAW_COLORS, draw_blobs, draw_label, find_color_blobs  # noqa: E402
from drone_vision.detection_utils import detect_all, forward, load_model  # noqa: E402

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def image_paths(inputs: list[str]) -> list[Path]:
    paths: list[Path] = []
    for item in inputs:
        path = Path(item)
        if path.is_dir():
            paths.extend(sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES))
        else:
            paths.append(path)
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("images", nargs="+", help="image files or folders")
    parser.add_argument("--model", default=str(ROOT / "assets" / "yolov8n.onnx"), help="YOLOv8 ONNX file")
    parser.add_argument("--out", default=str(ROOT / "debug" / "vision"), help="folder for annotated copies")
    parser.add_argument("--confidence", type=float, default=0.35, help="YOLO confidence threshold")
    parser.add_argument("--min-area", type=float, default=0.002, help="smallest color blob, fraction of frame")
    parser.add_argument("--colors", default=",".join(COLOR_RANGES), help="comma-separated colors to look for")
    parser.add_argument("--no-yolo", action="store_true", help="skip YOLO, colors only")
    args = parser.parse_args()

    net = None
    if not args.no_yolo:
        if not Path(args.model).is_file():
            print(f"YOLO model not found at {args.model} (use --model, or --no-yolo for colors only)")
            return 1
        net = load_model(args.model)
    colors = [c.strip() for c in args.colors.split(",") if c.strip() in COLOR_RANGES]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    paths = image_paths(args.images)
    if not paths:
        print("No images found.")
        return 1
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path}: could not read")
            continue
        annotated = image.copy()

        objects = []
        if net is not None:
            predictions, scale, left, top = forward(net, image)
            objects = detect_all(image, predictions, scale, left, top, confidence_threshold=args.confidence)
        blobs = find_color_blobs(image, colors, args.min_area)

        draw_blobs(annotated, blobs)
        for item in objects:
            x_min, y_min, x_max, y_max = item["bbox"]
            draw_color = DRAW_COLORS.get(item["color"], (255, 255, 255))
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            draw_label(
                annotated,
                f"{item['class_name']} {item['color']} {item['confidence']:.2f}",
                (x_min, y_min),
                draw_color,
            )
        out_path = out_dir / f"{path.stem}_vision.jpg"
        cv2.imwrite(str(out_path), annotated)

        print(f"{path}:")
        if net is not None:
            if objects:
                for item in objects:
                    print(
                        f"  object  {item['class_name']:<14} color={item['color']:<7} "
                        f"confidence={item['confidence']:.2f}  center={item['center_norm']}"
                    )
            else:
                print("  object  none above confidence", args.confidence)
        if blobs:
            for blob in blobs:
                print(
                    f"  color   {blob['color']:<7} area={blob['area_fraction'] * 100:5.1f}%  "
                    f"center={blob['center_norm']}"
                )
        else:
            print("  color   none")
        print(f"  saved   {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
