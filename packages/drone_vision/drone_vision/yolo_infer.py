"""YOLOv8n inference shared by the simulator node and the real camera.

This module does not import ROS. The box math matches yolo_detector.py.
"""

from __future__ import annotations

import cv2
import numpy as np

from drone_vision.vision_lib import COLOR_BGR, color_inside, specified_for


MODEL_INPUT_SIZE = 640
COCO_CLASS_NAMES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic_light", "fire_hydrant", "stop_sign", "parking_meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports_ball", "kite", "baseball_bat", "baseball_glove", "skateboard",
    "surfboard", "tennis_racket", "bottle", "wine_glass", "cup", "fork", "knife",
    "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot_dog", "pizza", "donut", "cake", "chair", "couch", "potted_plant", "bed",
    "dining_table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell_phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy_bear", "hair_drier", "toothbrush",
)


def load_net(model_path: str):
    net = cv2.dnn.readNetFromONNX(model_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net


def _letterbox(image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    height, width = image.shape[:2]
    scale = min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height)
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height))
    left = (MODEL_INPUT_SIZE - resized_width) // 2
    top = (MODEL_INPUT_SIZE - resized_height) // 2
    padded = np.full((MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, 3), 114, dtype=np.uint8)
    padded[top : top + resized_height, left : left + resized_width] = resized
    return padded, scale, left, top


def infer(net, image: np.ndarray, confidence: float, nms: float, watched: list[int] | None):
    """Return (score, [x0, y0, x1, y1], class_id) in the original image."""
    height, width = image.shape[:2]
    padded, scale, left, top = _letterbox(image)
    blob = cv2.dnn.blobFromImage(
        padded, scalefactor=1.0 / 255.0, size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE), swapRB=True, crop=False,
    )
    net.setInput(blob)
    predictions = net.forward()[0].T
    class_scores = predictions[:, 4:]
    if watched is None:
        class_ids = np.argmax(class_scores, axis=1)
        target_scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    else:
        masked = np.full_like(class_scores, -1.0)
        masked[:, watched] = class_scores[:, watched]
        class_ids = np.argmax(masked, axis=1)
        target_scores = masked[np.arange(masked.shape[0]), class_ids]
    candidate_indices = np.flatnonzero(target_scores >= confidence)
    if candidate_indices.size == 0:
        return []

    grouped: dict[int, list[tuple[list[int], float]]] = {}
    for index in candidate_indices:
        class_id = int(class_ids[int(index)])
        center_x, center_y, box_width, box_height = predictions[int(index), :4]
        x_min = int(round((center_x - box_width / 2 - left) / scale))
        y_min = int(round((center_y - box_height / 2 - top) / scale))
        box_width_px = max(1, int(round(box_width / scale)))
        box_height_px = max(1, int(round(box_height / scale)))
        grouped.setdefault(class_id, []).append(
            ([x_min, y_min, box_width_px, box_height_px], float(target_scores[int(index)]))
        )

    detections = []
    for class_id, items in grouped.items():
        boxes = [item[0] for item in items]
        scores = [item[1] for item in items]
        selected = cv2.dnn.NMSBoxes(boxes, scores, confidence, nms)
        if len(selected) == 0:
            continue
        for raw_index in np.asarray(selected).reshape(-1):
            box, score = items[int(raw_index)]
            x_min, y_min, box_width_px, box_height_px = box
            x_min = min(max(x_min, 0), width - 1)
            y_min = min(max(y_min, 0), height - 1)
            x_max = min(max(x_min + box_width_px, x_min + 1), width)
            y_max = min(max(y_min + box_height_px, y_min + 1), height)
            detections.append((score, [float(x_min), float(y_min), float(x_max), float(y_max)], class_id))
    detections.sort(key=lambda item: item[0], reverse=True)
    if watched is None:
        return detections[:1]
    return detections


def row_for(image: np.ndarray, detection) -> dict:
    confidence, (x_min, y_min, x_max, y_max), class_id = detection
    height, width = image.shape[:2]
    color = color_inside(image, int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min))
    expected = specified_for(class_id)
    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0
    name = COCO_CLASS_NAMES[class_id] if 0 <= class_id < len(COCO_CLASS_NAMES) else f"coco_class_{class_id}"
    return {
        "class_id": class_id,
        "name": name,
        "label": expected["label"] if expected else name,
        "confidence": round(float(confidence), 3),
        "color": color[0] if color else "",
        "color_confidence": color[1] if color else 0.0,
        "expected_color": expected["color"] if expected else "",
        "color_ok": bool(expected and color and color[0] == expected["color"]),
        "center_x": center_x / width,
        "center_y": center_y / height,
        "error_x": (center_x - (width - 1) / 2.0) / max((width - 1) / 2.0, 1.0),
        "error_y": (center_y - (height - 1) / 2.0) / max((height - 1) / 2.0, 1.0),
        "width": (x_max - x_min) / width,
        "height": (y_max - y_min) / height,
        "x_min": x_min,
        "y_min": y_min,
        "x_max": x_max,
        "y_max": y_max,
    }


def draw_rows(image: np.ndarray, rows: list[dict]) -> np.ndarray:
    canvas = image.copy()
    height, width = canvas.shape[:2]
    cx, cy = width // 2, height // 2
    cv2.line(canvas, (cx - 10, cy), (cx + 10, cy), (200, 200, 200), 1)
    cv2.line(canvas, (cx, cy - 10), (cx, cy + 10), (200, 200, 200), 1)
    for row in rows:
        color = COLOR_BGR.get(row["color"] or "", (0, 255, 0))
        top_left = (int(row["x_min"]), int(row["y_min"]))
        bottom_right = (int(row["x_max"]), int(row["y_max"]))
        label = f"{row['name']} {row['confidence']:.2f}"
        if row["color"]:
            label += f" {row['color']} {row['color_confidence']:.2f}"
        cv2.rectangle(canvas, top_left, bottom_right, color, 2)
        label_at = (top_left[0], max(top_left[1] - 6, 14))
        cv2.putText(canvas, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(canvas, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    return canvas
