"""YOLOv8 ONNX helpers shared by the YOLO node and the offline test script.

Pure NumPy/OpenCV: no ROS imports. `detect_all` reports every object YOLO
finds (any COCO class), each labeled with its main color from color_utils.
"""

from __future__ import annotations

import cv2
import numpy as np

from .color_utils import dominant_color

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


def class_name(class_id: int) -> str:
    if 0 <= class_id < len(COCO_CLASS_NAMES):
        return COCO_CLASS_NAMES[class_id]
    return f"coco_class_{class_id}"


def load_model(model_path: str) -> cv2.dnn.Net:
    net = cv2.dnn.readNetFromONNX(model_path)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    return net


def letterbox(image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    """Resize without changing aspect ratio and pad to 640x640."""
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


def forward(net: cv2.dnn.Net, image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
    """Run the network once. Returns (predictions [N, 84], scale, left, top)."""
    padded, scale, left, top = letterbox(image)
    blob = cv2.dnn.blobFromImage(
        padded,
        scalefactor=1.0 / 255.0,
        size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
        swapRB=True,
        crop=False,
    )
    net.setInput(blob)
    return net.forward()[0].T, scale, left, top


def detect_all(
    image: np.ndarray,
    predictions: np.ndarray,
    scale: float,
    left: int,
    top: int,
    confidence_threshold: float = 0.35,
    nms_threshold: float = 0.45,
    max_box_area_fraction: float = 1.0,
    max_detections: int = 10,
) -> list[dict]:
    """Every object above the threshold, with its color. Highest confidence first.

    Each result has: class_id, class_name, confidence, color, color_fraction,
    bbox in pixels (x_min, y_min, x_max, y_max) and center_norm (x, y in [0, 1]).
    """
    height, width = image.shape[:2]
    class_scores = predictions[:, 4:]
    class_ids = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    keep = np.flatnonzero(scores >= confidence_threshold)
    if keep.size == 0:
        return []

    boxes: list[list[int]] = []
    nms_boxes: list[list[int]] = []
    kept_scores: list[float] = []
    for index in keep:
        center_x, center_y, box_width, box_height = predictions[index, :4]
        x_min = int(round((center_x - box_width / 2 - left) / scale))
        y_min = int(round((center_y - box_height / 2 - top) / scale))
        w = int(round(box_width / scale))
        h = int(round(box_height / scale))
        boxes.append([x_min, y_min, w, h])
        # Shift each class far apart so NMS only suppresses boxes of the same
        # class (older OpenCV has no batched NMS).
        offset = int(class_ids[index]) * 10000
        nms_boxes.append([x_min + offset, y_min + offset, w, h])
        kept_scores.append(float(scores[index]))

    selected = cv2.dnn.NMSBoxes(nms_boxes, kept_scores, confidence_threshold, nms_threshold)
    results = []
    for chosen in np.asarray(selected).reshape(-1)[:max_detections]:
        chosen = int(chosen)
        x_min, y_min, w, h = boxes[chosen]
        x_min = min(max(x_min, 0), width - 1)
        y_min = min(max(y_min, 0), height - 1)
        x_max = min(max(x_min + w, x_min + 1), width)
        y_max = min(max(y_min + h, y_min + 1), height)
        if (x_max - x_min) * (y_max - y_min) / float(width * height) > max_box_area_fraction:
            continue
        class_id = int(class_ids[keep[chosen]])
        color, color_fraction = dominant_color(image, (x_min, y_min, x_max, y_max))
        results.append(
            {
                "class_id": class_id,
                "class_name": class_name(class_id),
                "confidence": round(kept_scores[chosen], 3),
                "color": color,
                "color_fraction": color_fraction,
                "bbox": [x_min, y_min, x_max, y_max],
                "center_norm": [
                    round((x_min + x_max) / 2.0 / width, 3),
                    round((y_min + y_max) / 2.0 / height, 3),
                ],
            }
        )
    results.sort(key=lambda item: item["confidence"], reverse=True)
    return results
