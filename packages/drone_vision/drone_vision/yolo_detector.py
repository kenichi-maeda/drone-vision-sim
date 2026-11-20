"""Read-only YOLOv8n COCO detector for the Duckiedrone front camera."""

from __future__ import annotations

import json
import os
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Header, String

from drone_vision.vision_lib import COLOR_BGR, color_inside, resolve_specified, specified_for


ROBOT_NAME = os.environ.get("VEHICLE_NAME", "birdie")
CAMERA_TOPIC = f"/{ROBOT_NAME}/camera_node/image/compressed"
ANNOTATED_TOPIC = "/drone_vision/yolo_image/compressed"
OBJECTS_TOPIC = "/drone_vision/yolo_objects"
FLIGHT_REQUEST_TOPIC = "/drone_vision/flight_request"
MODEL_INPUT_SIZE = 640
COCO_STOP_SIGN_CLASS = 11
FLIGHT_MAX_AREA = 0.18
VIEW_MAX_AREA = 0.45
COCO_CLASS_NAMES = dict(
    enumerate(
        (
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
    )
)


class YoloDetector(Node):
    """Detect the allowed COCO classes and report class, box, and color."""

    def __init__(self) -> None:
        super().__init__("yolo_detector")
        # Leave cores for the Matrix physics loop. OpenCV otherwise takes every core.
        cv2.setNumThreads(2)
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.10)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("inference_every_n_frames", 2)
        self.declare_parameter("target_class_id", COCO_STOP_SIGN_CLASS)
        # Classes drawn on the camera. [0] is only the declared array type;
        # the launcher replaces it. The flight message still prefers target_class_id.
        self.declare_parameter("allowed_class_ids", [0])
        model_path = str(self.get_parameter("model_path").value)
        if not model_path:
            raise ValueError("model_path must point to the YOLOv8n ONNX file")
        self._confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self._nms_threshold = float(self.get_parameter("nms_threshold").value)
        self._inference_every_n_frames = max(
            1, int(self.get_parameter("inference_every_n_frames").value)
        )
        self._target_class_id = int(self.get_parameter("target_class_id").value)
        if self._target_class_id != -1 and not 0 <= self._target_class_id < 80:
            raise ValueError("target_class_id must be -1 (any class) or a COCO class ID in [0, 79]")
        allowed = [int(c) for c in self.get_parameter("allowed_class_ids").value]
        self._allowed_class_ids = None if allowed == [0] else allowed
        if self._allowed_class_ids is not None and not all(
            0 <= c < 80 for c in self._allowed_class_ids
        ):
            raise ValueError("allowed_class_ids must be COCO class IDs in [0, 79]")
        self._watched_class_ids = self._watched_classes()
        self._target_class_name = COCO_CLASS_NAMES.get(
            self._target_class_id, f"coco_class_{self._target_class_id}"
        )
        self._target_description = (
            "any COCO class" if self._target_class_id == -1 else self._target_class_name
        )
        self._frame_count = 0
        self._last_log_at = 0.0
        self._last_miss_log_at = 0.0
        self._net = cv2.dnn.readNetFromONNX(model_path)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._image_sub = self.create_subscription(
            CompressedImage, CAMERA_TOPIC, self._on_image, qos
        )
        self._detection_pub = self.create_publisher(
            Float32MultiArray, "/drone_vision/yolo_detection", 10
        )
        self._objects_pub = self.create_publisher(String, OBJECTS_TOPIC, 10)
        self._frame_pub = self.create_publisher(Header, "/drone_vision/yolo_frame", 10)
        # Camera frame with every reported detection drawn on it, for live viewing.
        # Only encoded while something is subscribed, so it costs nothing otherwise.
        self._annotated_pub = self.create_publisher(
            CompressedImage, ANNOTATED_TOPIC, QoSProfile(depth=1)
        )
        self.create_subscription(String, FLIGHT_REQUEST_TOPIC, self._on_flight_request, 10)
        watched = (
            "any"
            if self._watched_class_ids is None
            else ",".join(COCO_CLASS_NAMES[c] for c in self._watched_class_ids)
        )
        self.get_logger().info(
            f"YOLOv8n COCO detector listening on {CAMERA_TOPIC}; loaded from {model_path}; "
            f"watched={watched}; flight target_class_id={self._target_class_id} "
            f"({self._target_description}); confidence_threshold="
            f"{self._confidence_threshold:.2f}; running every "
            f"{self._inference_every_n_frames} camera frames. Publishing "
            "[class_id, confidence, center_x_norm, center_y_norm, "
            "error_x_norm, error_y_norm, width_norm, height_norm] on "
            "/drone_vision/yolo_detection. Perception only; no flight commands."
        )

    def _watched_classes(self) -> list[int] | None:
        if self._allowed_class_ids is None:
            if self._target_class_id == -1:
                return None
            return [self._target_class_id]
        watched = list(self._allowed_class_ids)
        if self._target_class_id != -1 and self._target_class_id not in watched:
            watched.append(self._target_class_id)
        return watched

    @staticmethod
    def _letterbox(image: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        """Resize without changing aspect ratio and pad to 640x640."""
        height, width = image.shape[:2]
        scale = min(MODEL_INPUT_SIZE / width, MODEL_INPUT_SIZE / height)
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(image, (resized_width, resized_height))
        left = (MODEL_INPUT_SIZE - resized_width) // 2
        top = (MODEL_INPUT_SIZE - resized_height) // 2
        padded = np.full(
            (MODEL_INPUT_SIZE, MODEL_INPUT_SIZE, 3), 114, dtype=np.uint8
        )
        padded[top : top + resized_height, left : left + resized_width] = resized
        return padded, scale, left, top

    def _infer(self, image: np.ndarray) -> list[tuple[float, list[float], int]]:
        """Return confidence and original-image box for every kept hit."""
        height, width = image.shape[:2]
        padded, scale, left, top = self._letterbox(image)
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
            swapRB=True,
            crop=False,
        )
        self._net.setInput(blob)
        output = self._net.forward()
        predictions = output[0].T
        class_scores = predictions[:, 4:]
        if self._watched_class_ids is None:
            class_ids = np.argmax(class_scores, axis=1)
            target_scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        else:
            masked = np.full_like(class_scores, -1.0)
            masked[:, self._watched_class_ids] = class_scores[:, self._watched_class_ids]
            class_ids = np.argmax(masked, axis=1)
            target_scores = masked[np.arange(masked.shape[0]), class_ids]
        candidate_indices = np.flatnonzero(target_scores >= self._confidence_threshold)
        if candidate_indices.size == 0:
            return []

        grouped: dict[int, list[tuple[int, list[int], float]]] = {}
        for index in candidate_indices:
            class_id = int(class_ids[int(index)])
            center_x, center_y, box_width, box_height = predictions[int(index), :4]
            x_min = int(round((center_x - box_width / 2 - left) / scale))
            y_min = int(round((center_y - box_height / 2 - top) / scale))
            box_width_px = max(1, int(round(box_width / scale)))
            box_height_px = max(1, int(round(box_height / scale)))
            grouped.setdefault(class_id, []).append(
                (int(index), [x_min, y_min, box_width_px, box_height_px], float(target_scores[int(index)]))
            )

        detections: list[tuple[float, list[float], int]] = []
        for class_id, items in grouped.items():
            boxes = [item[1] for item in items]
            scores = [item[2] for item in items]
            selected = cv2.dnn.NMSBoxes(
                boxes, scores, self._confidence_threshold, self._nms_threshold
            )
            if len(selected) == 0:
                continue
            for raw_index in np.asarray(selected).reshape(-1):
                _prediction_index, box, score = items[int(raw_index)]
                x_min, y_min, box_width_px, box_height_px = box
                x_min = min(max(x_min, 0), width - 1)
                y_min = min(max(y_min, 0), height - 1)
                x_max = min(max(x_min + box_width_px, x_min + 1), width)
                y_max = min(max(y_min + box_height_px, y_min + 1), height)
                detections.append(
                    (score, [float(x_min), float(y_min), float(x_max), float(y_max)], class_id)
                )
        detections.sort(key=lambda item: item[0], reverse=True)
        if self._watched_class_ids is None:
            return detections[:1]
        return detections

    def _on_flight_request(self, message: String) -> None:
        spec = resolve_specified(message.data)
        if spec is None:
            self.get_logger().error(f"Ignoring unknown flight target {message.data!r}")
            return
        self.set_parameters([
            Parameter("target_class_id", Parameter.Type.INTEGER, int(spec["class_id"]))
        ])
        self._target_class_id = int(spec["class_id"])
        self._target_description = spec["label"]
        self.get_logger().info(
            f"Flight detections now follow {spec['label']} "
            f"({spec['name']}, {spec['color']})"
        )

    def _publish_annotated(
        self,
        header: Header,
        image: np.ndarray,
        rows: list[dict],
    ) -> None:
        """Publish the fisheye frame with every reported box drawn on it."""
        if self._annotated_pub.get_subscription_count() == 0:
            return
        height, width = image.shape[:2]
        cx, cy = width // 2, height // 2
        cv2.line(image, (cx - 10, cy), (cx + 10, cy), (200, 200, 200), 1)
        cv2.line(image, (cx, cy - 10), (cx, cy + 10), (200, 200, 200), 1)
        for row in rows:
            color = COLOR_BGR.get(row["color"] or "", (0, 255, 0))
            top_left = (int(row["x_min"]), int(row["y_min"]))
            bottom_right = (int(row["x_max"]), int(row["y_max"]))
            label = f"{row['name']} {row['confidence']:.2f}"
            if row["color"]:
                label += f" {row['color']} {row['color_confidence']:.2f}"
            cv2.rectangle(image, top_left, bottom_right, color, 2)
            label_at = (top_left[0], max(top_left[1] - 6, 14))
            cv2.putText(image, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(image, label, label_at, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        annotated = CompressedImage()
        annotated.header = header
        annotated.format = "jpeg"
        annotated.data = encoded.tobytes()
        self._annotated_pub.publish(annotated)

    def _row(self, image: np.ndarray, detection: tuple[float, list[float], int]) -> dict:
        confidence, (x_min, y_min, x_max, y_max), class_id = detection
        height, width = image.shape[:2]
        color = color_inside(
            image, int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min)
        )
        color_name = color[0] if color else ""
        color_confidence = color[1] if color else 0.0
        expected = specified_for(class_id)
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        return {
            "class_id": class_id,
            "name": COCO_CLASS_NAMES.get(class_id, f"coco_class_{class_id}"),
            "label": expected["label"] if expected else COCO_CLASS_NAMES.get(class_id, ""),
            "confidence": round(confidence, 3),
            "color": color_name,
            "color_confidence": color_confidence,
            "expected_color": expected["color"] if expected else "",
            "color_ok": bool(expected and color_name == expected["color"]),
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

    def _on_image(self, message: CompressedImage) -> None:
        self._frame_count += 1
        if self._frame_count % self._inference_every_n_frames:
            return
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("Could not decode camera frame for YOLO")
            return
        started_at = monotonic()
        detections = self._infer(image)
        self._frame_pub.publish(message.header)
        area_limit = float(image.shape[0] * image.shape[1])
        visible = []
        for detection in detections:
            box = detection[1]
            area = (box[2] - box[0]) * (box[3] - box[1]) / area_limit
            if area <= VIEW_MAX_AREA:
                visible.append(detection)
        rows = [self._row(image, detection) for detection in visible]
        inference_ms = (monotonic() - started_at) * 1000.0
        objects_msg = String()
        objects_msg.data = json.dumps({"objects": [
            {key: value for key, value in row.items() if not key.startswith("x_") and key not in ("y_min", "y_max")}
            for row in rows
        ]})
        self._objects_pub.publish(objects_msg)
        self._publish_annotated(message.header, image, rows)

        flight_rows = [
            row for row in rows
            if row["width"] * row["height"] <= FLIGHT_MAX_AREA
            and row["color_ok"]
            and (self._target_class_id == -1 or row["class_id"] == self._target_class_id)
        ]
        flight_rows.sort(key=lambda row: row["confidence"], reverse=True)
        if not flight_rows:
            now = monotonic()
            if now - self._last_miss_log_at >= 3.0:
                seen = ", ".join(
                    f"{row['name']}/{row['color'] or '?'} {row['confidence']:.2f}" for row in rows
                ) or "none"
                self.get_logger().info(
                    f"No flight target {self._target_description} above confidence "
                    f"{self._confidence_threshold:.2f}; visible={seen}; inference_ms={inference_ms:.1f}"
                )
                self._last_miss_log_at = now
            return

        row = flight_rows[0]
        detection_msg = Float32MultiArray()
        detection_msg.data = [
            float(row["class_id"]),
            float(row["confidence"]),
            float(row["center_x"]),
            float(row["center_y"]),
            float(row["error_x"]),
            float(row["error_y"]),
            float(row["width"]),
            float(row["height"]),
        ]
        self._detection_pub.publish(detection_msg)

        now = monotonic()
        if now - self._last_log_at >= 0.5:
            seen = ", ".join(
                f"{item['name']} {item['confidence']:.2f} {item['color'] or '?'} "
                f"{'ok' if item['color_ok'] else 'color-mismatch'}"
                for item in rows
            )
            self.get_logger().info(
                f"visible={seen}; inference_ms={inference_ms:.1f}"
            )
            self._last_log_at = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
