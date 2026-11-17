"""Read-only YOLOv8n COCO detector for the Duckiedrone front camera.

Two outputs from one network pass:
  /drone_vision/yolo_detection  the single flight target (unchanged format,
                                read by safe_hover)
  /drone_vision/detections      every object YOLO sees, each with its OpenCV
                                color, as JSON (for identification tests)
"""

from __future__ import annotations

import json
import os
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Header, String

from .color_utils import DRAW_COLORS, dominant_color, draw_label
from .detection_utils import detect_all, forward


# The camera topic is namespaced by the robot's name (VEHICLE_NAME in Duckietown containers).
CAMERA_TOPIC = f"/{os.environ.get('VEHICLE_NAME') or 'birdie'}/camera_node/image/compressed"
ANNOTATED_TOPIC = "/drone_vision/yolo_image/compressed"
ALL_DETECTIONS_TOPIC = "/drone_vision/detections"
MAX_BOX_AREA_FRACTION = 0.18


def _iou(a: list[float], b: list[float]) -> float:
    """Overlap of two (x_min, y_min, x_max, y_max) boxes, 0 to 1."""
    width = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = width * height
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0
COCO_STOP_SIGN_CLASS = 11
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
    """Detect one configurable COCO class and report its 2D image location."""

    def __init__(self) -> None:
        super().__init__("yolo_detector")
        self.declare_parameter("model_path", "")
        self.declare_parameter("confidence_threshold", 0.10)
        self.declare_parameter("nms_threshold", 0.45)
        self.declare_parameter("inference_every_n_frames", 2)
        self.declare_parameter("target_class_id", COCO_STOP_SIGN_CLASS)
        # Optional allowlist, used only with target_class_id=-1: report the
        # best detection among these COCO classes and ignore all others.
        self.declare_parameter("allowed_class_ids", [0])
        # Also publish every detected object (any class) with its color on
        # /drone_vision/detections. Does not change the flight target.
        self.declare_parameter("report_all_detections", True)
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
        # [0] is the declared default only to fix the parameter's array type;
        # treat it as "no allowlist" unless the caller passes something else.
        allowed = [int(c) for c in self.get_parameter("allowed_class_ids").value]
        self._allowed_class_ids = None if allowed == [0] else allowed
        if self._allowed_class_ids is not None and not all(
            0 <= c < 80 for c in self._allowed_class_ids
        ):
            raise ValueError("allowed_class_ids must be COCO class IDs in [0, 79]")
        self._target_class_name = COCO_CLASS_NAMES.get(
            self._target_class_id, f"coco_class_{self._target_class_id}"
        )
        self._target_description = (
            "any COCO class" if self._target_class_id == -1 else self._target_class_name
        )
        self._report_all = bool(self.get_parameter("report_all_detections").value)
        self._frame_count = 0
        self._last_log_at = 0.0
        self._last_all_log_at = 0.0
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
        self._frame_pub = self.create_publisher(Header, "/drone_vision/yolo_frame", 10)
        self._all_pub = self.create_publisher(String, ALL_DETECTIONS_TOPIC, 10)
        # Camera frame with the reported detection drawn on it, for live viewing.
        # Only encoded while something is subscribed, so it costs nothing otherwise.
        self._annotated_pub = self.create_publisher(
            CompressedImage, ANNOTATED_TOPIC, QoSProfile(depth=1)
        )
        self.get_logger().info(
            f"YOLOv8n COCO detector loaded from {model_path}; target_class_id="
            f"{self._target_class_id} ({self._target_description}); confidence_threshold="
            f"{self._confidence_threshold:.2f}; running every "
            f"{self._inference_every_n_frames} camera frames. Publishing "
            "[class_id, confidence, center_x_norm, center_y_norm, "
            "error_x_norm, error_y_norm, width_norm, height_norm] on "
            "/drone_vision/yolo_detection. Perception only; no flight commands."
        )
        if self._target_class_id == COCO_STOP_SIGN_CLASS:
            self.get_logger().info("Stop-sign detection is YOLO-only; no red-shape fallback")

    def _infer(
        self, image: np.ndarray, predictions: np.ndarray, scale: float, left: int, top: int
    ) -> tuple[float, list[float], int] | None:
        """Return confidence and original-image box for the best target hit.

        `predictions`, `scale`, `left`, `top` come from detection_utils.forward,
        so the network runs once per frame for both outputs.
        """
        height, width = image.shape[:2]
        class_scores = predictions[:, 4:]
        if self._target_class_id == -1:
            if self._allowed_class_ids is not None:
                masked = np.full_like(class_scores, -1.0)
                masked[:, self._allowed_class_ids] = class_scores[:, self._allowed_class_ids]
                class_scores = masked
            class_ids = np.argmax(class_scores, axis=1)
            target_scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        else:
            class_ids = np.full(class_scores.shape[0], self._target_class_id, dtype=np.int64)
            target_scores = class_scores[:, self._target_class_id]
        candidate_indices = np.flatnonzero(
            target_scores >= self._confidence_threshold
        )
        if candidate_indices.size == 0:
            return None

        boxes: list[list[int]] = []
        scores: list[float] = []
        for index in candidate_indices:
            center_x, center_y, box_width, box_height = predictions[index, :4]
            x_min = int(round((center_x - box_width / 2 - left) / scale))
            y_min = int(round((center_y - box_height / 2 - top) / scale))
            box_width_px = int(round(box_width / scale))
            box_height_px = int(round(box_height / scale))
            boxes.append([x_min, y_min, box_width_px, box_height_px])
            scores.append(float(target_scores[index]))

        selected = cv2.dnn.NMSBoxes(
            boxes, scores, self._confidence_threshold, self._nms_threshold
        )
        if len(selected) == 0:
            return None
        best_index = int(np.asarray(selected).reshape(-1)[0])
        x_min, y_min, box_width_px, box_height_px = boxes[best_index]
        x_min = min(max(x_min, 0), width - 1)
        y_min = min(max(y_min, 0), height - 1)
        x_max = min(max(x_min + box_width_px, x_min + 1), width)
        y_max = min(max(y_min + box_height_px, y_min + 1), height)
        prediction_index = int(candidate_indices[int(np.asarray(selected).reshape(-1)[0])])
        detected_class = int(class_ids[prediction_index])
        return (scores[best_index], [float(x_min), float(y_min), float(x_max), float(y_max)], detected_class)

    def _publish_annotated(
        self,
        header: Header,
        image: np.ndarray,
        detection: tuple[float, list[float], int] | None,
        target_color: str = "",
        all_detections: list[dict] | None = None,
    ) -> None:
        """Publish the frame with the boxes drawn on it (viewer only).

        Every detected object gets a thin box in its own color; the flight
        target gets a thick green box on top.
        """
        if self._annotated_pub.get_subscription_count() == 0:
            return
        height, width = image.shape[:2]
        # Image center: the controller steers to put the target here.
        cx, cy = width // 2, height // 2
        cv2.line(image, (cx - 10, cy), (cx + 10, cy), (200, 200, 200), 1)
        cv2.line(image, (cx, cy - 10), (cx, cy + 10), (200, 200, 200), 1)
        for item in all_detections or []:
            if detection is not None and item["class_id"] == detection[2] and _iou(item["bbox"], detection[1]) > 0.7:
                continue  # the flight target, drawn below with its own label
            x_min, y_min, x_max, y_max = item["bbox"]
            box_color = DRAW_COLORS.get(item["color"], (255, 255, 255))
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), box_color, 1)
            draw_label(
                image,
                f"{item['class_name']} {item['color']} {item['confidence']:.2f}",
                (x_min, y_min),
                box_color,
            )
        if detection is not None:
            confidence, (x_min, y_min, x_max, y_max), class_id = detection
            label = (
                f"{COCO_CLASS_NAMES.get(class_id, f'coco_class_{class_id}')} {target_color} {confidence:.2f}"
            )
            top_left = (int(x_min), int(y_min))
            cv2.rectangle(image, top_left, (int(x_max), int(y_max)), (0, 255, 0), 2)
            draw_label(image, label, top_left, (0, 255, 0))
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        annotated = CompressedImage()
        annotated.header = header
        annotated.format = "jpeg"
        annotated.data = encoded.tobytes()
        self._annotated_pub.publish(annotated)

    def _on_image(self, message: CompressedImage) -> None:
        self._frame_count += 1
        if self._frame_count % self._inference_every_n_frames:
            return
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("Could not decode camera frame for YOLO")
            return
        started_at = monotonic()
        predictions, scale, left, top = forward(self._net, image)
        detection = self._infer(image, predictions, scale, left, top)
        # A heartbeat means YOLO actually processed this camera frame, even
        # when the target was absent. Preserve its capture timestamp.
        self._frame_pub.publish(message.header)
        detector_method = "yolov8n_coco"
        # Reject oversized model boxes; the old Matrix scene made the COCO
        # model label a building as a stop sign. The controller uses this bound.
        oversized_detection = detection is not None and (
            (detection[1][2] - detection[1][0])
            * (detection[1][3] - detection[1][1])
            / float(image.shape[0] * image.shape[1])
            > MAX_BOX_AREA_FRACTION
        )
        if oversized_detection:
            detection = None
        inference_ms = (monotonic() - started_at) * 1000.0
        # Colors are read before anything is drawn on the image.
        target_color = dominant_color(image, tuple(detection[1]))[0] if detection is not None else ""
        all_detections = None
        if self._report_all:
            all_detections = detect_all(
                image, predictions, scale, left, top,
                confidence_threshold=self._confidence_threshold,
                nms_threshold=self._nms_threshold,
                max_box_area_fraction=MAX_BOX_AREA_FRACTION,
            )
            self._all_pub.publish(String(data=json.dumps(all_detections)))
            now = monotonic()
            if now - self._last_all_log_at >= 1.0:
                seen = ", ".join(
                    f"{d['class_name']} ({d['color']}) {d['confidence']:.2f}" for d in all_detections
                ) or "nothing"
                self.get_logger().info(f"YOLO sees: {seen}")
                self._last_all_log_at = now
        self._publish_annotated(message.header, image, detection, target_color, all_detections)
        if detection is None:
            now = monotonic()
            if now - self._last_miss_log_at >= 3.0:
                self.get_logger().info(
                    f"No {self._target_description} detection above confidence "
                    f"{self._confidence_threshold:.2f}; inference_ms={inference_ms:.1f}"
                )
                self._last_miss_log_at = now
            return

        confidence, (x_min, y_min, x_max, y_max), class_id = detection
        height, width = image.shape[:2]
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        error_x = (center_x - (width - 1) / 2.0) / max((width - 1) / 2.0, 1.0)
        error_y = (center_y - (height - 1) / 2.0) / max((height - 1) / 2.0, 1.0)

        detection_msg = Float32MultiArray()
        detection_msg.data = [
            float(class_id),
            confidence,
            center_x / width,
            center_y / height,
            error_x,
            error_y,
            (x_max - x_min) / width,
            (y_max - y_min) / height,
        ]
        self._detection_pub.publish(detection_msg)

        now = monotonic()
        if now - self._last_log_at >= 0.5:
            self.get_logger().info(
                f"class={COCO_CLASS_NAMES.get(class_id, f'coco_class_{class_id}')}, color={target_color}, "
                f"detector={detector_method}, "
                f"confidence={confidence:.3f}, "
                f"center_px=({center_x:.1f}, {center_y:.1f}), "
                f"error_norm=({error_x:+.3f}, {error_y:+.3f}), "
                f"bbox_px=({x_min:.1f}, {y_min:.1f})-({x_max:.1f}, {y_max:.1f}), "
                f"inference_ms={inference_ms:.1f}"
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
