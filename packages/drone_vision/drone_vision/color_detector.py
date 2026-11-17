"""Read-only OpenCV color detector for the Duckiedrone camera.

Finds the largest blob of each configured color in every processed frame and
publishes them as JSON. Perception only: it sends no flight commands.

Topics:
  in   /<robot name>/camera_node/image/compressed
  out  /drone_vision/color_detections         std_msgs/String (JSON list)
  out  /drone_vision/color_image/compressed   annotated frame, only while watched
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
from std_msgs.msg import String

from .color_utils import COLOR_RANGES, draw_blobs, find_color_blobs

# The camera topic is namespaced by the robot's name (VEHICLE_NAME in Duckietown containers).
CAMERA_TOPIC = f"/{os.environ.get('VEHICLE_NAME') or 'birdie'}/camera_node/image/compressed"
DETECTIONS_TOPIC = "/drone_vision/color_detections"
ANNOTATED_TOPIC = "/drone_vision/color_image/compressed"


class ColorDetector(Node):
    def __init__(self) -> None:
        super().__init__("color_detector")
        self.declare_parameter("colors", list(COLOR_RANGES))
        self.declare_parameter("min_area_fraction", 0.002)
        self.declare_parameter("process_every_n_frames", 2)
        self._colors = [str(c) for c in self.get_parameter("colors").value if str(c) in COLOR_RANGES]
        if not self._colors:
            raise ValueError(f"colors must include at least one of {list(COLOR_RANGES)}")
        self._min_area_fraction = float(self.get_parameter("min_area_fraction").value)
        self._every_n = max(1, int(self.get_parameter("process_every_n_frames").value))
        self._frame_count = 0
        self._last_log_at = 0.0

        self.create_subscription(
            CompressedImage, CAMERA_TOPIC, self._on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._detections_pub = self.create_publisher(String, DETECTIONS_TOPIC, 10)
        self._annotated_pub = self.create_publisher(CompressedImage, ANNOTATED_TOPIC, QoSProfile(depth=1))
        self.get_logger().info(
            f"Color detector watching {CAMERA_TOPIC} for {', '.join(self._colors)} "
            f"(min area {self._min_area_fraction * 100:.2f}% of frame). Publishing JSON on "
            f"{DETECTIONS_TOPIC}. Perception only; no flight commands."
        )

    def _on_image(self, message: CompressedImage) -> None:
        self._frame_count += 1
        if self._frame_count % self._every_n:
            return
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("Could not decode camera frame for color detection")
            return
        started_at = monotonic()
        blobs = find_color_blobs(image, self._colors, self._min_area_fraction)
        elapsed_ms = (monotonic() - started_at) * 1000.0

        self._detections_pub.publish(String(data=json.dumps(blobs)))

        if self._annotated_pub.get_subscription_count() > 0:
            draw_blobs(image, blobs)
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                annotated = CompressedImage()
                annotated.header = message.header
                annotated.format = "jpeg"
                annotated.data = encoded.tobytes()
                self._annotated_pub.publish(annotated)

        now = monotonic()
        if now - self._last_log_at >= 1.0:
            seen = ", ".join(
                f"{b['color']} {b['area_fraction'] * 100:.1f}% @x={b['center_norm'][0]:.2f}" for b in blobs
            ) or "none"
            self.get_logger().info(f"colors: {seen}; process_ms={elapsed_ms:.1f}")
            self._last_log_at = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
