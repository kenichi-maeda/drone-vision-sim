"""Safely verify that the simulated Duckiedrone camera publishes usable frames."""

from time import monotonic, time
from pathlib import Path

from mavros_msgs.msg import State
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


CAMERA_TOPIC = "/birdie/camera_node/image/compressed"
FLIGHT_FRAME_DIR = Path("/tmp/duckiedrone_flight_frames")
FLIGHT_FRAME_INTERVAL_S = 1.0
MAX_FLIGHT_FRAMES = 40


class CameraProbe(Node):
    """Subscribe to the camera stream and log low-rate health telemetry only."""

    def __init__(self) -> None:
        super().__init__("camera_probe")
        # This exactly matches the camera publisher discovered in the simulator.
        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._received_frames = 0
        self._snapshot_written = False
        self._snapshot_index = 0
        self._snapshot_every_frames = 30
        self._armed = False
        self._last_flight_frame_at = 0.0
        self._flight_frame_index = 0
        FLIGHT_FRAME_DIR.mkdir(parents=True, exist_ok=True)
        # This directory is owned by this probe; remove only its prior captures
        # so a new run cannot be mistaken for stale flight imagery.
        for old_frame in FLIGHT_FRAME_DIR.glob("flight_frame_*.jpg"):
            old_frame.unlink()
        self._subscription = self.create_subscription(
            CompressedImage,
            CAMERA_TOPIC,
            self._on_image,
            camera_qos,
        )
        state_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._state_subscription = self.create_subscription(
            State, "/mavros/state", self._on_state, state_qos
        )
        self.get_logger().info(f"Listening read-only on {CAMERA_TOPIC}")
        self.get_logger().info(
            f"Armed-flight image capture: every {FLIGHT_FRAME_INTERVAL_S:.1f} s, "
            f"maximum {MAX_FLIGHT_FRAMES} JPEGs in {FLIGHT_FRAME_DIR}"
        )

    def _on_state(self, state: State) -> None:
        self._armed = state.armed

    def _on_image(self, image: CompressedImage) -> None:
        """Report one out of every 30 received frames without decoding it."""
        self._received_frames += 1
        if not self._snapshot_written:
            snapshot = Path("/tmp/duckiedrone_camera.jpg")
            snapshot.write_bytes(bytes(image.data))
            self._snapshot_written = True
            self.get_logger().info(f"Saved a diagnostic frame to {snapshot}")
        now = monotonic()
        if (
            self._armed
            and self._flight_frame_index < MAX_FLIGHT_FRAMES
            and now - self._last_flight_frame_at >= FLIGHT_FRAME_INTERVAL_S
        ):
            frame_path = FLIGHT_FRAME_DIR / (
                f"flight_frame_{self._flight_frame_index:02d}.jpg"
            )
            frame_path.write_bytes(bytes(image.data))
            stamp = image.header.stamp
            image_stamp = stamp.sec + stamp.nanosec * 1e-9
            self.get_logger().info(
                f"Saved armed-flight image {self._flight_frame_index:02d} "
                f"to {frame_path} (image_stamp={image_stamp:.3f})"
            )
            self._flight_frame_index += 1
            self._last_flight_frame_at = now
        if (
            self._received_frames % self._snapshot_every_frames == 0
        ):
            sample_slot = self._snapshot_index % 20
            snapshot = Path(
                f"/tmp/duckiedrone_camera_sample_{sample_slot:02d}.jpg"
            )
            snapshot.write_bytes(bytes(image.data))
            self.get_logger().info(f"Updated rolling camera sample {sample_slot:02d}")
            self._snapshot_index += 1
        if self._received_frames % 30 != 0:
            return

        stamp = image.header.stamp
        capture_time = stamp.sec + stamp.nanosec * 1e-9
        age_ms = (time() - capture_time) * 1_000 if capture_time else float("nan")
        self.get_logger().info(
            f"frame={self._received_frames}, format={image.format!r}, "
            f"bytes={len(image.data)}, age_ms={age_ms:.1f}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraProbe()
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
