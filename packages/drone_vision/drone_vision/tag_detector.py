"""Detect AprilTag 36h11 markers in the simulated Duckiedrone camera stream.

This node is intentionally perception-only. It logs image-space detections,
a bounded P-only image-centering suggestion, and the latest MAVROS local pose;
it publishes no flight-control topic.
"""

from __future__ import annotations

import csv
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32MultiArray, String


CAMERA_TOPIC = "/birdie/camera_node/image/compressed"
CAMERA_INFO_TOPIC = "/birdie/camera_node/camera_info"
LOCAL_POSE_TOPIC = "/mavros/local_position/pose"
UPSCALE_FALLBACK = 4
MAX_TRACK_AGE_S = 2.0
MAX_TRACK_FORWARD_BACKWARD_ERROR_PX = 1.5
MIN_TRACK_FEATURES = 6
CALIBRATION_CAPTURE_PHASES = {
    "baseline", "step", "return", "landing", "tag_centering"
}
MAX_MOTION_CSV_ROWS = 1000


class TagDetector(Node):
    """Report AprilTag image-space offsets and optional metric poses."""

    def __init__(self) -> None:
        super().__init__("tag_detector")
        # Duckietown traffic-sign AprilTags are specified as 6.5 cm squares.
        # Override this for a custom marker with a different printed size.
        self.declare_parameter("tag_size_m", 0.065)
        self.declare_parameter("control_tag_id", 5)
        self.declare_parameter("dry_run_kp", 0.25)
        self.declare_parameter("dry_run_max_step", 0.15)
        self.declare_parameter("observation_log_interval_s", 0.25)
        self.declare_parameter("motion_csv_path", "/tmp/duckiedrone_tag_motion.csv")
        self._tag_size_m = float(self.get_parameter("tag_size_m").value)
        self._control_tag_id = int(self.get_parameter("control_tag_id").value)
        self._dry_run_kp = float(self.get_parameter("dry_run_kp").value)
        self._dry_run_max_step = abs(
            float(self.get_parameter("dry_run_max_step").value)
        )
        self._observation_log_interval_s = max(
            0.05, float(self.get_parameter("observation_log_interval_s").value)
        )
        self._flight_phase = "unknown"
        self._calibration_phase: str | None = None
        self._mavros_state: State | None = None
        self._mavros_extended_state: ExtendedState | None = None
        self._px4_target_local: PositionTarget | None = None
        self._px4_target_received_at = 0.0
        self._motion_csv_rows = 0
        self._motion_csv_limit_reported = False
        self._motion_csv_file = None
        self._motion_csv_writer = None
        csv_path = Path(str(self.get_parameter("motion_csv_path").value))
        try:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._motion_csv_file = csv_path.open("w", newline="", encoding="utf-8")
            self._motion_csv_writer = csv.writer(self._motion_csv_file)
            self._motion_csv_writer.writerow(
                [
                    "image_stamp_s", "pose_stamp_s", "phase", "tag_id",
                    "center_x_px", "center_y_px", "error_x_norm", "error_y_norm",
                    "bbox_x_min_px", "bbox_y_min_px", "bbox_x_max_px", "bbox_y_max_px",
                    "pose_x_m", "pose_y_m", "pose_z_m", "pose_age_ms", "detector_scale",
                    "detection_method", "tracking_age_ms",
                    "armed", "mode", "landed_state",
                    "px4_target_echo_x_m", "px4_target_echo_y_m",
                    "px4_target_echo_z_m", "px4_target_echo_age_ms",
                ]
            )
            self._motion_csv_file.flush()
        except OSError as exc:
            self.get_logger().warning(f"Could not open motion CSV {csv_path}: {exc}")
        self._camera_matrix: np.ndarray | None = None
        self._distortion: np.ndarray | None = None
        self._local_pose: PoseStamped | None = None
        self._local_pose_received_at = 0.0
        self._last_no_tag_log = 0.0
        self._last_detection_log: dict[int, float] = {}
        self._last_correlation_sample: dict[
            int, tuple[float, float, float, float, float, float]
        ] = {}
        self._tracked_tag_corners: np.ndarray | None = None
        self._tracked_tag_gray: np.ndarray | None = None
        self._tracked_tag_id: int | None = None
        self._last_verified_detection_at = 0.0

        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._info_subscription = self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, camera_qos
        )
        self._image_subscription = self.create_subscription(
            CompressedImage, CAMERA_TOPIC, self._on_image, camera_qos
        )
        self._tag_error_pub = self.create_publisher(
            Float32MultiArray, "/drone_vision/tag_error", 10
        )
        # MAVROS local pose publishes BEST_EFFORT; RELIABLE would not match it.
        pose_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pose_subscription = self.create_subscription(
            PoseStamped, LOCAL_POSE_TOPIC, self._on_local_pose, pose_qos
        )
        self._state_subscription = self.create_subscription(
            State, "/mavros/state", self._on_mavros_state, pose_qos
        )
        self._extended_state_subscription = self.create_subscription(
            ExtendedState, "/mavros/extended_state", self._on_extended_state, pose_qos
        )
        self._target_echo_subscription = self.create_subscription(
            PositionTarget,
            "/mavros/setpoint_raw/target_local",
            self._on_px4_target_echo,
            pose_qos,
        )
        self._phase_subscription = self.create_subscription(
            String,
            "/drone_vision/calibration_phase",
            self._on_calibration_phase,
            10,
        )

        dictionary_id = cv2.aruco.DICT_APRILTAG_36h11
        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, parameters)
        self._run_self_check()
        pose_note = (
            f"metric pose enabled; tag_size_m={self._tag_size_m:.3f} m"
            if self._tag_size_m > 0.0
            else "metric pose disabled until the physical tag size is calibrated"
        )
        self.get_logger().info(
            f"Looking for AprilTag 36h11 markers on {CAMERA_TOPIC}; {pose_note}; "
            "P-only image-centering dry-run enabled (no MAVROS commands); "
            f"publishing target tag {self._control_tag_id} image error on "
            "/drone_vision/tag_error; "
            f"logging ground observations at 1 Hz and armed-flight observations "
            f"every {self._observation_log_interval_s:.2f} s to {csv_path} "
            f"(maximum {MAX_MOTION_CSV_ROWS} rows)"
        )

    def _on_calibration_phase(self, phase: String) -> None:
        # Preserve the more detailed MAVROS-derived label during normal/manual
        # flights; safe_hover publishes "idle" even when AUTO.TAKEOFF is active.
        self._calibration_phase = (
            phase.data if phase.data in CALIBRATION_CAPTURE_PHASES else None
        )
        self._update_flight_phase()

    def _on_mavros_state(self, state: State) -> None:
        self._mavros_state = state
        self._update_flight_phase()

    def _on_extended_state(self, state: ExtendedState) -> None:
        self._mavros_extended_state = state
        self._update_flight_phase()

    def _on_px4_target_echo(self, target: PositionTarget) -> None:
        # MAVROS republishes PX4's POSITION_TARGET_LOCAL_NED telemetry here
        # when the autopilot reports a local target. This is read-only feedback.
        self._px4_target_local = target
        self._px4_target_received_at = monotonic()

    def _update_flight_phase(self) -> None:
        if self._calibration_phase is not None:
            self._flight_phase = self._calibration_phase
            return
        state = self._mavros_state
        extended = self._mavros_extended_state
        if state is None:
            self._flight_phase = "unknown"
            return

        mode = state.mode.upper()
        landed_state = None if extended is None else extended.landed_state
        if mode == "AUTO.TAKEOFF" or landed_state == ExtendedState.LANDED_STATE_TAKEOFF:
            self._flight_phase = "takeoff"
        elif mode == "AUTO.LAND" or landed_state == ExtendedState.LANDED_STATE_LANDING:
            self._flight_phase = "landing"
        elif state.armed and landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self._flight_phase = "airborne"
        elif state.armed:
            self._flight_phase = "armed_ground"
        elif landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            self._flight_phase = "ground"
        else:
            self._flight_phase = "disarmed"

    def destroy_node(self) -> None:
        if self._motion_csv_file is not None:
            self._motion_csv_file.close()
        super().destroy_node()

    def _run_self_check(self) -> None:
        """Verify this OpenCV build can decode a generated 36h11 tag."""
        tag_id = 1
        marker = cv2.aruco.generateImageMarker(self._dictionary, tag_id, 160)
        test_image = np.full((240, 240), 255, dtype=np.uint8)
        test_image[40:200, 40:200] = marker
        _corners, ids, _rejected = self._detector.detectMarkers(test_image)
        if ids is not None and tag_id in ids.flatten():
            self.get_logger().info("AprilTag self-check passed (36h11, id=1)")
        else:
            self.get_logger().error("AprilTag self-check FAILED; map testing is paused")

    def _on_camera_info(self, info: CameraInfo) -> None:
        """Keep the current pinhole calibration for metric pose estimation."""
        self._camera_matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        self._distortion = np.asarray(info.d, dtype=np.float64)

    def _on_local_pose(self, pose: PoseStamped) -> None:
        """Cache the latest MAVROS pose for read-only image/vehicle correlation."""
        self._local_pose = pose
        self._local_pose_received_at = monotonic()

    def _on_image(self, image: CompressedImage) -> None:
        """Decode a JPEG and periodically report tag image-space geometry."""
        frame = cv2.imdecode(np.frombuffer(image.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Could not decode a compressed camera frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detector.detectMarkers(frame)
        detection_scale = 1
        if ids is None:
            # The sign tag occupies only a few pixels at the camera's current
            # range. Retry a missed frame at higher sampling resolution. Scale
            # corner coordinates back before reporting/estimating its pose.
            enlarged = cv2.resize(
                frame,
                None,
                fx=UPSCALE_FALLBACK,
                fy=UPSCALE_FALLBACK,
                interpolation=cv2.INTER_CUBIC,
            )
            corners, ids, _rejected = self._detector.detectMarkers(enlarged)
            if ids is not None:
                detection_scale = UPSCALE_FALLBACK
                corners = [tag_corners / detection_scale for tag_corners in corners]

        now = monotonic()
        detection_method = f"aruco_{detection_scale}x"
        tracking_age_s: float | None = None
        if ids is not None:
            target_indices = np.flatnonzero(ids.flatten() == self._control_tag_id)
            if target_indices.size:
                target_index = int(target_indices[0])
                self._tracked_tag_id = self._control_tag_id
                self._tracked_tag_corners = corners[target_index].astype(np.float32)
                self._tracked_tag_gray = gray
                self._last_verified_detection_at = now
            elif self._tracked_tag_id == self._control_tag_id:
                tracked = self._track_control_tag(gray, now)
                if tracked is not None:
                    tracked_corners, tracking_age_s = tracked
                    corners = [tracked_corners]
                    ids = np.asarray([[self._control_tag_id]], dtype=np.int32)
                    detection_method = "optical_flow"
                    detection_scale = 0
        elif self._tracked_tag_id == self._control_tag_id:
            tracked = self._track_control_tag(gray, now)
            if tracked is not None:
                tracked_corners, tracking_age_s = tracked
                corners = [tracked_corners]
                ids = np.asarray([[self._control_tag_id]], dtype=np.int32)
                detection_method = "optical_flow"
                detection_scale = 0

        if ids is None:
            self._clear_tag_track()
            self._log_no_tag()
            return

        height, width = frame.shape[:2]
        if self._camera_matrix is not None:
            optical_cx = float(self._camera_matrix[0, 2])
            optical_cy = float(self._camera_matrix[1, 2])
        else:
            optical_cx = (width - 1) / 2.0
            optical_cy = (height - 1) / 2.0

        for tag_id, tag_corners in zip(ids.flatten(), corners):
            points = tag_corners.reshape(4, 2)
            center = points.mean(axis=0)
            x_min, y_min = points.min(axis=0)
            x_max, y_max = points.max(axis=0)
            # Normalized error is in [-1, 1] relative to the calibrated optical
            # center (positive y points down in the image).
            error_x = (center[0] - optical_cx) / max(optical_cx, 1.0)
            error_y = (center[1] - optical_cy) / max(height - 1 - optical_cy, 1.0)

            tag_id = int(tag_id)
            if tag_id == self._control_tag_id:
                # Float32MultiArray data is [error_x_norm, error_y_norm].
                # Publish on every detected camera frame for the flight node;
                # logs remain throttled separately below.
                tag_error = Float32MultiArray()
                tag_error.data = [float(error_x), float(error_y)]
                self._tag_error_pub.publish(tag_error)
            state = self._mavros_state
            capture_active = (
                self._motion_csv_writer is not None
                and self._motion_csv_rows < MAX_MOTION_CSV_ROWS
            )
            flight_active = bool(state is not None and state.armed)
            log_interval = (
                self._observation_log_interval_s
                if capture_active and flight_active
                else 1.0
            )
            if now - self._last_detection_log.get(tag_id, 0.0) < log_interval:
                continue
            self._last_detection_log[tag_id] = now
            details = (
                f"tag_id={tag_id}, detection_method={detection_method}, "
                f"image={width}x{height}, "
                f"center_px=({center[0]:.1f}, {center[1]:.1f}), "
                f"error_norm=({error_x:+.3f}, {error_y:+.3f}), "
                f"bbox_px=({x_min:.1f}, {y_min:.1f})-({x_max:.1f}, {y_max:.1f})"
            )
            if tracking_age_s is not None:
                details += f", tracking_age_s={tracking_age_s:.2f}"
            # P-only image-space recommendation. The values are normalized
            # half-frame shifts toward the optical center, not vehicle-axis
            # velocities; camera-to-body calibration is not established.
            image_du = float(
                np.clip(
                    -self._dry_run_kp * error_x,
                    -self._dry_run_max_step,
                    self._dry_run_max_step,
                )
            )
            image_dv = float(
                np.clip(
                    -self._dry_run_kp * error_y,
                    -self._dry_run_max_step,
                    self._dry_run_max_step,
                )
            )
            details += (
                f", dry_run_P_image_delta_half_frame=({image_du:+.3f}, "
                f"{image_dv:+.3f}) (suggestion only; no flight command)"
            )
            image_stamp = image.header.stamp
            details += (
                f", image_stamp={image_stamp.sec}.{image_stamp.nanosec:09d}"
                f", flight_phase={self._flight_phase}"
            )
            pose_age_ms: float | None = None
            pose_stamp_s: float | None = None
            pose_x = pose_y = pose_z = None
            if self._local_pose is not None:
                pose = self._local_pose.pose.position
                pose_age_ms = (now - self._local_pose_received_at) * 1000.0
                pose_stamp = self._local_pose.header.stamp
                pose_stamp_s = pose_stamp.sec + pose_stamp.nanosec * 1e-9
                pose_x, pose_y, pose_z = pose.x, pose.y, pose.z
                details += (
                    f", local_pose_m=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f})"
                    f", pose_age_ms={pose_age_ms:.0f}"
                    f", pose_stamp={pose_stamp.sec}.{pose_stamp.nanosec:09d}"
                )
                if pose_age_ms <= 250.0:
                    previous = self._last_correlation_sample.get(tag_id)
                    if previous is not None:
                        (prev_time, prev_u, prev_v, prev_x, prev_y, prev_z) = previous
                        details += (
                            f", motion_observation_dt_s={now - prev_time:.2f}"
                            f", delta_image_px=({center[0] - prev_u:+.1f}, "
                            f"{center[1] - prev_v:+.1f})"
                            f", delta_local_pose_m=({pose.x - prev_x:+.3f}, "
                            f"{pose.y - prev_y:+.3f}, {pose.z - prev_z:+.3f})"
                        )
                    self._last_correlation_sample[tag_id] = (
                        now,
                        float(center[0]),
                        float(center[1]),
                        float(pose.x),
                        float(pose.y),
                        float(pose.z),
                    )
            else:
                details += ", local_pose=unavailable"
            px4_target = self._px4_target_local
            target_echo_age_ms: float | None = None
            target_x = target_y = target_z = None
            if px4_target is not None:
                target_echo_age_ms = (now - self._px4_target_received_at) * 1000.0
                target_x = px4_target.position.x
                target_y = px4_target.position.y
                target_z = px4_target.position.z
                details += (
                    f", px4_target_echo_m=({target_x:.3f}, {target_y:.3f}, "
                    f"{target_z:.3f}), target_echo_age_ms={target_echo_age_ms:.0f}"
                )
            else:
                details += ", px4_target_echo=unavailable"
            if capture_active:
                self._motion_csv_writer.writerow(
                    [
                        image_stamp.sec + image_stamp.nanosec * 1e-9,
                        pose_stamp_s,
                        self._flight_phase,
                        tag_id,
                        f"{center[0]:.3f}",
                        f"{center[1]:.3f}",
                        f"{error_x:.6f}",
                        f"{error_y:.6f}",
                        f"{x_min:.3f}",
                        f"{y_min:.3f}",
                        f"{x_max:.3f}",
                        f"{y_max:.3f}",
                        pose_x,
                        pose_y,
                        pose_z,
                        None if pose_age_ms is None else f"{pose_age_ms:.1f}",
                        detection_scale,
                        detection_method,
                        (
                            None
                            if tracking_age_s is None
                            else f"{tracking_age_s * 1000.0:.1f}"
                        ),
                        None if state is None else state.armed,
                        None if state is None else state.mode,
                        (
                            None
                            if self._mavros_extended_state is None
                            else self._mavros_extended_state.landed_state
                        ),
                        target_x,
                        target_y,
                        target_z,
                        (
                            None
                            if target_echo_age_ms is None
                            else f"{target_echo_age_ms:.1f}"
                        ),
                    ]
                )
                self._motion_csv_file.flush()
                self._motion_csv_rows += 1
            elif (
                self._motion_csv_rows >= MAX_MOTION_CSV_ROWS
                and not self._motion_csv_limit_reported
            ):
                self.get_logger().warning(
                    f"Motion CSV reached its {MAX_MOTION_CSV_ROWS}-row limit; "
                    "further high-rate capture is disabled"
                )
                self._motion_csv_limit_reported = True
            if self._tag_size_m > 0.0 and detection_method != "optical_flow":
                pose = self._estimate_pose(tag_corners)
                if pose is not None:
                    details += (
                        f", camera_position_m=({pose[0]:.3f}, "
                        f"{pose[1]:.3f}, {pose[2]:.3f})"
                    )
            self.get_logger().info(details)

    def _clear_tag_track(self) -> None:
        self._tracked_tag_corners = None
        self._tracked_tag_gray = None
        self._tracked_tag_id = None
        self._last_verified_detection_at = 0.0

    def _track_control_tag(
        self, gray: np.ndarray, now: float
    ) -> tuple[np.ndarray, float] | None:
        """Bridge short detector misses using checked optical-flow tracking."""
        previous_gray = self._tracked_tag_gray
        previous_corners = self._tracked_tag_corners
        age_s = now - self._last_verified_detection_at
        if (
            previous_gray is None
            or previous_corners is None
            or age_s > MAX_TRACK_AGE_S
        ):
            self._clear_tag_track()
            return None

        # A 6.5 cm sign is only about 20–30 pixels wide in this camera view.
        # Tracking just its four outer corners is brittle under the small
        # amount of blur seen during flight. Track multiple high-contrast
        # points inside the last verified tag quad, then fit a robust partial
        # affine transform to move the quad as a whole.
        height, width = previous_gray.shape[:2]
        mask = np.zeros((height, width), dtype=np.uint8)
        polygon = np.round(previous_corners.reshape(4, 2)).astype(np.int32)
        cv2.fillConvexPoly(mask, polygon, 255)
        features = cv2.goodFeaturesToTrack(
            previous_gray,
            maxCorners=40,
            qualityLevel=0.01,
            minDistance=2.0,
            mask=mask,
            blockSize=3,
        )
        if features is None or len(features) < MIN_TRACK_FEATURES:
            self._clear_tag_track()
            return None

        next_points, status_forward, _error = cv2.calcOpticalFlowPyrLK(
            previous_gray,
            gray,
            features,
            None,
            winSize=(21, 21),
            maxLevel=3,
        )
        if next_points is None or status_forward is None:
            self._clear_tag_track()
            return None
        previous_reconstructed, status_backward, _back_error = cv2.calcOpticalFlowPyrLK(
            gray,
            previous_gray,
            next_points,
            None,
            winSize=(21, 21),
            maxLevel=3,
        )
        if previous_reconstructed is None or status_backward is None:
            self._clear_tag_track()
            return None

        forward_backward_error = np.linalg.norm(
            features.reshape(-1, 2) - previous_reconstructed.reshape(-1, 2),
            axis=1,
        )
        good = (
            (status_forward.reshape(-1) != 0)
            & (status_backward.reshape(-1) != 0)
            & np.isfinite(forward_backward_error)
            & (forward_backward_error <= MAX_TRACK_FORWARD_BACKWARD_ERROR_PX)
        )
        if np.count_nonzero(good) < MIN_TRACK_FEATURES:
            self._clear_tag_track()
            return None

        source = features.reshape(-1, 2)[good]
        destination = next_points.reshape(-1, 2)[good]
        transform, inliers = cv2.estimateAffinePartial2D(
            source,
            destination,
            method=cv2.RANSAC,
            ransacReprojThreshold=2.0,
            maxIters=100,
            confidence=0.95,
            refineIters=5,
        )
        if transform is None or inliers is None:
            self._clear_tag_track()
            return None
        inlier_mask = inliers.reshape(-1).astype(bool)
        inlier_count = int(np.count_nonzero(inlier_mask))
        if inlier_count < MIN_TRACK_FEATURES or inlier_count / len(source) < 0.6:
            self._clear_tag_track()
            return None

        previous_quad = previous_corners.reshape(4, 2).astype(np.float32)
        points = cv2.transform(previous_quad[None, :, :], transform)[0]
        height, width = gray.shape[:2]
        in_bounds = bool(
            np.all(points[:, 0] >= 0)
            and np.all(points[:, 0] < width)
            and np.all(points[:, 1] >= 0)
            and np.all(points[:, 1] < height)
        )
        previous_area = abs(cv2.contourArea(previous_quad))
        current_area = abs(cv2.contourArea(points.astype(np.float32)))
        area_ratio = current_area / max(previous_area, 1.0)
        convex = cv2.isContourConvex(np.round(points).astype(np.int32))
        valid = bool(
            in_bounds
            and convex
            and 0.65 <= area_ratio <= 1.55
            and current_area >= 80.0
        )
        if not valid:
            self._clear_tag_track()
            return None

        tracked_corners = points.reshape(1, 4, 2).astype(np.float32)
        self._tracked_tag_corners = tracked_corners
        self._tracked_tag_gray = gray
        return tracked_corners, age_s

    def _estimate_pose(self, corners: np.ndarray) -> tuple[float, float, float] | None:
        """Return the tag center in the optical-camera frame, in metres."""
        if self._camera_matrix is None or self._distortion is None:
            return None

        half_size = self._tag_size_m / 2.0
        object_points = np.array(
            [
                [-half_size, half_size, 0.0],
                [half_size, half_size, 0.0],
                [half_size, -half_size, 0.0],
                [-half_size, -half_size, 0.0],
            ],
            dtype=np.float64,
        )
        success, _rotation, translation = cv2.solvePnP(
            object_points,
            corners.reshape(4, 2).astype(np.float64),
            self._camera_matrix,
            self._distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None
        return tuple(float(value) for value in translation.reshape(3))

    def _log_no_tag(self) -> None:
        """Avoid flooding logs while the current map contains no marker."""
        now = monotonic()
        if now - self._last_no_tag_log >= 5.0:
            self.get_logger().info("No AprilTag 36h11 detected yet")
            self._last_no_tag_log = now


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TagDetector()
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
