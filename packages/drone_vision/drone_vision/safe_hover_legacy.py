"""Legacy, feature-rich PX4/MAVROS test controller retained for reference.

This node is deliberately separate from perception. It requires an explicit
``enabled:=true`` parameter and lands after a short, fixed hover. The default
mode delegates takeoff to PX4's documented AUTO.TAKEOFF behavior. An optional
OFFBOARD position-hold mode is retained for controller experiments; an
optional local-X calibration step is available after a verified OFFBOARD
handoff or in the legacy OFFBOARD-only mode. This is
intended only for the local Duckiematrix virtual drone.
"""

from __future__ import annotations

from math import atan2, cos, isfinite, sin
from time import monotonic

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String


class SafeHover(Node):
    """Run one bounded takeoff/hover/land test in Duckiematrix."""
    def __init__(self) -> None:
        super().__init__("safe_hover")
        self.declare_parameter("enabled", False)
        self.declare_parameter("flight_mode", "AUTO.TAKEOFF")
        self.declare_parameter("height_m", 0.60)
        self.declare_parameter("hover_seconds", 5.0)
        self.declare_parameter("climb_timeout_seconds", 18.0)
        self.declare_parameter("altitude_tolerance_m", 0.10)
        self.declare_parameter("warmup_seconds", 2.0)
        self.declare_parameter("hard_timeout_seconds", 35.0)
        self.declare_parameter("landing_timeout_seconds", 20.0)
        # A brief pose-stream gap can occur during Matrix takeoff. Keep the
        # AUTO.TAKEOFF phase tolerant of that gap, but abort if pose stays stale.
        self.declare_parameter("pose_stale_timeout_seconds", 1.5)
        self.declare_parameter("calibration_x_step_enabled", False)
        self.declare_parameter("calibration_x_step_m", 0.15)
        self.declare_parameter("calibration_baseline_seconds", 2.0)
        self.declare_parameter("calibration_step_seconds", 3.0)
        self.declare_parameter("calibration_return_seconds", 2.0)
        self.declare_parameter("handoff_to_offboard", False)
        self.declare_parameter("handoff_rise_m", 0.40)
        self.declare_parameter("handoff_timeout_seconds", 4.0)
        self.declare_parameter("tag_centering_enabled", False)
        self.declare_parameter("tag_centering_seconds", 8.0)
        self.declare_parameter("tag_centering_kp_m", 0.20)
        self.declare_parameter("tag_centering_deadband", 0.04)
        self.declare_parameter("tag_centering_step_limit_m", 0.005)
        self.declare_parameter("tag_centering_max_offset_m", 0.10)
        self.declare_parameter("tag_loss_timeout_seconds", 0.75)
        self.declare_parameter("object_search_enabled", False)
        self.declare_parameter("object_search_seconds", 20.0)
        self.declare_parameter("object_search_initial_hold_seconds", 2.0)
        # Positive yaw moved the car farther toward the image's right edge in
        # the 2026-10-31 Matrix scan, so search in the opposite direction.
        self.declare_parameter("object_search_yaw_rate_rad_s", -0.31)
        # Keep this aligned with the configured YOLO target-class threshold;
        # requiring a higher value discarded fresh observations in Matrix.
        self.declare_parameter("object_search_detection_confidence", 0.10)
        self.declare_parameter("object_search_max_bbox_area", 0.18)
        self.declare_parameter("object_search_confirmation_count", 3)
        self.declare_parameter("object_search_max_acquisition_error_x", 0.45)
        self.declare_parameter("object_search_hold_after_detection_seconds", 1.5)
        self.declare_parameter("object_search_preferred_side", "any")
        self.declare_parameter("object_search_side_deadband", 0.10)
        self.declare_parameter("object_centering_enabled", False)
        self.declare_parameter("object_centering_seconds", 5.0)
        self.declare_parameter("object_centering_kp_rad", 0.20)
        self.declare_parameter("object_centering_deadband", 0.08)
        self.declare_parameter("object_centering_step_limit_rad", 0.03)
        self.declare_parameter("object_centering_max_offset_rad", 0.35)
        # YOLO inference can occasionally skip a frame. While stale, the
        # controller holds its last bounded yaw target (it never keeps
        # integrating an old error), then lands if detections do not resume.
        self.declare_parameter("object_centering_loss_timeout_seconds", 1.5)
        self.declare_parameter("object_approach_enabled", False)
        self.declare_parameter("object_approach_speed_m_s", 0.08)
        # Stop after a relative apparent-size increase from the first stable
        # centered detection. This adapts to target class and initial range;
        # it is visual scale, not calibrated metric distance.
        self.declare_parameter("object_approach_size_growth_ratio", 3.0)
        self.declare_parameter("object_approach_center_tolerance", 0.20)
        self.declare_parameter("object_approach_loss_timeout_seconds", 2.0)
        self.declare_parameter("object_approach_timeout_seconds", 25.0)

        # MAVROS local pose/state and position setpoint topics use BEST_EFFORT.
        # A RELIABLE subscription is incompatible with that publisher.
        mavros_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pose: PoseStamped | None = None
        self._pose_received_at = 0.0
        self._state: State | None = None
        self._state_received_at = 0.0
        self._extended_state: ExtendedState | None = None
        self._extended_state_received_at = 0.0
        self._started_at: float | None = None
        self._climb_started_at: float | None = None
        self._within_altitude_since: float | None = None
        self._hover_started_at: float | None = None
        self._landing_started_at: float | None = None
        self._landing_disarmed_since: float | None = None
        self._last_progress_log_at = 0.0
        self._landing_timeout_reported = False
        self._last_wait_message: str | None = None
        self._phase = "idle"
        self._last_request_at = 0.0
        self._target: PoseStamped | None = None
        self._base_x: float | None = None
        self._base_y: float | None = None
        self._calibration_phase: str | None = None
        self._calibration_phase_started_at: float | None = None
        self._takeoff_mode_accepted = False
        self._auto_start_z: float | None = None
        self._handoff_requested_at: float | None = None
        self._handoff_prestream_started_at: float | None = None
        self._selected_mode: str | None = None
        self._handoff_enabled: bool | None = None
        self._handoff_rise_m: float | None = None
        self._calibration_enabled: bool | None = None
        self._tag_centering_enabled: bool | None = None
        self._tag_error_x: float | None = None
        self._tag_error_received_at = 0.0
        self._last_tag_error_used_at = 0.0
        self._tag_centering_started_at: float | None = None
        self._last_tag_control_at = 0.0
        self._tag_centering_start_x: float | None = None
        self._object_search_enabled: bool | None = None
        self._object_centering_enabled: bool | None = None
        self._object_search_started_at: float | None = None
        self._object_search_start_yaw: float | None = None
        self._object_search_angle_rad = 0.0
        self._object_search_detection_received_at = 0.0
        self._object_search_detection_confidence: float | None = None
        self._object_search_error_x: float | None = None
        self._object_search_error_y: float | None = None
        self._object_search_bbox_area: float | None = None
        self._object_search_detection_count = 0
        self._object_search_preferred_target_seen = False
        self._last_object_search_detection_at = 0.0
        self._object_search_detection_seen_at: float | None = None
        self._object_centering_started_at: float | None = None
        self._object_centering_start_yaw: float | None = None
        self._last_yolo_detection_used_at = 0.0
        self._last_yolo_control_at = 0.0
        self._object_approach_enabled = False
        self._object_approach_started_at: float | None = None
        self._object_approach_yaw: float | None = None
        self._object_approach_start_yaw: float | None = None
        self._object_approach_distance_m = 0.0
        self._object_approach_initial_bbox_area: float | None = None
        self._object_approach_stop_bbox_area: float | None = None
        self._object_approach_last_update_at = 0.0

        flight_mode = str(self.get_parameter("flight_mode").value)
        if flight_mode not in ("AUTO.TAKEOFF", "OFFBOARD"):
            raise ValueError("flight_mode must be AUTO.TAKEOFF or OFFBOARD")

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._on_pose, mavros_qos
        )
        self.create_subscription(State, "/mavros/state", self._on_state, mavros_qos)
        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._on_extended_state,
            mavros_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "/drone_vision/tag_error",
            self._on_tag_error,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/drone_vision/yolo_detection",
            self._on_yolo_detection,
            10,
        )
        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", mavros_qos
        )
        self._calibration_phase_pub = self.create_publisher(
            String, "/drone_vision/calibration_phase", 10
        )
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.create_timer(0.05, self._tick)  # PX4 Offboard requires a steady stream.

        self.get_logger().info(
            "Safe hover is DISABLED. Start only with enabled:=true; "
            f"flight_mode={flight_mode}; waits up to "
            f"{float(self.get_parameter('climb_timeout_seconds').value):.0f} s for "
            "airborne confirmation, holds for "
            f"{float(self.get_parameter('hover_seconds').value):.0f} s, then "
            "requests AUTO.LAND. +X calibration and tag centering after OFFBOARD "
            "handoff are opt-in. YOLO search and image centering are opt-in."
        )

    def _on_pose(self, pose: PoseStamped) -> None:
        self._pose = pose
        self._pose_received_at = monotonic()

    def _on_state(self, state: State) -> None:
        self._state = state
        self._state_received_at = monotonic()
        if self._phase == "landing":
            if state.armed:
                self._landing_disarmed_since = None
            elif self._landing_disarmed_since is None:
                self._landing_disarmed_since = self._state_received_at

    def _on_extended_state(self, state: ExtendedState) -> None:
        self._extended_state = state
        self._extended_state_received_at = monotonic()

    def _on_tag_error(self, error: Float32MultiArray) -> None:
        if len(error.data) < 2:
            return
        error_x = float(error.data[0])
        error_y = float(error.data[1])
        if not isfinite(error_x) or not isfinite(error_y):
            return
        self._tag_error_x = error_x
        self._tag_error_received_at = monotonic()

    def _on_yolo_detection(self, detection: Float32MultiArray) -> None:
        # yolo_detector publishes [class_id, confidence, center_x, center_y,
        # error_x, error_y, width, height]. Reject implausibly large boxes:
        # in the Matrix scene the COCO model briefly enclosed much of a
        # building as a target while the real object was tiny at the frame
        # edge. Confirm a candidate over multiple consecutive inferences
        # before interrupting the yaw search.
        if len(detection.data) < 8:
            return
        confidence = float(detection.data[1])
        error_x = float(detection.data[4])
        error_y = float(detection.data[5])
        box_width = float(detection.data[6])
        box_height = float(detection.data[7])
        now = monotonic()
        max_bbox_area = float(self.get_parameter("object_search_max_bbox_area").value)
        confidence_threshold = float(
            self.get_parameter("object_search_detection_confidence").value
        )
        if (
            not isfinite(confidence)
            or not 0.0 <= confidence <= 1.0
            or not isfinite(error_x)
            or not -1.0 <= error_x <= 1.0
            or not isfinite(error_y)
            or not -1.0 <= error_y <= 1.0
            or not isfinite(box_width)
            or not 0.0 < box_width <= 1.0
            or not isfinite(box_height)
            or not 0.0 < box_height <= 1.0
            or box_width * box_height > max_bbox_area
            or confidence < confidence_threshold
        ):
            # A single empty/low-confidence message is a detector miss, not
            # proof that the target disappeared. Clear the confirmation run
            # but retain the timestamp of the last valid observation so the
            # approach can hold position and reacquire before its loss timeout.
            self._object_search_detection_count = 0
            self._last_object_search_detection_at = 0.0
            return
        preferred_side = str(
            self.get_parameter("object_search_preferred_side").value
        )
        side_deadband = float(
            self.get_parameter("object_search_side_deadband").value
        )
        if (
            self._phase == "object_search"
            and preferred_side != "any"
            and not self._object_search_preferred_target_seen
        ):
            on_preferred_side = (
                error_x <= -side_deadband
                if preferred_side == "left"
                else error_x >= side_deadband
            )
            if not on_preferred_side:
                # Ignore the already-visible candidate until a detection
                # appears on the requested image side during the yaw scan.
                self._object_search_detection_count = 0
                self._last_object_search_detection_at = 0.0
                self._object_search_detection_received_at = 0.0
                return
            self._object_search_preferred_target_seen = True
        if now - self._last_object_search_detection_at > 0.5:
            self._object_search_detection_count = 0
        self._object_search_detection_count += 1
        self._last_object_search_detection_at = now
        self._object_search_detection_confidence = confidence
        self._object_search_error_x = error_x
        self._object_search_error_y = error_y
        self._object_search_bbox_area = box_width * box_height
        self._object_search_detection_received_at = now

    def _enabled(self) -> bool:
        return bool(self.get_parameter("enabled").value)

    def _tick(self) -> None:
        now = monotonic()
        phase_msg = String()
        phase_msg.data = self._calibration_phase or self._phase
        self._calibration_phase_pub.publish(phase_msg)
        if not self._enabled():
            if self._phase in (
                "warmup",
                "climbing",
                "handoff",
                "hovering",
                "tag_centering",
                "object_search",
                "landing",
            ):
                self._request_land()
            return
        if self._phase == "done":
            return
        if self._phase == "landing":
            self._request_land()
            return
        if self._pose is None:
            self._log_once("Waiting for /mavros/local_position/pose")
            return
        pose_stale_timeout = float(
            self.get_parameter("pose_stale_timeout_seconds").value
        )
        if now - self._pose_received_at > pose_stale_timeout:
            # Never request AUTO.TAKEOFF/arm from an old position sample. If
            # takeoff was already accepted, request landing and keep confirming
            # until fresh MAVROS state proves the aircraft is down.
            if self._started_at is None or (
                self._phase == "warmup"
                and not self._takeoff_mode_accepted
                and self._state is not None
                and not self._state.armed
            ):
                self._log_once(
                    "Waiting for fresh /mavros/local_position/pose; not starting flight"
                )
                return
            self.get_logger().error("Pose became stale; requesting AUTO.LAND")
            self._request_land()
            return

        flight_mode = self._selected_mode or str(self.get_parameter("flight_mode").value)
        handoff_enabled = (
            self._handoff_enabled
            if self._handoff_enabled is not None
            else bool(self.get_parameter("handoff_to_offboard").value)
        )
        calibration_enabled = (
            self._calibration_enabled
            if self._calibration_enabled is not None
            else bool(self.get_parameter("calibration_x_step_enabled").value)
        )
        tag_centering_enabled = (
            self._tag_centering_enabled
            if self._tag_centering_enabled is not None
            else bool(self.get_parameter("tag_centering_enabled").value)
        )
        object_search_enabled = (
            self._object_search_enabled
            if self._object_search_enabled is not None
            else bool(self.get_parameter("object_search_enabled").value)
        )
        object_centering_enabled = bool(
            self.get_parameter("object_centering_enabled").value
        )
        object_approach_enabled = bool(
            self.get_parameter("object_approach_enabled").value
        )
        if tag_centering_enabled and (
            flight_mode != "AUTO.TAKEOFF" or not handoff_enabled
        ):
            self.get_logger().error(
                "Tag centering requires flight_mode:=AUTO.TAKEOFF and "
                "handoff_to_offboard:=true; refusing takeoff"
            )
            self._phase = "done"
            return
        if tag_centering_enabled and calibration_enabled:
            self.get_logger().error(
                "Tag centering and fixed X calibration are mutually exclusive; "
                "refusing takeoff"
            )
            self._phase = "done"
            return
        if object_search_enabled and (
            flight_mode != "AUTO.TAKEOFF" or not handoff_enabled
        ):
            self.get_logger().error(
                "YOLO object search requires flight_mode:=AUTO.TAKEOFF and "
                "handoff_to_offboard:=true; refusing takeoff"
            )
            self._phase = "done"
            return
        if object_search_enabled and (tag_centering_enabled or calibration_enabled):
            self.get_logger().error(
                "YOLO object search cannot be combined with tag centering or "
                "X-axis calibration; refusing takeoff"
            )
            self._phase = "done"
            return
        if object_centering_enabled and not object_search_enabled:
            self.get_logger().error(
                "YOLO centering requires object_search_enabled:=true; refusing takeoff"
            )
            self._phase = "done"
            return
        if object_approach_enabled and not object_centering_enabled:
            self.get_logger().error(
                "YOLO approach requires object_centering_enabled:=true; refusing takeoff"
            )
            self._phase = "done"
            return
        if flight_mode == "AUTO.TAKEOFF" and calibration_enabled and not handoff_enabled:
            self.get_logger().error(
                "X-axis calibration requires OFFBOARD or an AUTO.TAKEOFF to "
                "OFFBOARD handoff; refusing takeoff"
            )
            if self._started_at is None:
                self._phase = "done"
            else:
                self._request_land()
            return

        if calibration_enabled and self._started_at is None:
            step = float(self.get_parameter("calibration_x_step_m").value)
            durations = [
                float(self.get_parameter(name).value)
                for name in (
                    "calibration_baseline_seconds",
                    "calibration_step_seconds",
                    "calibration_return_seconds",
                )
            ]
            if not (0.0 < step <= 0.15) or any(not 0.5 <= d <= 5.0 for d in durations):
                self.get_logger().error(
                    "Calibration requires 0 < X step <= 0.15 m and each phase "
                    "duration between 0.5 and 5 s; refusing takeoff"
                )
                self._phase = "done"
                return
        if tag_centering_enabled and self._started_at is None:
            seconds = float(self.get_parameter("tag_centering_seconds").value)
            kp = float(self.get_parameter("tag_centering_kp_m").value)
            deadband = float(self.get_parameter("tag_centering_deadband").value)
            step_limit = float(self.get_parameter("tag_centering_step_limit_m").value)
            max_offset = float(self.get_parameter("tag_centering_max_offset_m").value)
            loss_timeout = float(self.get_parameter("tag_loss_timeout_seconds").value)
            if not (
                3.0 <= seconds <= 10.0
                and 0.1 <= kp <= 0.8
                and 0.01 <= deadband <= 0.10
                and 0.001 <= step_limit <= 0.02
                and step_limit <= max_offset <= 0.15
                and 0.3 <= loss_timeout <= 1.5
            ):
                self.get_logger().error(
                    "Tag-centering limits are outside the allowed ranges; "
                    "refusing takeoff"
                )
                self._phase = "done"
                return
        if object_search_enabled and self._started_at is None:
            duration = float(self.get_parameter("object_search_seconds").value)
            initial_hold = float(
                self.get_parameter("object_search_initial_hold_seconds").value
            )
            yaw_rate = float(
                self.get_parameter("object_search_yaw_rate_rad_s").value
            )
            confidence = float(
                self.get_parameter("object_search_detection_confidence").value
            )
            max_bbox_area = float(
                self.get_parameter("object_search_max_bbox_area").value
            )
            confirmation_count = int(
                self.get_parameter("object_search_confirmation_count").value
            )
            max_acquisition_error = float(
                self.get_parameter("object_search_max_acquisition_error_x").value
            )
            preferred_side = str(
                self.get_parameter("object_search_preferred_side").value
            )
            side_deadband = float(
                self.get_parameter("object_search_side_deadband").value
            )
            hold = float(
                self.get_parameter(
                    "object_search_hold_after_detection_seconds"
                ).value
            )
            if not (
                5.0 <= duration <= 25.0
                and 0.0 <= initial_hold <= 5.0
                and 0.10 <= abs(yaw_rate) <= 0.50
                and 0.10 <= confidence <= 0.80
                and 0.02 <= max_bbox_area <= 0.50
                and 1 <= confirmation_count <= 10
                and 0.10 <= max_acquisition_error <= 1.0
                and 0.5 <= hold <= 4.0
                and preferred_side in ("any", "left", "right")
                and 0.05 <= side_deadband <= 0.50
            ):
                self.get_logger().error(
                    "YOLO search settings are outside safe bounds; refusing takeoff"
                )
                self._phase = "done"
                return
        if object_centering_enabled and self._started_at is None:
            seconds = float(self.get_parameter("object_centering_seconds").value)
            kp = float(self.get_parameter("object_centering_kp_rad").value)
            deadband = float(self.get_parameter("object_centering_deadband").value)
            step_limit = float(
                self.get_parameter("object_centering_step_limit_rad").value
            )
            max_offset = float(
                self.get_parameter("object_centering_max_offset_rad").value
            )
            loss_timeout = float(
                self.get_parameter("object_centering_loss_timeout_seconds").value
            )
            if not (
                2.0 <= seconds <= 6.0
                and 0.05 <= kp <= 0.50
                and 0.02 <= deadband <= 0.20
                and 0.005 <= step_limit <= 0.08
                and step_limit <= max_offset <= 0.50
                and 0.3 <= loss_timeout <= 1.5
            ):
                self.get_logger().error(
                    "YOLO-centering settings are outside safe bounds; refusing takeoff"
                )
                self._phase = "done"
                return
        if object_approach_enabled and self._started_at is None:
            speed = float(self.get_parameter("object_approach_speed_m_s").value)
            growth_ratio = float(
                self.get_parameter("object_approach_size_growth_ratio").value
            )
            center_tolerance = float(
                self.get_parameter("object_approach_center_tolerance").value
            )
            loss_timeout = float(
                self.get_parameter("object_approach_loss_timeout_seconds").value
            )
            timeout = float(
                self.get_parameter("object_approach_timeout_seconds").value
            )
            if not (
                0.02 <= speed <= 0.25
                and 2.0 <= growth_ratio <= 30.0
                and 0.05 <= center_tolerance <= 0.25
                and 0.5 <= loss_timeout <= 2.0
                and 2.0 <= timeout <= 30.0
            ):
                self.get_logger().error(
                    "YOLO approach limits are outside safe bounds; refusing takeoff"
                )
                self._phase = "done"
                return
        if handoff_enabled and self._started_at is None:
            rise = float(self.get_parameter("handoff_rise_m").value)
            height = float(self.get_parameter("height_m").value)
            if flight_mode != "AUTO.TAKEOFF" or not (0.25 <= rise < height <= 0.80):
                self.get_logger().error(
                    "Handoff requires flight_mode:=AUTO.TAKEOFF and "
                    "0.25 <= handoff_rise_m < height_m <= 0.80; refusing takeoff"
                )
                self._phase = "done"
                return
        if self._started_at is None:
            self._selected_mode = flight_mode
            self._handoff_enabled = handoff_enabled
            self._calibration_enabled = calibration_enabled
            self._tag_centering_enabled = tag_centering_enabled
            self._object_approach_enabled = object_approach_enabled
            self._object_search_enabled = object_search_enabled
            self._object_centering_enabled = object_centering_enabled
            self._handoff_rise_m = float(self.get_parameter("handoff_rise_m").value)
            if flight_mode == "AUTO.TAKEOFF":
                self._auto_start_z = self._pose.pose.position.z
            if flight_mode == "OFFBOARD" or handoff_enabled:
                self._target = PoseStamped()
                self._target.header.frame_id = self._pose.header.frame_id or "map"
                self._target.pose.position.x = self._pose.pose.position.x
                self._target.pose.position.y = self._pose.pose.position.y
                self._base_x = self._target.pose.position.x
                self._base_y = self._target.pose.position.y
                self._target.pose.position.z = self._pose.pose.position.z + float(
                    self.get_parameter("height_m").value
                )
                self._target.pose.orientation = self._pose.pose.orientation
            self._started_at = now
            self._phase = "warmup"
            if flight_mode == "OFFBOARD":
                self.get_logger().info(
                    "Hover test enabled: holding X/Y; "
                    f"start_z={self._pose.pose.position.z:.2f} m, "
                    f"target_z={self._target.pose.position.z:.2f} m"
                )
            else:
                self.get_logger().info(
                    "Takeoff test enabled: requesting PX4 AUTO.TAKEOFF before "
                    f"arming; offboard_handoff={handoff_enabled}; "
                    f"x_step_calibration={calibration_enabled}"
                    f"; tag_centering={tag_centering_enabled}; "
                    f"yolo_search={object_search_enabled}"
                )

        # AUTO.TAKEOFF owns the vehicle until the handoff threshold. Sending
        # position setpoints during that mode can make PX4's multicopter
        # position controller report invalid setpoints and trigger blind-land.
        # Start the required OFFBOARD stream only after takeoff has risen far
        # enough; direct OFFBOARD tests still stream from their first tick.
        stream_ready = (
            flight_mode == "OFFBOARD"
            or self._handoff_prestream_started_at is not None
        )
        if self._target is not None and stream_ready:
            assert self._target is not None
            self._target.header.stamp = self.get_clock().now().to_msg()
            self._setpoint_pub.publish(self._target)

        assert self._started_at is not None
        hard_timeout = float(self.get_parameter("hard_timeout_seconds").value)
        # The YOLO experiment has bounded sub-phases (AUTO.TAKEOFF/handoff,
        # up to one search sweep, then up to six seconds of centering). The
        # legacy global deadline was shorter than that complete sequence and
        # could interrupt centering even though each phase was within its own
        # limit. Preserve the existing base watchdog and add only the explicitly
        # enabled perception windows; each still has its own timeout and the
        # landing watchdog remains independent.
        if self._object_search_enabled:
            hard_timeout += float(
                self.get_parameter("object_search_seconds").value
            )
            hard_timeout += float(
                self.get_parameter("object_search_initial_hold_seconds").value
            )
            if self._object_centering_enabled:
                hard_timeout += float(
                    self.get_parameter("object_centering_seconds").value
                )
            if self._object_approach_enabled:
                hard_timeout += float(
                    self.get_parameter("object_approach_timeout_seconds").value
                )
        if now - self._started_at >= hard_timeout:
            self.get_logger().error("Overall test timeout; requesting AUTO.LAND")
            self._request_land()
            return

        if now - self._started_at < float(self.get_parameter("warmup_seconds").value):
            return

        state = self._state
        if state is None or now - self._state_received_at > 0.5 or not state.connected:
            self._log_once("Waiting for /mavros/state")
            return

        if flight_mode == "AUTO.TAKEOFF":
            self._tick_auto_takeoff(now, state)
            return

        if state.mode != "OFFBOARD":
            self._within_altitude_since = None
            if self._calibration_phase is not None:
                self.get_logger().error(
                    "OFFBOARD lost during X-axis calibration; requesting AUTO.LAND"
                )
                self._request_land()
                return
            self._request_offboard(now)
            return
        if not state.armed:
            self._within_altitude_since = None
            if self._calibration_phase is not None:
                self.get_logger().error(
                    "Vehicle disarmed during X-axis calibration; requesting AUTO.LAND"
                )
                self._request_land()
                return
            self._request_arm(now)
            return

        if self._hover_started_at is None:
            if self._climb_started_at is None:
                self._climb_started_at = now
                self._phase = "climbing"
                self.get_logger().info("OFFBOARD armed; checking climb to target altitude")

            current_z = self._pose.pose.position.z
            target_z = self._target.pose.position.z
            altitude_error = target_z - current_z
            tolerance = float(self.get_parameter("altitude_tolerance_m").value)
            if abs(altitude_error) <= tolerance:
                if self._within_altitude_since is None:
                    self._within_altitude_since = now
                elif now - self._within_altitude_since >= 0.5:
                    self._hover_started_at = now
                    self._phase = "hovering"
                    self.get_logger().info(
                        "Target altitude reached; starting timed hold: "
                        f"current_z={current_z:.2f} m, target_z={target_z:.2f} m"
                    )
                    return
            else:
                self._within_altitude_since = None

            if now - self._last_progress_log_at >= 1.0:
                self.get_logger().info(
                    "Climb progress: "
                    f"current_z={current_z:.2f} m, target_z={target_z:.2f} m, "
                    f"remaining={altitude_error:.2f} m, "
                    f"x_error={self._target.pose.position.x - self._pose.pose.position.x:+.2f} m, "
                    f"y_error={self._target.pose.position.y - self._pose.pose.position.y:+.2f} m"
                )
                self._last_progress_log_at = now

            climb_timed_out = now - self._climb_started_at >= float(
                self.get_parameter("climb_timeout_seconds").value
            )
            # If the vehicle first enters the altitude band at the deadline,
            # allow the 0.5 s stability dwell to complete. If it leaves the
            # band after the deadline, _within_altitude_since resets and we
            # land on this tick.
            if climb_timed_out and self._within_altitude_since is None:
                self.get_logger().error(
                    "Climb deadline expired before target altitude was held "
                    "stably; "
                    f"current_z={current_z:.2f} m, target_z={target_z:.2f} m, "
                    f"remaining={altitude_error:.2f} m; requesting AUTO.LAND"
                )
                self._request_land()
            return

        if self._calibration_phase is not None:
            self._tick_calibration(now)
            return

        if now - self._last_progress_log_at >= 1.0:
            current_z = self._pose.pose.position.z
            target_z = self._target.pose.position.z
            self.get_logger().info(
                f"Holding altitude: current_z={current_z:.2f} m, "
                f"target_z={target_z:.2f} m, "
                f"error={target_z - current_z:+.2f} m"
            )
            self._last_progress_log_at = now
        if self._calibration_enabled:
            self._calibration_phase = "baseline"
            self._calibration_phase_started_at = now
            self.get_logger().info(
                "X-axis calibration starting: baseline hold at current X/Y for "
                f"{float(self.get_parameter('calibration_baseline_seconds').value):.1f} s"
            )
            return

        if now - self._hover_started_at >= float(self.get_parameter("hover_seconds").value):
            self.get_logger().info("Timed hold complete; requesting AUTO.LAND")
            self._request_land()

    def _tick_auto_takeoff(self, now: float, state: State) -> None:
        """Use PX4's known-good takeoff mode, then land after a brief hold."""
        assert self._started_at is not None

        if state.mode == "AUTO.LAND":
            extended = self._extended_state
            landed_and_disarmed = (
                not state.armed
                and extended is not None
                and now - self._extended_state_received_at <= 2.0
                and extended.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
            )
            if not landed_and_disarmed:
                self._request_land()
                return
            # PX4 may remain in AUTO.LAND after a completed landing. That is
            # safe to leave only when MAVROS confirms both disarmed and on
            # ground; continue below to request AUTO.TAKEOFF for a new run.
        if not self._takeoff_mode_accepted:
            extended = self._extended_state
            # MAVROS may publish extended_state more slowly than pose/state.
            extended_fresh = (
                extended is not None and now - self._extended_state_received_at <= 2.0
            )
            if not extended_fresh:
                self._log_once("Waiting for fresh /mavros/extended_state before takeoff")
                return
            if extended.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                self.get_logger().error(
                    "AUTO.TAKEOFF test requires a confirmed, landed drone; "
                    f"landed_state={extended.landed_state}. Refusing to start."
                )
                self._phase = "done"
                return
            if state.armed:
                self.get_logger().error(
                    "AUTO.TAKEOFF test requires the drone to be disarmed; refusing to start"
                )
                self._phase = "done"
                return
            if state.mode != "AUTO.TAKEOFF":
                self._request_mode(now, "AUTO.TAKEOFF")
                return
            self._takeoff_mode_accepted = True

        if not state.armed:
            if self._climb_started_at is not None:
                self.get_logger().error("Disarmed during takeoff; ending test")
                if self._handoff_requested_at is not None:
                    self._request_land()
                else:
                    self._phase = "done"
                return
            self._request_arm(now)
            return

        if self._climb_started_at is None:
            self._climb_started_at = now
            self._phase = "climbing"
            self.get_logger().info(
                "AUTO.TAKEOFF armed; waiting for MAVROS to report airborne"
            )

        extended = self._extended_state
        extended_fresh = (
            extended is not None and now - self._extended_state_received_at <= 2.0
        )
        landed_state = extended.landed_state if extended_fresh else None

        if self._handoff_enabled:
            self._tick_handoff(now, state, landed_state)
            return

        # Do not start the hold clock on the transient TAKEOFF state: wait until
        # MAVROS reports IN_AIR, so ascent time is not counted as hover time.
        if landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            if self._hover_started_at is None:
                self._hover_started_at = now
                self._phase = "hovering"
                self.get_logger().info(
                    "Airborne confirmed by /mavros/extended_state; starting "
                    f"{float(self.get_parameter('hover_seconds').value):.1f} s hold"
                )
            elif now - self._hover_started_at >= float(
                self.get_parameter("hover_seconds").value
            ):
                self.get_logger().info("Timed hold complete; requesting AUTO.LAND")
                self._request_land()
            return

        if now - self._last_progress_log_at >= 1.0:
            z = self._pose.pose.position.z if self._pose is not None else float("nan")
            self.get_logger().info(
                "Waiting for airborne confirmation: "
                f"landed_state={landed_state}, local_z={z:.2f} m, "
                f"mode={state.mode}, armed={state.armed}"
            )
            self._last_progress_log_at = now

        if now - self._climb_started_at >= float(
            self.get_parameter("climb_timeout_seconds").value
        ):
            self.get_logger().error(
                "AUTO.TAKEOFF did not report airborne before the climb deadline; "
                "requesting AUTO.LAND"
            )
            self._request_land()

    def _tick_handoff(self, now: float, state: State, landed_state: int | None) -> None:
        """Switch a rising AUTO.TAKEOFF flight to a bounded OFFBOARD hold."""
        assert self._pose is not None and self._auto_start_z is not None
        assert self._climb_started_at is not None and self._target is not None

        if self._handoff_requested_at is None:
            rise = self._pose.pose.position.z - self._auto_start_z
            assert self._handoff_rise_m is not None
            threshold = self._handoff_rise_m
            if landed_state in (
                ExtendedState.LANDED_STATE_TAKEOFF,
                ExtendedState.LANDED_STATE_IN_AIR,
            ) and rise >= threshold:
                if self._handoff_prestream_started_at is None:
                    # Begin a fresh position/yaw target at the measured
                    # airborne X/Y. Keep the precomputed bounded altitude.
                    self._target.pose.position.x = self._pose.pose.position.x
                    self._target.pose.position.y = self._pose.pose.position.y
                    self._target.pose.orientation = self._pose.pose.orientation
                    self._handoff_prestream_started_at = now
                    self._phase = "handoff_prestream"
                    self.get_logger().info(
                        f"Takeoff rise={rise:.2f} m; starting OFFBOARD "
                        "setpoint pre-stream at measured airborne X/Y"
                    )
                    return
                if now - self._handoff_prestream_started_at >= 1.0:
                    self._handoff_requested_at = now
                    self._phase = "handoff"
                    self.get_logger().info(
                        f"OFFBOARD setpoint pre-stream complete; requesting "
                        f"OFFBOARD at target_z={self._target.pose.position.z:.2f} m"
                    )
                    self._request_mode(now, "OFFBOARD")
                return
            if now - self._last_progress_log_at >= 1.0:
                self.get_logger().info(
                    f"AUTO.TAKEOFF rise={rise:.2f} m; waiting for "
                    f"{threshold:.2f} m before OFFBOARD handoff; "
                    f"landed_state={landed_state}"
                )
                self._last_progress_log_at = now
            if now - self._climb_started_at >= float(
                self.get_parameter("climb_timeout_seconds").value
            ):
                self.get_logger().error(
                    "Takeoff did not reach the handoff rise before the deadline; "
                    "requesting AUTO.LAND"
                )
                self._request_land()
            return

        if state.mode != "OFFBOARD":
            if self._calibration_phase is not None or self._phase in (
                "tag_centering",
                "object_search",
            ):
                self.get_logger().error(
                    "OFFBOARD lost during the controller test; requesting AUTO.LAND"
                )
                self._request_land()
                return
            # AUTO.TAKEOFF remains in control during the 1 s OFFBOARD
            # setpoint pre-stream. Do not treat that interval as a failed
            # mode handoff.
            if self._handoff_requested_at is None:
                return
            if now - self._handoff_requested_at >= float(
                self.get_parameter("handoff_timeout_seconds").value
            ):
                self.get_logger().error(
                    "OFFBOARD handoff was not confirmed; requesting AUTO.LAND"
                )
                self._request_land()
            else:
                self._request_mode(now, "OFFBOARD")
            return

        if landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            self.get_logger().error("Ground contact during OFFBOARD hold; requesting AUTO.LAND")
            self._request_land()
            return
        if self._hover_started_at is None:
            self._hover_started_at = now
            self._phase = (
                "object_search"
                if self._object_search_enabled
                else "tag_centering"
                if self._tag_centering_enabled
                else "hovering"
            )
            if self._tag_centering_enabled or self._object_search_enabled:
                # Start from the measured airborne location to avoid snapping
                # back to the ground takeoff coordinates at mode handoff.
                self._target.pose.position.x = self._pose.pose.position.x
                self._target.pose.position.y = self._pose.pose.position.y
                self._base_x = self._pose.pose.position.x
                self._base_y = self._pose.pose.position.y
                self._target.pose.orientation = self._pose.pose.orientation
            if self._tag_centering_enabled:
                self._tag_centering_start_x = self._base_x
                self._tag_centering_started_at = now
                self._last_tag_control_at = now
                # Do not steer from a takeoff-phase image after handing off;
                # wait for a new observation from the stable OFFBOARD phase.
                self._last_tag_error_used_at = self._tag_error_received_at
            if self._object_search_enabled:
                self._object_search_started_at = now
                self._object_search_start_yaw = self._yaw_from_quaternion(
                    self._pose.pose.orientation
                )
                self._object_search_angle_rad = 0.0
                self._object_search_detection_seen_at = None
                # Ignore any detection that arrived before the airborne scan.
                self._object_search_detection_received_at = now
                self._object_search_detection_confidence = None
                self._object_search_error_x = None
                self._object_search_detection_count = 0
                self._last_object_search_detection_at = 0.0
                self._object_search_preferred_target_seen = False
            self.get_logger().info(
                "OFFBOARD handoff confirmed; "
                + (
                    "starting bounded in-place YOLO yaw search; "
                    f"preferred image side="
                    f"{self.get_parameter('object_search_preferred_side').value}"
                    if self._object_search_enabled
                    else "starting bounded tag-centering test"
                    if self._tag_centering_enabled
                    else "holding the "
                    f"{float(self.get_parameter('height_m').value):.2f} m target for "
                    f"{float(self.get_parameter('hover_seconds').value):.1f} s"
                )
            )
            if self._object_search_enabled:
                self._calibration_phase = "object_search"
            elif self._tag_centering_enabled:
                self._calibration_phase = "tag_centering"
            elif self._calibration_enabled:
                self._calibration_phase = "baseline"
                self._calibration_phase_started_at = now
                self.get_logger().info(
                    "X-axis calibration starting after OFFBOARD handoff: "
                    f"baseline {float(self.get_parameter('calibration_baseline_seconds').value):.1f} s, "
                    f"+X step {float(self.get_parameter('calibration_step_seconds').value):.1f} s, "
                    f"return {float(self.get_parameter('calibration_return_seconds').value):.1f} s"
                )
        elif self._object_search_enabled:
            self._tick_object_search(now)
        elif self._tag_centering_enabled:
            self._tick_tag_centering(now)
        elif self._calibration_phase is not None:
            self._tick_calibration(now)
        elif now - self._hover_started_at >= float(
            self.get_parameter("hover_seconds").value
        ):
            self.get_logger().info("Handoff hold complete; requesting AUTO.LAND")
            self._request_land()

    def _tick_tag_centering(self, now: float) -> None:
        """Make slow, bounded local-X position corrections from tag image error."""
        assert self._target is not None
        assert self._tag_centering_started_at is not None
        assert self._tag_centering_start_x is not None

        if now - self._tag_error_received_at > float(
            self.get_parameter("tag_loss_timeout_seconds").value
        ):
            self.get_logger().error(
                "Target AprilTag lost or stale during centering; requesting AUTO.LAND"
            )
            self._request_land()
            return

        duration = float(self.get_parameter("tag_centering_seconds").value)
        if now - self._tag_centering_started_at >= duration:
            self.get_logger().info(
                "Bounded tag-centering interval complete; requesting AUTO.LAND"
            )
            self._request_land()
            return

        # Calibration showed local X+ moves the tag left in the image, so
        # positive image error commands a small positive local-X correction.
        if (
            now - self._last_tag_control_at >= 0.20
            and self._tag_error_x is not None
            and self._tag_error_received_at > self._last_tag_error_used_at
        ):
            error_x = self._tag_error_x
            deadband = float(self.get_parameter("tag_centering_deadband").value)
            if abs(error_x) > deadband:
                kp = float(self.get_parameter("tag_centering_kp_m").value)
                step_limit = float(
                    self.get_parameter("tag_centering_step_limit_m").value
                )
                max_offset = float(
                    self.get_parameter("tag_centering_max_offset_m").value
                )
                delta_x = max(-step_limit, min(step_limit, kp * error_x))
                current_offset = self._target.pose.position.x - self._tag_centering_start_x
                desired_offset = max(
                    -max_offset, min(max_offset, current_offset + delta_x)
                )
                self._target.pose.position.x = (
                    self._tag_centering_start_x + desired_offset
                )
            self._last_tag_control_at = now
            self._last_tag_error_used_at = self._tag_error_received_at

        if now - self._last_progress_log_at >= 1.0:
            offset = self._target.pose.position.x - self._tag_centering_start_x
            self.get_logger().info(
                "Tag-centering: "
                f"error_x={self._tag_error_x:+.3f}, "
                f"local_x_offset={offset:+.3f} m, "
                f"elapsed={now - self._tag_centering_started_at:.1f}/"
                f"{duration:.1f} s"
            )
            self._last_progress_log_at = now

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        """Extract yaw from a ROS quaternion (x, y, z, w)."""
        return atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0
            - 2.0
            * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _tick_object_search(self, now: float) -> None:
        """Hold the initial heading, then yaw-scan until detection or timeout."""
        assert self._target is not None
        assert self._object_search_started_at is not None
        assert self._object_search_start_yaw is not None

        if self._calibration_phase == "object_approach":
            self._tick_object_approach(now)
            return

        elapsed = now - self._object_search_started_at
        confidence_threshold = float(
            self.get_parameter("object_search_detection_confidence").value
        )
        confidence = self._object_search_detection_confidence
        error_x = self._object_search_error_x
        max_acquisition_error = float(
            self.get_parameter("object_search_max_acquisition_error_x").value
        )
        fresh_detection = (
            confidence is not None
            and error_x is not None
            and self._object_search_detection_received_at
            >= self._object_search_started_at
            and now - self._object_search_detection_received_at <= 0.5
            and confidence >= confidence_threshold
            and abs(error_x) <= max_acquisition_error
            and self._object_search_detection_count
            >= int(self.get_parameter("object_search_confirmation_count").value)
        )
        if fresh_detection and self._object_search_detection_seen_at is None:
            self._object_search_detection_seen_at = now
            self._target.pose.orientation = self._pose.pose.orientation
            if self._object_centering_enabled:
                approach_tolerance = float(
                    self.get_parameter("object_approach_center_tolerance").value
                )
                if (
                    self._object_approach_enabled
                    and self._object_search_detection_count
                    >= int(
                        self.get_parameter(
                            "object_search_confirmation_count"
                        ).value
                    )
                    and abs(error_x) <= approach_tolerance
                    and now - self._object_search_detection_received_at <= 0.5
                ):
                    self.get_logger().info(
                        "YOLO target is already freshly centered; skipping the "
                        "extra yaw-centering interval and starting approach"
                    )
                    self._start_object_approach(now)
                    return
                self._object_centering_started_at = now
                self._object_centering_start_yaw = self._yaw_from_quaternion(
                    self._pose.pose.orientation
                )
                self._target.pose.position.x = self._pose.pose.position.x
                self._target.pose.position.y = self._pose.pose.position.y
                self._target.pose.position.z = self._pose.pose.position.z
                # Use the just-confirmed candidate for the initial bounded
                # yaw correction; don't wait for another frame while a target
                # near the image edge could disappear.
                self._last_yolo_detection_used_at = 0.0
                self._last_yolo_control_at = now
                self._phase = "object_centering"
                self._calibration_phase = "object_centering"
                self.get_logger().info(
                    "YOLO found the target: "
                    f"confidence={confidence:.3f}, "
                    f"error_x={self._object_search_error_x:+.3f}; stopping yaw "
                    "and starting bounded image-centering test"
                )
            else:
                self.get_logger().info(
                    "YOLO found the target during yaw search: "
                    f"confidence={confidence:.3f}; stopping rotation and holding "
                    "position briefly"
                )

        if self._object_search_detection_seen_at is not None:
            if self._object_centering_enabled:
                self._tick_object_centering(now)
                return
            hold = float(
                self.get_parameter(
                    "object_search_hold_after_detection_seconds"
                ).value
            )
            if now - self._object_search_detection_seen_at >= hold:
                self.get_logger().info(
                    "YOLO search verification hold complete; requesting AUTO.LAND"
                )
                self._request_land()
                return
        else:
            duration = float(self.get_parameter("object_search_seconds").value)
            initial_hold = float(
                self.get_parameter("object_search_initial_hold_seconds").value
            )
            yaw_rate = float(
                self.get_parameter("object_search_yaw_rate_rad_s").value
            )
            if elapsed >= initial_hold + duration:
                self.get_logger().warning(
                    "YOLO search timed out without a confident target detection; "
                    "requesting AUTO.LAND"
                )
                self._request_land()
                return

            # Preserve the takeoff heading briefly: the sign is already
            # visible/detectable from the grounded camera, and immediately
            # rotating can sweep it out of view before three fresh model
            # observations arrive. If it isn't reacquired, then scan one turn.
            scan_elapsed = max(0.0, elapsed - initial_hold)
            self._object_search_angle_rad = yaw_rate * scan_elapsed
            yaw = self._object_search_start_yaw + self._object_search_angle_rad
            self._target.pose.orientation.x = 0.0
            self._target.pose.orientation.y = 0.0
            self._target.pose.orientation.z = sin(yaw / 2.0)
            self._target.pose.orientation.w = cos(yaw / 2.0)

            # Derive the command from elapsed wall time, not the number of
            # controller callbacks. Clamping each callback's dt to 0.1 s
            # caused a slow/loaded executor to accumulate only half the
            # requested search rotation over a 16 s sweep.

        if now - self._last_progress_log_at >= 1.0:
            self.get_logger().info(
                "YOLO yaw search: "
                f"angle={self._object_search_angle_rad:.2f} rad, "
                f"elapsed={elapsed:.1f} s, "
                f"initial_hold={initial_hold:.1f} s, "
                f"confirmations={self._object_search_detection_count}/"
                f"{int(self.get_parameter('object_search_confirmation_count').value)}, "
                f"latest_confidence={confidence if confidence is not None else 'none'}"
            )
            self._last_progress_log_at = now

    def _tick_object_centering(self, now: float) -> None:
        """Use bounded yaw-only corrections to center a fresh YOLO detection."""
        assert self._target is not None
        assert self._object_centering_started_at is not None
        assert self._object_centering_start_yaw is not None

        loss_timeout = float(
            self.get_parameter("object_centering_loss_timeout_seconds").value
        )
        if now - self._object_search_detection_received_at > loss_timeout:
            self.get_logger().error(
                "YOLO target lost or stale during centering; requesting AUTO.LAND"
            )
            self._request_land()
            return

        duration = float(self.get_parameter("object_centering_seconds").value)
        if now - self._object_centering_started_at >= duration:
            if self._object_approach_enabled:
                self._start_object_approach(now)
            else:
                self.get_logger().info(
                    "Bounded YOLO image-centering interval complete; requesting AUTO.LAND"
                )
                self._request_land()
            return

        received_at = self._object_search_detection_received_at
        confidence_threshold = float(
            self.get_parameter("object_search_detection_confidence").value
        )
        if (
            now - self._last_yolo_control_at >= 0.20
            and received_at > self._last_yolo_detection_used_at
            and self._object_search_error_x is not None
            and self._object_search_detection_confidence is not None
            and self._object_search_detection_confidence >= confidence_threshold
        ):
            error_x = self._object_search_error_x
            deadband = float(self.get_parameter("object_centering_deadband").value)
            if abs(error_x) > deadband:
                kp = float(self.get_parameter("object_centering_kp_rad").value)
                step_limit = float(
                    self.get_parameter("object_centering_step_limit_rad").value
                )
                max_offset = float(
                    self.get_parameter("object_centering_max_offset_rad").value
                )
                # Positive image error means the object is right of image
                # center, so yaw the camera right (negative ROS yaw). The
                # total yaw change is tightly capped pending validation.
                delta_yaw = max(-step_limit, min(step_limit, -kp * error_x))
                current_yaw = self._yaw_from_quaternion(
                    self._target.pose.orientation
                )
                current_offset = atan2(
                    sin(current_yaw - self._object_centering_start_yaw),
                    cos(current_yaw - self._object_centering_start_yaw),
                )
                desired_offset = max(
                    -max_offset,
                    min(max_offset, current_offset + delta_yaw),
                )
                target_yaw = self._object_centering_start_yaw + desired_offset
                self._target.pose.orientation.x = 0.0
                self._target.pose.orientation.y = 0.0
                self._target.pose.orientation.z = sin(target_yaw / 2.0)
                self._target.pose.orientation.w = cos(target_yaw / 2.0)
            self._last_yolo_control_at = now
            self._last_yolo_detection_used_at = received_at

        if now - self._last_progress_log_at >= 1.0:
            yaw_offset = atan2(
                sin(
                    self._yaw_from_quaternion(self._target.pose.orientation)
                    - self._object_centering_start_yaw
                ),
                cos(
                    self._yaw_from_quaternion(self._target.pose.orientation)
                    - self._object_centering_start_yaw
                ),
            )
            self.get_logger().info(
                "YOLO image-centering: "
                f"error_x={self._object_search_error_x:+.3f}, "
                f"confidence={self._object_search_detection_confidence:.3f}, "
                f"yaw_setpoint_offset={yaw_offset:+.3f} rad, "
                f"pose_m=({self._pose.pose.position.x:+.3f}, "
                f"{self._pose.pose.position.y:+.3f}), yaw={self._yaw_from_quaternion(self._pose.pose.orientation):+.2f} rad, "
                f"elapsed={now - self._object_centering_started_at:.1f}/"
                f"{duration:.1f} s"
            )
            self._last_progress_log_at = now

    def _start_object_approach(self, now: float) -> None:
        """Begin a very short, level, forward-only approach after centering."""
        assert self._target is not None and self._pose is not None
        self._object_approach_started_at = now
        self._object_approach_yaw = self._yaw_from_quaternion(
            self._pose.pose.orientation
        )
        self._object_approach_start_yaw = self._object_approach_yaw
        self._object_approach_distance_m = 0.0
        self._object_approach_last_update_at = now
        self._target.pose.position.x = self._pose.pose.position.x
        self._target.pose.position.y = self._pose.pose.position.y
        center_tolerance = float(
            self.get_parameter("object_approach_center_tolerance").value
        )
        if (
            self._object_search_detection_count
            < int(self.get_parameter("object_search_confirmation_count").value)
            or self._object_search_error_x is None
            or abs(self._object_search_error_x) > center_tolerance
            or now - self._object_search_detection_received_at > 0.5
        ):
            self.get_logger().warning(
                "YOLO target was not freshly centered at approach handoff; "
                "refusing forward motion and requesting AUTO.LAND"
            )
            self._request_land()
            return
        self._object_approach_initial_bbox_area = self._object_search_bbox_area
        if (
            self._object_approach_initial_bbox_area is None
            or self._object_approach_initial_bbox_area <= 0.0
        ):
            self.get_logger().warning(
                "YOLO approach has no valid initial box size; refusing forward motion"
            )
            self._request_land()
            return
        max_detection_area = float(
            self.get_parameter("object_search_max_bbox_area").value
        )
        self._object_approach_stop_bbox_area = min(
            self._object_approach_initial_bbox_area
            * float(self.get_parameter("object_approach_size_growth_ratio").value),
            max_detection_area * 0.85,
        )
        self._calibration_phase = "object_approach"
        self.get_logger().info(
            "YOLO image-centering window complete; starting visual-size approach: "
            f"speed<={float(self.get_parameter('object_approach_speed_m_s').value):.2f} m/s, "
            f"initial_bbox_area={self._object_approach_initial_bbox_area:.4f}, "
            "stop when the centered box grows "
            f"{self._object_approach_stop_bbox_area / self._object_approach_initial_bbox_area:.1f}x; "
            "25 s safety timeout, altitude held, target-loss lands"
        )

    def _tick_object_approach(self, now: float) -> None:
        """Step forward cautiously while YOLO remains fresh and centered."""
        assert self._target is not None
        assert self._object_approach_started_at is not None
        assert self._object_approach_yaw is not None

        elapsed = now - self._object_approach_started_at
        timeout = float(self.get_parameter("object_approach_timeout_seconds").value)
        loss_timeout = float(
            self.get_parameter("object_approach_loss_timeout_seconds").value
        )
        if elapsed >= timeout:
            self.get_logger().warning(
                "YOLO approach time limit reached; requesting AUTO.LAND"
            )
            self._request_land()
            return
        if now - self._object_search_detection_received_at > loss_timeout:
            self.get_logger().error(
                "YOLO target lost during approach; holding position and requesting AUTO.LAND"
            )
            self._request_land()
            return

        area = self._object_search_bbox_area
        initial_area = self._object_approach_initial_bbox_area
        if initial_area is None or initial_area <= 0.0:
            self.get_logger().error(
                "YOLO approach lost its initial size reference; requesting AUTO.LAND"
            )
            self._request_land()
            return
        size_growth_ratio = float(
            self.get_parameter("object_approach_size_growth_ratio").value
        )
        stop_area = self._object_approach_stop_bbox_area
        if stop_area is None or stop_area <= 0.0:
            self.get_logger().error(
                "YOLO approach lost its visual-size goal; requesting AUTO.LAND"
            )
            self._request_land()
            return

        # Continue visual yaw alignment during translation. A small residual
        # image error at the centering/approach handoff can grow as the drone
        # moves; holding position alone would otherwise stall the approach.
        received_at = self._object_search_detection_received_at
        confidence = self._object_search_detection_confidence
        confidence_threshold = float(
            self.get_parameter("object_search_detection_confidence").value
        )
        if (
            received_at > self._last_yolo_detection_used_at
            and now - received_at <= 0.5
            and self._object_search_error_x is not None
            and confidence is not None
            and confidence >= confidence_threshold
            and self._object_search_detection_count
            >= int(self.get_parameter("object_search_confirmation_count").value)
        ):
            error_x = self._object_search_error_x
            deadband = float(self.get_parameter("object_centering_deadband").value)
            if abs(error_x) > deadband:
                assert self._object_approach_start_yaw is not None
                kp = float(self.get_parameter("object_centering_kp_rad").value)
                step_limit = float(
                    self.get_parameter("object_centering_step_limit_rad").value
                )
                max_offset = float(
                    self.get_parameter("object_centering_max_offset_rad").value
                )
                delta_yaw = max(-step_limit, min(step_limit, -kp * error_x))
                current_yaw = self._yaw_from_quaternion(
                    self._target.pose.orientation
                )
                current_offset = atan2(
                    sin(current_yaw - self._object_approach_start_yaw),
                    cos(current_yaw - self._object_approach_start_yaw),
                )
                desired_offset = max(
                    -max_offset,
                    min(max_offset, current_offset + delta_yaw),
                )
                self._object_approach_yaw = (
                    self._object_approach_start_yaw + desired_offset
                )
                self._target.pose.orientation.x = 0.0
                self._target.pose.orientation.y = 0.0
                self._target.pose.orientation.z = sin(
                    self._object_approach_yaw / 2.0
                )
                self._target.pose.orientation.w = cos(
                    self._object_approach_yaw / 2.0
                )
            self._last_yolo_detection_used_at = received_at
        if area is not None and area >= stop_area:
            confirmations = int(
                self.get_parameter("object_search_confirmation_count").value
            )
            cue_is_confirmed = (
                self._object_search_detection_count >= confirmations
                and now - self._object_search_detection_received_at <= 0.5
                and self._object_search_error_x is not None
                and abs(self._object_search_error_x)
                <= float(
                    self.get_parameter("object_approach_center_tolerance").value
                )
            )
            if cue_is_confirmed:
                self.get_logger().info(
                    f"YOLO target reached confirmed relative-size goal "
                    f"({area / initial_area:.1f}x >= {size_growth_ratio:.1f}x initial box area); "
                    "requesting AUTO.LAND"
                )
                self._request_land()
                return
            # The box is large but the estimate is not yet stable/centered.
            # Hold rather than taking another step closer.
            if now - self._last_progress_log_at >= 1.0:
                self.get_logger().info(
                    "YOLO approach holding for stable centered size cue: "
                    f"bbox_area={area:.4f}, "
                    f"confirmations={self._object_search_detection_count}/"
                    f"{confirmations}, error_x={self._object_search_error_x}"
                )
                self._last_progress_log_at = now
            return

        fresh_and_centered = (
            self._object_search_detection_count
            >= int(self.get_parameter("object_search_confirmation_count").value)
            and self._object_search_error_x is not None
            and abs(self._object_search_error_x)
            <= float(self.get_parameter("object_approach_center_tolerance").value)
            and now - self._object_search_detection_received_at <= 0.5
        )
        dt = min(max(0.0, now - self._object_approach_last_update_at), 0.1)
        self._object_approach_last_update_at = now
        if not fresh_and_centered:
            # Freeze the moving setpoint whenever the target is stale or
            # horizontally off-center. Target-loss timeout then lands.
            if now - self._last_progress_log_at >= 1.0:
                self.get_logger().info(
                    "YOLO approach holding for fresh centered target: "
                    f"confirmations={self._object_search_detection_count}/"
                    f"{int(self.get_parameter('object_search_confirmation_count').value)}, "
                    f"error_x={self._object_search_error_x}, "
                    f"elapsed={elapsed:.1f}/{timeout:.1f} s"
                )
                self._last_progress_log_at = now
            return

        speed_limit = float(
            self.get_parameter("object_approach_speed_m_s").value
        )
        # Reduce forward speed as the relative-size goal approaches. Position
        # setpoints advance continuously; there is no arbitrary step count or
        # fixed travel distance.
        size_fraction = min(1.0, max(0.0, (area or 0.0) / stop_area))
        speed = speed_limit * (1.0 - 0.8 * size_fraction)
        distance = speed * dt
        self._target.pose.position.x += distance * cos(self._object_approach_yaw)
        self._target.pose.position.y += distance * sin(self._object_approach_yaw)
        self._object_approach_distance_m += distance
        if now - self._last_progress_log_at >= 1.0:
            self.get_logger().info(
                f"YOLO approach: speed={speed:.3f} m/s, "
                f"distance={self._object_approach_distance_m:.2f} m, "
                f"size_growth={(area or 0.0) / initial_area:.1f}x/"
                f"{stop_area / initial_area:.1f}x, "
                f"error_x={self._object_search_error_x:+.3f}, "
                f"elapsed={elapsed:.1f}/{timeout:.1f} s"
            )
            self._last_progress_log_at = now

    def _tick_calibration(self, now: float) -> None:
        """Run baseline -> small +X step -> return, then land."""
        assert self._target is not None
        assert self._base_x is not None and self._base_y is not None
        assert self._calibration_phase_started_at is not None

        elapsed = now - self._calibration_phase_started_at
        if self._calibration_phase == "baseline":
            duration = float(self.get_parameter("calibration_baseline_seconds").value)
            if elapsed >= duration:
                step = abs(float(self.get_parameter("calibration_x_step_m").value))
                if step <= 0.0 or step > 0.15:
                    self.get_logger().error(
                        "Calibration X step must be >0 and <=0.15 m; requesting AUTO.LAND"
                    )
                    self._request_land()
                    return
                self._target.pose.position.x = self._base_x + step
                self._calibration_phase = "step"
                self._calibration_phase_started_at = now
                self.get_logger().info(
                    f"X-axis calibration step: local map X +{step:.2f} m; "
                    f"target_x={self._target.pose.position.x:.2f} m, "
                    f"target_y={self._base_y:.2f} m"
                )
            return

        if self._calibration_phase == "step":
            duration = float(self.get_parameter("calibration_step_seconds").value)
            if elapsed >= duration:
                self._target.pose.position.x = self._base_x
                self._target.pose.position.y = self._base_y
                self._calibration_phase = "return"
                self._calibration_phase_started_at = now
                self.get_logger().info(
                    "X-axis calibration return: restoring original local map X/Y "
                    f"({self._base_x:.2f}, {self._base_y:.2f}) m"
                )
            return

        if self._calibration_phase == "return":
            duration = float(self.get_parameter("calibration_return_seconds").value)
            if elapsed >= duration:
                self.get_logger().info(
                    "X-axis calibration complete; requesting AUTO.LAND"
                )
                self._request_land()

    def _request_offboard(self, now: float) -> None:
        self._request_mode(now, "OFFBOARD")

    def _request_mode(self, now: float, mode: str) -> None:
        if now - self._last_request_at < 1.0 or not self._mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.custom_mode = mode
        self._mode_client.call_async(request)
        self._last_request_at = now
        self.get_logger().info(f"Requesting {mode} mode")

    def _request_arm(self, now: float) -> None:
        if now - self._last_request_at < 1.0 or not self._arm_client.service_is_ready():
            return
        request = CommandBool.Request()
        request.value = True
        self._arm_client.call_async(request)
        self._last_request_at = now
        self.get_logger().info("Requesting arm")

    def _request_land(self) -> None:
        if self._phase == "done":
            return
        now = monotonic()
        if self._phase != "landing":
            self._phase = "landing"
            self._calibration_phase = "landing"
            self._landing_started_at = now
            self._landing_disarmed_since = None
            self.get_logger().info("Landing requested; waiting for landed and disarmed state")

        state_fresh = (
            self._state is not None
            and now - self._state_received_at <= 2.0
            and self._landing_started_at is not None
            and self._state_received_at >= self._landing_started_at
        )
        extended_fresh = (
            self._extended_state is not None
            and now - self._extended_state_received_at <= 2.0
            and self._landing_started_at is not None
            and self._extended_state_received_at >= self._landing_started_at
        )
        landed = (
            extended_fresh
            and self._extended_state.landed_state
            == ExtendedState.LANDED_STATE_ON_GROUND
        )
        disarmed_stable = (
            self._landing_disarmed_since is not None
            and now - self._landing_disarmed_since >= 1.0
        )
        disarmed = state_fresh and not self._state.armed and disarmed_stable
        if landed and disarmed:
            self._phase = "done"
            self._calibration_phase = None
            self.get_logger().info("Landed and disarmed confirmed; hover test finished")
            return

        mode_is_land = self._state is not None and self._state.mode == "AUTO.LAND"
        if (
            not mode_is_land
            and now - self._last_request_at >= 1.0
            and self._mode_client.service_is_ready()
        ):
            request = SetMode.Request()
            request.custom_mode = "AUTO.LAND"
            self._mode_client.call_async(request)
            self._last_request_at = now
            self.get_logger().info("Requesting AUTO.LAND")

        assert self._landing_started_at is not None
        if (
            now - self._landing_started_at
            >= float(self.get_parameter("landing_timeout_seconds").value)
            and not self._landing_timeout_reported
        ):
            self.get_logger().error(
                "Landing was not confirmed within the timeout; "
                f"armed={None if self._state is None else self._state.armed}, "
                "landed_state="
                f"{None if self._extended_state is None else self._extended_state.landed_state}"
            )
            self._landing_timeout_reported = True

    def _log_once(self, message: str) -> None:
        if self._last_wait_message != message:
            self._last_wait_message = message
            self.get_logger().info(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafeHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._request_land()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
