"""Compact YOLO-guided takeoff, visual approach, and landing node.

The controller uses PX4 AUTO.TAKEOFF, then a streamed MAVROS local-position
setpoint in OFFBOARD. YOLO supplies image-space target center and box size;
there is no metric range estimate. The approach advances continuously at a
bounded speed and lands when the centered box reaches a relative-size cue.
"""

from __future__ import annotations

from math import asin, atan2, cos, isfinite, sin
from time import monotonic, time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Header, String

from drone_vision.vision_lib import resolve_specified


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def centering_factor(error: float, tolerance: float, max_error: float) -> float:
    if max_error <= tolerance:
        return 0.0
    return clamp((max_error - abs(error)) / (max_error - tolerance), 0.0, 1.0)


def yaw_of(q) -> float:
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pitch_of(q) -> float:
    return asin(clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0))


def set_yaw(pose: PoseStamped, yaw: float) -> None:
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = sin(yaw / 2.0)
    pose.pose.orientation.w = cos(yaw / 2.0)


class SafeHover(Node):
    """One explicitly enabled, bounded visual-approach flight in Duckiematrix."""

    def __init__(self) -> None:
        super().__init__("safe_hover")
        defaults = {
            "enabled": False,
            "flight_mode": "AUTO.TAKEOFF",
            "height_m": 0.60,
            "hover_seconds": 2.0,
            "climb_timeout_seconds": 150.0,
            "warmup_seconds": 2.0,
            # Covers the slow ground spool before the search, centering, and
            # approach budgets. A shorter cap expired during the search.
            "hard_timeout_seconds": 170.0,
            "landing_timeout_seconds": 40.0,
            "pose_stale_timeout_seconds": 1.5,
            "handoff_to_offboard": True,
            "handoff_rise_m": 0.40,
            # The simulator acknowledges a mode change a few seconds late.
            "handoff_timeout_seconds": 12.0,
            "object_search_enabled": True,
            # Used only when the requested object was never seen. A negative
            # rate turns toward a target on the right. A sighting, including
            # one from before takeoff, replaces this with a yaw toward that
            # image error so a stop sign on the left is not swept away.
            "object_search_seconds": 40.0,
            "object_search_initial_hold_seconds": 0.5,
            "object_search_yaw_rate_rad_s": -0.31,
            "object_search_detection_confidence": 0.08,
            "object_search_max_bbox_area": 0.18,
            "object_search_confirmation_count": 2,
            # The stop sign is still a reliable detection well off center.
            # Acquiring it there avoids a blind yaw that carries it out of frame.
            "object_search_max_acquisition_error_x": 0.80,
            # Unguided search ignores a box this far above or below center.
            # A requested object skips this gate; the stop sign is near the
            # top of the forward camera even when it is horizontally centered.
            "object_search_max_abs_error_y": 0.80,
            "object_tracking_max_error_x_delta": 0.30,
            "object_tracking_max_error_y_delta": 0.50,
            "object_tracking_max_area_ratio": 4.0,
            "object_centering_enabled": True,
            # The simulator yaws slowly, so centering needs a longer window
            # and a larger yaw step before the box is inside the approach gate.
            "object_centering_seconds": 25.0,
            "object_centering_kp_rad": 1.50,
            "object_centering_deadband": 0.08,
            # A half-radian step yaws a narrow sign out of the fisheye frame
            # before the next detection can correct it.
            "object_centering_step_limit_rad": 0.20,
            "object_centering_max_offset_rad": 1.20,
            "object_centering_loss_timeout_seconds": 4.0,
            # One in-tolerance frame is not enough; the box has to stay
            # centered before the drone starts moving toward it.
            "object_centering_hold_seconds": 1.5,
            "object_approach_enabled": True,
            "object_approach_speed_m_s": 0.08,
            "object_approach_size_growth_ratio": 2.5,
            "object_approach_center_tolerance": 0.20,
            "object_approach_max_error_x": 0.45,
            # A forward camera loses a ground object out the bottom of the
            # frame once the drone is over it. That is the arrival cue.
            "object_approach_arrival_error_y": 0.75,
            # The stop sign flickers below the detector threshold for a few
            # seconds. These waits hold position instead of treating that gap
            # as a lost target. They do not command forward motion.
            "object_approach_loss_timeout_seconds": 5.0,
            "object_approach_reacquire_seconds": 8.0,
            "object_approach_reacquire_max_error_x": 0.80,
            "object_approach_timeout_seconds": 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        status_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose: PoseStamped | None = None
        self.pose_at = self.state_at = self.ext_at = self.vision_at = 0.0
        self.state: State | None = None
        self.estimator: EstimatorStatus | None = None
        self.ext: ExtendedState | None = None
        self.target: PoseStamped | None = None
        self.phase = "idle"
        self.flight_phase_pub = self.create_publisher(
            String, "/drone_vision/calibration_phase", 10
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_cb,
            qos,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            status_qos,
        )
        self.create_subscription(
            EstimatorStatus,
            "/mavros/estimator_status",
            self._estimator_cb,
            status_qos,
        )
        self.create_subscription(
            ExtendedState,
            "/mavros/extended_state",
            self._ext_cb,
            qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "/drone_vision/yolo_detection",
            self._detection_cb,
            10,
        )
        self.create_subscription(Header, "/drone_vision/yolo_frame", self._vision_cb, 10)
        self.create_subscription(String, "/drone_vision/flight_request", self._flight_request_cb, 10)
        self.flight_status_pub = self.create_publisher(String, "/drone_vision/flight_status", 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            qos,
        )
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self.pose_stream_requested = False

        self.started = self.climb_started = self.handoff_started = None
        self.last_climb_tick = 0.0
        self.prestream_started = self.airborne_started = self.search_started = None
        self.center_started = self.approach_started = self.landing_started = None
        self.center_hold_since = None
        self.reacquire_started = None
        self.disarmed_at = None
        self.start_z = self.search_yaw = self.center_yaw = None
        self.approach_start_yaw = self.approach_yaw = None
        self.reacquire_yaw = None
        self.approach_reacquire_attempted = False
        self.initial_area = self.stop_area = None
        self.approach_distance = 0.0
        self.last_request = self.last_log = self.last_approach_tick = 0.0
        self.last_detection_at = self.last_valid_detection_at = 0.0
        self.last_control_detection_at = 0.0
        self.detection_count = 0
        self.target_class_id: int | None = None
        self.requested_class_id: int | None = None
        self.search_hint_error: float | None = None
        self.hint_applied_error: float | None = None
        self.hint_nudges = 0
        self.hint_nudge_at = 0.0
        self.flight_label = ""
        self._pending_flight = ""
        self.det_candidate_class_id: int | None = None
        self.det = None  # confidence, error_x, error_y, normalized box area, class_id
        self.ground_error_y = None
        self.ground_area = None
        self.body_rose = False
        self.pose_gap_retry_at = 0.0
        self.found_at = None
        self.arm_requested = False
        self.land_timeout_logged = False
        self.wait_message = None

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "Safe hover disabled by default. AUTO.TAKEOFF + OFFBOARD handoff, "
            "YOLO in-place search/centering, continuous image-size approach, "
            "and AUTO.LAND are opt-in; no arbitrary approach step count."
        )

    def _pose_cb(self, msg: PoseStamped) -> None:
        self.pose, self.pose_at = msg, monotonic()

    def _state_cb(self, msg: State) -> None:
        self.state, self.state_at = msg, monotonic()
        if self.phase == "landing":
            if msg.armed:
                self.disarmed_at = None
            elif self.disarmed_at is None:
                self.disarmed_at = self.state_at

    def _estimator_cb(self, msg: EstimatorStatus) -> None:
        self.estimator = msg

    def _estimator_healthy(self) -> bool:
        est = self.estimator
        return est is not None and (
            est.attitude_status_flag
            and est.velocity_horiz_status_flag
            and est.velocity_vert_status_flag
            and est.pos_horiz_rel_status_flag
            and not est.gps_glitch_status_flag
            and not est.accel_error_status_flag
        )

    def _ext_cb(self, msg: ExtendedState) -> None:
        self.ext, self.ext_at = msg, monotonic()

    def _vision_cb(self, msg: Header) -> None:
        capture_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        age = time() - capture_time
        if capture_time > 0.0 and -0.2 <= age <= 2.0:
            self.vision_at = monotonic()

    def _detection_cb(self, msg: Float32MultiArray) -> None:
        # yolo_detector wire format: class, confidence, cx, cy, ex, ey, w, h.
        if len(msg.data) < 8:
            return
        class_id, confidence, ex, ey, width, height = map(
            float, (msg.data[0], msg.data[1], msg.data[4], msg.data[5], msg.data[6], msg.data[7])
        )
        now = monotonic()
        if not all(map(isfinite, (class_id, confidence, ex, ey, width, height))):
            self.detection_count = 0
            return
        if class_id < 0 or class_id != int(class_id):
            self.detection_count = 0
            return
        if self.requested_class_id is not None and int(class_id) != self.requested_class_id:
            return
        area = width * height
        # Once a target is locked, follow its continuous YOLO box even if the
        # classifier changes labels as viewpoint/scale changes. Reject jumps
        # to unrelated boxes using image-space displacement and area ratio.
        if self.target_class_id is not None and int(class_id) != self.target_class_id:
            if self.det is None:
                return
            area_ratio = area / max(self.det[3], 1e-9)
            if (abs(ex - self.det[1]) > float(self._p("object_tracking_max_error_x_delta"))
                    or abs(ey - self.det[2]) > float(self._p("object_tracking_max_error_y_delta"))
                    or area_ratio > float(self._p("object_tracking_max_area_ratio"))
                    or area_ratio < 1.0 / float(self._p("object_tracking_max_area_ratio"))):
                return
            class_id = float(self.target_class_id)
        valid = (0.0 <= confidence <= 1.0 and confidence >= float(self.get_parameter("object_search_detection_confidence").value)
                 and -1.0 <= ex <= 1.0 and -1.0 <= ey <= 1.0
                 and 0.0 < width <= 1.0 and 0.0 < height <= 1.0
                 and area <= float(self.get_parameter("object_search_max_bbox_area").value))
        if not valid:
            self.detection_count = 0
            return
        # Inference in this simulator often arrives slower than 2 Hz.
        # A shorter gap reset the confirmation count before it could reach
        # the acquisition threshold, so the scan yawed past the duckie.
        if now - self.last_detection_at > 1.5 or self.det_candidate_class_id != int(class_id):
            self.detection_count = 0
        self.det_candidate_class_id = int(class_id)
        self.detection_count += 1
        self.last_detection_at = self.last_valid_detection_at = now
        self.det = (confidence, ex, ey, area, int(class_id))
        self.search_hint_error = ex

    def _flight_request_cb(self, message: String) -> None:
        self._pending_flight = message.data.strip()

    def _say_flight(self, text: str) -> None:
        status = String()
        status.data = text
        self.flight_status_pub.publish(status)
        self.get_logger().info(text)

    def _clear_mission(self) -> None:
        self.started = self.climb_started = self.handoff_started = None
        self.last_climb_tick = 0.0
        self.prestream_started = self.airborne_started = self.search_started = None
        self.center_started = self.approach_started = self.landing_started = None
        self.center_hold_since = None
        self.reacquire_started = None
        self.disarmed_at = None
        self.start_z = self.search_yaw = self.center_yaw = None
        self.approach_start_yaw = self.approach_yaw = None
        self.reacquire_yaw = None
        self.approach_reacquire_attempted = False
        self.initial_area = self.stop_area = None
        self.approach_distance = 0.0
        self.last_request = self.last_log = self.last_approach_tick = 0.0
        self.last_detection_at = self.last_valid_detection_at = 0.0
        self.last_control_detection_at = 0.0
        self.detection_count = 0
        self.target_class_id = None
        self.det_candidate_class_id = None
        self.det = None
        self.ground_error_y = None
        self.ground_area = None
        self.body_rose = False
        self.pose_gap_retry_at = 0.0
        self.search_hint_error = None
        self.hint_applied_error = None
        self.hint_nudges = 0
        self.hint_nudge_at = 0.0
        self.pose_stream_requested = False
        self.found_at = None
        self.arm_requested = False
        self.land_timeout_logged = False
        self.wait_message = None

    def _apply_flight_request(self) -> None:
        label = self._pending_flight
        if not label:
            return
        self._pending_flight = ""
        spec = resolve_specified(label)
        if spec is None:
            self._say_flight("Unknown object. Choose duckie or stop sign.")
            return
        if self.state is not None and self.state.armed:
            self._say_flight(f"Cannot fly to {spec['label']} while the drone is armed.")
            return
        if self.phase not in ("idle", "done"):
            self._say_flight(f"Cannot fly to {spec['label']} until the current flight finishes.")
            return
        if (
            self.ext is not None
            and monotonic() - self.ext_at < 2.0
            and self.ext.landed_state != ExtendedState.LANDED_STATE_ON_GROUND
        ):
            self._say_flight(f"Cannot fly to {spec['label']} until the drone is on the ground.")
            return
        self.requested_class_id = int(spec["class_id"])
        self.flight_label = spec["label"]
        self._clear_mission()
        self.phase = "idle"
        self.set_parameters([Parameter("enabled", Parameter.Type.BOOL, True)])
        self._say_flight(f"Flying to {spec['label']}")

    def _request_pose_stream(self) -> None:
        # A fresh PX4 onboard link does not publish local position until asked.
        # Without it the takeoff pose goes stale and the flight lands immediately.
        if self.pose_stream_requested or not self.command_client.service_is_ready():
            return
        for message_id in (32, 30):
            request = CommandLong.Request()
            request.command = 511
            request.param1 = float(message_id)
            request.param2 = 50000.0
            self.command_client.call_async(request)
        self.pose_stream_requested = True
        self.get_logger().info("Requested 20 Hz local position and attitude")

    def _p(self, key: str):
        return self.get_parameter(key).value

    def _log_once(self, message: str) -> None:
        if message != self.wait_message:
            self.wait_message = message
            self.get_logger().info(message)

    def _tick(self) -> None:
        now = monotonic()
        self._apply_flight_request()
        phase_msg = String()
        phase_msg.data = self.phase
        self.flight_phase_pub.publish(phase_msg)
        if bool(self._p("enabled")) and self.phase not in ("done", "landing"):
            self._request_pose_stream()
        if not bool(self._p("enabled")):
            if self.phase not in ("idle", "done"):
                self._land()
            return
        if self.phase == "done":
            return
        if self.phase == "landing":
            self._land()
            return
        if self.pose is None:
            self._log_once("Waiting for /mavros/local_position/pose")
            return
        stale = float(self._p("pose_stale_timeout_seconds"))
        if now - self.pose_at > stale:
            # The same 1.5 s limit still lands once the drone has actually climbed.
            # Before that, a slow simulator frame waits instead of aborting takeoff.
            risen = (
                self.start_z is not None
                and self.pose is not None
                and self.pose.pose.position.z - float(self.start_z) >= 0.15
            )
            if not risen:
                self._log_once("Waiting for fresh MAVROS pose")
                return
            # One extra request at the takeoff handoff. A single dropped
            # position sample there was landing a climb the camera had confirmed.
            if self.pose_gap_retry_at == 0.0 and self.phase in ("prestream", "handoff", "search"):
                self.pose_gap_retry_at = now
                self.pose_stream_requested = False
                self._request_pose_stream()
                self.get_logger().warning("Pose gap after handoff; requesting the position stream again")
                return
            if 0.0 < self.pose_gap_retry_at and now - self.pose_gap_retry_at < 2.0:
                return
            self.get_logger().error("MAVROS pose stale; requesting AUTO.LAND")
            self._land()
            return

        if now - self.vision_at > 2.0:
            if self.started is None:
                self._log_once("Waiting for YOLO inference on fresh camera frames")
            else:
                self.get_logger().error("YOLO camera stream stale; requesting AUTO.LAND")
                self._land()
            return

        if not self._estimator_healthy():
            if self.started is None:
                self._log_once("Waiting for healthy PX4 estimator")
            else:
                self.get_logger().error("PX4 estimator unhealthy; requesting AUTO.LAND")
                self._land()
            return

        if self.started is None:
            if not self._validate():
                self.phase = "done"
                return
            self.started = now
            self.start_z = self.pose.pose.position.z
            self.target = PoseStamped()
            self.target.header.frame_id = self.pose.header.frame_id or "map"
            self.target.pose = self.pose.pose
            self.target.pose.position.z = self.start_z + float(self._p("handoff_rise_m"))
            self.phase = "warmup"
            self.get_logger().info(
                f"Mission enabled from current pose z={self.start_z:.2f} m; "
                f"takeoff handoff at +{float(self._p('handoff_rise_m')):.2f} m"
            )

        assert self.started is not None and self.target is not None
        total = (
            float(self._p("hard_timeout_seconds"))
            + float(self._p("object_search_seconds"))
            + float(self._p("object_centering_seconds"))
            + float(self._p("object_approach_timeout_seconds"))
            + float(self._p("object_approach_reacquire_seconds"))
        )
        if now - self.started > total:
            self.get_logger().error("Mission watchdog expired; landing")
            self._land()
            return
        # AUTO.TAKEOFF owns the climb. Stream setpoints only once that climb
        # has reached the handoff height.
        if self.phase in ("prestream", "handoff", "search", "center", "approach", "reacquire"):
            self.target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(self.target)

        if now - self.started < float(self._p("warmup_seconds")):
            return
        if self.state is None or not self.state.connected:
            self._log_once("Waiting for connected /mavros/state")
            return
        if self.phase in ("warmup", "takeoff"):
            self._takeoff(now)
        elif self.phase == "prestream":
            self._handoff(now)
        elif self.phase == "handoff":
            self._handoff(now)
        elif self.phase == "search":
            self._search(now)
        elif self.phase == "center":
            self._center(now)
        elif self.phase == "approach":
            self._approach(now)
        elif self.phase == "reacquire":
            self._reacquire(now)

    def _validate(self) -> bool:
        checks = [
            (self._p("flight_mode") == "AUTO.TAKEOFF", "flight_mode must be AUTO.TAKEOFF"),
            (bool(self._p("handoff_to_offboard")), "handoff_to_offboard must be enabled"),
            (0.25 <= float(self._p("handoff_rise_m")) < float(self._p("height_m")) <= 0.80, "require 0.25 <= handoff rise < height <= 0.80 m"),
            (bool(self._p("object_search_enabled")), "object_search_enabled must be enabled"),
            (bool(self._p("object_centering_enabled")), "object_centering_enabled must be enabled"),
            (bool(self._p("object_approach_enabled")), "object_approach_enabled must be enabled"),
            (0.02 <= float(self._p("object_approach_speed_m_s")) <= 0.25, "approach speed must be 0.02–0.25 m/s"),
            (2.0 <= float(self._p("object_approach_size_growth_ratio")) <= 30.0, "visual size-growth ratio must be 2–30"),
            (float(self._p("object_approach_center_tolerance")) < float(self._p("object_approach_max_error_x")) <= 1.0, "approach max image error must exceed center tolerance and be at most 1.0"),
        ]
        for ok, error in checks:
            if not ok:
                self.get_logger().error(f"Refusing takeoff: {error}")
                return False
        return True

    def _call_mode(self, now: float, mode: str) -> None:
        if now - self.last_request < 1.0 or not self.mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.custom_mode = mode
        self.mode_client.call_async(request)
        self.last_request = now
        self.get_logger().info(f"Requesting {mode}")

    def _takeoff(self, now: float) -> None:
        assert self.state is not None and self.started is not None and self.target is not None
        ext_fresh = self.ext is not None and now - self.ext_at < 2.0
        if not ext_fresh:
            self._log_once("Waiting for fresh /mavros/extended_state before takeoff")
            return
        if self.state.armed is False and self.ext.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            self.get_logger().error(f"Not confirmed on ground (landed_state={self.ext.landed_state}); refusing takeoff")
            self.phase = "done"
            return
        if self.state.armed and self.climb_started is None and not self.arm_requested:
            self.get_logger().error("Already armed before test; refusing to take control")
            self.phase = "done"
            return
        if not self.state.armed:
            if self.state.mode != "AUTO.TAKEOFF":
                self._call_mode(now, "AUTO.TAKEOFF")
                return
            if now - self.last_request >= 1.0 and self.arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
                self.last_request = now
                self.arm_requested = True
                self.get_logger().info("Requesting arm for AUTO.TAKEOFF")
            return
        if self.climb_started is None:
            self.climb_started = now
            self.phase = "takeoff"
            self.get_logger().info("AUTO.TAKEOFF armed; waiting for airborne rise")
        rise = self.pose.pose.position.z - float(self.start_z)
        landed_state = self.ext.landed_state if now - self.ext_at < 2.0 else -1
        airborne = landed_state in (
            ExtendedState.LANDED_STATE_TAKEOFF, ExtendedState.LANDED_STATE_IN_AIR
        )
        # The local-position climb can walk while the body is still on the
        # ground. Hand off only after the target moves down or grows in the image.
        fresh_det = self.det is not None and now - self.last_valid_detection_at <= 1.5
        if fresh_det and self.ground_error_y is None:
            self.ground_error_y = self.det[2]
            self.ground_area = self.det[3]
        if fresh_det and self.ground_area not in (None, 0.0) and not self.body_rose:
            # A real climb moves the sign down the image or makes it larger.
            # A 0.12 error flicker on the ground does neither.
            grew = self.det[3] >= self.ground_area * 1.5
            # Require some estimator rise so a ground-level flicker cannot
            # hand off, and accept the smaller image shift seen on a real climb.
            moved_down = rise >= 0.30 and self.det[2] - self.ground_error_y >= 0.12
            if grew or moved_down:
                self.body_rose = True
                self.get_logger().info(
                    f"Camera confirms climb: error_y {self.ground_error_y:+.2f} -> {self.det[2]:+.2f}, "
                    f"area {self.ground_area:.4f} -> {self.det[3]:.4f}"
                )
        if (
            self.ground_error_y is not None
            and not self.body_rose
            and self.last_valid_detection_at > 0.0
            and now - self.last_valid_detection_at > 10.0
            and rise >= 0.40
        ):
            self.get_logger().error("Target left the camera during takeoff; landing")
            self._land()
            return
        if self.body_rose and (airborne or self.state.mode != "AUTO.TAKEOFF"):
            if self.prestream_started is None:
                self.target.pose.position.x = self.pose.pose.position.x
                self.target.pose.position.y = self.pose.pose.position.y
                self.target.pose.orientation = self.pose.pose.orientation
                self.target.pose.position.z = self.pose.pose.position.z
                self.prestream_started = now
                self.phase = "prestream"
                self.get_logger().info(
                    f"Takeoff rise {rise:.2f} m (camera confirmed); streaming OFFBOARD setpoints"
                )
            return
        if self.state.mode != "AUTO.TAKEOFF":
            self._call_mode(now, "AUTO.TAKEOFF")
            return
        if self.climb_started is not None and now - self.climb_started > float(self._p("climb_timeout_seconds")):
            self.get_logger().error(f"Takeoff failed to rise {float(self._p('handoff_rise_m')):.2f} m before deadline; landing")
            self._land()
        elif now - self.last_log > 1.0:
            seen_y = f"{self.det[2]:+.2f}" if fresh_det else "none"
            seen_area = f"{self.det[3]:.4f}" if fresh_det else "none"
            self.get_logger().info(
                f"AUTO.TAKEOFF rise={rise:+.2f} m, pitch={pitch_of(self.pose.pose.orientation):+.2f} rad, "
                f"landed_state={landed_state}, error_y={seen_y}, area={seen_area}; waiting for the camera to show a climb"
            )
            self.last_log = now

    def _handoff(self, now: float) -> None:
        assert self.state is not None
        if self.phase == "prestream" and self.prestream_started is not None:
            if now - self.prestream_started >= 1.0:
                self.handoff_started = now
                self.phase = "handoff"
                self._call_mode(now, "OFFBOARD")
            return
        if self.state.mode == "OFFBOARD":
            self.airborne_started = now
            self.search_started = now
            self.search_yaw = yaw_of(self.pose.pose.orientation)
            self.target.pose.position.x = self.pose.pose.position.x
            self.target.pose.position.y = self.pose.pose.position.y
            self.target.pose.position.z = self.pose.pose.position.z
            self.target.pose.orientation = self.pose.pose.orientation
            # The detection that proved the climb is the lock. A slow frame
            # must not wipe it and force a blind nudge.
            if (
                self.body_rose
                and self.det is not None
                and now - self.last_valid_detection_at <= 1.5
                and self.detection_count < int(self._p("object_search_confirmation_count"))
            ):
                self.detection_count = int(self._p("object_search_confirmation_count"))
            fresh = (
                self.det is not None
                and now - self.last_valid_detection_at <= 1.5
                and self.detection_count >= int(self._p("object_search_confirmation_count"))
            )
            if not fresh:
                self.det = None
                self.target_class_id = None
                self.det_candidate_class_id = None
                self.detection_count = 0
                self.last_detection_at = 0.0
                self.last_valid_detection_at = 0.0
                self.last_control_detection_at = 0.0
            self.phase = "search"
            self.get_logger().info("OFFBOARD confirmed; beginning in-place target search")
            return
        if self.handoff_started is None:
            self.handoff_started = now
        if now - self.handoff_started > float(self._p("handoff_timeout_seconds")):
            self.get_logger().error("OFFBOARD handoff timed out; landing")
            self._land()
        else:
            self._call_mode(now, "OFFBOARD")

    def _confirmed(self, now: float) -> bool:
        return (self.det is not None and self.detection_count >= int(self._p("object_search_confirmation_count"))
                and now - self.last_valid_detection_at <= 1.5
                and (self.target_class_id is not None
                     or self.requested_class_id is not None
                     or abs(self.det[2]) <= float(self._p("object_search_max_abs_error_y")))
                and self.det[0] >= float(self._p("object_search_detection_confidence"))
                and (self.target_class_id is None or self.det[4] == self.target_class_id))

    def _search(self, now: float) -> None:
        assert self.search_started is not None and self.search_yaw is not None
        elapsed = now - self.search_started
        initial_hold = float(self._p("object_search_initial_hold_seconds"))
        duration = float(self._p("object_search_seconds"))
        if elapsed > initial_hold + duration:
            self.get_logger().warning("Target not found during bounded in-place scan; landing")
            self._land()
            return
        if self._confirmed(now) and abs(self.det[1]) <= float(self._p("object_search_max_acquisition_error_x")):
            self.target_class_id = self.det[4]
            self.target.pose.orientation = self.pose.pose.orientation
            self.center_started = now
            self.center_hold_since = None
            self.center_yaw = yaw_of(self.pose.pose.orientation)
            self.phase = "center"
            self.get_logger().info(f"YOLO target acquired (error_x={self.det[1]:+.3f}); yaw-centering in place")
            return
        # A target already in frame but outside the acquisition gate is yawed
        # toward center. The open-loop sweep is only for an empty view.
        if self._confirmed(now):
            if elapsed > initial_hold and self.last_valid_detection_at > self.last_control_detection_at:
                self._steer(self.det[1], self.search_yaw)
                self.last_control_detection_at = self.last_valid_detection_at
            if now - self.last_log > 1.0:
                self.get_logger().info(
                    f"Search yaw toward target: error_x={self.det[1]:+.3f}, "
                    f"yaw={yaw_of(self.pose.pose.orientation):+.2f}"
                )
                self.last_log = now
            return
        if elapsed > initial_hold:
            if self.search_hint_error is not None:
                # One capped step toward the last sighting. A second step
                # stacked on the first and yawed past the sign.
                if self.hint_nudges == 0:
                    self._steer(self.search_hint_error, self.search_yaw)
                    self.hint_applied_error = self.search_hint_error
                    self.hint_nudges += 1
                    self.hint_nudge_at = now
                    self.get_logger().info(
                        f"Search nudge {self.hint_nudges} toward hint_error_x={self.search_hint_error:+.2f}"
                    )
            else:
                # Keep the yaw setpoint a small lead ahead of the measured yaw.
                # Integrating the sweep from the start leaves a multi-radian error,
                # and that error made the estimator drop out.
                measured_yaw = yaw_of(self.pose.pose.orientation)
                rate = float(self._p("object_search_yaw_rate_rad_s"))
                lead = float(self._p("object_centering_step_limit_rad"))
                set_yaw(self.target, measured_yaw + (lead if rate >= 0.0 else -lead))
        if now - self.last_log > 1.0:
            hint = "none" if self.search_hint_error is None else f"{self.search_hint_error:+.2f}"
            self.get_logger().info(
                f"Searching: elapsed={elapsed:.1f}s, detections={self.detection_count}, "
                f"hint_error_x={hint}, "
                f"yaw={yaw_of(self.pose.pose.orientation):+.2f}, "
                f"target_yaw={yaw_of(self.target.pose.orientation):+.2f}"
            )
            self.last_log = now

    def _steer(self, error: float, reference_yaw: float) -> float:
        """Yaw so a positive image error (target on the right) decreases."""
        measured_yaw = yaw_of(self.pose.pose.orientation)
        delta = 0.0
        if abs(error) > float(self._p("object_centering_deadband")):
            limit = float(self._p("object_centering_step_limit_rad"))
            delta = clamp(-float(self._p("object_centering_kp_rad")) * error,
                          -limit, limit)
        offset = measured_yaw + delta - reference_yaw
        offset = atan2(sin(offset), cos(offset))
        limit = float(self._p("object_centering_max_offset_rad"))
        commanded_yaw = reference_yaw + clamp(offset, -limit, limit)
        set_yaw(self.target, commanded_yaw)
        return commanded_yaw

    def _center(self, now: float) -> None:
        assert self.center_started is not None and self.center_yaw is not None
        if now - self.last_valid_detection_at > float(self._p("object_centering_loss_timeout_seconds")):
            self.get_logger().error("Target lost during yaw centering; landing")
            self._land()
            return
        inside = self._confirmed(now) and abs(self.det[1]) <= float(self._p("object_approach_center_tolerance"))
        if inside:
            if self.center_hold_since is None:
                self.center_hold_since = now
            elif now - self.center_hold_since >= float(self._p("object_centering_hold_seconds")):
                self._begin_approach(now)
                return
        else:
            self.center_hold_since = None
        if now - self.center_started > float(self._p("object_centering_seconds")):
            self.get_logger().warning("Could not center target within time limit; landing")
            self._land()
            return
        if (not inside and self._confirmed(now)
                and self.last_valid_detection_at > self.last_control_detection_at):
            self._steer(self.det[1], self.center_yaw)
            self.last_control_detection_at = self.last_valid_detection_at
        if now - self.last_log > 1.0:
            error = self.det[1] if self._confirmed(now) else float("nan")
            self.get_logger().info(
                f"Centering: error_x={error:+.3f}, "
                f"yaw={yaw_of(self.pose.pose.orientation):+.2f}, "
                f"target_yaw={yaw_of(self.target.pose.orientation):+.2f}, "
                f"elapsed={now - self.center_started:.1f}s"
            )
            self.last_log = now

    def _begin_approach(self, now: float) -> None:
        assert self.det is not None
        if not self._confirmed(now) or abs(self.det[1]) > float(self._p("object_approach_center_tolerance")) or self.det[3] <= 0:
            return
        self.approach_started = self.last_approach_tick = now
        self.approach_start_yaw = yaw_of(self.pose.pose.orientation)
        self.approach_yaw = self.approach_start_yaw
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation
        self.initial_area = self.det[3]
        self.stop_area = min(self.initial_area * float(self._p("object_approach_size_growth_ratio")),
                             float(self._p("object_search_max_bbox_area")) * 0.85)
        self.phase = "approach"
        self.get_logger().info(
            f"Centered target; continuous approach at <= {float(self._p('object_approach_speed_m_s')):.2f} m/s; "
            f"land on stable box-area growth to {self.stop_area / self.initial_area:.2f}x (or timeout/loss)"
        )

    def _approach(self, now: float) -> None:
        assert self.approach_started is not None and self.approach_start_yaw is not None
        assert self.approach_yaw is not None
        dt = min(0.1, max(0.0, now - self.last_approach_tick))
        self.last_approach_tick = now
        elapsed = now - self.approach_started
        if elapsed >= float(self._p("object_approach_timeout_seconds")):
            self.get_logger().warning("Approach time limit reached; landing")
            self._land()
            return
        if now - self.last_valid_detection_at > float(self._p("object_approach_loss_timeout_seconds")):
            if (self.det is not None
                    and self.det[2] >= float(self._p("object_approach_arrival_error_y"))
                    and abs(self.det[1]) <= float(self._p("object_approach_max_error_x"))):
                self.get_logger().info(
                    "Target dropped out the bottom of the frame after a centered approach; landing"
                )
                self._land()
                return
            if self.approach_reacquire_attempted:
                self.get_logger().error("Target still lost after reacquisition attempt; landing")
                self._land()
                return
            self.approach_reacquire_attempted = True
            self.reacquire_started = now
            self.reacquire_yaw = yaw_of(self.pose.pose.orientation)
            self.hint_applied_error = None
            # Freeze translation at the measured pose before scanning, so
            # target loss can never cause blind forward motion.
            self.target.pose.position = self.pose.pose.position
            self.target.pose.orientation = self.pose.pose.orientation
            self.phase = "reacquire"
            self.get_logger().warning(
                "Target lost during approach; holding position and starting one bounded yaw reacquisition"
            )
            return
        if not self._confirmed(now):
            return
        assert self.det is not None and self.initial_area is not None and self.stop_area is not None
        _, error, error_y, area, _ = self.det
        # Positive error_y means the object has dropped into the lower half
        # of the forward camera, which is the cue that the drone has reached
        # a ground object. A tall sign stays in the upper half while the
        # drone is still approaching it.
        if (error_y >= float(self._p("object_approach_arrival_error_y"))
                and abs(error) <= float(self._p("object_approach_max_error_x"))):
            self.get_logger().info(
                f"Target is under the forward camera (error_y={error_y:+.3f}); landing on the object"
            )
            self._land()
            return
        centered = abs(error) <= float(self._p("object_approach_center_tolerance"))
        # A 2.5x growth of the tiny ground sliver is still a distant sign.
        # Keep closing until the box is actually large, or it drops under the camera.
        arrival_area = max(self.stop_area, 0.02)
        if area >= arrival_area:
            if centered:
                self.get_logger().info(
                    f"Reached visual-size goal ({area / self.initial_area:.2f}x, area={area:.4f}, "
                    f"error_y={error_y:+.2f}); landing"
                )
                self._land()
            return  # Never advance further while waiting for a stable cue.

        # Small, bounded image-based yaw corrections keep the target on-axis.
        if self.last_valid_detection_at > self.last_control_detection_at:
            self.approach_yaw = self._steer(error, self.approach_start_yaw)
            self.last_control_detection_at = self.last_valid_detection_at
        alignment = centering_factor(
            error,
            float(self._p("object_approach_center_tolerance")),
            float(self._p("object_approach_max_error_x")),
        )
        if alignment == 0.0:
            return
        fraction = clamp(area / arrival_area, 0.0, 1.0)
        speed = (float(self._p("object_approach_speed_m_s"))
                 * (1.0 - 0.8 * fraction) * alignment)
        distance = speed * dt
        measured_yaw = yaw_of(self.pose.pose.orientation)
        self.target.pose.position.x += distance * cos(measured_yaw)
        self.target.pose.position.y += distance * sin(measured_yaw)
        self.approach_distance += distance
        if now - self.last_log > 1.0:
            self.get_logger().info(f"Approach: commanded={self.approach_distance:.2f}m, box={area / self.initial_area:.2f}x/{self.stop_area / self.initial_area:.2f}x, image_error_x={error:+.3f}, alignment={alignment:.2f}")
            self.last_log = now

    def _reacquire(self, now: float) -> None:
        assert self.reacquire_started is not None and self.reacquire_yaw is not None
        elapsed = now - self.reacquire_started
        if (self._confirmed(now)
                and abs(self.det[1]) <= float(self._p("object_approach_reacquire_max_error_x"))):
            # Resume from a position hold. Keep the original size goal and
            # overall approach deadline; do not reset progress or extend flight.
            self.target.pose.position = self.pose.pose.position
            self.target.pose.orientation = self.pose.pose.orientation
            self.approach_yaw = yaw_of(self.pose.pose.orientation)
            self.approach_start_yaw = self.approach_yaw
            self.last_approach_tick = now
            self.last_control_detection_at = 0.0
            self.phase = "approach"
            self.get_logger().info(
                f"Target reacquired (error_x={self.det[1]:+.3f}); resuming approach from held position"
            )
            return
        if elapsed >= float(self._p("object_approach_reacquire_seconds")):
            self.get_logger().error("Target not reacquired during bounded scan; landing")
            self._land()
            return
        # One correction back toward the last sighting, then hold. Repeating a
        # frozen error winds the yaw and carries the sign out of frame.
        if self.search_hint_error is not None:
            if self.hint_applied_error != self.search_hint_error:
                self._steer(self.search_hint_error, self.reacquire_yaw)
                self.hint_applied_error = self.search_hint_error
            return
        measured_yaw = yaw_of(self.pose.pose.orientation)
        rate = float(self._p("object_search_yaw_rate_rad_s"))
        lead = float(self._p("object_centering_step_limit_rad"))
        set_yaw(self.target, measured_yaw + (lead if rate >= 0.0 else -lead))

    def _land(self) -> None:
        now = monotonic()
        if self.phase != "landing":
            self.phase, self.landing_started, self.disarmed_at = "landing", now, None
            self.get_logger().info("Requesting AUTO.LAND; waiting for ground and disarm confirmation")
        state_fresh = self.state is not None and now - self.state_at < 2.0 and self.state_at >= self.landing_started
        ext_fresh = self.ext is not None and now - self.ext_at < 2.0 and self.ext_at >= self.landing_started
        if state_fresh and not self.state.armed:
            if self.disarmed_at is None:
                self.disarmed_at = now
        else:
            self.disarmed_at = None
        landed = ext_fresh and self.ext.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        disarmed = state_fresh and not self.state.armed and self.disarmed_at is not None and now - self.disarmed_at >= 1.0
        if landed and disarmed:
            self.phase = "done"
            if self.flight_label and self.approach_distance > 0.2:
                self._say_flight(f"Landed after flying to {self.flight_label}")
            elif self.flight_label:
                self._say_flight(f"Flight to {self.flight_label} ended before reaching it")
            self.get_logger().info("Landing confirmed: on ground and disarmed")
            return
        if (self.state is None or self.state.mode != "AUTO.LAND") and now - self.last_request >= 1.0 and self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = "AUTO.LAND"
            self.mode_client.call_async(req)
            self.last_request = now
            self.get_logger().info("Requesting AUTO.LAND")
        if self.landing_started is not None and now - self.landing_started >= float(self._p("landing_timeout_seconds")) and not self.land_timeout_logged:
            self.get_logger().error(f"Landing not confirmed: armed={None if self.state is None else self.state.armed}, landed_state={None if self.ext is None else self.ext.landed_state}")
            self.land_timeout_logged = True


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafeHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._land()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
