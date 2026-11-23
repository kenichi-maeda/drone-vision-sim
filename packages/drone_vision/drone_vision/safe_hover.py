"""Compact YOLO-guided takeoff, visual approach, and landing node.

The controller uses PX4 AUTO.TAKEOFF, then a streamed MAVROS local-position
setpoint in OFFBOARD. YOLO supplies image-space target center and box size;
there is no metric range estimate. The approach advances continuously at a
bounded speed and lands when the centered box reaches a relative-size cue.
"""

from __future__ import annotations

import json
from math import asin, atan2, cos, isfinite, pi, sin
from time import monotonic, time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, ExtendedState, State
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
            "object_search_detection_confidence": 0.05,
            "object_search_max_bbox_area": 0.90,
            "object_search_confirmation_count": 2,
            # The stop sign is still a reliable detection well off center.
            # Acquiring it there avoids a blind yaw that carries it out of frame.
            "object_search_max_acquisition_error_x": 0.80,
            # Unguided search ignores a box this far above or below center.
            # A requested object skips this gate; the stop sign is near the
            # top of the forward camera even when it is horizontally centered.
            "object_search_max_abs_error_y": 0.80,
            "object_tracking_max_error_x_delta": 0.30,
            "object_tracking_max_error_y_delta": 1.20,
            "object_tracking_max_area_ratio": 12.0,
            "object_centering_enabled": True,
            # The simulator yaws slowly, so centering needs a longer window
            # and a larger yaw step before the box is inside the approach gate.
            "object_centering_seconds": 40.0,
            "object_centering_kp_rad": 1.50,
            "object_centering_deadband": 0.08,
            # A half-radian step yaws a narrow sign out of the fisheye frame
            # before the next detection can correct it.
            "object_centering_step_limit_rad": 0.20,
            "object_centering_max_offset_rad": 1.20,
            "object_centering_loss_timeout_seconds": 10.0,
            # One in-tolerance frame is not enough; the box has to stay
            # centered before the drone starts moving toward it.
            "object_centering_hold_seconds": 1.5,
            "object_approach_enabled": True,
            "object_approach_speed_m_s": 0.08,
            "object_approach_size_growth_ratio": 2.5,
            "object_approach_center_tolerance": 0.28,
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
            "object_approach_timeout_seconds": 55.0,
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
        self.create_subscription(String, "/drone_vision/yolo_objects", self._objects_cb, 10)
        self.flight_status_pub = self.create_publisher(String, "/drone_vision/flight_status", 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            qos,
        )
        self.thrust_pub = self.create_publisher(
            AttitudeTarget,
            "/mavros/setpoint_raw/attitude",
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
        self.approach_centered_since = None
        self.visual_anchor = None
        self.last_visual_tick = 0.0
        self.horizon_creep = 0.0
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
        self.mission = ""
        self.requested_color = ""
        self.catalog: dict[str, dict] = {}
        self.orbit_last_yaw = None
        self.orbit_turned = 0.0
        self.recall_yaw = None
        self.recall_dwell_at = None
        self.arrived_at = None
        self.reached = False
        self.orbit_started = None
        self.seek_ready_at = 0.0
        self._pending_flight = ""
        self.det_candidate_class_id: int | None = None
        self.det = None  # confidence, error_x, error_y, normalized box area, class_id
        self.ground_error_y = None
        self.ground_area = None
        self.body_rose = False
        self.pose_gap_retry_at = 0.0
        self.estimator_bad_since = None
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
        # GPS vertical velocity in this sandbox walks the height estimate, so
        # that fusion is off and velocity_vert_status_flag stays false. Baro
        # holds the height. The other flags are the ones that mean the
        # estimate can be flown.
        return est is not None and (
            est.attitude_status_flag
            and est.velocity_horiz_status_flag
            and est.pos_horiz_rel_status_flag
            and not est.gps_glitch_status_flag
            and not est.accel_error_status_flag
        )

    def _ext_cb(self, msg: ExtendedState) -> None:
        self.ext, self.ext_at = msg, monotonic()

    def _vision_cb(self, msg: Header) -> None:
        capture_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        age = time() - capture_time
        # The camera stamp runs a fraction of a second ahead of this clock.
        # A frame that just arrived is live even when its stamp is in the future.
        if capture_time > 0.0 and age <= 2.0:
            self.vision_at = monotonic()

    def _detection_cb(self, msg: Float32MultiArray) -> None:
        # yolo_detector wire format: class, confidence, cx, cy, ex, ey, w, h.
        if len(msg.data) < 8:
            return
        class_id, confidence, ex, ey, width, height = map(
            float, (msg.data[0], msg.data[1], msg.data[4], msg.data[5], msg.data[6], msg.data[7])
        )
        now = monotonic()
        if now < self.seek_ready_at:
            return
        if not all(map(isfinite, (class_id, confidence, ex, ey, width, height))):
            self.detection_count = 0
            return
        if class_id < 0 or class_id != int(class_id):
            self.detection_count = 0
            return
        if self.requested_class_id is not None and int(class_id) != self.requested_class_id:
            return
        area = width * height
        # Once a target is locked, follow one continuous box. A second object of
        # the same class (another blue bot) is a jump, not a new label.
        if self.target_class_id is not None and self.det is not None:
            area_ratio = area / max(self.det[3], 1e-9)
            if (abs(ex - self.det[1]) > float(self._p("object_tracking_max_error_x_delta"))
                    or abs(ey - self.det[2]) > float(self._p("object_tracking_max_error_y_delta"))
                    or area_ratio > float(self._p("object_tracking_max_area_ratio"))
                    or area_ratio < 1.0 / float(self._p("object_tracking_max_area_ratio"))):
                return
            if int(class_id) != self.target_class_id:
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
        self.approach_centered_since = None
        self.visual_anchor = None
        self.last_visual_tick = 0.0
        self.horizon_creep = 0.0
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
        self.estimator_bad_since = None
        self.search_hint_error = None
        self.hint_applied_error = None
        self.hint_nudges = 0
        self.hint_nudge_at = 0.0
        self.pose_stream_requested = False
        self.found_at = None
        self.arm_requested = False
        self.land_timeout_logged = False
        self.wait_message = None
        self.orbit_last_yaw = None
        self.orbit_turned = 0.0
        self.recall_yaw = None
        self.recall_dwell_at = None
        self.arrived_at = None
        self.reached = False
        self.orbit_started = None
        self.seek_ready_at = 0.0

    def _objects_cb(self, message: String) -> None:
        if self.pose is None or self.phase not in ("orbit", "choose", "search", "recall", "center"):
            return
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        yaw = yaw_of(self.pose.pose.orientation)
        for item in payload.get("objects", []):
            if not item.get("color_ok"):
                continue
            label = str(item.get("label") or "")
            confidence = float(item.get("confidence") or 0.0)
            if not label or confidence < 0.05:
                continue
            error_x = float(item.get("error_x") or 0.0)
            bearing = atan2(sin(yaw - error_x * 0.8), cos(yaw - error_x * 0.8))
            previous = self.catalog.get(label)
            if previous is None or confidence >= float(previous["confidence"]):
                self.catalog[label] = {
                    "confidence": confidence,
                    "bearing": bearing,
                    "color": item.get("color") or "",
                }

    def _apply_flight_request(self) -> None:
        label = self._pending_flight
        if not label:
            return
        self._pending_flight = ""
        if label.lower() == "scan":
            if self.phase == "choose":
                self._say_flight("Already up. Type an object and press Fly to it.")
                return
            if self.state is not None and self.state.armed:
                self._say_flight("Already in the air.")
                return
            self.mission = "scan"
            self.flight_label = ""
            self.requested_class_id = None
            self.requested_color = ""
            self.catalog = {}
            self._clear_mission()
            self.phase = "idle"
            self.set_parameters([Parameter("enabled", Parameter.Type.BOOL, True)])
            self._say_flight("Lifting off to look around")
            return
        spec = resolve_specified(label)
        if spec is None:
            self._say_flight("Unknown object. Choose a colored duck or the stop sign.")
            return
        if self.phase == "choose" and self.state is not None and self.state.armed:
            self.mission = "seek"
            self.requested_class_id = int(spec["class_id"])
            self.requested_color = spec["color"]
            self.flight_label = spec["label"]
            self._say_flight(f"Flying to {spec['label']}")
            self._begin_seek(monotonic())
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
        self.mission = "seek"
        self.requested_class_id = int(spec["class_id"])
        self.requested_color = spec["color"]
        self.flight_label = spec["label"]
        self.catalog = {}
        self._clear_mission()
        self.phase = "idle"
        self.set_parameters([Parameter("enabled", Parameter.Type.BOOL, True)])
        self._say_flight(f"Flying to {spec['label']}. Looking around first.")

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
            # The same 1.5 s limit still waits once the drone has actually climbed.
            # A dropped MAVROS stream used to land a flight that was still on course.
            # Ask for the stream again and keep the last setpoint until it returns.
            risen = self.body_rose
            if not risen:
                self._log_once("Waiting for fresh MAVROS pose")
                return
            if self.pose_gap_retry_at == 0.0 or now - self.pose_gap_retry_at >= 2.0:
                self.pose_gap_retry_at = now
                self.pose_stream_requested = False
                self._request_pose_stream()
                self.get_logger().warning("Pose gap; requesting the position stream again")
            if now - self.pose_at < 18.0:
                return
            self.get_logger().error("MAVROS pose stale; requesting AUTO.LAND")
            self._land()
            return
        self.pose_gap_retry_at = 0.0

        if now - self.vision_at > 6.0:
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
                # A yaw during the look-around drops a flag for a sample or two.
                # Land only if the estimate stays unusable.
                if self.estimator_bad_since is None:
                    self.estimator_bad_since = now
                    est = self.estimator
                    self.get_logger().warning(
                        "Estimator flag blip: "
                        f"att={None if est is None else est.attitude_status_flag} "
                        f"vh={None if est is None else est.velocity_horiz_status_flag} "
                        f"ph={None if est is None else est.pos_horiz_rel_status_flag} "
                        f"glitch={None if est is None else est.gps_glitch_status_flag} "
                        f"acc={None if est is None else est.accel_error_status_flag}"
                    )
                elif now - self.estimator_bad_since >= 6.0:
                    self.get_logger().error("PX4 estimator unhealthy; requesting AUTO.LAND")
                    self._land()
            return
        self.estimator_bad_since = None

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
        if self.phase != "choose" and now - self.started > total:
            self.get_logger().error("Mission watchdog expired; landing")
            self._land()
            return
        # Position setpoints follow the bad vertical velocity and cut thrust.
        # The climb sends a level attitude with a fixed thrust instead.
        if (
            self.phase in ("warmup", "takeoff")
            and self.pose is not None
            and not self.body_rose
        ):
            thrust = AttitudeTarget()
            thrust.header.stamp = self.get_clock().now().to_msg()
            thrust.type_mask = 7
            thrust.orientation = self.pose.pose.orientation
            thrust.thrust = 0.74
            self.thrust_pub.publish(thrust)
        elif self.phase in ("prestream", "handoff", "orbit", "choose", "recall", "search", "center", "approach", "reacquire", "arrived"):
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
        elif self.phase == "orbit":
            self._orbit(now)
        elif self.phase == "choose":
            self._choose(now)
        elif self.phase == "recall":
            self._recall(now)
        elif self.phase == "search":
            self._search(now)
        elif self.phase == "center":
            self._center(now)
        elif self.phase == "approach":
            self._approach(now)
        elif self.phase == "reacquire":
            self._reacquire(now)
        elif self.phase == "arrived":
            self._arrived(now)

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
        # AUTO.TAKEOFF reports landed_state TAKEOFF before MAVROS flips armed.
        in_takeoff_handshake = self.arm_requested or self.state.mode == "OFFBOARD"
        if (
            self.state.armed is False
            and self.ext.landed_state != ExtendedState.LANDED_STATE_ON_GROUND
            and not in_takeoff_handshake
        ):
            # The land detector follows the wandering height estimate, so it can
            # say "in air" while the body is still on the grass.
            self._log_once(
                f"Land detector says {self.ext.landed_state} while disarmed; arming anyway"
            )
        if self.state.armed and self.climb_started is None and not self.arm_requested:
            self.get_logger().error("Already armed before test; refusing to take control")
            self.phase = "done"
            return
        if not self.state.armed:
            if self.state.mode != "OFFBOARD":
                self._call_mode(now, "OFFBOARD")
                return
            if now - self.last_request >= 1.0 and self.arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
                self.last_request = now
                self.arm_requested = True
                self.get_logger().info("Requesting arm for OFFBOARD climb")
            return
        if self.climb_started is None:
            self.climb_started = now
            self.phase = "takeoff"
            self.get_logger().info("OFFBOARD armed; climbing until the camera shows it")
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
            # The height estimate wanders while the body is still on the grass,
            # so it cannot prove a climb. The picture can: a real climb moves
            # the target down the frame or makes the box larger.
            moved_down = self.det[2] - self.ground_error_y >= 0.12
            # A one-frame flicker can move the box before the drone has left
            # the ground. Keep the fixed thrust until the climb has had time.
            # Stop as the object crosses the crosshair. Climbing on past that
            # leaves it in the bottom of the fisheye for the rest of the flight.
            level = (
                self.climb_started is not None
                and now - self.climb_started >= 2.5
                and rise >= 0.20
                and self.ground_error_y is not None
                and self.ground_error_y <= -0.05
                and -0.12 <= self.det[2] <= 0.06
            )
            held = self.climb_started is not None and now - self.climb_started >= 4.0
            if level or (held and (grew or moved_down)):
                self.body_rose = True
                self.get_logger().info(
                    f"Camera confirms climb: error_y {self.ground_error_y:+.2f} -> {self.det[2]:+.2f}, "
                    f"area {self.ground_area:.4f} -> {self.det[3]:.4f}"
                )
        if (
            not self.body_rose
            and self.climb_started is not None
            and now - self.climb_started >= 1.0
            and rise >= 0.55
        ):
            self.body_rose = True
            self.get_logger().info(f"Climb capped at rise {rise:.2f} m")
        if (
            not self.body_rose
            and self.climb_started is not None
            and now - self.climb_started >= 2.0
            and rise >= 0.80
        ):
            self.body_rose = True
            self.get_logger().info(f"Climb capped at rise {rise:.2f} m")
        if (
            not self.body_rose
            and self.climb_started is not None
            and now - self.climb_started >= 1.8
        ):
            self.body_rose = True
            self.get_logger().info("Climb time elapsed; starting the look-around")
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
                self.visual_anchor = self.pose.pose.position.z
                self.prestream_started = now
                self.phase = "prestream"
                self.get_logger().info(
                    f"Takeoff rise {rise:.2f} m; holding the current estimate "
                    f"z={self.pose.pose.position.z:.2f} m"
                )
            return
        if self.climb_started is not None and now - self.climb_started > float(self._p("climb_timeout_seconds")):
            self.get_logger().error("Climb produced no camera change before the deadline; landing")
            self._land()
        elif now - self.last_log > 1.0:
            seen_y = f"{self.det[2]:+.2f}" if fresh_det else "none"
            self.get_logger().info(
                f"Fixed-thrust climb in {self.state.mode}: landed_state={landed_state}, error_y={seen_y}"
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
            self._begin_orbit(now)
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
                and now - self.last_valid_detection_at <= 3.2
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
        if self.start_z is not None and self.target is not None:
            self._ease_down()
            goal = float(self.start_z) + 0.60
            if self.target.pose.position.z > goal + 0.08:
                self.target.pose.position.z = max(goal, self.target.pose.position.z - 0.01)
        # One confident frame is enough. The stop sign often clears 0.2 for a
        # single inference and then drops under the two-frame confirmation.
        live = (
            self.det is not None
            and now - self.last_valid_detection_at <= 1.5
            and self.det[0] >= 0.12
            and abs(self.det[1]) <= float(self._p("object_search_max_acquisition_error_x"))
        )
        if live and (self._confirmed(now) or self.detection_count >= 1):
            self.visual_anchor = self.pose.pose.position.z
            self.target_class_id = self.det[4]
            self.target.pose.orientation = self.pose.pose.orientation
            self.center_started = now
            self.center_hold_since = None
            self.center_yaw = yaw_of(self.pose.pose.orientation)
            self.phase = "center"
            self.get_logger().info(
                f"YOLO target acquired (error_x={self.det[1]:+.3f}, error_y={self.det[2]:+.3f}); "
                "yaw-centering in place"
            )
            return
        # A target already in frame but outside the acquisition gate is yawed
        # toward center. The open-loop sweep is only for an empty view.
        if self._confirmed(now):
            if elapsed > initial_hold and self.last_valid_detection_at > self.last_control_detection_at:
                self._steer(self.det[1], self.search_yaw)
                self.last_control_detection_at = self.last_valid_detection_at
            self._nudge_height(self.det[2], now)
            if now - self.last_log > 1.0:
                self.get_logger().info(
                    f"Search yaw toward target: error_x={self.det[1]:+.3f}, "
                    f"yaw={yaw_of(self.pose.pose.orientation):+.2f}"
                )
                self.last_log = now
            return
        if elapsed > initial_hold and self.search_hint_error is not None:
            measured_yaw = yaw_of(self.pose.pose.orientation)
            commanded_yaw = yaw_of(self.target.pose.orientation)
            caught_up = abs(atan2(sin(commanded_yaw - measured_yaw), cos(commanded_yaw - measured_yaw))) < 0.08
            if caught_up and abs(self.search_hint_error) > 0.10:
                # One step from the current heading. Waiting until the last
                # step is done keeps a stale edge error from winding past it.
                self._steer(self.search_hint_error, measured_yaw)
                self.search_hint_error *= 0.65
                self.get_logger().info(
                    f"Search step toward hint_error_x={self.search_hint_error / 0.65:+.2f}"
                )
            if self.det is not None and now - self.last_valid_detection_at <= 8.0:
                self._nudge_height(self.det[2], now)
                self._creep_toward(self.det[1], self.det[2])
            return
        if elapsed > initial_hold:
            hint_is_live = (
                self.search_hint_error is not None
                and now - self.last_valid_detection_at <= 2.0
            )
            if hint_is_live and self.hint_nudges == 0:
                # One capped step toward a sighting that is still on screen.
                # A second step stacked on the first and yawed past the sign.
                self._steer(self.search_hint_error, self.search_yaw)
                self.hint_applied_error = self.search_hint_error
                self.hint_nudges += 1
                self.hint_nudge_at = now
                self.get_logger().info(
                    f"Search nudge {self.hint_nudges} toward hint_error_x={self.search_hint_error:+.2f}"
                )
            elif not hint_is_live:
                # The old hint is gone. Keep turning until the object is seen again.
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

    def _nudge_height(self, error_y: float, now: float) -> None:
        """Move the held altitude so a ground object climbs toward the crosshair."""
        if self.target is None:
            return
        anchor = self.visual_anchor
        if anchor is None and self.pose is not None:
            anchor = self.pose.pose.position.z
            self.visual_anchor = anchor
        if anchor is None:
            return
        dt = min(0.1, max(0.0, now - self.last_visual_tick)) if self.last_visual_tick else 0.05
        self.last_visual_tick = now
        z = self.target.pose.position.z
        rate = 0.28 if abs(error_y) > 0.35 else 0.18
        if error_y > 0.12:
            z = max(float(anchor) - 0.80, z - rate * dt)
        elif error_y < -0.12:
            ceiling = float(anchor) + (1.40 if error_y < -0.35 else 0.25)
            z = min(ceiling, z + rate * dt)
        self.target.pose.position.z = z

    def _ease_down(self) -> None:
        """Ramp a runaway climb back to the height where the stop-sign face is visible."""
        if self.start_z is None or self.target is None:
            return
        ceiling = float(self.start_z) + 0.50
        if self.target.pose.position.z > ceiling:
            self.target.pose.position.z = max(ceiling, self.target.pose.position.z - 0.004)

    def _creep_toward(self, error_x: float, error_y: float) -> None:
        """Close a little on a horizon object so the box grows toward the crosshair."""
        if self.target is None or self.pose is None:
            return
        if error_y > -0.28 or abs(error_x) > 0.55:
            return
        flown = getattr(self, "horizon_creep", 0.0)
        if flown >= 1.00:
            return
        # A box that already fills the frame is the sign beside the camera.
        # Another meter of creep would fly into it.
        if self.det is not None and self.det[3] >= 0.02:
            return
        step = 0.0035
        yaw = yaw_of(self.pose.pose.orientation)
        self.target.pose.position.x += step * cos(yaw)
        self.target.pose.position.y += step * sin(yaw)
        self.horizon_creep = flown + step

    def _center(self, now: float) -> None:
        assert self.center_started is not None and self.center_yaw is not None
        if now - self.last_valid_detection_at > float(self._p("object_centering_loss_timeout_seconds")):
            self.center_retries = getattr(self, "center_retries", 0) + 1
            if self.center_retries > 2:
                self.get_logger().error("Target lost during yaw centering; landing")
                self._land()
                return
            self.get_logger().warning("Target lost during yaw centering; searching again")
            self.search_started = now
            self.search_yaw = yaw_of(self.pose.pose.orientation)
            self.phase = "search"
            return
        confirmed = self._confirmed(now)
        on_crosshair = confirmed and abs(self.det[1]) <= 0.22 and abs(self.det[2]) <= 0.22
        if on_crosshair and self.det[3] >= 0.0004:
            if self.center_hold_since is None:
                self.center_hold_since = now
            elif now - self.center_hold_since >= 0.5:
                self.get_logger().info(
                    f"Object is on the crosshair (error_x={self.det[1]:+.3f}, "
                    f"error_y={self.det[2]:+.3f}, area={self.det[3]:.4f}); landing beside it"
                )
                self._arrive(now)
                return
        else:
            if not on_crosshair:
                self.center_hold_since = None
            if self.det is not None and now - self.last_valid_detection_at <= 4.0:
                self._nudge_height(self.det[2], now)
                self._creep_toward(self.det[1], self.det[2])
        if now - self.center_started > float(self._p("object_centering_seconds")):
            self.get_logger().warning("Could not center target within time limit; landing")
            self._land()
            return
        if (not on_crosshair and self._confirmed(now)
                and self.last_valid_detection_at > self.last_control_detection_at):
            self._steer(self.det[1], self.center_yaw)
            self.last_control_detection_at = self.last_valid_detection_at
        if now - self.last_log > 1.0:
            error = self.det[1] if self._confirmed(now) else float("nan")
            error_y = self.det[2] if self._confirmed(now) else float("nan")
            self.get_logger().info(
                f"Centering: error_x={error:+.3f}, error_y={error_y:+.3f}, "
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
        self.approach_centered_since = None
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
                    and self.det[3] >= 0.02
                    and 0.22 < self.det[2] <= 0.45
                    and abs(self.det[1]) <= 0.22):
                self.get_logger().info(
                    "Close target slipped just below the crosshair; landing beside it"
                )
                self._arrive(now)
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
        # Land only when the object is actually near the camera crosshair.
        # A speck at the bottom of the frame is not an arrival.
        at_crosshair = abs(error) <= 0.22 and abs(error_y) <= 0.22 and area >= 0.0004
        if at_crosshair:
            if (
                self.approach_centered_since is not None
                and now - self.approach_centered_since >= 0.8
            ):
                self.get_logger().info(
                    f"Object is on the crosshair (error_x={error:+.3f}, error_y={error_y:+.3f}, "
                    f"area={area:.4f}); landing beside it"
                )
                self._arrive(now)
                return
            if self.approach_centered_since is None:
                self.approach_centered_since = now
            return
        self.approach_centered_since = None
        self._nudge_height(error_y, now)
        centered = abs(error) <= float(self._p("object_approach_center_tolerance"))
        arrival_area = max(self.stop_area, 0.02)

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
        # Stay put until the object is on the crosshair. Forward motion pushes
        # a ground object down the fisheye and out of the arrival window.
        if abs(error) > 0.22 or abs(error_y) > 0.22 or area >= 0.0030:
            forward = 0.0
        else:
            forward = 0.0
        speed = (float(self._p("object_approach_speed_m_s"))
                 * (1.0 - 0.8 * fraction) * alignment * forward)
        distance = speed * dt
        measured_yaw = yaw_of(self.pose.pose.orientation)
        self.target.pose.position.x += distance * cos(measured_yaw)
        self.target.pose.position.y += distance * sin(measured_yaw)
        self.approach_distance += distance
        if now - self.last_log > 1.0:
            self.get_logger().info(
                f"Approach: commanded={self.approach_distance:.2f}m, "
                f"box={area / self.initial_area:.2f}x, error_x={error:+.3f}, "
                f"error_y={error_y:+.3f}, z={self.target.pose.position.z:.2f}, "
                f"alignment={alignment:.2f}"
            )
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

    def _begin_orbit(self, now: float) -> None:
        self.phase = "orbit"
        self.orbit_started = now
        self.orbit_last_yaw = yaw_of(self.pose.pose.orientation)
        self.orbit_turned = 0.0
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation
        self.get_logger().info("OFFBOARD confirmed; turning once to see everything around")

    def _orbit(self, now: float) -> None:
        measured = yaw_of(self.pose.pose.orientation)
        if self.orbit_last_yaw is None:
            self.orbit_last_yaw = measured
        delta = atan2(sin(measured - self.orbit_last_yaw), cos(measured - self.orbit_last_yaw))
        if abs(delta) > 0.005:
            self.orbit_turned += abs(delta)
        self.orbit_last_yaw = measured
        self._ease_down()
        live_sign = (
            self.flight_label
            and self.det is not None
            and now - self.last_valid_detection_at <= 1.5
            and self.det[0] >= 0.20
            and abs(self.det[1]) <= 0.50
            and self.orbit_turned >= 0.4
        )
        if live_sign:
            self.get_logger().info(
                f"Target in view during the look-around "
                f"(error_x={self.det[1]:+.3f}, error_y={self.det[2]:+.3f}); centering"
            )
            self._lock_target(now)
            return
        lead = float(self._p("object_centering_step_limit_rad"))
        set_yaw(self.target, measured + lead)
        seen = (not self.flight_label) or (self.flight_label in self.catalog)
        # Once the requested object has been cataloged, most of a turn is enough.
        # Waiting out the last part of the circle is when the estimator drops.
        if (seen and self.orbit_turned >= 5.0) or (
            self.orbit_started is not None and now - self.orbit_started > 75.0
        ):
            self._finish_orbit(now)

    def _finish_orbit(self, now: float) -> None:
        names = ", ".join(
            f"{label} {item['confidence']:.2f}"
            for label, item in sorted(self.catalog.items(), key=lambda pair: -float(pair[1]["confidence"]))
        ) or "nothing clearly"
        self.get_logger().info(f"Scan finished. Catalog: {names}")
        if self.mission == "seek" and self.flight_label:
            self._say_flight(f"Saw {names}. Flying to {self.flight_label}")
            self._begin_seek(now)
            return
        self.started = now
        self.phase = "choose"
        self._say_flight(f"Scan finished. Saw {names}. Type an object and press Fly to it.")

    def _choose(self, now: float) -> None:
        del now
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation

    def _begin_seek(self, now: float) -> None:
        self.started = now
        self.seek_ready_at = now + 0.4
        self.det = None
        self.detection_count = 0
        self.det_candidate_class_id = None
        self.last_detection_at = 0.0
        self.last_valid_detection_at = 0.0
        self.last_control_detection_at = 0.0
        self.target_class_id = self.requested_class_id
        self.search_started = now
        self.search_yaw = yaw_of(self.pose.pose.orientation)
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation
        entry = self.catalog.get(self.flight_label)
        if entry is not None:
            self.recall_yaw = float(entry["bearing"])
            self.recall_dwell_at = None
            self.phase = "recall"
            self.get_logger().info(
                f"Turning back toward {self.flight_label} "
                f"(bearing {self.recall_yaw:+.2f}, confidence {float(entry['confidence']):.2f})"
            )
            return
        self.phase = "search"
        self.get_logger().info(f"{self.flight_label} was not in the scan; searching in place")

    def _lock_target(self, now: float) -> None:
        if self.pose is not None:
            self.visual_anchor = self.pose.pose.position.z
        self.target_class_id = self.det[4]
        self.target.pose.orientation = self.pose.pose.orientation
        self.center_started = now
        self.center_hold_since = None
        self.center_yaw = yaw_of(self.pose.pose.orientation)
        self.phase = "center"
        self.get_logger().info(
            f"YOLO target acquired (error_x={self.det[1]:+.3f}); yaw-centering in place"
        )

    def _recall(self, now: float) -> None:
        assert self.recall_yaw is not None
        measured = yaw_of(self.pose.pose.orientation)
        live = (
            self.det is not None
            and now - self.last_valid_detection_at <= 1.5
            and self.det[0] >= 0.12
            and abs(self.det[1]) <= float(self._p("object_search_max_acquisition_error_x"))
        )
        if live and (self._confirmed(now) or self.detection_count >= 1):
            self._lock_target(now)
            return
        error = atan2(sin(self.recall_yaw - measured), cos(self.recall_yaw - measured))
        self._ease_down()
        if abs(error) >= 0.18 and self.recall_dwell_at is None:
            lead = float(self._p("object_centering_step_limit_rad"))
            set_yaw(self.target, measured + clamp(error, -lead, lead))
            return
        if self.recall_dwell_at is None:
            self.recall_dwell_at = now
            self.get_logger().info(
                f"Facing {self.flight_label}; holding that heading until it is visible"
            )
        set_yaw(self.target, self.recall_yaw)
        if now - self.recall_dwell_at > 12.0:
            self.search_started = now
            self.search_yaw = measured
            self.phase = "search"
            self.get_logger().info(
                f"Still no view of {self.flight_label}; searching from this hover"
            )
            return
        if now - self.last_log > 1.0:
            self.get_logger().info(
                f"Recall hold: yaw={measured:+.2f}, z={self.target.pose.position.z:.2f}"
            )
            self.last_log = now

    def _arrive(self, now: float) -> None:
        if self.phase == "arrived":
            return
        self.reached = True
        self.phase = "arrived"
        self.arrived_at = now
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation
        self.get_logger().info(f"Arrived beside {self.flight_label}; holding it in the camera")
        self._say_flight(f"Arrived at {self.flight_label}")

    def _arrived(self, now: float) -> None:
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target.pose.orientation = self.pose.pose.orientation
        if self.arrived_at is not None and now - self.arrived_at >= 1.4:
            self._land()

    def _land(self) -> None:
        now = monotonic()
        # A nose-up landing can bring a horizon sign into the top of the frame
        # after the search gave up. Track that box instead of finishing the landing.
        if (
            not self.reached
            and self.phase == "landing"
            and self.pose is not None
            and self.target is not None
            and self.det is not None
            and now - self.last_valid_detection_at <= 1.5
            and self.det[0] >= 0.07
            and abs(self.det[1]) <= 0.25
        ):
            self.phase = "center"
            self.center_started = now
            self.center_hold_since = None
            self.center_yaw = yaw_of(self.pose.pose.orientation)
            self.landing_started = None
            self.land_timeout_logged = False
            self.target.pose.position.x = self.pose.pose.position.x
            self.target.pose.position.y = self.pose.pose.position.y
            self.target.pose.position.z = self.pose.pose.position.z
            self.target.pose.orientation = self.pose.pose.orientation
            self.visual_anchor = self.pose.pose.position.z
            self.last_request = 0.0
            self._call_mode(now, "OFFBOARD")
            self.get_logger().info(
                f"Target appeared during landing (error_x={self.det[1]:+.3f}, "
                f"error_y={self.det[2]:+.3f}); centering on it"
            )
            return
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
        # The land detector follows the bad height estimate and can stay in
        # LANDING after the motors have stopped. A disarm is the completion.
        if disarmed and (landed or (self.landing_started is not None and now - self.landing_started >= 6.0)):
            self.phase = "done"
            if self.reached and self.flight_label:
                self._say_flight(f"Landed beside {self.flight_label}")
            elif self.flight_label and self.approach_distance > 0.2:
                self._say_flight(f"Landed after flying to {self.flight_label}")
            elif self.flight_label:
                self._say_flight(f"Flight to {self.flight_label} ended before reaching it")
            self.get_logger().info("Landing confirmed: on ground and disarmed")
            return
        if self.landing_started is not None and now - self.landing_started >= 6.0 and self.state is not None and self.state.armed:
            if now - self.last_request >= 1.0 and self.arm_client.service_is_ready():
                req = CommandBool.Request()
                req.value = False
                self.arm_client.call_async(req)
                self.last_request = now
                self.get_logger().info("Requesting disarm after landing")
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
