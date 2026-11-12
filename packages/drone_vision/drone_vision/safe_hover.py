"""Compact YOLO-guided takeoff, visual approach, and landing node.

The controller uses PX4 AUTO.TAKEOFF, then a streamed MAVROS local-position
setpoint in OFFBOARD. YOLO supplies image-space target center and box size;
there is no metric range estimate. The approach advances continuously at a
bounded speed and lands when the centered box reaches a relative-size cue.
"""

from __future__ import annotations

from math import atan2, cos, isfinite, sin
from time import monotonic, time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import EstimatorStatus, ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Header, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def centering_factor(error: float, tolerance: float, max_error: float) -> float:
    if max_error <= tolerance:
        return 0.0
    return clamp((max_error - abs(error)) / (max_error - tolerance), 0.0, 1.0)


def yaw_of(q) -> float:
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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
            "climb_timeout_seconds": 30.0,
            "warmup_seconds": 2.0,
            # Allow the slow AUTO.TAKEOFF / OFFBOARD startup observed in
            # Matrix before the bounded search and approach phase budgets.
            "hard_timeout_seconds": 70.0,
            "landing_timeout_seconds": 40.0,
            "pose_stale_timeout_seconds": 1.5,
            "handoff_to_offboard": True,
            "handoff_rise_m": 0.40,
            "handoff_timeout_seconds": 4.0,
            "object_search_enabled": True,
            # At 0.31 rad/s this covers just over one full 360-degree yaw
            # sweep. Positive yaw turns the visible right-of-center sign toward
            # the camera center in the current test map.
            "object_search_seconds": 22.0,
            "object_search_initial_hold_seconds": 0.5,
            "object_search_yaw_rate_rad_s": 0.31,
            "object_search_detection_confidence": 0.10,
            "object_search_max_bbox_area": 0.18,
            "object_search_confirmation_count": 3,
            "object_search_max_acquisition_error_x": 0.45,
            "object_search_max_abs_error_y": 0.60,
            "object_tracking_max_error_x_delta": 0.30,
            "object_tracking_max_error_y_delta": 0.50,
            "object_tracking_max_area_ratio": 4.0,
            "object_centering_enabled": True,
            "object_centering_seconds": 6.0,
            "object_centering_kp_rad": 0.60,
            "object_centering_deadband": 0.08,
            "object_centering_step_limit_rad": 0.12,
            "object_centering_max_offset_rad": 1.20,
            "object_centering_loss_timeout_seconds": 1.5,
            "object_approach_enabled": True,
            "object_approach_speed_m_s": 0.08,
            "object_approach_size_growth_ratio": 2.5,
            "object_approach_center_tolerance": 0.20,
            "object_approach_max_error_x": 0.45,
            "object_approach_loss_timeout_seconds": 2.0,
            "object_approach_reacquire_seconds": 2.0,
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
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            qos,
        )
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.started = self.climb_started = self.handoff_started = None
        self.prestream_started = self.airborne_started = self.search_started = None
        self.center_started = self.approach_started = self.landing_started = None
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
        self.det_candidate_class_id: int | None = None
        self.det = None  # confidence, error_x, error_y, normalized box area, class_id
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
        if now - self.last_detection_at > 0.5 or self.det_candidate_class_id != int(class_id):
            self.detection_count = 0
        self.det_candidate_class_id = int(class_id)
        self.detection_count += 1
        self.last_detection_at = self.last_valid_detection_at = now
        self.det = (confidence, ex, ey, area, int(class_id))

    def _p(self, key: str):
        return self.get_parameter(key).value

    def _log_once(self, message: str) -> None:
        if message != self.wait_message:
            self.wait_message = message
            self.get_logger().info(message)

    def _tick(self) -> None:
        now = monotonic()
        phase_msg = String()
        phase_msg.data = self.phase
        self.flight_phase_pub.publish(phase_msg)
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
            if self.started is None and self.phase == "idle":
                self._log_once("Waiting for fresh MAVROS pose; not starting flight")
            else:
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
            self.target.pose.position.z = self.start_z + float(self._p("height_m"))
            self.phase = "warmup"
            self.get_logger().info(
                f"Mission enabled from current pose z={self.start_z:.2f} m; "
                f"handoff target z={self.target.pose.position.z:.2f} m"
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
        # AUTO.TAKEOFF owns the aircraft until the handoff point. Then stream
        # local setpoints continuously before requesting OFFBOARD.
        if self.prestream_started is not None or self.phase in ("search", "center", "approach", "reacquire"):
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
        assert self.state is not None and self.started is not None
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
        if self.state.mode != "AUTO.TAKEOFF":
            self._call_mode(now, "AUTO.TAKEOFF")
            return
        if not self.state.armed:
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
        if rise >= float(self._p("handoff_rise_m")) and landed_state in (
            ExtendedState.LANDED_STATE_TAKEOFF, ExtendedState.LANDED_STATE_IN_AIR
        ):
            if self.prestream_started is None:
                self.target.pose.position.x = self.pose.pose.position.x
                self.target.pose.position.y = self.pose.pose.position.y
                self.target.pose.orientation = self.pose.pose.orientation
                self.prestream_started = now
                self.phase = "prestream"
                self.get_logger().info(f"Takeoff rise {rise:.2f} m; streaming OFFBOARD setpoints")
            return
        if self.climb_started is not None and now - self.climb_started > float(self._p("climb_timeout_seconds")):
            self.get_logger().error(f"Takeoff failed to rise {float(self._p('handoff_rise_m')):.2f} m before deadline; landing")
            self._land()
        elif now - self.last_log > 1.0:
            self.get_logger().info(f"AUTO.TAKEOFF rise={rise:+.2f} m, landed_state={landed_state}; waiting for handoff height")
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
            # Never accept a detection acquired while grounded/taking off as
            # the airborne search lock; require fresh post-handoff frames.
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
                and now - self.last_valid_detection_at <= 0.5
                and (self.target_class_id is not None
                     or abs(self.det[2]) <= float(self._p("object_search_max_abs_error_y")))
                and self.det[0] >= float(self._p("object_search_detection_confidence"))
                and (self.target_class_id is None or self.det[4] == self.target_class_id))

    def _search(self, now: float) -> None:
        assert self.search_started is not None and self.search_yaw is not None
        elapsed = now - self.search_started
        if self._confirmed(now) and abs(self.det[1]) <= float(self._p("object_search_max_acquisition_error_x")):
            self.target_class_id = self.det[4]
            self.target.pose.orientation = self.pose.pose.orientation
            if abs(self.det[1]) <= float(self._p("object_approach_center_tolerance")):
                self._begin_approach(now)
            else:
                self.center_started = now
                self.center_yaw = yaw_of(self.pose.pose.orientation)
                self.phase = "center"
                self.get_logger().info(f"YOLO target acquired (error_x={self.det[1]:+.3f}); yaw-centering in place")
            return
        initial_hold = float(self._p("object_search_initial_hold_seconds"))
        duration = float(self._p("object_search_seconds"))
        if elapsed > initial_hold + duration:
            self.get_logger().warning("Target not found during bounded in-place scan; landing")
            self._land()
            return
        if elapsed > initial_hold:
            yaw = self.search_yaw + float(self._p("object_search_yaw_rate_rad_s")) * (elapsed - initial_hold)
            set_yaw(self.target, yaw)
        if now - self.last_log > 1.0:
            self.get_logger().info(f"Searching: elapsed={elapsed:.1f}s, detections={self.detection_count}")
            self.last_log = now

    def _steer(self, error: float, reference_yaw: float) -> float:
        """Correct from measured yaw so repeated frames cannot wind up a turn."""
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
        if self._confirmed(now) and abs(self.det[1]) <= float(self._p("object_approach_center_tolerance")):
            self._begin_approach(now)
            return
        if now - self.center_started > float(self._p("object_centering_seconds")):
            self.get_logger().warning("Could not center target within time limit; landing")
            self._land()
            return
        if self._confirmed(now) and self.last_valid_detection_at > self.last_control_detection_at:
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
            if self.approach_reacquire_attempted:
                self.get_logger().error("Target still lost after reacquisition attempt; landing")
                self._land()
                return
            self.approach_reacquire_attempted = True
            self.reacquire_started = now
            self.reacquire_yaw = yaw_of(self.pose.pose.orientation)
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
        _, error, _, area, _ = self.det
        centered = abs(error) <= float(self._p("object_approach_center_tolerance"))
        if area >= self.stop_area:
            if centered:
                self.get_logger().info(f"Reached visual-size goal ({area / self.initial_area:.2f}x); landing")
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
        fraction = clamp(area / self.stop_area, 0.0, 1.0)
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
        yaw = self.reacquire_yaw + float(self._p("object_search_yaw_rate_rad_s")) * elapsed
        set_yaw(self.target, yaw)

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
