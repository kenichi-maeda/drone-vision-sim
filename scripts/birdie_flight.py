#!/usr/bin/env python3
"""Climb on offboard attitude thrust, circle once, fly to the duck, then land.

This runs beside MAVROS on birdie. It does not arm. Mission Control keeps
publishing throttle 0 on the manual-control stick, which chopped the last
climb, so this script does not use that stick. It commands motor thrust
directly. COM_RC_OVERRIDE has to stay 0, or that stick kicks the mode back
and the motors return to idle.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget, State, StatusText
from mavros_msgs.srv import SetMode
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32MultiArray, String


SCAN_YAW = 0.18
MAX_YAW = 0.18
PITCH_FORWARD = -0.03
# 50 percent with no height reading held a little above the target. Capping
# the same case at 48 percent only skimmed, and the 20 second "did not climb"
# cut then dropped it to 34 percent in the corner. Above a real 0.65 m, or
# on a climb faster than about 0.4 m/s, thrust eases toward 44 percent.
# Three feet is 0.91 m. The circle waits until the height has stayed near 0.50 m.
TARGET_HEIGHT = 0.50
SETTLE_HEIGHT = 0.40
TAKEOFF_THRUST = 0.46
HOVER_MIN = 0.40
HOVER_MAX = 0.50
MAX_HEIGHT = 0.65
DESCEND_THRUST = 0.34
MIN_THRUST = 0.34
MAX_THRUST = 0.50
# Attitude pitch and roll stay at 0. Level comes from SENS_BOARD_Y_OFF 1.66
# and SENS_BOARD_X_OFF -0.25.
DRIFT_ROLL = 0.0
DRIFT_PITCH = 0.0
FLYING_HEIGHT = 0.10
HOVER_SECONDS = 8.0
CLIMB_LIMIT = 20.0
FULL_TURN = math.tau
MISSION_SECONDS = 120.0
CENTER = 0.18
IGNORE_RATES = 7


def wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    return angle


def yaw_of(quat) -> float:
    siny = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny, cosy)


def quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    msg = Quaternion()
    msg.w = cr * cp * cy + sr * sp * sy
    msg.x = sr * cp * cy - cr * sp * sy
    msg.y = cr * sp * cy + sr * cp * sy
    msg.z = cr * cp * sy - sr * sp * cy
    return msg


class BirdieFlight(Node):
    def __init__(self) -> None:
        super().__init__("birdie_flight")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state: State | None = None
        self.duck: list[float] | None = None
        self.duck_at = 0.0
        self.yaw_now: float | None = None
        self.agl: float | None = None
        self.agl_at = 0.0
        self.mode_asked = 0.0
        self.mission_started: float | None = None
        self.centered_since: float | None = None
        self.last_tick: float | None = None
        self.yaw0: float | None = None
        self.yaw_travel = 0.0
        self.best_yaw: float | None = None
        self.best_heading: float | None = None
        self.best_error: float | None = None
        self.scan_done = False
        self.height_gap = 0.0
        self.stuck_low = 0.0
        self.range_rejects = 0
        self.turned = 0.0
        self.prev_yaw: float | None = None
        self.hover_credit = 0.0
        self.hover_done = False
        self.out_of_band = 0.0
        self.was_airborne = False
        self.airborne_since = None
        self.settled_since: float | None = None
        self.thrust_cmd = TAKEOFF_THRUST
        self.thrust_phase_name = "takeoff"
        self.t_base: float | None = None
        self.height_samples: list[float] = []
        self.last_raw: float | None = None
        self.h_f: float | None = None
        self.v_f = 0.0
        self.blind_at_cap = 0.0
        self.land_reason = ""
        self.thrust = 0.0
        self.phase = "waiting"
        self.last_log = 0.0
        self.attitude_pub = self.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", qos)
        self.status_pub = self.create_publisher(String, "/drone_vision/flight_status", 10)
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.thrust_scale = self.create_client(SetParameters, "/mavros/setpoint_raw/set_parameters")
        self.thrust_scale_set = False
        self.thrust_scale_busy = False
        self.create_subscription(State, "/mavros/state", self._state, 10)
        self.create_subscription(Float32MultiArray, "/drone_vision/yolo_detection", self._duck, 10)
        self.create_subscription(Imu, "/mavros/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(Range, "/birdie/altitude_node/altitude", self._range, qos_profile_sensor_data)
        self.create_subscription(StatusText, "/mavros/statustext/recv", self._flight_controller_text, qos_profile_sensor_data)
        self.create_timer(0.05, self._tick)
        self.create_timer(1.0, self._ensure_thrust_scale)
        self.get_logger().info("Flight script is running on birdie. It commands climb thrust and will not arm.")

    def _flight_controller_text(self, msg: StatusText) -> None:
        text = msg.text.strip()
        if text:
            self.get_logger().warn(f"Flight controller: {text}")

    def _ensure_thrust_scale(self) -> None:
        # The plugin default is NaN, and a NaN scaling makes MAVROS drop the
        # thrust message. The watch callback applies a live parameter change.
        if self.thrust_scale_set or self.thrust_scale_busy or not self.thrust_scale.service_is_ready():
            return
        parameter = Parameter()
        parameter.name = "thrust_scaling"
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0)
        request = SetParameters.Request()
        request.parameters = [parameter]
        self.thrust_scale_busy = True
        future = self.thrust_scale.call_async(request)
        future.add_done_callback(self._thrust_scale_done)

    def _thrust_scale_done(self, future) -> None:
        self.thrust_scale_busy = False
        result = future.result()
        if result and result.results and result.results[0].successful:
            self.thrust_scale_set = True
            self.get_logger().info("Thrust scaling is 1. Motor thrust will be sent.")
        else:
            self.get_logger().warning("Thrust scaling was not set.")

    def _state(self, msg: State) -> None:
        self.state = msg

    def _duck(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 8:
            self.duck = [float(value) for value in msg.data]
            self.duck_at = time.monotonic()

    def _imu(self, msg: Imu) -> None:
        self.yaw_now = yaw_of(msg.orientation)

    def _range(self, msg: Range) -> None:
        reading = float(msg.range)
        if reading < max(0.05, float(msg.min_range)) or reading > float(msg.max_range):
            return
        now = time.monotonic()
        # One wild sample is ignored. A reading that keeps coming is the
        # drone, including a fast climb. Rejecting it forever left the thrust
        # at a climb while the log already showed the cage ceiling.
        if self.agl is not None and abs(reading - self.agl) > 0.40:
            self.range_rejects = getattr(self, "range_rejects", 0) + 1
            if self.range_rejects < 3:
                return
        self.range_rejects = 0
        self.agl = reading
        self.agl_at = now

    def _fresh(self, now: float) -> list[float] | None:
        if self.duck is None or now - self.duck_at > 0.8:
            return None
        return self.duck

    def _height(self, now: float) -> float | None:
        if self.agl is None or now - self.agl_at > 0.5:
            return None
        return self.agl

    def _range_code(self, duck: list[float]) -> float:
        height = duck[7]
        code = duck[8] if len(duck) > 8 else 0.0
        if code != 0.0:
            return code
        if height > 0.40:
            return 3.0
        if height < 0.08:
            return 2.0
        return 4.0

    def _remember(self, duck: list[float]) -> None:
        error_x = abs(duck[4])
        if self._range_code(duck) == 4.0:
            return
        if self.yaw_now is None:
            return
        if self.best_error is None or error_x < self.best_error:
            self.best_error = error_x
            self.best_heading = self.yaw_now
            self.best_yaw = self.yaw_travel

    def _set_mode(self, mode: str) -> None:
        if not self.mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        self.mode_client.call_async(request)

    def _yaw_cmd(self) -> float:
        base = self.yaw0 if self.yaw0 is not None else (self.yaw_now if self.yaw_now is not None else 0.0)
        return base + self.yaw_travel

    def _attitude(self, pitch: float, yaw_rate: float, thrust: float, dt: float) -> None:
        if self.yaw0 is None and self.yaw_now is not None:
            self.yaw0 = self.yaw_now
        self.yaw_travel += yaw_rate * dt
        # Hold the heading captured at arm. Following the live IMU yaw let
        # every disturbance become the new setpoint, so a skim walked into
        # the corner. If the body is far from the command, rebase once.
        cmd_yaw = self._yaw_cmd()
        if self.yaw_now is not None and abs(wrap(cmd_yaw - self.yaw_now)) > 0.50:
            self.yaw0 = self.yaw_now
            self.yaw_travel = 0.0
            cmd_yaw = self.yaw_now
        self.thrust = 0.0 if thrust <= 0.0 else max(MIN_THRUST, min(MAX_THRUST, thrust))
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type_mask = IGNORE_RATES
        msg.orientation = quaternion(DRIFT_ROLL, pitch + DRIFT_PITCH, cmd_yaw)
        msg.thrust = float(self.thrust)
        self.attitude_pub.publish(msg)

    def _reset_mission(self) -> None:
        self.mission_started = None
        self.centered_since = None
        self.yaw0 = None
        self.yaw_travel = 0.0
        self.best_yaw = None
        self.best_heading = None
        self.best_error = None
        self.scan_done = False
        self.height_gap = 0.0
        self.stuck_low = 0.0
        self.range_rejects = 0
        self.turned = 0.0
        self.prev_yaw = None
        self.hover_credit = 0.0
        self.hover_done = False
        self.out_of_band = 0.0
        self.was_airborne = False
        self.airborne_since = None
        self.settled_since = None
        self.thrust_cmd = TAKEOFF_THRUST
        self.thrust_phase_name = "takeoff"
        self.t_base = None
        self.height_samples = []
        self.last_raw = None
        self.h_f = None
        self.v_f = 0.0
        self.blind_at_cap = 0.0
        self.land_reason = ""
        self.thrust = 0.0

    def _filtered_height(self, height: float | None, dt: float) -> bool:
        if height is None or height < 0.05:
            return False
        if self.h_f is None:
            self.h_f = height
            self.last_raw = height
            self.v_f = 0.0
            self.height_gap = 0.0
            return True
        dt = max(dt, 0.02)
        delta = height - self.h_f
        max_step = 2.0 * dt
        if abs(delta) > max_step:
            self.height_gap += dt
            followed = delta if self.height_gap >= 0.20 else math.copysign(max_step, delta)
        else:
            self.height_gap = 0.0
            followed = delta
        new_h = self.h_f + followed
        measured = followed / dt
        self.v_f += 0.35 * (measured - self.v_f)
        self.h_f = new_h
        self.last_raw = height
        return True

    def _slew_thrust(self, target: float, dt: float, up: float = 0.03, down: float = 0.08) -> float:
        delta = target - self.thrust_cmd
        limit = (up if delta >= 0.0 else down) * dt
        if abs(delta) > limit:
            delta = limit if delta > 0.0 else -limit
        self.thrust_cmd = max(HOVER_MIN, min(HOVER_MAX, self.thrust_cmd + delta))
        return self.thrust_cmd

    def _flight_thrust(self, dt: float, height: float | None) -> float:
        valid = self._filtered_height(height, dt)
        raw = height if height is not None and height >= 0.05 else None
        # The latest reading cuts thrust even when the filter is still catching up.
        too_high = (raw is not None and raw > MAX_HEIGHT) or (self.h_f is not None and self.h_f > MAX_HEIGHT)
        if too_high and self.v_f > -0.05:
            self.thrust_phase_name = "hold"
            self.t_base = min(self.thrust_cmd if self.t_base is None else self.t_base, 0.44)
            self.stuck_low = 0.0
            return self._slew_thrust(0.44, dt, down=0.20)
        if too_high:
            self.thrust_phase_name = "hold"
            self.t_base = min(self.thrust_cmd if self.t_base is None else self.t_base, 0.44)
            self.stuck_low = 0.0
            return self._slew_thrust(0.44, dt, down=0.10)
        if self.h_f is not None and self.v_f > 0.40:
            self.thrust_phase_name = "hold"
            self.t_base = min(self.thrust_cmd if self.t_base is None else self.t_base, 0.45)
            self.stuck_low = 0.0
            return self._slew_thrust(0.43, dt, down=0.12)
        if not valid:
            if self.h_f is not None and self.h_f > TARGET_HEIGHT:
                return self._slew_thrust(min(self.thrust_cmd, 0.44), dt, down=0.10)
            # 50 percent is the thrust that held a little high with no height.
            # 48 percent only skimmed, and a later cut to 34 percent pinned it.
            if self.thrust_phase_name == "takeoff":
                self.thrust_cmd = min(0.50, self.thrust_cmd + 0.015 * dt)
            return min(self.thrust_cmd, 0.50)
        if self.thrust_phase_name == "takeoff":
            self.thrust_cmd = min(0.50, self.thrust_cmd + 0.015 * dt)
            rising = self.h_f is not None and (self.v_f > 0.15 or self.h_f >= 0.35)
            if rising:
                self.thrust_phase_name = "hold"
                self.t_base = min(0.50, self.thrust_cmd)
            return min(self.thrust_cmd, 0.50)
        if self.t_base is None:
            self.t_base = min(0.50, self.thrust_cmd)
        error = TARGET_HEIGHT - self.h_f
        if self.h_f > 0.55:
            self.t_base = max(0.44, self.t_base - 0.04 * dt)
        elif self.h_f < 0.40 and self.v_f < 0.05:
            self.t_base = min(0.50, self.t_base + 0.02 * dt)
        desired = self.t_base + 0.03 * error - 0.06 * self.v_f
        return self._slew_thrust(desired, dt, up=0.03, down=0.08)

    def _motion(self, now: float, dt: float, elapsed: float) -> tuple[float, float, float, str]:
        height = self._height(now)
        thrust = self._flight_thrust(dt, height)
        if height is not None and height >= FLYING_HEIGHT:
            if not self.was_airborne:
                self.airborne_since = now
            self.was_airborne = True
        if self.yaw_now is not None:
            if self.prev_yaw is not None:
                self.turned += wrap(self.yaw_now - self.prev_yaw)
            self.prev_yaw = self.yaw_now
        duck = self._fresh(now)
        if not self.was_airborne:
            # A missing height is not proof it is still on the floor. The last
            # hop was already off the ground when this timer cut to 34 percent.
            return 0.0, 0.0, thrust, "climb"
        # A fresh reading inside the band earns hover time. One noisy sample
        # does not wipe it. A full second outside the band does.
        fresh = height is not None and self.h_f is not None
        in_band = fresh and 0.35 <= self.h_f <= MAX_HEIGHT
        if in_band:
            self.out_of_band = 0.0
            if not self.hover_done:
                self.hover_credit += dt
                if self.hover_credit >= HOVER_SECONDS:
                    self.hover_done = True
                    self.turned = 0.0
                    self.prev_yaw = self.yaw_now
        else:
            self.out_of_band += dt
            if self.out_of_band >= 1.0 and not self.hover_done:
                self.hover_credit = 0.0
        if not in_band and self.out_of_band >= 0.4:
            return 0.0, 0.0, thrust, "climb"
        if not self.hover_done:
            return 0.0, 0.0, thrust, "hover"
        if duck is not None:
            self._remember(duck)
        facing = duck is not None and abs(duck[4]) <= CENTER and self._range_code(duck) in (1.0, 2.0)
        if facing:
            if self.centered_since is None:
                self.centered_since = now
            # A positive error means the duck is on the right. Yaw the other way.
            yaw_rate = max(-MAX_YAW, min(MAX_YAW, -duck[4] * 0.8))
            if now - self.centered_since < 0.4:
                return 0.0, yaw_rate, thrust, "center"
            self.scan_done = True
            if self.yaw_now is not None:
                self.best_heading = self.yaw_now
        else:
            self.centered_since = None
        if not self.scan_done and self.turned < FULL_TURN:
            return 0.0, SCAN_YAW, thrust, "circle"
        if self.best_heading is None or self.yaw_now is None:
            self.land_reason = "none"
            return 0.0, 0.0, DESCEND_THRUST, "land"
        if duck is not None and self._range_code(duck) == 3.0:
            self.land_reason = "close"
            return 0.0, 0.0, DESCEND_THRUST, "land"
        remain = wrap(self.best_heading - self.yaw_now)
        if abs(remain) > 0.20 and not facing:
            yaw_rate = max(-MAX_YAW, min(MAX_YAW, remain * 0.8))
            return 0.0, yaw_rate, thrust, "return"
        if duck is None or self._range_code(duck) == 4.0 or abs(duck[4]) > CENTER:
            yaw_rate = 0.0 if duck is None else max(-MAX_YAW, min(MAX_YAW, -duck[4] * 0.8))
            return 0.0, yaw_rate, thrust, "center"
        yaw_rate = max(-MAX_YAW, min(MAX_YAW, -duck[4] * 0.8))
        return PITCH_FORWARD, yaw_rate, thrust, "approach"

    def _tick(self) -> None:
        now = time.monotonic()
        dt = 0.05 if self.last_tick is None else min(0.2, now - self.last_tick)
        self.last_tick = now
        armed = self.state is not None and bool(self.state.armed)
        mode = self.state.mode if self.state is not None else ""
        locked = self.state is not None and (self.state.system_status == 8 or "655360" in mode)
        if armed and locked:
            self.phase = "locked"
            self._attitude(0.0, 0.0, 0.0, dt)
            self._say(now, mode)
            return
        if not armed:
            self._reset_mission()
            if locked:
                self.phase = "locked"
                self._say(now, mode)
                return
            if mode not in ("", "STABILIZED") and now - self.mode_asked > 1.0:
                self._set_mode("STABILIZED")
                self.mode_asked = now
            self.phase = "waiting"
            self._say(now, mode)
            return
        if self.phase == "land":
            self._attitude(0.0, 0.0, DESCEND_THRUST, dt)
            self._say(now, mode)
            return
        if self.mission_started is None:
            self.mission_started = now
        elapsed = now - self.mission_started
        # The flight controller's first second is spool-up. Switching to
        # OFFBOARD during that second is when the last hop locked and cut
        # the motors. Wait until spool-up is over, while the setpoints
        # are already streaming.
        if elapsed > 1.2 and mode != "OFFBOARD" and now - self.mode_asked > 0.5:
            self._set_mode("OFFBOARD")
            self.mode_asked = now
        if elapsed > MISSION_SECONDS:
            self.land_reason = "time"
            self.phase = "land"
            self._attitude(0.0, 0.0, DESCEND_THRUST, dt)
            self._say(now, mode)
            return
        pitch, yaw_rate, thrust, phase = self._motion(now, dt, elapsed)
        self.phase = phase
        self._attitude(pitch, yaw_rate, thrust, dt)
        self._say(now, mode)

    def _say(self, now: float, mode: str) -> None:
        if now - self.last_log < 1.0:
            return
        self.last_log = now
        text = "Flight script running on birdie. Disarmed. After arming it climbs to about 0.50 m before it hovers, stays under 3 feet, holds level, then turns one circle, flies to the duck, and lands."
        height = self._height(now)
        if self.h_f is None:
            height_text = "no height" if height is None else f"sensor {height:.2f} m"
        elif height is not None and abs(height - self.h_f) > 0.15:
            height_text = f"height {self.h_f:.2f} m, sensor {height:.2f} m"
        else:
            height_text = f"height {self.h_f:.2f} m"
        pct = self.thrust * 100.0
        mode_text = mode or "no mode"
        if self.phase == "locked":
            text = "Flight controller locked and cut the motors."
        elif self.phase == "climb":
            text = f"Climbing. Motor thrust {pct:.0f} percent. Mode {mode_text}. {height_text}."
        elif self.phase == "hover":
            text = f"Hovering still. Motor thrust {pct:.0f} percent. Mode {mode_text}. {height_text}."
        elif self.phase == "circle":
            text = f"Hovering. Turning one full circle to find the duck. Motor thrust {pct:.0f} percent. Mode {mode_text}. {height_text}."
        elif self.phase == "return":
            text = f"Circle done. Turning back toward the duck. Motor thrust {pct:.0f} percent. Mode {mode_text}."
        elif self.phase == "center":
            text = f"Duck is in view. Facing it before moving forward. Motor thrust {pct:.0f} percent. Mode {mode_text}."
        elif self.phase == "approach":
            text = f"Flying toward the duck. Motor thrust {pct:.0f} percent. Mode {mode_text}."
        elif self.phase == "land":
            if self.land_reason == "none":
                text = "Full circle and no duck. Lowering the throttle to land."
            elif self.land_reason == "close":
                text = "Duck is close. Lowering the throttle to land."
            elif self.land_reason == "low":
                text = "It did not climb. Lowering the throttle."
            else:
                text = "Lowering the throttle to land."
        status = String()
        status.data = text
        self.status_pub.publish(status)
        self.get_logger().info(text)


def main() -> None:
    rclpy.init()
    node = BirdieFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
