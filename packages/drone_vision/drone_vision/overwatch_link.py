"""OVERWATCH drone-side link: camera frames up, flight commands down.

Runs in the project workbench on the drone (the virtual DD24, or a physical
drone running the same ROS 2 + PX4 + MAVROS stack). It serves one OVERWATCH
console at a time over TCP. A newer connection replaces the older one.

Wire format (both directions). Every message is

    uint32 big-endian length of everything after it | uint8 kind | payload

    kind 1: a UTF-8 JSON object
    kind 2: an image: uint32 big-endian header length | JSON header | JPEG bytes

Flight model. After a takeoff command, PX4 climbs in AUTO.TAKEOFF. Once it has
risen `handoff_rise_m`, this node streams a position setpoint and switches PX4
to OFFBOARD, where it holds position at `takeoff_altitude_m`. While the console
has enabled manual control, fresh velocity commands (body frame) nudge that
setpoint, which is kept on a short leash around the measured pose. Velocity
commands older than `command_timeout_s` are ignored, so the drone holds
position. If the console disconnects while this node is flying the drone, it
holds for `link_loss_land_s` and then lands. LAND uses AUTO.LAND; KILL
force-disarms through MAV_CMD_COMPONENT_ARM_DISARM.
"""

from __future__ import annotations

import json
import os
import queue
import socket
import struct
import threading
from math import atan2, cos, hypot, isfinite, sin
from time import monotonic, time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, CommandLong, SetMode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

PROTOCOL_VERSION = 1
KIND_JSON = 1
KIND_IMAGE = 2
MAX_MESSAGE_BYTES = 8 * 1024 * 1024

MAV_CMD_COMPONENT_ARM_DISARM = 400
FORCE_DISARM_MAGIC = 21196.0

LANDED_ON_GROUND = ExtendedState.LANDED_STATE_ON_GROUND
AIRBORNE_STATES = (ExtendedState.LANDED_STATE_IN_AIR, ExtendedState.LANDED_STATE_TAKEOFF)


def yaw_of(q) -> float:
    return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def encode_json(obj: dict) -> bytes:
    body = json.dumps(obj, separators=(",", ":")).encode()
    return struct.pack(">IB", len(body) + 1, KIND_JSON) + body


def encode_image(header: dict, jpeg: bytes) -> bytes:
    head = json.dumps(header, separators=(",", ":")).encode()
    payload = struct.pack(">I", len(head)) + head + jpeg
    return struct.pack(">IB", len(payload) + 1, KIND_IMAGE) + payload


class ClientConnection:
    """One console connection: a reader thread and a writer thread.

    The writer sends queued JSON first, then only the newest camera frame, so
    a slow console drops frames instead of building up delay.
    """

    def __init__(self, sock: socket.socket, address, on_message, on_closed) -> None:
        self.sock = sock
        self.address = address
        self._on_message = on_message
        self._on_closed = on_closed
        self._json: queue.SimpleQueue[bytes] = queue.SimpleQueue()
        self._frame: bytes | None = None
        self._wake = threading.Condition()
        self._closed = threading.Event()
        self._close_lock = threading.Lock()
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(5.0)
        threading.Thread(target=self._read_loop, daemon=True).start()
        threading.Thread(target=self._write_loop, daemon=True).start()

    @property
    def closed(self) -> bool:
        return self._closed.is_set()

    def send_json(self, obj: dict) -> None:
        if not self.closed:
            self._json.put(encode_json(obj))
            with self._wake:
                self._wake.notify()

    def offer_frame(self, data: bytes) -> None:
        with self._wake:
            self._frame = data
            self._wake.notify()

    def close(self) -> None:
        with self._close_lock:
            if self._closed.is_set():
                return
            self._closed.set()
        with self._wake:
            self._wake.notify_all()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
        self._on_closed(self)

    def _recv_exact(self, count: int) -> bytes | None:
        chunks = []
        while count:
            try:
                chunk = self.sock.recv(count)
            except socket.timeout:
                if self.closed:
                    return None
                continue
            except OSError:
                return None
            if not chunk:
                return None
            chunks.append(chunk)
            count -= len(chunk)
        return b"".join(chunks)

    def _read_loop(self) -> None:
        while not self.closed:
            head = self._recv_exact(5)
            if head is None:
                break
            length, kind = struct.unpack(">IB", head)
            if length < 1 or length > MAX_MESSAGE_BYTES:
                break
            body = self._recv_exact(length - 1)
            if body is None:
                break
            if kind != KIND_JSON:
                continue
            try:
                message = json.loads(body)
            except ValueError:
                continue
            if isinstance(message, dict):
                self._on_message(self, message)
        self.close()

    def _write_loop(self) -> None:
        while not self.closed:
            with self._wake:
                while not self.closed and self._json.empty() and self._frame is None:
                    self._wake.wait(timeout=1.0)
                frame, self._frame = self._frame, None
            try:
                while not self._json.empty():
                    self.sock.sendall(self._json.get_nowait())
                if frame is not None:
                    self.sock.sendall(frame)
            except (OSError, queue.Empty):
                break
        self.close()


class OverwatchLink(Node):
    def __init__(self) -> None:
        super().__init__("overwatch_link")
        robot = os.environ.get("VEHICLE_NAME", "birdie")
        defaults = {
            "port": 8765,
            "bind_address": "0.0.0.0",
            "camera_topic": f"/{robot}/camera_node/image/compressed",
            "takeoff_altitude_m": 0.8,
            "handoff_rise_m": 0.35,
            "min_altitude_m": 0.3,
            "max_altitude_m": 1.6,
            "takeoff_timeout_s": 30.0,
            "handoff_timeout_s": 6.0,
            "command_timeout_s": 0.3,
            "link_loss_land_s": 3.0,
            "pose_stale_s": 1.0,
            "leash_m": 0.25,
            "yaw_leash_rad": 0.5,
            "max_speed_m_s": 0.6,
            "max_yaw_rate_rad_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.robot = robot

        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CompressedImage, self._p("camera_topic"), self._on_image, camera_qos)
        self.create_subscription(State, "/mavros/state", self._on_state, latched)
        self.create_subscription(ExtendedState, "/mavros/extended_state", self._on_ext, best_effort)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self._on_pose, best_effort)
        self.setpoint_pub = self.create_publisher(PoseStamped, "/mavros/setpoint_position/local", best_effort)
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.command_client = self.create_client(CommandLong, "/mavros/cmd/command")

        # Vehicle state (written by ROS callbacks, read by the control timer).
        self.state: State | None = None
        self.ext: ExtendedState | None = None
        self.pose: PoseStamped | None = None
        self.pose_at = self.ext_at = 0.0
        self.ground_z: float | None = None

        # Control state (only touched in the ROS executor thread).
        self.phase = "idle"  # idle | takeoff | handoff | hold | landing
        self.phase_since = monotonic()
        self.target: PoseStamped | None = None
        self.target_yaw = 0.0
        self.start_z = 0.0
        self.handoff_stream_since: float | None = None
        self.last_request = 0.0
        self.manual_enabled = False
        self.velocity = (0.0, 0.0, 0.0, 0.0)  # forward, left, up, yaw rate
        self.velocity_at = 0.0
        self.last_tick = monotonic()

        # Networking.
        self.client: ClientConnection | None = None
        self.client_lost_at: float | None = None
        self.last_seq = -1
        self.inbox: queue.SimpleQueue = queue.SimpleQueue()
        self.frame_count = 0
        self.frame_times: list[float] = []
        self._client_lock = threading.Lock()
        # create_server sets SO_REUSEADDR, so a restarted link can rebind at once.
        self._server = socket.create_server((self._p("bind_address"), int(self._p("port"))))
        threading.Thread(target=self._accept_loop, daemon=True).start()

        self.create_timer(0.05, self._control_tick)
        self.create_timer(0.1, self._send_state)
        self.get_logger().info(
            f"OVERWATCH link on {self._p('bind_address')}:{self._p('port')}, camera "
            f"{self._p('camera_topic')}. Flight commands are accepted only from a connected console."
        )

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ---------------------------------------------------------------- network
    def _accept_loop(self) -> None:
        while True:
            try:
                sock, address = self._server.accept()
            except OSError:
                return
            # Queue the reset before the new connection can deliver commands.
            self.inbox.put(("connected", address))
            with self._client_lock:
                old = self.client
                conn = ClientConnection(sock, address, self._on_client_message, self._on_client_closed)
                self.client = conn
            if old is not None:
                old.close()
            conn.send_json({
                "type": "hello", "proto": PROTOCOL_VERSION, "robot": self.robot,
                "camera_topic": self._p("camera_topic"),
            })

    def _on_client_message(self, conn: ClientConnection, message: dict) -> None:
        if conn is self.client:
            self.inbox.put(("message", message))

    def _on_client_closed(self, conn: ClientConnection) -> None:
        with self._client_lock:
            if conn is self.client:
                self.client = None
                self.inbox.put(("disconnected", conn.address))

    def _send(self, obj: dict) -> None:
        client = self.client
        if client is not None:
            client.send_json(obj)

    def _event(self, level: str, text: str, **extra) -> None:
        getattr(self.get_logger(), "error" if level == "error" else "warning" if level == "warn" else "info")(text)
        self._send({"type": "event", "level": level, "text": text, "t": time(), **extra})

    # ------------------------------------------------------------- ROS input
    def _on_image(self, msg: CompressedImage) -> None:
        now = monotonic()
        self.frame_count += 1
        self.frame_times.append(now)
        if len(self.frame_times) > 40:
            del self.frame_times[:-40]
        client = self.client
        if client is None:
            return
        header = {
            "seq": self.frame_count,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "format": msg.format,
            "frame_id": msg.header.frame_id,
        }
        client.offer_frame(encode_image(header, bytes(msg.data)))

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_ext(self, msg: ExtendedState) -> None:
        self.ext, self.ext_at = msg, monotonic()

    def _on_pose(self, msg: PoseStamped) -> None:
        self.pose, self.pose_at = msg, monotonic()
        on_ground = self.ext is not None and self.ext.landed_state == LANDED_ON_GROUND
        if on_ground and self.phase == "idle":
            self.ground_z = msg.pose.position.z

    # -------------------------------------------------------------- commands
    def _drain_inbox(self) -> None:
        while True:
            try:
                kind, payload = self.inbox.get_nowait()
            except queue.Empty:
                return
            if kind == "connected":
                self.client_lost_at = None
                self.last_seq = -1
                self.manual_enabled = False
                self.velocity_at = 0.0
                self.get_logger().info(f"Console connected from {payload[0]}")
            elif kind == "disconnected":
                self.client_lost_at = monotonic()
                self.manual_enabled = False
                self.velocity_at = 0.0
                if self.phase in ("takeoff", "handoff", "hold"):
                    self.get_logger().warning(
                        f"Console disconnected while flying; holding, landing in {self._p('link_loss_land_s')} s")
                else:
                    self.get_logger().info("Console disconnected")
            elif kind == "message":
                self._handle(payload)

    def _ack(self, seq, cmd: str, ok: bool, detail: str = "") -> None:
        self._send({"type": "ack", "seq": seq, "cmd": cmd, "ok": ok, "detail": detail})

    def _handle(self, message: dict) -> None:
        if message.get("type") == "ping":
            self._send({"type": "pong", "t": message.get("t")})
            return
        if message.get("type") != "cmd":
            return
        seq = message.get("seq")
        if not isinstance(seq, int) or seq <= self.last_seq:
            return  # duplicate or out of order: never act on an older command
        self.last_seq = seq
        cmd = message.get("cmd")
        if cmd == "velocity":
            if not self.manual_enabled:
                return
            try:
                values = tuple(float(message.get(k, 0.0)) for k in ("forward", "left", "up", "yaw_rate"))
            except (TypeError, ValueError):
                return
            if not all(isfinite(v) for v in values):
                return
            speed = float(self._p("max_speed_m_s"))
            yaw_rate = float(self._p("max_yaw_rate_rad_s"))
            self.velocity = (clamp(values[0], -speed, speed), clamp(values[1], -speed, speed),
                             clamp(values[2], -speed, speed), clamp(values[3], -yaw_rate, yaw_rate))
            self.velocity_at = monotonic()
        elif cmd == "manual":
            self.manual_enabled = bool(message.get("enabled"))
            self.velocity_at = 0.0
            self._ack(seq, cmd, True, "manual control " + ("enabled" if self.manual_enabled else "disabled"))
        elif cmd == "hold":
            self.velocity_at = 0.0
            self._ack(seq, cmd, True, "holding position")
        elif cmd == "arm":
            self._call_arm(True, seq, cmd)
        elif cmd == "disarm":
            self._call_arm(False, seq, cmd)
        elif cmd == "takeoff":
            self._start_takeoff(seq)
        elif cmd == "land":
            self._start_landing(f"console {message.get('reason', 'land')}")
            self._ack(seq, cmd, True, "AUTO.LAND requested")
        elif cmd == "kill":
            self._kill(seq)
        else:
            self._ack(seq, str(cmd), False, "unknown command")

    def _call_arm(self, value: bool, seq, cmd: str) -> None:
        if not self.arm_client.service_is_ready():
            self._ack(seq, cmd, False, "arming service not available")
            return
        request = CommandBool.Request()
        request.value = value
        future = self.arm_client.call_async(request)
        future.add_done_callback(lambda f: self._ack(
            seq, cmd, bool(f.result() and f.result().success),
            "" if f.result() and f.result().success else "PX4 refused"))

    def _set_mode(self, mode: str) -> None:
        now = monotonic()
        if now - self.last_request < 1.0 or not self.mode_client.service_is_ready():
            return
        request = SetMode.Request()
        request.custom_mode = mode
        self.mode_client.call_async(request)
        self.last_request = now

    def _set_phase(self, phase: str) -> None:
        if phase != self.phase:
            self.get_logger().info(f"phase {self.phase} -> {phase}")
        self.phase, self.phase_since = phase, monotonic()

    def _start_takeoff(self, seq) -> None:
        if self.phase != "idle":
            self._ack(seq, "takeoff", False, f"busy ({self.phase})")
            return
        if self.pose is None or monotonic() - self.pose_at > float(self._p("pose_stale_s")):
            self._ack(seq, "takeoff", False, "no fresh local position from PX4")
            return
        if self.ext is None or self.ext.landed_state != LANDED_ON_GROUND:
            self._ack(seq, "takeoff", False, "not confirmed on the ground")
            return
        if self.state is None or not self.state.connected:
            self._ack(seq, "takeoff", False, "MAVROS is not connected to PX4")
            return
        self.start_z = self.pose.pose.position.z
        self.last_request = 0.0
        self._set_phase("takeoff")
        self._ack(seq, "takeoff", True, "AUTO.TAKEOFF requested")

    def _start_landing(self, reason: str) -> None:
        if self.phase != "landing":
            self._event("info", f"Landing ({reason})")
        self.manual_enabled = False
        self.velocity_at = 0.0
        self.last_request = 0.0
        self._set_phase("landing")

    def _kill(self, seq) -> None:
        self.manual_enabled = False
        self.velocity_at = 0.0
        self._set_phase("idle")  # stop streaming setpoints immediately
        if not self.command_client.service_is_ready():
            self._ack(seq, "kill", False, "command service not available")
            return
        request = CommandLong.Request()
        request.broadcast = False
        request.command = MAV_CMD_COMPONENT_ARM_DISARM
        request.confirmation = 0
        request.param1 = 0.0
        request.param2 = FORCE_DISARM_MAGIC
        future = self.command_client.call_async(request)
        future.add_done_callback(lambda f: self._ack(
            seq, "kill", bool(f.result() and f.result().success),
            "force-disarm sent" if f.result() and f.result().success else "PX4 refused force-disarm"))
        self._event("warn", "KILL: force-disarm requested")

    # --------------------------------------------------------------- control
    def _hold_here(self) -> None:
        self.target = PoseStamped()
        self.target.header.frame_id = self.pose.header.frame_id or "map"
        self.target.pose.position.x = self.pose.pose.position.x
        self.target.pose.position.y = self.pose.pose.position.y
        self.target.pose.position.z = self.pose.pose.position.z
        self.target_yaw = yaw_of(self.pose.pose.orientation)

    def _stream_target(self) -> None:
        self.target.pose.orientation.x = 0.0
        self.target.pose.orientation.y = 0.0
        self.target.pose.orientation.z = sin(self.target_yaw / 2.0)
        self.target.pose.orientation.w = cos(self.target_yaw / 2.0)
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target)

    def _apply_velocity(self, dt: float) -> None:
        fresh = self.manual_enabled and monotonic() - self.velocity_at <= float(self._p("command_timeout_s"))
        if not fresh:
            return
        forward, left, up, yaw_rate = self.velocity
        pose = self.pose.pose
        yaw = yaw_of(pose.orientation)
        t = self.target.pose.position
        t.x += (forward * cos(yaw) - left * sin(yaw)) * dt
        t.y += (forward * sin(yaw) + left * cos(yaw)) * dt
        t.z += up * dt
        self.target_yaw = wrap(self.target_yaw + yaw_rate * dt)
        # Keep the setpoint on a short leash so it cannot run away from the drone.
        leash = float(self._p("leash_m"))
        dx, dy = t.x - pose.position.x, t.y - pose.position.y
        distance = hypot(dx, dy)
        if distance > leash:
            t.x = pose.position.x + dx * leash / distance
            t.y = pose.position.y + dy * leash / distance
        t.z = clamp(t.z, pose.position.z - leash, pose.position.z + leash)
        t.z = clamp(t.z, self.start_z + float(self._p("min_altitude_m")),
                    self.start_z + float(self._p("max_altitude_m")))
        yaw_leash = float(self._p("yaw_leash_rad"))
        self.target_yaw = wrap(yaw + clamp(wrap(self.target_yaw - yaw), -yaw_leash, yaw_leash))

    def _control_tick(self) -> None:
        now = monotonic()
        dt = min(0.1, now - self.last_tick)
        self.last_tick = now
        self._drain_inbox()
        flying = self.phase in ("takeoff", "handoff", "hold")
        if flying and self.client_lost_at is not None and now - self.client_lost_at > float(self._p("link_loss_land_s")):
            self._start_landing("console link lost")
            return
        pose_fresh = self.pose is not None and now - self.pose_at <= float(self._p("pose_stale_s"))
        mode = self.state.mode if self.state else ""
        armed = bool(self.state and self.state.armed)

        if self.phase == "takeoff":
            if now - self.phase_since > float(self._p("takeoff_timeout_s")):
                self._event("error", "Takeoff did not reach handoff height in time")
                self._start_landing("takeoff timeout")
                return
            if not pose_fresh:
                return
            if mode != "AUTO.TAKEOFF":
                self._set_mode("AUTO.TAKEOFF")
                return
            if not armed:
                if now - self.last_request >= 1.0 and self.arm_client.service_is_ready():
                    request = CommandBool.Request()
                    request.value = True
                    self.arm_client.call_async(request)
                    self.last_request = now
                return
            rise = self.pose.pose.position.z - self.start_z
            airborne = self.ext is not None and self.ext.landed_state in AIRBORNE_STATES
            if rise >= float(self._p("handoff_rise_m")) and airborne:
                self._hold_here()
                self.target.pose.position.z = self.start_z + clamp(
                    float(self._p("takeoff_altitude_m")),
                    float(self._p("min_altitude_m")), float(self._p("max_altitude_m")))
                self.handoff_stream_since = now
                self._set_phase("handoff")
        elif self.phase == "handoff":
            if not pose_fresh:
                self._event("error", "Local position went stale during handoff")
                self._start_landing("stale pose")
                return
            self._stream_target()
            if mode == "OFFBOARD":
                self._set_phase("hold")
                self._event("info", "OFFBOARD: holding position")
            elif now - self.phase_since > float(self._p("handoff_timeout_s")):
                self._event("error", "PX4 did not accept OFFBOARD")
                self._start_landing("OFFBOARD refused")
            elif now - self.handoff_stream_since >= 1.0:
                self._set_mode("OFFBOARD")
        elif self.phase == "hold":
            if not armed or mode != "OFFBOARD":
                self._event("warn", f"PX4 left OFFBOARD (mode={mode}, armed={armed}); OVERWATCH released control")
                self._set_phase("idle")
                return
            if not pose_fresh:
                self._event("error", "Local position went stale; landing")
                self._start_landing("stale pose")
                return
            self._apply_velocity(dt)
            self._stream_target()
        elif self.phase == "landing":
            if mode != "AUTO.LAND" and armed:
                self._set_mode("AUTO.LAND")
            on_ground = self.ext is not None and self.ext.landed_state == LANDED_ON_GROUND
            if on_ground and not armed:
                self._event("info", "Landed and disarmed")
                self._set_phase("idle")

    def _send_state(self) -> None:
        if self.client is None:
            return
        now = monotonic()
        times = self.frame_times
        camera_hz = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0
        if times and now - times[-1] > 1.0:
            camera_hz = 0.0
        pose = None
        if self.pose is not None:
            p = self.pose.pose.position
            pose = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw_of(self.pose.pose.orientation),
                    "age_s": round(now - self.pose_at, 3)}
        reference_z = self.start_z if self.phase != "idle" else self.ground_z
        altitude = None
        if self.pose is not None and reference_z is not None:
            altitude = self.pose.pose.position.z - reference_z
        self._send({
            "type": "state",
            "t": time(),
            "mavros_connected": bool(self.state and self.state.connected),
            "armed": bool(self.state and self.state.armed),
            "mode": self.state.mode if self.state else "",
            "landed_state": self.ext.landed_state if self.ext else 0,
            "phase": self.phase,
            "manual_enabled": self.manual_enabled,
            "altitude_m": altitude,
            "pose": pose,
            "camera_hz": round(camera_hz, 1),
            "command_age_s": round(now - self.velocity_at, 3) if self.velocity_at else None,
        })

    def destroy_node(self) -> bool:
        try:
            self._server.close()
        except OSError:
            pass
        if self.client is not None:
            self.client.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OverwatchLink()
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
