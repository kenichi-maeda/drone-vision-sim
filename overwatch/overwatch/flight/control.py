"""Control modes, manual override, the motion gates, KILL and LAND NOW.

Motion commands are only produced from input that is fresh right now. Nothing
is buffered or replayed. When any gate is closed, a zero-velocity (hold)
command is sent instead, and the HUD shows the reason.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Callable

from ..events import EventLog
from .base import FlightAdapter, VehicleState
from .pursuit import NoPursuit, PursuitController, VelocityCommand

# Unit direction per held key: (forward, left, up, yaw_rate)
MOTION_KEYS: dict[str, tuple[int, int, int, int]] = {
    "forward": (1, 0, 0, 0), "back": (-1, 0, 0, 0),
    "left": (0, 1, 0, 0), "right": (0, -1, 0, 0),
    "up": (0, 0, 1, 0), "down": (0, 0, -1, 0),
    "yaw_left": (0, 0, 0, 1), "yaw_right": (0, 0, 0, -1),
}
ACTIONS = ("toggle_mode", "arm", "disarm", "takeoff", "land", "land_now", "kill")
ZERO = VelocityCommand()


@dataclass
class Health:
    feed_lost: bool
    inference_stalled: bool
    display_age_s: float | None  # age of the newest frame shown to the operator (already inferred)


class FlightControl:
    def __init__(self, cfg: dict[str, Any], adapter: FlightAdapter, events: EventLog,
                 pursuit: PursuitController | None = None,
                 notify: Callable[[str, str], None] | None = None) -> None:
        self.cfg = cfg
        self.adapter = adapter
        self.events = events
        self.pursuit = pursuit or NoPursuit()
        self.notify = notify or (lambda level, text: None)
        self.mode = "AUTONOMOUS"
        self.held: frozenset[str] = frozenset()
        self.typing = False
        self.input_at = 0.0
        self.command = ZERO
        self.block_reason: str | None = None
        self.velocity_input_on = False
        self._flag_sent_at = 0.0
        self._pending: dict[int, dict] = {}  # seq -> context, for logging KILL / LAND NOW acks
        self._last_target: dict | None = None
        adapter.on_ack(self._on_ack)

    # ------------------------------------------------------------------ input
    def set_input(self, keys: list[str], typing: bool, now: float) -> None:
        self.held = frozenset(k for k in keys if k in MOTION_KEYS)
        self.typing = bool(typing)
        self.input_at = now

    def clear_input(self) -> None:
        self.held = frozenset()

    # ---------------------------------------------------------------- actions
    async def set_mode(self, mode: str, source: str = "operator") -> str:
        if mode not in ("AUTONOMOUS", "MANUAL"):
            raise ValueError(mode)
        if mode == self.mode:
            return f"Already {mode}"
        self.mode = mode
        self.clear_input()
        self.events.record("MODE", mode=mode, source=source)
        return f"{mode} mode" + (": WASD/arrows move, R/F altitude, Q/E yaw" if mode == "MANUAL" else "")

    async def action(self, name: str, source: str = "key", target: dict | None = None) -> str:
        vehicle = self.adapter.vehicle_state()
        if name == "toggle_mode":
            return await self.set_mode("AUTONOMOUS" if self.mode == "MANUAL" else "MANUAL", source)
        if name in ("land_now", "kill"):
            return await self._emergency(name, source, vehicle, target)
        if name not in ACTIONS:
            raise ValueError(name)
        if not vehicle.link_connected:
            return f"{name.upper()} not sent: no link to the drone"
        seq = await getattr(self.adapter, name)("operator") if name == "land" else await getattr(self.adapter, name)()
        if seq is None:
            return f"{name.upper()} not sent: link dropped"
        self._pending[seq] = {"event": name.upper(), "log": False}
        return f"{name.upper()} sent"

    async def _emergency(self, name: str, source: str, vehicle: VehicleState, target: dict | None) -> str:
        self.clear_input()
        if name == "kill":
            action = self.cfg.get("estop_action", "kill")
            event = "KILL"
            seq = await (self.adapter.kill() if action == "kill" else self.adapter.land("estop"))
        else:
            action = "land"
            event = "LAND_NOW"
            seq = await self.adapter.land("land_now")
        delivered = seq is not None
        self.events.record(
            event, source=source, action=action, delivered=delivered, seq=seq,
            control_mode=self.mode, vehicle=vehicle.to_dict(), target=target,
        )
        if delivered:
            self._pending[seq] = {"event": event, "log": True}
            text = f"{event.replace('_', ' ')}: {'force-disarm' if action == 'kill' else 'AUTO.LAND'} sent"
            self.notify("warn", text)
            return text
        text = f"{event.replace('_', ' ')} NOT DELIVERED: no link to the drone"
        self.notify("error", text)
        return text

    def _on_ack(self, ack: dict) -> None:
        context = self._pending.pop(ack.get("seq"), None)
        if context is None:
            return
        if context["log"]:
            self.events.record("ACK", for_event=context["event"], seq=ack.get("seq"),
                               ok=ack.get("ok"), detail=ack.get("detail", ""))
        level = "info" if ack.get("ok") else "error"
        self.notify(level, f"{context['event'].replace('_', ' ')}: {'OK' if ack.get('ok') else 'REFUSED'}"
                           + (f" ({ack['detail']})" if ack.get("detail") else ""))

    # ------------------------------------------------------------------- loop
    def motion_gate(self, now: float, health: Health, vehicle: VehicleState) -> str | None:
        """Reason manual motion is blocked right now, or None if it is allowed."""
        if not vehicle.link_connected:
            return "LINK LOST"
        if health.feed_lost:
            return "FEED LOST"
        if health.inference_stalled:
            return "INFERENCE STALLED"
        if health.display_age_s is None or health.display_age_s > float(self.cfg["frame_timeout_s"]):
            return "NO FRESH FRAME"
        if self.typing:
            return "TYPING"
        if now - self.input_at > float(self.cfg["input_timeout_s"]):
            return "NO OPERATOR INPUT"
        return None

    def _manual_command(self) -> VelocityCommand:
        f = l = u = y = 0
        for key in self.held:
            df, dl, du, dy = MOTION_KEYS[key]
            f, l, u, y = f + df, l + dl, u + du, y + dy
        clip = lambda v: max(-1, min(1, v))  # noqa: E731
        return VelocityCommand(
            forward=clip(f) * float(self.cfg["planar_speed_m_s"]),
            left=clip(l) * float(self.cfg["planar_speed_m_s"]),
            up=clip(u) * float(self.cfg["vertical_speed_m_s"]),
            yaw_rate=clip(y) * float(self.cfg["yaw_rate_rad_s"]),
        )

    async def tick(self, now: float, health: Health, target) -> None:
        vehicle = self.adapter.vehicle_state()
        command: VelocityCommand | None = None
        if self.mode == "MANUAL":
            self.block_reason = self.motion_gate(now, health, vehicle)
            command = self._manual_command() if self.block_reason is None else ZERO
        else:
            self.block_reason = None
            fresh = not (health.feed_lost or health.inference_stalled) and vehicle.link_connected
            if fresh and target is not None and target.status == "TRACKING":
                command = self.pursuit.compute(target, vehicle)
        self.command = command or ZERO

        # The link accepts velocity input only while this flag is on. Keep it in
        # sync (the link clears it whenever the console reconnects).
        want = command is not None
        if vehicle.link_connected and (want != vehicle.manual_enabled or want != self.velocity_input_on):
            if now - self._flag_sent_at > 0.5:
                await self.adapter.set_manual(want)
                self._flag_sent_at = now
                self.velocity_input_on = want
        if command is not None and vehicle.link_connected:
            await self.adapter.send_velocity(command.forward, command.left, command.up, command.yaw_rate)

    def hud(self) -> dict:
        return {
            "mode": self.mode,
            "command": asdict(self.command),
            "held": sorted(self.held),
            "block_reason": self.block_reason,
            "estop_action": self.cfg.get("estop_action", "kill"),
            "vehicle": self.adapter.vehicle_state().to_dict(),
        }
