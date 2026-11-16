"""Flight commands through overwatch_link to PX4/MAVROS (SIM, and LIVE for an Ente/PX4 drone)."""

from __future__ import annotations

from time import monotonic

from ..link_client import LinkClient
from .base import FlightAdapter, VehicleState

LANDED_ON_GROUND = 1


class LinkFlight(FlightAdapter):
    name = "link"

    def __init__(self, client: LinkClient) -> None:
        super().__init__()
        self.client = client
        client.on_message(self._on_message)

    def start(self) -> None:
        self.client.start()

    def _on_message(self, message: dict) -> None:
        if message.get("type") == "ack":
            self._emit_ack(message)

    def vehicle_state(self) -> VehicleState:
        s = self.client.state
        if not self.client.connected or s is None:
            return VehicleState(link_connected=self.client.connected)
        landed_state = s.get("landed_state", 0)
        return VehicleState(
            link_connected=True,
            autopilot_connected=bool(s.get("mavros_connected")),
            armed=bool(s.get("armed")),
            mode=str(s.get("mode", "")),
            phase=str(s.get("phase", "")),
            on_ground=None if landed_state == 0 else landed_state == LANDED_ON_GROUND,
            altitude_m=s.get("altitude_m"),
            manual_enabled=bool(s.get("manual_enabled")),
            state_age_s=round(monotonic() - self.client.state_at, 2),
        )

    async def set_manual(self, enabled: bool) -> int | None:
        return await self.client.command("manual", enabled=enabled)

    async def send_velocity(self, forward: float, left: float, up: float, yaw_rate: float) -> int | None:
        return await self.client.command("velocity", forward=forward, left=left, up=up, yaw_rate=yaw_rate)

    async def arm(self) -> int | None:
        return await self.client.command("arm")

    async def disarm(self) -> int | None:
        return await self.client.command("disarm")

    async def takeoff(self) -> int | None:
        return await self.client.command("takeoff")

    async def land(self, reason: str) -> int | None:
        return await self.client.command("land", reason=reason)

    async def kill(self) -> int | None:
        return await self.client.command("kill")
