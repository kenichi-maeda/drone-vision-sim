"""No vehicle (replay mode). Every command is refused."""

from __future__ import annotations

from .base import FlightAdapter, VehicleState


class NullFlight(FlightAdapter):
    name = "none"

    def vehicle_state(self) -> VehicleState:
        return VehicleState()

    async def set_manual(self, enabled: bool) -> int | None:
        return None

    async def send_velocity(self, forward: float, left: float, up: float, yaw_rate: float) -> int | None:
        return None

    async def arm(self) -> int | None:
        return None

    async def disarm(self) -> int | None:
        return None

    async def takeoff(self) -> int | None:
        return None

    async def land(self, reason: str) -> int | None:
        return None

    async def kill(self) -> int | None:
        return None
