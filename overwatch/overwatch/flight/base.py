"""Flight adapter interface: the only way OVERWATCH commands a vehicle."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from typing import Callable

AckListener = Callable[[dict], None]


@dataclass
class VehicleState:
    link_connected: bool = False      # OVERWATCH can talk to the drone-side link
    autopilot_connected: bool = False # the link can talk to the flight controller
    armed: bool = False
    mode: str = ""                    # flight controller mode, e.g. OFFBOARD, AUTO.LAND
    phase: str = ""                   # link phase: idle, takeoff, handoff, hold, landing
    on_ground: bool | None = None
    altitude_m: float | None = None
    manual_enabled: bool = False
    state_age_s: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)


class FlightAdapter(ABC):
    """Commands return a sequence number for matching acknowledgements, or None
    if the command could not be sent."""

    name = "flight"

    def __init__(self) -> None:
        self._ack_listeners: list[AckListener] = []

    def on_ack(self, listener: AckListener) -> None:
        self._ack_listeners.append(listener)

    def _emit_ack(self, ack: dict) -> None:
        for listener in self._ack_listeners:
            listener(ack)

    def start(self) -> None:
        """Begin connecting (non-blocking)."""

    @abstractmethod
    def vehicle_state(self) -> VehicleState: ...

    @abstractmethod
    async def set_manual(self, enabled: bool) -> int | None: ...

    @abstractmethod
    async def send_velocity(self, forward: float, left: float, up: float, yaw_rate: float) -> int | None:
        """Body-frame velocity: forward/left/up in m/s, yaw_rate in rad/s (positive = left)."""

    @abstractmethod
    async def arm(self) -> int | None: ...

    @abstractmethod
    async def disarm(self) -> int | None: ...

    @abstractmethod
    async def takeoff(self) -> int | None: ...

    @abstractmethod
    async def land(self, reason: str) -> int | None: ...

    @abstractmethod
    async def kill(self) -> int | None:
        """Stop the motors immediately (force-disarm). The vehicle falls."""
