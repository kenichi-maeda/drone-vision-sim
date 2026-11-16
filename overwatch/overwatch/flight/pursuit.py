"""Autonomous pursuit: interface only. Nothing here moves the drone yet.

FlightControl calls `compute()` at the control rate, but only when all of these hold:
  * the control mode is AUTONOMOUS,
  * the target status is TRACKING,
  * the camera feed and inference are fresh,
  * the flight adapter is connected.
A returned VelocityCommand is sent through the same flight adapter and the same
dead-man timers as manual input. Returning None sends nothing, so the drone
keeps holding position.

To add pursuit, implement PursuitController and pass it to FlightControl
(see overwatch/app.py). TargetState comes from TargetManager.get_target_state():
it has the target's bounding box, track ID and image-frame position
(`center_norm`, `error_norm` measured from the image center, `area_norm`).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from ..targeting.manager import TargetState
from .base import VehicleState


@dataclass(frozen=True)
class VelocityCommand:
    forward: float = 0.0   # m/s, body frame
    left: float = 0.0      # m/s
    up: float = 0.0        # m/s
    yaw_rate: float = 0.0  # rad/s, positive turns left


class PursuitController(Protocol):
    def compute(self, target: TargetState, vehicle: VehicleState) -> VelocityCommand | None: ...


class NoPursuit:
    """The default: pursuit is not implemented, so AUTONOMOUS only holds position."""

    def compute(self, target: TargetState, vehicle: VehicleState) -> VelocityCommand | None:
        return None
