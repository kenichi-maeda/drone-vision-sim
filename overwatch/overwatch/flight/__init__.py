"""Flight adapters. Swapping SIM -> LIVE changes only this and the feed source."""

from __future__ import annotations

from typing import Any

from ..link_client import shared_client
from .base import FlightAdapter, VehicleState


def make_flight(source_cfg: dict[str, Any]) -> FlightAdapter:
    kind = source_cfg["flight"]
    if kind == "link":
        from .link import LinkFlight
        link = source_cfg["link"]
        return LinkFlight(shared_client(link["host"], link["port"]))
    if kind == "none":
        from .null import NullFlight
        return NullFlight()
    if kind == "pidrone":
        from .pidrone import PidroneFlight
        return PidroneFlight(source_cfg["rosbridge_url"])
    raise ValueError(f"Unknown flight adapter '{kind}'")


__all__ = ["FlightAdapter", "VehicleState", "make_flight"]
