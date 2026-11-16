"""STUB: flight adapter for a Daffy (ROS 1 pidrone_pkg) drone.

Not implemented. The physical drone's software stack has not been decided yet.
An Ente/PX4 drone does not need this: it uses LinkFlight with overwatch_link
running on the drone.

Mapping, from the DD24 manual (software-architecture/nodes) and h2r/pidrone_pkg
(web/js/main.js, scripts/flight_controller_node.py). Talk to the drone's
rosbridge (the stock web UI uses it; default port 9090, not verified):

  OVERWATCH call        pidrone topic                      message
  arm()                 /pidrone/desired/mode              pidrone_pkg/Mode {mode: "ARMED"}
  disarm(), kill()      /pidrone/desired/mode              pidrone_pkg/Mode {mode: "DISARMED"}
  takeoff()             /pidrone/desired/mode              pidrone_pkg/Mode {mode: "FLYING"}
  send_velocity(...)    /pidrone/desired/twist             geometry_msgs/Twist (velocity mode;
                                                           publish /pidrone/position_control false first)
  land(reason)          no land mode in pidrone_pkg; the web UI disarms. Decide
                        whether to descend with a negative z twist first.
  safety heartbeat      /pidrone/heartbeat/web_interface   std_msgs/Empty at >= 1 Hz.
                        flight_controller_node disarms if it is missing for 3 s,
                        which gives a dead-man behavior. Stop sending it whenever
                        OVERWATCH must not be in control.
  vehicle_state()       /pidrone/mode (current mode), /pidrone/battery, /pidrone/range (height)

Check every mapping on the real drone before flying.
"""

from __future__ import annotations

from .base import FlightAdapter, VehicleState


class PidroneFlight(FlightAdapter):
    name = "pidrone"

    def __init__(self, rosbridge_url: str) -> None:
        super().__init__()
        self.rosbridge_url = rosbridge_url
        raise NotImplementedError(
            "The Daffy pidrone flight adapter is a stub (overwatch/flight/pidrone.py). For an "
            "Ente/PX4 drone, use the 'live' source with overwatch_link instead.")

    def vehicle_state(self) -> VehicleState:  # pragma: no cover - stub
        return VehicleState()

    async def set_manual(self, enabled: bool) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def send_velocity(self, forward, left, up, yaw_rate) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def arm(self) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def disarm(self) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def takeoff(self) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def land(self, reason: str) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError

    async def kill(self) -> int | None:  # pragma: no cover - stub
        raise NotImplementedError
