import asyncio
import json

import pytest

from overwatch.events import EventLog
from overwatch.flight.base import FlightAdapter, VehicleState
from overwatch.flight.control import FlightControl, Health
from overwatch.flight.pursuit import VelocityCommand
from overwatch.targeting.manager import TargetState

CFG = {"estop_action": "kill", "planar_speed_m_s": 0.3, "vertical_speed_m_s": 0.2, "yaw_rate_rad_s": 0.6,
       "input_timeout_s": 0.25, "frame_timeout_s": 0.5, "inference_timeout_s": 1.0}
FRESH = Health(feed_lost=False, inference_stalled=False, display_age_s=0.05)


class FakeAdapter(FlightAdapter):
    def __init__(self, connected=True):
        super().__init__()
        self.connected = connected
        self.calls = []
        self.manual = False
        self.seq = 0

    def vehicle_state(self):
        return VehicleState(link_connected=self.connected, autopilot_connected=self.connected,
                            manual_enabled=self.manual, armed=True, mode="OFFBOARD", phase="hold")

    async def _cmd(self, *call):
        if not self.connected:
            return None
        self.calls.append(call)
        self.seq += 1
        return self.seq

    async def set_manual(self, enabled):
        self.manual = enabled
        return await self._cmd("manual", enabled)

    async def send_velocity(self, forward, left, up, yaw_rate):
        return await self._cmd("velocity", forward, left, up, yaw_rate)

    async def arm(self):
        return await self._cmd("arm")

    async def disarm(self):
        return await self._cmd("disarm")

    async def takeoff(self):
        return await self._cmd("takeoff")

    async def land(self, reason):
        return await self._cmd("land", reason)

    async def kill(self):
        return await self._cmd("kill")


def run(coro):
    return asyncio.run(coro)


def make(tmp_path, connected=True, cfg=CFG, pursuit=None):
    adapter = FakeAdapter(connected)
    events = EventLog(tmp_path)
    return FlightControl(cfg, adapter, events, pursuit=pursuit), adapter, events


def velocities(adapter):
    return [c[1:] for c in adapter.calls if c[0] == "velocity"]


def test_autonomous_sends_no_motion(tmp_path):
    control, adapter, _ = make(tmp_path)
    control.set_input(["forward"], False, 10.0)
    run(control.tick(10.0, FRESH, TargetState(status="TRACKING", track_id=1)))
    assert velocities(adapter) == []


def test_manual_keys_map_to_body_velocity(tmp_path):
    control, adapter, _ = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward", "left", "up", "yaw_right"], False, 10.0)
    run(control.tick(10.0, FRESH, None))
    assert ("manual", True) in adapter.calls
    assert velocities(adapter)[-1] == (0.3, 0.3, 0.2, -0.6)
    control.set_input(["forward", "back"], False, 10.1)
    run(control.tick(10.1, FRESH, None))
    assert velocities(adapter)[-1] == (0.0, 0.0, 0.0, 0.0)


@pytest.mark.parametrize("health, reason", [
    (Health(True, False, 0.05), "FEED LOST"),
    (Health(False, True, 0.05), "INFERENCE STALLED"),
    (Health(False, False, 0.9), "NO FRESH FRAME"),
    (Health(False, False, None), "NO FRESH FRAME"),
])
def test_stale_video_holds_zero(tmp_path, health, reason):
    control, adapter, _ = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], False, 10.0)
    run(control.tick(10.0, health, None))
    assert control.block_reason == reason
    assert velocities(adapter)[-1] == (0.0, 0.0, 0.0, 0.0)


def test_stale_operator_input_holds_zero(tmp_path):
    control, adapter, _ = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], False, 10.0)
    run(control.tick(10.3, FRESH, None))  # input is 300 ms old
    assert control.block_reason == "NO OPERATOR INPUT"
    assert velocities(adapter)[-1] == (0.0, 0.0, 0.0, 0.0)


def test_typing_holds_zero(tmp_path):
    control, adapter, _ = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], True, 10.0)
    run(control.tick(10.0, FRESH, None))
    assert control.block_reason == "TYPING" and velocities(adapter)[-1] == (0.0, 0.0, 0.0, 0.0)


def test_link_lost_sends_nothing(tmp_path):
    control, adapter, _ = make(tmp_path, connected=False)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], False, 10.0)
    run(control.tick(10.0, FRESH, None))
    assert control.block_reason == "LINK LOST" and adapter.calls == []


def test_mode_toggle_clears_held_keys(tmp_path):
    control, adapter, _ = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], False, 10.0)
    run(control.action("toggle_mode"))
    assert control.mode == "AUTONOMOUS" and control.held == frozenset()


def read_log(events):
    return [json.loads(line) for line in events.path.read_text().splitlines()]


def test_kill_is_logged_with_context(tmp_path):
    control, adapter, events = make(tmp_path)
    run(control.set_mode("MANUAL"))
    control.set_input(["forward"], False, 10.0)
    text = run(control.action("kill", source="key", target={"status": "TRACKING", "track_id": 4}))
    assert "KILL" in text and ("kill",) in adapter.calls
    assert control.held == frozenset()
    assert control.mode == "AUTONOMOUS"  # manual control must be re-taken on purpose
    entry = [e for e in read_log(events) if e["event"] == "KILL"][0]
    assert entry["delivered"] and entry["action"] == "kill" and entry["control_mode"] == "MANUAL"
    assert entry["target"]["track_id"] == 4 and entry["vehicle"]["armed"] is True
    adapter._emit_ack({"type": "ack", "seq": entry["seq"], "cmd": "kill", "ok": True, "detail": "force-disarm sent"})
    ack = [e for e in read_log(events) if e["event"] == "ACK"][0]
    assert ack["for_event"] == "KILL" and ack["ok"] is True


def test_kill_configured_as_land(tmp_path):
    control, adapter, events = make(tmp_path, cfg={**CFG, "estop_action": "land"})
    run(control.action("kill"))
    assert ("land", "estop") in adapter.calls and ("kill",) not in adapter.calls
    assert read_log(events)[-1]["action"] == "land"


def test_land_now_logged_and_kill_not_delivered_when_link_down(tmp_path):
    control, adapter, events = make(tmp_path)
    run(control.action("land_now", source="button"))
    assert ("land", "land_now") in adapter.calls
    assert read_log(events)[-1]["event"] == "LAND_NOW" and read_log(events)[-1]["source"] == "button"
    control2, _, events2 = make(tmp_path / "b", connected=False)
    text = run(control2.action("kill"))
    assert "NOT DELIVERED" in text
    entry = read_log(events2)[-1]
    assert entry["event"] == "KILL" and entry["delivered"] is False


def test_pursuit_hook_runs_only_when_tracking_and_fresh(tmp_path):
    class Pursuit:
        def compute(self, target, vehicle):
            return VelocityCommand(forward=0.1)

    control, adapter, _ = make(tmp_path, pursuit=Pursuit())
    run(control.tick(10.0, FRESH, TargetState(status="LOST", track_id=1)))
    assert velocities(adapter) == []
    run(control.tick(10.0, Health(True, False, None), TargetState(status="TRACKING", track_id=1)))
    assert velocities(adapter) == []
    run(control.tick(10.0, FRESH, TargetState(status="TRACKING", track_id=1)))
    assert velocities(adapter)[-1] == (0.1, 0.0, 0.0, 0.0)
