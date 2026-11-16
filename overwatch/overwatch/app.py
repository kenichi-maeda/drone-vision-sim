"""The OVERWATCH runtime: wires feed -> perception -> targeting -> UI, and the
control loop -> flight adapter.

Frames are never queued. The feed callback keeps only the newest frame, the
inference worker always takes the newest one, and each UI client gets only the
newest packet. Each packet carries one frame and the detections computed on
that frame, so the operator sees exactly what YOLO saw.
"""

from __future__ import annotations

import asyncio
import json
import logging
import struct
from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from time import monotonic, time
from typing import Any

from .config import active_source, resolve
from .events import EventLog
from .feed import Frame, make_feed
from .flight import make_flight
from .flight.control import ACTIONS, MOTION_KEYS, FlightControl, Health
from .nl import CommandEngine, Vocabulary
from .nl.llm_fallback import make_fallback
from .perception import Perception, PerceptionResult, make_perception
from .recorder import Recorder
from .targeting import TargetManager

log = logging.getLogger("overwatch")


class UIClient:
    """One browser connection. Holds at most one pending frame packet."""

    def __init__(self, send_bytes, send_text) -> None:
        self._send_bytes = send_bytes
        self._send_text = send_text
        self._frame: bytes | None = None
        self._texts: deque[str] = deque(maxlen=200)
        self._wake = asyncio.Event()
        self.closed = False

    def push_frame(self, packet: bytes) -> None:
        self._frame = packet
        self._wake.set()

    def push_json(self, obj: dict) -> None:
        self._texts.append(json.dumps(obj, default=str))
        self._wake.set()

    async def run(self) -> None:
        while not self.closed:
            await self._wake.wait()
            self._wake.clear()
            while self._texts:
                await self._send_text(self._texts.popleft())
            frame, self._frame = self._frame, None
            if frame is not None:
                await self._send_bytes(frame)


class Runtime:
    def __init__(self, cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.source_name = cfg["source"]
        source = active_source(cfg)
        self.feed = make_feed(source)
        self.flight = make_flight(source)
        self.has_vehicle = source["flight"] != "none"
        self.events = EventLog(resolve(cfg["logging"]["events_dir"]))
        self.targets = TargetManager(**cfg["targeting"])
        self.vocab = Vocabulary(cfg["perception"]["classes"])
        self.commands = CommandEngine(self.vocab, cfg["nl"]["clarify_timeout_s"],
                                      make_fallback(cfg["nl"].get("llm_fallback", {})))
        self.control = FlightControl(cfg["flight"], self.flight, self.events, notify=self.notify)
        recording = cfg.get("recording", {})
        self.recorder = Recorder(resolve(recording.get("dir", "recordings")), float(recording.get("interval_s", 0.5)))
        self.perception: Perception | None = None
        self.perception_status = "loading model"
        self.latest: Frame | None = None
        self.first_frame_at: float | None = None
        self.displayed: Frame | None = None
        self.displayed_result: PerceptionResult | None = None
        self.last_inferred_at = 0.0
        self.inference_ms = 0.0
        self.display_times: deque[float] = deque(maxlen=30)
        self.results: OrderedDict[int, PerceptionResult] = OrderedDict()
        self.clients: set[UIClient] = set()
        self.log_lines: deque[dict] = deque(maxlen=60)
        self._new_frame = asyncio.Event()
        self._tasks: list[asyncio.Task] = []
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="perception")

    # ------------------------------------------------------------- lifecycle
    async def start(self) -> None:
        self.flight.start()
        await self.feed.start(self._on_frame)
        self._tasks = [
            asyncio.create_task(self._load_perception(), name="load perception"),
            asyncio.create_task(self._inference_loop(), name="inference"),
            asyncio.create_task(self._control_loop(), name="control"),
            asyncio.create_task(self._status_loop(), name="status"),
        ]
        self.notify("info", f"Source {self.source_name.upper()}: feed {self.feed.describe()}, flight {self.flight.name}")

    async def stop(self) -> None:
        for task in self._tasks:
            task.cancel()
        if self.control.mode == "MANUAL":
            try:
                await self.flight.set_manual(False)
            except Exception:
                pass
        await self.feed.stop()
        if self.perception:
            self.perception.close()
        self._executor.shutdown(wait=False, cancel_futures=True)

    async def _load_perception(self) -> None:
        try:
            self.perception = await asyncio.get_running_loop().run_in_executor(
                self._executor, make_perception, self.cfg["perception"])
            self.perception_status = self.perception.describe()
            self.notify("info", f"Perception ready: {self.perception_status}")
        except Exception as error:
            self.perception_status = f"failed: {error}"
            self.notify("error", f"Perception failed to load: {error}")
            log.exception("perception load failed")

    # ------------------------------------------------------------- pipeline
    def _on_frame(self, frame: Frame) -> None:
        if self.first_frame_at is None:
            self.first_frame_at = frame.received_at
        self.latest = frame
        self._new_frame.set()
        if self.recorder.active:
            self.recorder.offer(frame, self.flight.vehicle_state().to_dict())

    async def _inference_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            await self._new_frame.wait()
            self._new_frame.clear()
            frame, perception = self.latest, self.perception
            if frame is None or perception is None:
                continue
            try:
                result = await loop.run_in_executor(self._executor, perception.infer, frame.jpeg, frame.seq)
            except Exception as error:
                self.notify("error", f"Inference failed: {error}")
                await asyncio.sleep(0.2)
                continue
            now = monotonic()
            self.last_inferred_at = now
            self.inference_ms = result.inference_ms
            self.display_times.append(now)
            self.targets.update(result, now)
            for level, text in self.targets.messages:
                self.notify(level, text)
            self.targets.messages.clear()
            self.displayed, self.displayed_result = frame, result
            self.results[frame.seq] = result
            while len(self.results) > 90:
                self.results.popitem(last=False)
            self._broadcast_frame(frame, result, now)

    def _broadcast_frame(self, frame: Frame, result: PerceptionResult, now: float) -> None:
        header = {
            "type": "frame", "seq": frame.seq, "width": result.width, "height": result.height,
            "inference_ms": round(result.inference_ms, 1),
            "detections": [d.to_dict() for d in result.detections],
            "target": self.targets.get_target_state(now).to_dict(),
            "cursor": self.targets.cursor,
        }
        head = json.dumps(header).encode()
        packet = struct.pack(">I", len(head)) + head + frame.jpeg
        for client in list(self.clients):
            client.push_frame(packet)

    # ----------------------------------------------------------------- health
    def health(self, now: float) -> Health:
        timeout = float(self.cfg["flight"]["frame_timeout_s"])
        feed_lost = self.feed.last_frame_at == 0.0 or now - self.feed.last_frame_at > timeout
        reference = self.last_inferred_at or (self.first_frame_at or now)
        stalled = (not feed_lost) and (
            self.perception is None or now - reference > float(self.cfg["flight"]["inference_timeout_s"]))
        display_age = now - self.displayed.received_at if self.displayed is not None else None
        return Health(feed_lost=feed_lost, inference_stalled=stalled, display_age_s=display_age)

    def banners(self, now: float, health: Health) -> list[dict]:
        banners = []
        vehicle = self.flight.vehicle_state()
        if self.has_vehicle and not vehicle.link_connected:
            banners.append({"level": "critical", "text": "LINK LOST", "detail": "no connection to the drone"})
        elif self.has_vehicle and not vehicle.autopilot_connected:
            banners.append({"level": "critical", "text": "AUTOPILOT OFFLINE", "detail": "link up, PX4 not connected"})
        if health.feed_lost:
            banners.append({"level": "critical", "text": "FEED LOST",
                            "detail": "no camera frames; motion commands held at zero"})
        elif health.inference_stalled:
            text = "MODEL LOADING" if self.perception is None and "failed" not in self.perception_status \
                else "INFERENCE STALLED"
            banners.append({"level": "critical", "text": text,
                            "detail": f"{self.perception_status}; display frozen, motion held at zero"})
        if self.targets.status == "LOST":
            banners.append({"level": "warn", "text": "TARGET LOST",
                            "detail": f"#{self.targets.track_id} {self.targets.describe(self.targets.last)}"})
        return banners

    # ------------------------------------------------------------ control loop
    async def _control_loop(self) -> None:
        period = 1.0 / float(self.cfg["flight"].get("control_rate_hz", 20))
        while True:
            started = monotonic()
            self.targets.check(started)
            for level, text in self.targets.messages:
                self.notify(level, text)
            self.targets.messages.clear()
            try:
                await self.control.tick(started, self.health(started), self.targets.get_target_state(started))
            except Exception as error:
                log.exception("control tick failed")
                self.notify("error", f"Control loop error: {error}")
            await asyncio.sleep(max(0.0, period - (monotonic() - started)))

    def status(self) -> dict:
        now = monotonic()
        health = self.health(now)
        times = self.display_times
        display_fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and now - times[-1] < 1.0 else 0.0
        return {
            "type": "status", "t": time(), "source": self.source_name,
            "feed": {"name": self.feed.describe(), "connected": self.feed.connected(), "fps": round(self.feed.fps(), 1)},
            "display_fps": round(display_fps, 1),
            "inference_ms": round(self.inference_ms, 1),
            "perception": self.perception_status,
            "health": {"feed_lost": health.feed_lost, "inference_stalled": health.inference_stalled,
                       "display_age_s": None if health.display_age_s is None else round(health.display_age_s, 2)},
            "banners": self.banners(now, health),
            "control": self.control.hud(),
            "target": self.targets.get_target_state(now).to_dict(),
            "cursor": self.targets.cursor,
            "recording": {"active": self.recorder.active, "frames": self.recorder.count,
                          "folder": str(self.recorder.folder) if self.recorder.folder else None},
        }

    async def _status_loop(self) -> None:
        while True:
            status = self.status()
            for client in list(self.clients):
                client.push_json(status)
            await asyncio.sleep(0.2)

    # --------------------------------------------------------------- messages
    def notify(self, level: str, text: str) -> None:
        entry = {"type": "log", "level": level, "text": text, "t": time()}
        self.log_lines.append(entry)
        log.log(logging.WARNING if level in ("warn", "error") else logging.INFO, text)
        for client in list(self.clients):
            client.push_json(entry)

    # ------------------------------------------------------- operator inputs
    async def handle_ui(self, message: dict, source: str = "key") -> dict | None:
        kind = message.get("type")
        now = monotonic()
        if kind == "input":
            keys = [k for k in message.get("keys", []) if k in MOTION_KEYS]
            self.control.set_input(keys, bool(message.get("typing")), now)
            return None
        if kind == "action":
            name = message.get("action")
            if name not in ACTIONS:
                return {"type": "reply", "kind": "error", "text": f"Unknown action {name}"}
            target = self.targets.get_target_state(now).to_dict()
            text = await self.control.action(name, source=message.get("source", source), target=target)
            self.notify("warn" if name in ("kill", "land_now") else "info", text)
            return None
        if kind == "click":
            result = self.results.get(int(message.get("seq", -1)))
            text = self.targets.lock_at(result, float(message.get("x", -1)), float(message.get("y", -1)), now)
        elif kind == "cycle":
            text = self.targets.cycle(1 if int(message.get("direction", 1)) >= 0 else -1)
        elif kind == "lock":
            text = self.targets.confirm(now)
        elif kind == "release":
            text = self.targets.release()
        elif kind == "text":
            reply = self.commands.handle(str(message.get("text", ""))[:300], self.targets, now)
            self.notify("info", f"> {message.get('text', '')}")
            self.notify("error" if reply.kind in ("error", "none") else "info", reply.text)
            return {"type": "reply", "kind": reply.kind, "text": reply.text, "options": reply.options,
                    "query": reply.query, "via": reply.via}
        else:
            return {"type": "reply", "kind": "error", "text": f"Unknown message type {kind}"}
        self.notify("info", text)
        return {"type": "reply", "kind": "info", "text": text}
