#!/usr/bin/env python3
"""A stand-in for overwatch_link, for testing the console without the simulator.

Speaks the real wire protocol, serves JPEG frames from a folder in a loop, and
simulates the link's flight phases (takeoff -> hold -> landing). It does not fly
anything. It records every command it receives to a JSONL file so that tests
can check what the console sent.

    .venv/bin/python tools/fake_link.py --frames recordings/latest --port 8765
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path
from time import monotonic, time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from overwatch.protocol import encode_image, encode_json, read_message  # noqa: E402


class FakeDrone:
    def __init__(self, log_path: Path | None) -> None:
        self.phase, self.armed, self.mode, self.landed = "idle", False, "AUTO.LOITER", 1
        self.altitude, self.manual, self.phase_at = 0.0, False, monotonic()
        self.velocity, self.velocity_at, self.last_seq = (0.0, 0.0, 0.0, 0.0), 0.0, -1
        self.log_path = log_path

    def log(self, message: dict) -> None:
        if self.log_path:
            with open(self.log_path, "a") as handle:
                handle.write(json.dumps({"t": time(), **message}) + "\n")

    def handle(self, message: dict) -> dict | None:
        if message.get("type") != "cmd" or message.get("seq", -1) <= self.last_seq:
            return None
        self.last_seq = message["seq"]
        self.log(message)
        cmd = message["cmd"]
        if cmd == "velocity":
            if self.manual:
                self.velocity = tuple(float(message[k]) for k in ("forward", "left", "up", "yaw_rate"))
                self.velocity_at = monotonic()
            return None
        ok, detail = True, ""
        if cmd == "manual":
            self.manual = bool(message.get("enabled"))
        elif cmd == "arm":
            self.armed = True
        elif cmd == "disarm":
            ok = self.landed == 1
            self.armed = self.armed and not ok
            detail = "" if ok else "PX4 refused"
        elif cmd == "takeoff":
            ok = self.phase == "idle" and self.landed == 1
            if ok:
                self.phase, self.armed, self.mode, self.phase_at = "takeoff", True, "AUTO.TAKEOFF", monotonic()
            detail = "AUTO.TAKEOFF requested" if ok else f"busy ({self.phase})"
        elif cmd == "land":
            self.phase, self.mode, self.phase_at, self.manual = "landing", "AUTO.LAND", monotonic(), False
            detail = "AUTO.LAND requested"
        elif cmd == "kill":
            self.phase, self.armed, self.manual, self.mode = "idle", False, False, "AUTO.LOITER"
            self.altitude, self.landed = 0.0, 1
            detail = "force-disarm sent"
        return {"type": "ack", "seq": message["seq"], "cmd": cmd, "ok": ok, "detail": detail}

    def step(self, dt: float) -> None:
        now = monotonic()
        if self.phase == "takeoff":
            self.altitude += 0.4 * dt
            self.landed = 3
            if self.altitude >= 0.8:
                self.phase, self.mode, self.landed = "hold", "OFFBOARD", 2
        elif self.phase == "hold" and self.manual and now - self.velocity_at < 0.3:
            self.altitude = min(1.6, max(0.3, self.altitude + self.velocity[2] * dt))
        elif self.phase == "landing":
            self.altitude = max(0.0, self.altitude - 0.3 * dt)
            if self.altitude == 0.0:
                self.phase, self.armed, self.landed, self.mode = "idle", False, 1, "AUTO.LOITER"

    def state(self) -> dict:
        return {"type": "state", "t": time(), "mavros_connected": True, "armed": self.armed, "mode": self.mode,
                "landed_state": self.landed, "phase": self.phase, "manual_enabled": self.manual,
                "altitude_m": round(self.altitude, 3), "pose": None, "camera_hz": 15.0,
                "command_age_s": round(monotonic() - self.velocity_at, 3) if self.velocity_at else None}


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--frames", required=True, help="folder of .jpg frames (or a recording folder)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--log", help="write received commands to this JSONL file")
    args = parser.parse_args()
    folder = Path(args.frames)
    folder = folder / "frames" if (folder / "frames").is_dir() else folder
    files = sorted(folder.glob("*.jpg"))
    if not files:
        sys.exit(f"No .jpg frames in {folder}")
    frames = [f.read_bytes() for f in files]
    drone = FakeDrone(Path(args.log) if args.log else None)
    paused = Path(args.log).with_suffix(".pause") if args.log else None

    async def serve(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        writer.write(encode_json({"type": "hello", "proto": 1, "robot": "fake", "camera_topic": "/fake"}))
        drone.last_seq = -1
        drone.manual = False

        async def receive() -> None:
            while True:
                _, message, _ = await read_message(reader)
                ack = drone.handle(message)
                if ack:
                    writer.write(encode_json(ack))

        async def transmit() -> None:
            seq, last = 0, monotonic()
            while True:
                now = monotonic()
                drone.step(now - last)
                last = now
                if not (paused and paused.exists()):  # touch <log>.pause to simulate a dropped camera
                    seq += 1
                    writer.write(encode_image({"seq": seq, "stamp": time(), "format": "jpeg"},
                                              frames[seq % len(frames)]))
                writer.write(encode_json(drone.state()))
                await writer.drain()
                await asyncio.sleep(1.0 / args.fps)

        tasks = [asyncio.create_task(receive()), asyncio.create_task(transmit())]
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            writer.close()

    server = await asyncio.start_server(serve, "127.0.0.1", args.port)
    print(f"Fake link on 127.0.0.1:{args.port} serving {len(frames)} frames from {folder}", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
