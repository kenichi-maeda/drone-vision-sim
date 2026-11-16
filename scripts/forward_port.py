#!/usr/bin/env python3
"""Forward a TCP port from the workspace devcontainer to a virtual robot.

Nodes in the workbench listen inside the virtual robot's own network, which
the Mac cannot reach. Run this inside the devcontainer, which uses host
networking, so OrbStack exposes the forwarded port on the Mac as
localhost:<port>. Stop it with Ctrl-C.

    python3 scripts/forward_port.py --robot scooby               # OVERWATCH link, port 8765
    python3 scripts/forward_port.py --robot scooby --port 8090   # camera_view
"""

from __future__ import annotations

import argparse
import asyncio
import subprocess
import sys


def robot_ip(robot: str) -> str:
    out = subprocess.run(
        ["docker", "inspect", f"dts-virtual-{robot}", "--format",
         "{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}"],
        capture_output=True, text=True,
    )
    ips = out.stdout.split()
    if out.returncode != 0 or not ips:
        sys.exit(f"Virtual robot '{robot}' is not running (container dts-virtual-{robot}).")
    return ips[0]


async def pump(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        while data := await reader.read(1 << 16):
            writer.write(data)
            await writer.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        writer.close()


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="scooby", help="virtual robot name (default: scooby)")
    parser.add_argument("--port", type=int, default=8765, help="port on the robot (default: 8765)")
    parser.add_argument("--listen-port", type=int, help="local port (default: same as --port)")
    args = parser.parse_args()
    target = robot_ip(args.robot)
    listen_port = args.listen_port or args.port

    async def handle(client_r: asyncio.StreamReader, client_w: asyncio.StreamWriter) -> None:
        try:
            robot_r, robot_w = await asyncio.open_connection(target, args.port)
        except OSError as error:
            print(f"Cannot reach {target}:{args.port} ({error}); is the workbench running?", flush=True)
            client_w.close()
            return
        await asyncio.gather(pump(client_r, robot_w), pump(robot_r, client_w))

    server = await asyncio.start_server(handle, "0.0.0.0", listen_port)
    print(f"Forwarding :{listen_port} -> {args.robot} ({target}):{args.port}", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
