#!/usr/bin/env python3
"""Expose the live camera view (the `camera_view` node) to a browser.

The node serves inside the virtual Duckiebot's container network. This forwards
a port on this machine to it and prints the URL to open, for example from the
Mac that runs the Matrix renderer. Stop it with Ctrl-C.

    python3 scripts/view_camera.py            # default robot birdie, port 8090
    python3 scripts/view_camera.py --port 8091
"""

from __future__ import annotations

import argparse
import asyncio
import subprocess
import sys


def robot_ip(robot: str) -> str:
    """IP of the virtual robot's container, as seen from this machine."""
    out = subprocess.run(
        ["docker", "inspect", f"dts-virtual-{robot}", "--format",
         "{{range .NetworkSettings.Networks}}{{.IPAddress}} {{end}}"],
        capture_output=True, text=True,
    )
    ips = out.stdout.split()
    if out.returncode != 0 or not ips:
        sys.exit(f"Virtual robot '{robot}' is not running (container dts-virtual-{robot}).")
    return ips[0]


def host_ip() -> str:
    """First address of this machine, for the URL to open from another computer."""
    out = subprocess.run(["hostname", "-I"], capture_output=True, text=True)
    return out.stdout.split()[0] if out.stdout.split() else "localhost"


async def pump(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        while data := await reader.read(65536):
            writer.write(data)
            await writer.drain()
    except (ConnectionError, asyncio.CancelledError):
        pass
    finally:
        writer.close()


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--robot", default="birdie", help="virtual robot name (default: birdie)")
    parser.add_argument("--port", type=int, default=8090,
                        help="port of the camera_view node (default: 8090)")
    args = parser.parse_args()
    target = robot_ip(args.robot)

    async def handle(client_r: asyncio.StreamReader, client_w: asyncio.StreamWriter) -> None:
        try:
            robot_r, robot_w = await asyncio.open_connection(target, args.port)
        except OSError:
            client_w.close()  # camera_view is not running (CAMERA_VIEWER=false?)
            return
        await asyncio.gather(pump(client_r, robot_w), pump(robot_r, client_w))

    server = await asyncio.start_server(handle, "0.0.0.0", args.port)
    print(f"Forwarding port {args.port} -> {args.robot} ({target}). Open in a browser:", flush=True)
    print(f"  http://{host_ip()}:{args.port}/", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
