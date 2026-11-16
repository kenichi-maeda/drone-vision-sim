"""Standalone perception server, for running YOLO on another machine.

    cd overwatch && .venv/bin/python -m overwatch.perception.server --port 8710

Then set `perception.host: ws://<this machine>:8710` in the console's config.
It uses the perception section of the same config file (model, prompts, color).
"""

from __future__ import annotations

import argparse
import json
import logging
import struct

from websockets.sync.server import serve

from ..config import load_config
from .yolo_world import YoloWorldPerception


def main() -> None:
    parser = argparse.ArgumentParser(description="OVERWATCH perception server")
    parser.add_argument("--config", help="path to overwatch.yaml")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8710)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    perception = YoloWorldPerception(load_config(args.config)["perception"])

    def handle(ws) -> None:
        for message in ws:
            try:
                (seq,) = struct.unpack(">I", message[:4])
                ws.send(json.dumps(perception.infer(bytes(message[4:]), seq).to_dict()))
            except Exception as error:  # report, keep serving
                ws.send(json.dumps({"error": f"{type(error).__name__}: {error}"}))

    with serve(handle, args.host, args.port, max_size=None) as server:
        logging.info("Perception server on ws://%s:%d (%s)", args.host, args.port, perception.describe())
        server.serve_forever()


if __name__ == "__main__":
    main()
