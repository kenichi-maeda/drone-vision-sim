"""Start the OVERWATCH console:  cd overwatch && .venv/bin/python -m overwatch [--source sim|live|replay]"""

from __future__ import annotations

import argparse
import logging

import uvicorn

from .config import load_config
from .ui.server import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="OVERWATCH operator console")
    parser.add_argument("--config", help="path to overwatch.yaml (default: config/overwatch.yaml)")
    parser.add_argument("--source", help="override the configured source (sim, live, replay, ...)")
    parser.add_argument("--port", type=int, help="override ui.port")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config, args.source)
    host = cfg["ui"].get("host", "127.0.0.1")
    port = args.port or int(cfg["ui"].get("port", 8700))
    print(f"OVERWATCH ({cfg['source'].upper()}) -> open http://{host}:{port}/", flush=True)
    uvicorn.run(create_app(cfg), host=host, port=port, log_level="warning", ws_max_size=1 << 20)


if __name__ == "__main__":
    main()
