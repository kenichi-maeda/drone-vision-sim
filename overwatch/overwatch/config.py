"""Load overwatch.yaml and resolve the active source."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from . import ROOT

DEFAULT_CONFIG = ROOT / "config" / "overwatch.yaml"
REQUIRED = ("source", "sources", "perception", "targeting", "nl", "flight", "keys", "ui", "logging")


def load_config(path: str | Path | None = None, source: str | None = None) -> dict[str, Any]:
    path = Path(path) if path else DEFAULT_CONFIG
    with open(path) as handle:
        cfg = yaml.safe_load(handle)
    missing = [key for key in REQUIRED if key not in cfg]
    if missing:
        raise ValueError(f"{path}: missing sections: {', '.join(missing)}")
    if source:
        cfg["source"] = source
    if cfg["source"] not in cfg["sources"]:
        raise ValueError(f"{path}: source '{cfg['source']}' is not one of {sorted(cfg['sources'])}")
    cfg["_path"] = str(path)
    return cfg


def active_source(cfg: dict[str, Any]) -> dict[str, Any]:
    return cfg["sources"][cfg["source"]]


def resolve(path: str) -> Path:
    """Resolve a config path relative to the overwatch/ directory."""
    p = Path(path)
    return p if p.is_absolute() else ROOT / p
