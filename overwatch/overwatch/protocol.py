"""Wire format of the overwatch_link TCP connection.

Mirrors packages/drone_vision/drone_vision/overwatch_link.py. Every message:

    uint32 big-endian length of everything after it | uint8 kind | payload

    kind 1: UTF-8 JSON object
    kind 2: image: uint32 big-endian header length | JSON header | JPEG bytes
"""

from __future__ import annotations

import asyncio
import json
import struct

PROTOCOL_VERSION = 1
KIND_JSON = 1
KIND_IMAGE = 2
MAX_MESSAGE_BYTES = 8 * 1024 * 1024


def encode_json(obj: dict) -> bytes:
    body = json.dumps(obj, separators=(",", ":")).encode()
    return struct.pack(">IB", len(body) + 1, KIND_JSON) + body


def encode_image(header: dict, jpeg: bytes) -> bytes:
    head = json.dumps(header, separators=(",", ":")).encode()
    payload = struct.pack(">I", len(head)) + head + jpeg
    return struct.pack(">IB", len(payload) + 1, KIND_IMAGE) + payload


def decode_body(kind: int, body: bytes) -> tuple[str, dict, bytes | None]:
    """Return ("json", obj, None) or ("image", header, jpeg)."""
    if kind == KIND_JSON:
        return "json", json.loads(body), None
    if kind == KIND_IMAGE:
        (head_len,) = struct.unpack(">I", body[:4])
        header = json.loads(body[4:4 + head_len])
        return "image", header, body[4 + head_len:]
    raise ValueError(f"unknown message kind {kind}")


async def read_message(reader: asyncio.StreamReader) -> tuple[str, dict, bytes | None]:
    head = await reader.readexactly(5)
    length, kind = struct.unpack(">IB", head)
    if length < 1 or length > MAX_MESSAGE_BYTES:
        raise ValueError(f"bad message length {length}")
    body = await reader.readexactly(length - 1)
    return decode_body(kind, body)
