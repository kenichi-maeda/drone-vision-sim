"""Architecture rules that must hold no matter how the code evolves."""

import ast
import asyncio
from pathlib import Path

from overwatch.protocol import encode_image, encode_json, read_message

PKG = Path(__file__).resolve().parent.parent / "overwatch"


def imports(folder: Path) -> set[str]:
    names = set()
    for file in folder.rglob("*.py"):
        for node in ast.walk(ast.parse(file.read_text())):
            if isinstance(node, ast.ImportFrom):
                names.add(("." * node.level) + (node.module or ""))
            elif isinstance(node, ast.Import):
                names.update(alias.name for alias in node.names)
    return names


def test_flight_never_imports_natural_language():
    assert not any("nl" in name.split(".") for name in imports(PKG / "flight"))


def test_natural_language_never_imports_flight():
    assert not any("flight" in name.split(".") for name in imports(PKG / "nl"))


def test_adapters_are_the_only_sim_live_seam():
    # Only feed/ and flight/ may know how a vehicle is reached.
    for folder in ("perception", "targeting", "nl"):
        names = imports(PKG / folder)
        assert not any("link_client" in n or "protocol" in n for n in names), folder


def test_wire_protocol_round_trip():
    async def go():
        reader = asyncio.StreamReader()
        reader.feed_data(encode_json({"type": "state", "armed": True}))
        reader.feed_data(encode_image({"seq": 7}, b"\xff\xd8jpeg"))
        reader.feed_eof()
        return await read_message(reader), await read_message(reader)

    first, second = asyncio.run(go())
    assert first == ("json", {"type": "state", "armed": True}, None)
    assert second == ("image", {"seq": 7}, b"\xff\xd8jpeg")


def test_drone_side_uses_the_same_wire_format():
    link = (PKG.parent.parent / "packages/drone_vision/drone_vision/overwatch_link.py").read_text()
    for snippet in ('struct.pack(">IB", len(body) + 1, KIND_JSON)', 'struct.pack(">I", len(head)) + head + jpeg',
                    "KIND_JSON = 1", "KIND_IMAGE = 2"):
        assert snippet in link
