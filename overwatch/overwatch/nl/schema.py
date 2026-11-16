"""The structured target query. Every natural-language path produces this, and
every query goes through validate() before it can select anything.

It only describes which detected object to lock or release. Nothing in nl/
can command flight (tests/test_import_boundary.py enforces that flight/ never
imports nl/).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields

from ..perception.color import COLORS

ACTIONS = ("lock", "release")
SELECTORS = ("leftmost", "rightmost", "topmost", "bottommost", "closest", "farthest",
             "largest", "smallest", "center", "other")
MAX_ORDINAL = 20
MAX_TRACK_ID = 1_000_000


class QueryError(ValueError):
    pass


@dataclass(frozen=True)
class TargetQuery:
    action: str = "lock"
    color: str | None = None
    class_name: str | None = None
    selector: str | None = None
    ordinal: int | None = None    # "second from the left": ordinal=2, selector=leftmost
    track_id: int | None = None   # "#7"
    index: int | None = None      # a bare number: an option of a clarifying question, else a track ID

    def to_dict(self) -> dict:
        return {k: v for k, v in asdict(self).items() if v is not None}

    def describe(self) -> str:
        parts = [self.color or "", self.class_name or "object"]
        return " ".join(p for p in parts if p)


def validate(query: TargetQuery, class_names: list[str] | tuple[str, ...]) -> TargetQuery:
    if query.action not in ACTIONS:
        raise QueryError(f"action must be one of {ACTIONS}")
    if query.color is not None and query.color not in COLORS:
        raise QueryError(f"unknown color '{query.color}'")
    if query.class_name is not None and query.class_name not in class_names:
        raise QueryError(f"unknown class '{query.class_name}'")
    if query.selector is not None and query.selector not in SELECTORS:
        raise QueryError(f"unknown selector '{query.selector}'")
    for name, limit in (("ordinal", MAX_ORDINAL), ("track_id", MAX_TRACK_ID), ("index", MAX_ORDINAL)):
        value = getattr(query, name)
        if value is not None and (not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= limit):
            raise QueryError(f"{name} must be an integer from 1 to {limit}")
    if query.ordinal is not None and query.selector not in (None, "leftmost", "rightmost"):
        raise QueryError("an ordinal can only count from the left or the right")
    if query.action == "release" and any(
            getattr(query, f.name) is not None for f in fields(query) if f.name != "action"):
        raise QueryError("release takes no other fields")
    if query.action == "lock" and all(
            getattr(query, f.name) is None for f in fields(query) if f.name != "action"):
        raise QueryError("lock needs at least one of color, class, selector, ordinal, track_id or index")
    return query


def query_from_dict(data: dict, class_names: list[str] | tuple[str, ...]) -> TargetQuery:
    """Build and validate a query from untrusted JSON (the optional LLM fallback)."""
    if not isinstance(data, dict):
        raise QueryError("query must be a JSON object")
    allowed = {f.name for f in fields(TargetQuery)}
    unknown = set(data) - allowed
    if unknown:
        raise QueryError(f"unknown fields: {sorted(unknown)}")
    cleaned = {k: v for k, v in data.items() if v is not None}
    return validate(TargetQuery(**cleaned), class_names)


def json_schema(class_names: list[str] | tuple[str, ...]) -> dict:
    """JSON Schema of TargetQuery, for structured output from the LLM fallback."""
    return {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "action": {"type": "string", "enum": list(ACTIONS)},
            "color": {"type": "string", "enum": list(COLORS)},
            "class_name": {"type": "string", "enum": list(class_names)},
            "selector": {"type": "string", "enum": list(SELECTORS)},
            "ordinal": {"type": "integer", "minimum": 1, "maximum": MAX_ORDINAL},
            "track_id": {"type": "integer", "minimum": 1, "maximum": MAX_TRACK_ID},
            "index": {"type": "integer", "minimum": 1, "maximum": MAX_ORDINAL},
        },
        "required": ["action"],
    }
