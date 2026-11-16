"""Optional Claude fallback for natural-language targeting. Off by default.

It runs only when the deterministic parser cannot parse a command. It sees the
operator's text and the list of class names, nothing else: no detections, no
vehicle state. Claude returns JSON constrained to the TargetQuery schema (plus
an "unknown" action for text it cannot map). That JSON then goes through the
same local validation as parsed commands (schema.query_from_dict). The result
is only ever a TargetQuery, which can select or release a target and nothing
more. This module has no path to flight control.

Failure is always graceful. With no credentials, no network, a timeout or a
refusal, it raises FallbackUnavailable, and the console tells the operator to
rephrase.

Enable it in config/overwatch.yaml (nl.llm_fallback.enabled: true). It needs
`pip install -r requirements-llm.txt` and credentials (ANTHROPIC_API_KEY, or an
`ant auth login` profile).
"""

from __future__ import annotations

import json
import logging
from typing import Any

from .schema import ACTIONS, TargetQuery, json_schema, query_from_dict

log = logging.getLogger("overwatch.nl.llm")

SYSTEM_PROMPT = """You convert a drone operator's typed targeting command into a JSON query.
The console shows camera detections; each has a class, a color and an image position.
The query selects one detected object to lock as the target, or releases the lock.

Fields:
- action: "lock" or "release". Use "unknown" if the text is not a request to select or release
  a detected object, or if you cannot tell which object it describes.
- color: one of the allowed colors, only if the operator named or clearly implied one.
- class_name: one of the allowed classes, only if the operator named one (map synonyms,
  e.g. "duckiebot" -> "robot"). If they named an object that is not in the list, use "unknown".
- selector: leftmost, rightmost, topmost, bottommost, closest, farthest, largest, smallest,
  center, or other ("the other one"). "closest" means nearest the drone.
- ordinal: with selector leftmost/rightmost, the count from that side ("second from the right").
- track_id: when the operator gives an object number such as "#7".

Never guess. Leave out any field the operator did not express."""


class FallbackUnavailable(RuntimeError):
    pass


def _llm_schema(class_names: list[str]) -> dict:
    schema = json_schema(class_names)
    schema["properties"]["action"] = {"type": "string", "enum": [*ACTIONS, "unknown"]}
    return schema


class ClaudeFallback:
    def __init__(self, cfg: dict[str, Any]) -> None:
        try:
            import anthropic
        except ImportError as error:
            raise FallbackUnavailable("install requirements-llm.txt to use the LLM fallback") from error
        self._anthropic = anthropic
        self.model = cfg.get("model", "claude-opus-5-5")
        self.timeout_s = float(cfg.get("timeout_s", 4.0))
        # No automatic retries: the operator is waiting, and a slow reply is a failed reply.
        self.client = anthropic.Anthropic(timeout=self.timeout_s, max_retries=0)

    def __call__(self, text: str, class_names: list[str]) -> TargetQuery | None:
        anthropic = self._anthropic
        try:
            response = self.client.beta.messages.create(
                model=self.model,
                max_tokens=4096,
                system=SYSTEM_PROMPT,
                messages=[{"role": "user", "content":
                           f"Allowed classes: {', '.join(class_names)}\nOperator command: {text}"}],
                output_config={
                    "effort": "low",
                    "format": {"type": "json_schema", "schema": _llm_schema(class_names)},
                },
                # Server-side refusal fallback (routes a declined request to another model).
                betas=["server-side-fallback-2026-07-01"],
                fallbacks="default",
            )
        except anthropic.APIConnectionError as error:  # includes timeouts
            raise FallbackUnavailable("no connection to the Claude API") from error
        except anthropic.AuthenticationError as error:
            raise FallbackUnavailable("no valid Claude API credentials") from error
        except anthropic.RateLimitError as error:
            raise FallbackUnavailable("Claude API rate limit") from error
        except anthropic.APIStatusError as error:
            raise FallbackUnavailable(f"Claude API error {error.status_code}") from error
        except TypeError as error:  # raised when no credentials can be resolved at all
            raise FallbackUnavailable("no Claude API credentials configured") from error

        if response.stop_reason == "refusal":
            log.info("LLM fallback refused the command")
            return None
        if response.stop_reason == "max_tokens":
            raise FallbackUnavailable("Claude reply was cut off")
        text_block = next((b.text for b in response.content if b.type == "text"), None)
        if text_block is None:
            return None
        try:
            data = json.loads(text_block)
        except ValueError:
            return None
        if data.get("action") == "unknown":
            return None
        # Same validation as every parsed command; raises QueryError on anything off-schema.
        return query_from_dict(data, class_names)


def make_fallback(cfg: dict[str, Any]):
    """Return the fallback callable, or None if disabled or unavailable."""
    if not cfg.get("enabled", False):
        return None
    try:
        return ClaudeFallback(cfg)
    except FallbackUnavailable as error:
        log.warning("LLM fallback disabled: %s", error)
        return None
