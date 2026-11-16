"""The Claude fallback without network: a stub client stands in for the SDK."""

import json
from types import SimpleNamespace

import anthropic
import httpx2 as httpx
import pytest

from overwatch.nl.llm_fallback import ClaudeFallback, FallbackUnavailable, _llm_schema, make_fallback
from overwatch.nl.schema import QueryError, TargetQuery


class StubMessages:
    def __init__(self, reply=None, error=None):
        self.reply, self.error, self.kwargs = reply, error, None

    def create(self, **kwargs):
        self.kwargs = kwargs
        if self.error:
            raise self.error
        return self.reply


def fallback_with(reply=None, error=None):
    fb = ClaudeFallback({"model": "claude-opus-5-5", "timeout_s": 1})
    stub = StubMessages(reply, error)
    fb.client = SimpleNamespace(beta=SimpleNamespace(messages=stub))
    return fb, stub


def text_reply(obj, stop_reason="end_turn"):
    return SimpleNamespace(stop_reason=stop_reason, content=[SimpleNamespace(type="text", text=json.dumps(obj))])


def test_disabled_by_default():
    assert make_fallback({}) is None
    assert make_fallback({"enabled": False}) is None


def test_maps_text_to_validated_query():
    fb, stub = fallback_with(text_reply({"action": "lock", "color": "blue", "class_name": "robot"}))
    assert fb("the azure fellow", ["robot", "duckie"]) == TargetQuery(color="blue", class_name="robot")
    schema = stub.kwargs["output_config"]["format"]["schema"]
    assert schema["properties"]["class_name"]["enum"] == ["robot", "duckie"]
    assert stub.kwargs["model"] == "claude-opus-5-5"


def test_unknown_refusal_and_bad_json_return_none():
    assert fallback_with(text_reply({"action": "unknown"}))[0]("hmm", ["robot"]) is None
    assert fallback_with(text_reply({}, stop_reason="refusal"))[0]("hmm", ["robot"]) is None
    bad = SimpleNamespace(stop_reason="end_turn", content=[SimpleNamespace(type="text", text="not json")])
    assert fallback_with(bad)[0]("hmm", ["robot"]) is None


def test_off_schema_output_is_rejected():
    fb, _ = fallback_with(text_reply({"action": "lock", "class_name": "cube"}))
    with pytest.raises(QueryError):
        fb("the cube", ["robot"])
    fb, _ = fallback_with(text_reply({"action": "lock", "color": "red", "fly": True}))
    with pytest.raises(QueryError):
        fb("x", ["robot"])


def test_network_and_auth_failures_are_graceful():
    request = httpx.Request("POST", "https://api.anthropic.com/v1/messages")
    for error in (anthropic.APIConnectionError(request=request),
                  anthropic.APITimeoutError(request=request),
                  TypeError("Could not resolve authentication method")):
        fb, _ = fallback_with(error=error)
        with pytest.raises(FallbackUnavailable):
            fb("x", ["robot"])


def test_schema_has_an_escape_hatch():
    assert "unknown" in _llm_schema(["robot"])["properties"]["action"]["enum"]
