"""Typed command -> parse -> (optional LLM fallback) -> validate -> resolve -> lock.

Holds the conversation state that resolution needs:
  * a pending clarifying question (its candidates, oldest first expires after
    `clarify_timeout_s`), so a follow-up like "the left one", "2" or "#8"
    resolves against the options just offered;
  * the track IDs matched by the previous command, for "the other one".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from ..targeting.manager import TargetManager
from .parser import ParseResult, Vocabulary, parse
from .resolver import Outcome, describe, resolve
from .schema import QueryError, TargetQuery, validate

EXAMPLES = "Try: 'lock the red robot', 'leftmost duckie', 'closest blue one', 'the other one', '#7', 'release'."

# fallback(text, class_names) -> dict for query_from_dict, or None. It never sees detections.
Fallback = Callable[[str, list[str]], TargetQuery | None]


@dataclass
class Reply:
    text: str
    kind: str                       # locked | released | clarify | none | error
    options: list[dict] = field(default_factory=list)  # for clarify: [{n, track_id, label}]
    query: dict | None = None
    via: str = "parser"             # parser | llm


class CommandEngine:
    def __init__(self, vocab: Vocabulary, clarify_timeout_s: float = 15.0, fallback: Fallback | None = None) -> None:
        self.vocab = vocab
        self.clarify_timeout_s = clarify_timeout_s
        self.fallback = fallback
        self.pending: list[int] | None = None
        self.pending_at = 0.0
        self.previous: list[int] = []

    def _pending_active(self, now: float) -> bool:
        if self.pending is not None and now - self.pending_at > self.clarify_timeout_s:
            self.pending = None
        return self.pending is not None

    def handle(self, text: str, targets: TargetManager, now: float) -> Reply:
        parsed: ParseResult = parse(text, self.vocab)
        query, via = parsed.query, "parser"
        if query is None and self.fallback is not None:
            try:
                query = self.fallback(text, self.vocab.class_names)
                via = "llm"
            except Exception as error:  # the fallback must never break the console
                query = None
                fallback_error = f" (LLM fallback unavailable: {type(error).__name__})"
            else:
                fallback_error = "" if query is not None else " (LLM fallback could not interpret it either)"
        else:
            fallback_error = ""
        if query is None:
            if parsed.unknown:
                words = ", ".join(f"'{w}'" for w in parsed.unknown)
                known = ", ".join(self.vocab.class_names)
                return Reply(f"I don't know {words}. Objects I can detect: {known}.{fallback_error} {EXAMPLES}", "error")
            return Reply(f"I didn't understand that.{fallback_error} {EXAMPLES}", "error")
        try:
            query = validate(query, self.vocab.class_names)
        except QueryError as error:
            return Reply(f"Invalid command: {error}. {EXAMPLES}", "error", via=via)

        result = targets.result
        width, height = (result.width, result.height) if result else (1, 1)
        visible = targets.lockable()

        if query.action == "release":
            self.pending = None
            return Reply(targets.release(), "released", query=query.to_dict(), via=via)

        outcome: Outcome | None = None
        if self._pending_active(now):
            options = [d for d in visible if d.track_id in self.pending]
            options.sort(key=lambda d: d.center[0])
            if query.index is not None and query.track_id is None:
                if 1 <= query.index <= len(options):
                    outcome = Outcome("lock", track_id=options[query.index - 1].track_id, matched=self.pending)
                else:
                    outcome = Outcome("none", f"Choose 1 to {len(options)}.")
            elif query.ordinal is not None and query.selector is None:
                if query.ordinal <= len(options):
                    outcome = Outcome("lock", track_id=options[query.ordinal - 1].track_id, matched=self.pending)
            elif query.track_id is None:
                attempt = resolve(query, options, width, height, targets.track_id, self.pending)
                if attempt.kind != "none":
                    outcome = attempt
        if outcome is None:
            if query.index is not None and query.track_id is None:
                query = TargetQuery(track_id=query.index)  # outside a question, a bare number is a track ID
            outcome = resolve(query, visible, width, height, targets.track_id, self.previous)

        if outcome.matched:
            self.previous = list(outcome.matched)
        if outcome.kind == "lock":
            self.pending = None
            return Reply(targets.lock(outcome.track_id, now), "locked", query=query.to_dict(), via=via)
        if outcome.kind == "clarify":
            self.pending = [d.track_id for d in outcome.candidates]
            self.pending_at = now
            options = [{"n": i, "track_id": d.track_id, "label": describe(d)}
                       for i, d in enumerate(outcome.candidates, 1)]
            return Reply(outcome.message, "clarify", options=options, query=query.to_dict(), via=via)
        return Reply(outcome.message, "none", query=query.to_dict(), via=via)
