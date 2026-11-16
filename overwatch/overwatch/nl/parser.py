"""Deterministic parser: typed command -> TargetQuery.

Understands colors, classes (with synonyms from the config), positions
("leftmost", "closest", "in the middle"), ordinals ("second from the right"),
track IDs ("#7"), "the other one", and release commands. Words it does not
know are reported, not ignored, so "the red toaster" fails instead of locking
some red object.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .schema import TargetQuery

COLOR_WORDS = {
    "red": "red", "crimson": "red", "scarlet": "red",
    "orange": "orange",
    "yellow": "yellow", "gold": "yellow", "golden": "yellow",
    "green": "green", "lime": "green",
    "blue": "blue", "navy": "blue", "azure": "blue",
    "purple": "purple", "magenta": "purple", "violet": "purple", "pink": "purple", "lilac": "purple",
}

# Multi-word phrases are matched before single words; order matters.
SELECTOR_PHRASES = [
    ("far left", "leftmost"), ("far right", "rightmost"),
    ("left most", "leftmost"), ("right most", "rightmost"),
    ("on the left", "leftmost"), ("on the right", "rightmost"),
    ("in the middle", "center"), ("in the center", "center"), ("in the centre", "center"),
    ("top most", "topmost"), ("bottom most", "bottommost"),
]
SELECTOR_WORDS = {
    "leftmost": "leftmost", "left": "leftmost",
    "rightmost": "rightmost", "right": "rightmost",
    "topmost": "topmost", "top": "topmost", "upper": "topmost", "highest": "topmost",
    "bottommost": "bottommost", "bottom": "bottommost", "lowest": "bottommost",
    "closest": "closest", "nearest": "closest", "near": "closest", "close": "closest",
    "closer": "closest", "nearer": "closest",
    "farthest": "farthest", "furthest": "farthest", "far": "farthest", "farther": "farthest",
    "further": "farthest", "distant": "farthest",
    "largest": "largest", "biggest": "largest", "big": "largest", "large": "largest",
    "bigger": "largest", "larger": "largest",
    "smallest": "smallest", "tiniest": "smallest", "small": "smallest", "tiny": "smallest",
    "smaller": "smallest", "little": "smallest",
    "center": "center", "centre": "center", "middle": "center", "central": "center",
    "other": "other", "another": "other", "else": "other",
}
ORDINAL_WORDS = {
    "first": 1, "1st": 1, "second": 2, "2nd": 2, "third": 3, "3rd": 3, "fourth": 4, "4th": 4,
    "fifth": 5, "5th": 5, "sixth": 6, "6th": 6,
}
RELEASE_PATTERN = re.compile(
    r"\b(release|unlock|clear|cancel|drop|forget|disengage|let go|stop (tracking|targeting)|"
    r"un ?lock|no target)\b")
STOPWORDS = {
    "lock", "locks", "onto", "on", "to", "target", "targets", "track", "tracking", "select",
    "designate", "follow", "focus", "pick", "choose", "get", "grab", "aim", "at", "the", "a", "an",
    "that", "this", "please", "one", "ones", "object", "objects", "thing", "item", "it", "of", "from",
    "side", "most", "in", "is", "now", "go", "for", "with", "and", "then", "me", "i", "want", "id",
    "number", "no", "option", "instead", "now", "position", "located", "there", "toward", "towards",
}
GENERIC_NOUNS = {"object", "objects", "thing", "things", "one", "ones", "item", "items", "target"}


@dataclass
class ParseResult:
    query: TargetQuery | None
    unknown: list[str] = field(default_factory=list)


class Vocabulary:
    def __init__(self, classes: list[dict]) -> None:
        self.class_names = [entry["name"] for entry in classes]
        self.phrases: list[tuple[str, str]] = []
        self.words: dict[str, str] = {}
        for entry in classes:
            for synonym in [entry["name"], *entry.get("synonyms", [])]:
                synonym = synonym.lower().strip()
                if " " in synonym:
                    self.phrases.append((synonym, entry["name"]))
                else:
                    self.words[synonym] = entry["name"]
        self.phrases.sort(key=lambda item: -len(item[0]))

    def class_of(self, word: str) -> str | None:
        if word in self.words:
            return self.words[word]
        for stem in (word[:-1] if word.endswith("s") else None, word[:-2] if word.endswith("es") else None):
            if stem and stem in self.words:
                return self.words[stem]
        return None


def normalize(text: str) -> str:
    text = text.lower().replace("-", " ").replace("’", "'")
    text = re.sub(r"[^a-z0-9# ]+", " ", text)
    return re.sub(r"\s+", " ", text).strip()


def parse(text: str, vocab: Vocabulary) -> ParseResult:
    s = f" {normalize(text)} "
    if not s.strip():
        return ParseResult(None)
    if RELEASE_PATTERN.search(s):
        return ParseResult(TargetQuery(action="release"))

    fields: dict = {}
    # Track IDs: "#7", "id 7", "track 7", "number 7".
    match = re.search(r"(?:#\s*|\b(?:id|track|number|no)\s+)(\d+)\b", s)
    if match:
        fields["track_id"] = int(match.group(1))
        s = s[:match.start()] + " " + s[match.end():]
    # Ordinal direction: "second from the left/right".
    match = re.search(r"\b(\w+) from the (left|right)\b", s)
    if match and match.group(1) in ORDINAL_WORDS:
        fields["ordinal"] = ORDINAL_WORDS[match.group(1)]
        fields["selector"] = "leftmost" if match.group(2) == "left" else "rightmost"
        s = s[:match.start()] + " " + s[match.end():]
    for phrase, selector in SELECTOR_PHRASES:
        if f" {phrase} " in s:
            fields.setdefault("selector", selector)
            s = s.replace(f" {phrase} ", " ")
    for phrase, class_name in vocab.phrases:
        if f" {phrase} " in s:
            fields.setdefault("class_name", class_name)
            s = s.replace(f" {phrase} ", " ")

    unknown: list[str] = []
    for word in s.split():
        if word.isdigit():
            fields.setdefault("index", int(word))
        elif word in COLOR_WORDS:
            fields.setdefault("color", COLOR_WORDS[word])
        elif word in ORDINAL_WORDS:
            fields.setdefault("ordinal", ORDINAL_WORDS[word])
        elif word in SELECTOR_WORDS:
            fields.setdefault("selector", SELECTOR_WORDS[word])
        elif vocab.class_of(word):
            fields.setdefault("class_name", vocab.class_of(word))
        elif word in STOPWORDS or word in GENERIC_NOUNS:
            continue
        else:
            unknown.append(word)
    # A bare number next to a track word was already taken; a lone number is an index.
    if "track_id" in fields and "index" in fields:
        fields.pop("index")
    if unknown or not fields:
        return ParseResult(None, unknown)
    if fields.get("ordinal") is not None and fields.get("selector") not in (None, "leftmost", "rightmost"):
        return ParseResult(None, [])
    return ParseResult(TargetQuery(action="lock", **fields))
