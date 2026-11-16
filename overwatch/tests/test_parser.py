import pytest

from overwatch.nl.parser import Vocabulary, parse
from overwatch.nl.schema import QueryError, TargetQuery, query_from_dict, validate
from tests.conftest import CLASSES

V = Vocabulary(CLASSES)


def q(text):
    return parse(text, V).query


@pytest.mark.parametrize("text, expected", [
    ("lock onto the red robot", TargetQuery(color="red", class_name="robot")),
    ("target the leftmost blue object", TargetQuery(color="blue", selector="leftmost")),
    ("track the closest duckie", TargetQuery(class_name="duckie", selector="closest")),
    ("Lock on the RED Duckiebot!", TargetQuery(color="red", class_name="robot")),
    ("the rubber duck on the right", TargetQuery(class_name="duckie", selector="rightmost")),
    ("the far left robot", TargetQuery(class_name="robot", selector="leftmost")),
    ("lock the right-most bot", TargetQuery(class_name="robot", selector="rightmost")),
    ("the robot in the middle", TargetQuery(class_name="robot", selector="center")),
    ("biggest one", TargetQuery(selector="largest")),
    ("the farthest duck", TargetQuery(class_name="duckie", selector="farthest")),
    ("nearest red thing", TargetQuery(color="red", selector="closest")),
    ("the other one", TargetQuery(selector="other")),
    ("the other red robot", TargetQuery(color="red", class_name="robot", selector="other")),
    ("second red robot from the right", TargetQuery(color="red", class_name="robot", ordinal=2, selector="rightmost")),
    ("the second one", TargetQuery(ordinal=2)),
    ("#7", TargetQuery(track_id=7)),
    ("lock track 12", TargetQuery(track_id=12)),
    ("2", TargetQuery(index=2)),
    ("magenta robots", TargetQuery(color="purple", class_name="robot")),
    ("lock onto the quadcopter", TargetQuery(class_name="drone")),
])
def test_parses(text, expected):
    assert q(text) == expected


@pytest.mark.parametrize("text", ["release", "clear the lock", "unlock", "let go", "stop tracking", "Release target"])
def test_release(text):
    assert q(text) == TargetQuery(action="release")


def test_unknown_words_are_reported_not_ignored():
    result = parse("lock onto the red cube", V)
    assert result.query is None
    assert result.unknown == ["cube"]


def test_nonsense_and_empty_fail():
    assert parse("", V).query is None
    assert parse("lock onto it", V).query is None
    assert parse("make me a sandwich", V).query is None


def test_validate_rejects_bad_queries():
    with pytest.raises(QueryError):
        validate(TargetQuery(color="teal"), ["robot"])
    with pytest.raises(QueryError):
        validate(TargetQuery(class_name="cube"), ["robot"])
    with pytest.raises(QueryError):
        validate(TargetQuery(ordinal=2, selector="closest"), ["robot"])
    with pytest.raises(QueryError):
        validate(TargetQuery(action="release", color="red"), ["robot"])
    with pytest.raises(QueryError):
        validate(TargetQuery(), ["robot"])
    with pytest.raises(QueryError):
        validate(TargetQuery(track_id=0), ["robot"])


def test_query_from_dict_rejects_extra_fields_and_types():
    assert query_from_dict({"action": "lock", "color": "red"}, ["robot"]) == TargetQuery(color="red")
    with pytest.raises(QueryError):
        query_from_dict({"action": "lock", "color": "red", "fly_to": "target"}, ["robot"])
    with pytest.raises(QueryError):
        query_from_dict({"action": "lock", "track_id": "7"}, ["robot"])
    with pytest.raises(QueryError):
        query_from_dict({"action": "lock", "track_id": True}, ["robot"])
    with pytest.raises(QueryError):
        query_from_dict({"action": "takeoff"}, ["robot"])
