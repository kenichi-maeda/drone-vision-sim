from overwatch.nl import CommandEngine, Vocabulary
from overwatch.nl.schema import TargetQuery
from overwatch.targeting import TargetManager
from tests.conftest import CLASSES, scene


def setup():
    targets = TargetManager()
    targets.update(scene(), now=0.0)
    return CommandEngine(Vocabulary(CLASSES), clarify_timeout_s=15), targets


def test_unique_match_locks():
    engine, targets = setup()
    reply = engine.handle("lock onto the blue robot", targets, 1.0)
    assert reply.kind == "locked" and targets.track_id == 5


def test_color_and_position():
    engine, targets = setup()
    assert engine.handle("leftmost red robot", targets, 1.0).kind == "locked"
    assert targets.track_id == 3
    engine.handle("rightmost red robot", targets, 1.0)
    assert targets.track_id == 8


def test_closest_and_farthest_use_box_bottom():
    engine, targets = setup()
    engine.handle("closest duckie", targets, 1.0)
    assert targets.track_id == 11
    engine.handle("farthest robot", targets, 1.0)
    assert targets.track_id == 8
    engine.handle("closest object", targets, 1.0)
    assert targets.track_id == 11


def test_ambiguous_asks_instead_of_guessing():
    engine, targets = setup()
    reply = engine.handle("lock onto the red robot", targets, 1.0)
    assert reply.kind == "clarify"
    assert targets.track_id is None
    assert [o["track_id"] for o in reply.options] == [3, 8]  # listed left to right
    assert "Which one?" in reply.text


def test_clarification_answers():
    for answer, expected in [("2", 8), ("the left one", 3), ("#8", 8), ("first", 3), ("the far one", 8)]:
        engine, targets = setup()
        engine.handle("red robot", targets, 1.0)
        reply = engine.handle(answer, targets, 2.0)
        assert reply.kind == "locked", answer
        assert targets.track_id == expected, answer


def test_clarification_out_of_range_and_expiry():
    engine, targets = setup()
    engine.handle("red robot", targets, 1.0)
    assert engine.handle("5", targets, 2.0).kind == "none"
    engine.handle("red robot", targets, 1.0)
    # After the question expires, a bare number is a track ID again.
    reply = engine.handle("5", targets, 20.0)
    assert reply.kind == "locked" and targets.track_id == 5


def test_the_other_one():
    engine, targets = setup()
    engine.handle("leftmost red robot", targets, 1.0)
    assert targets.track_id == 3
    engine.handle("the other one", targets, 2.0)
    assert targets.track_id == 8
    engine.handle("the other one", targets, 3.0)
    assert targets.track_id == 3


def test_the_other_one_after_a_clarification():
    engine, targets = setup()
    engine.handle("red robot", targets, 1.0)
    engine.handle("1", targets, 2.0)
    engine.handle("the other one", targets, 3.0)
    assert targets.track_id == 8


def test_the_other_one_falls_back_to_same_class_and_color():
    engine, targets = setup()
    targets.lock(11, 1.0)  # locked by click, no previous command
    reply = engine.handle("the other one", targets, 2.0)
    assert reply.kind == "locked" and targets.track_id == 12


def test_no_match_lists_what_is_in_view():
    engine, targets = setup()
    reply = engine.handle("lock the purple robot", targets, 1.0)
    assert reply.kind == "none" and "No purple robot in view" in reply.text and "#5 blue robot" in reply.text
    assert targets.track_id is None


def test_unknown_object_word():
    engine, targets = setup()
    reply = engine.handle("lock onto the red cube", targets, 1.0)
    assert reply.kind == "error" and "'cube'" in reply.text
    assert targets.track_id is None


def test_ordinal_from_side():
    engine, targets = setup()
    engine.handle("second robot from the left", targets, 1.0)
    assert targets.track_id == 5
    engine.handle("second robot from the right", targets, 1.0)
    assert targets.track_id == 8


def test_release():
    engine, targets = setup()
    engine.handle("blue robot", targets, 1.0)
    reply = engine.handle("release", targets, 2.0)
    assert reply.kind == "released" and targets.track_id is None


def test_fallback_only_runs_when_parser_fails_and_is_validated():
    calls = []

    def fallback(text, class_names):
        calls.append(text)
        return TargetQuery(color="blue", class_name="robot")

    targets = TargetManager()
    targets.update(scene(), 0.0)
    engine = CommandEngine(Vocabulary(CLASSES), fallback=fallback)
    engine.handle("lock onto the blue robot", targets, 1.0)
    assert calls == []  # the parser handled it
    reply = engine.handle("go for the azure-ish fellow over yonder", targets, 2.0)
    assert calls and reply.kind == "locked" and reply.via == "llm" and targets.track_id == 5


def test_fallback_output_is_validated():
    def bad_fallback(text, class_names):
        return TargetQuery(class_name="cube")  # not a configured class

    targets = TargetManager()
    targets.update(scene(), 0.0)
    engine = CommandEngine(Vocabulary(CLASSES), fallback=bad_fallback)
    reply = engine.handle("zap the gizmo", targets, 1.0)
    assert reply.kind == "error" and "Invalid command" in reply.text and targets.track_id is None


def test_fallback_failure_is_graceful():
    def broken(text, class_names):
        raise RuntimeError("no network")

    targets = TargetManager()
    targets.update(scene(), 0.0)
    engine = CommandEngine(Vocabulary(CLASSES), fallback=broken)
    reply = engine.handle("zap the gizmo", targets, 1.0)
    assert reply.kind == "error" and "LLM fallback unavailable: no network" in reply.text
