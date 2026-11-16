from overwatch.perception.base import PerceptionResult
from overwatch.targeting import TargetManager
from tests.conftest import det, scene


def frame(seq, detections):
    return PerceptionResult(frame_seq=seq, width=480, height=640, detections=detections)


def test_lock_follows_track_as_it_moves():
    tm = TargetManager(lost_after_s=0.5)
    tm.update(scene(1), 0.0)
    assert tm.lock(5, 0.0).startswith("Locked #5")
    tm.update(frame(2, [det(5, "robot", "blue", 260, 380, 350, 450)]), 0.1)
    state = tm.get_target_state(0.1)
    assert state.status == "TRACKING" and state.track_id == 5
    assert state.bbox_xyxy_px == (260, 380, 350, 450)
    assert abs(state.center_norm[0] - 305 / 480) < 1e-9
    assert abs(state.error_norm[0] - (305 - 240) / 240) < 1e-9


def test_brief_dropout_is_not_lost():
    tm = TargetManager(lost_after_s=0.5)
    tm.update(scene(1), 0.0)
    tm.lock(5, 0.0)
    tm.update(frame(2, []), 0.3)
    assert tm.status == "TRACKING"


def test_target_lost_then_back():
    tm = TargetManager(lost_after_s=0.5, reacquire_window_s=3.0)
    tm.update(scene(1), 0.0)
    tm.lock(5, 0.0)
    tm.update(frame(2, []), 0.6)
    assert tm.status == "LOST"
    assert tm.get_target_state(0.6).status == "LOST"
    assert any("TARGET LOST" in text for _, text in tm.messages)
    tm.update(frame(3, [det(5, "robot", "blue", 200, 380, 290, 450)]), 1.0)
    assert tm.status == "TRACKING"


def test_lock_ages_out_without_new_frames():
    tm = TargetManager(lost_after_s=0.5)
    tm.update(scene(1), 0.0)
    tm.lock(5, 0.0)
    tm.check(0.4)
    assert tm.status == "TRACKING"
    tm.check(0.7)  # no update() calls at all: the feed is gone
    assert tm.status == "LOST" and tm.get_target_state(0.7).status == "LOST"


def test_reacquire_unique_same_class_and_color_with_new_id():
    tm = TargetManager(lost_after_s=0.5, reacquire_window_s=3.0)
    tm.update(scene(1), 0.0)
    tm.lock(5, 0.0)
    tm.update(frame(2, [det(3, "robot", "red", 40, 300, 120, 360)]), 0.6)
    assert tm.status == "LOST"
    tm.update(frame(3, [det(3, "robot", "red", 40, 300, 120, 360), det(42, "robot", "blue", 0, 380, 60, 450)]), 1.0)
    assert tm.status == "TRACKING" and tm.track_id == 42


def test_no_auto_reacquire_when_ambiguous_or_too_late():
    tm = TargetManager(lost_after_s=0.5, reacquire_window_s=3.0)
    tm.update(scene(1), 0.0)
    tm.lock(3, 0.0)  # red robot
    tm.update(frame(2, []), 0.6)
    two_reds = [det(20, "robot", "red", 30, 300, 100, 360), det(21, "robot", "red", 300, 300, 380, 360)]
    tm.update(frame(3, two_reds), 1.0)
    assert tm.status == "LOST" and tm.cursor == 20  # nearest to last position is offered
    tm2 = TargetManager(lost_after_s=0.5, reacquire_window_s=3.0)
    tm2.update(scene(1), 0.0)
    tm2.lock(5, 0.0)
    tm2.update(frame(2, []), 0.6)
    tm2.update(frame(3, [det(42, "robot", "blue", 0, 380, 60, 450)]), 5.0)
    assert tm2.status == "LOST"


def test_cycle_left_to_right_and_confirm():
    tm = TargetManager()
    tm.update(scene(1), 0.0)
    order = []
    for _ in range(7):
        tm.cycle(1)
        order.append(tm.cursor)
    assert order[:6] == [11, 3, 5, 12, 8, 6] and order[6] == 11
    tm.cycle(-1)
    assert tm.cursor == 6
    assert tm.confirm(0.0).startswith("Locked #6")
    assert tm.cursor is None


def test_click_uses_the_frame_that_was_shown():
    tm = TargetManager()
    shown = scene(1)
    tm.update(shown, 0.0)
    tm.update(frame(2, [d for d in scene(2).detections]), 0.1)
    assert tm.lock_at(shown, 245, 400, 0.1).startswith("Locked #5")
    assert tm.lock_at(shown, 5, 5, 0.1) == "No object there"
    assert tm.lock_at(None, 245, 400, 0.1).startswith("That frame is too old")


def test_click_on_untracked_object():
    tm = TargetManager()
    shown = frame(1, [det(None, "robot", "red", 0, 0, 100, 100)])
    tm.update(shown, 0.0)
    assert "not tracked yet" in tm.lock_at(shown, 50, 50, 0.0)


def test_release_and_no_target_state():
    tm = TargetManager()
    tm.update(scene(1), 0.0)
    assert tm.get_target_state(0.0).status == "NONE"
    tm.lock(3, 0.0)
    assert tm.release() == "Released #3"
    assert tm.get_target_state(0.0).status == "NONE"
