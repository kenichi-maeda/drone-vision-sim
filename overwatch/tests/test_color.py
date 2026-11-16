import numpy as np
import pytest

from overwatch.perception.color import classify_color

BGR = {"red": (30, 30, 210), "orange": (20, 120, 240), "yellow": (20, 210, 230), "green": (40, 180, 40),
       "blue": (210, 70, 30), "purple": (190, 40, 160)}


def patch(color_bgr, size=100, background=(110, 110, 110)):
    image = np.full((200, 200, 3), background, np.uint8)
    image[50:50 + size, 50:50 + size] = color_bgr
    return image, (50, 50, 50 + size, 50 + size)


@pytest.mark.parametrize("name", list(BGR))
def test_pure_colors(name):
    image, box = patch(BGR[name])
    assert classify_color(image, box)[0] == name


def test_gray_black_white_are_unknown():
    for value in ((110, 110, 110), (10, 10, 10), (245, 245, 245)):
        image, box = patch(value)
        assert classify_color(image, box)[0] == "unknown"


def test_outer_margin_is_ignored():
    # A red object whose box is padded with green grass on every side: the grass
    # is 36% of the box, but only the inner 60% is sampled.
    image = np.zeros((200, 200, 3), np.uint8)
    image[:] = BGR["green"]
    image[70:130, 70:130] = BGR["red"]
    box = (55, 55, 145, 145)
    assert classify_color(image, box, margin=0.2)[0] == "red"
    assert classify_color(image, box, margin=0.0)[0] in ("green", "unknown")


def test_dark_wheels_and_gray_do_not_dilute_the_body_color():
    image, box = patch(BGR["blue"])
    image[100:150, 50:150] = (15, 15, 15)  # black lower half (wheels, shadow)
    color, fraction = classify_color(image, box)
    assert color == "blue" and fraction > 0.9


def test_mixed_colors_below_threshold_are_unknown():
    image = np.full((200, 200, 3), 110, np.uint8)
    image[50:150, 50:100] = BGR["red"]
    image[50:150, 100:150] = BGR["blue"]
    assert classify_color(image, (50, 50, 150, 150), min_fraction=0.6)[0] == "unknown"


def test_tiny_and_out_of_bounds_boxes():
    image, _ = patch(BGR["red"])
    assert classify_color(image, (60, 60, 62, 62))[0] == "red"
    assert classify_color(image, (500, 500, 600, 600))[0] == "unknown"
