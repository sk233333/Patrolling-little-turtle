import math
from types import SimpleNamespace

import pytest

from my_turtle_pkg.patrol_math import (
    clamp,
    normalize_angle,
    remaining_distance,
    validate_waypoints,
)


def point(x, y):
    return SimpleNamespace(x=x, y=y)


def test_ut01_normalize_cross_pi():
    assert normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_ut02_normalize_multi_turn():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)


def test_ut03_remaining_distance():
    waypoints = [point(0.0, 0.0), point(3.0, 4.0), point(3.0, 10.0)]
    assert remaining_distance(point(0.0, 0.0), waypoints, 0) == pytest.approx(11.0)


def test_ut04_empty_waypoints():
    with pytest.raises(ValueError):
        validate_waypoints([])


def test_ut05_length_mismatch():
    with pytest.raises(ValueError):
        validate_waypoints([1.0, 2.0], [1.0, 2.0, 3.0])


def test_ut06_out_of_range():
    with pytest.raises(ValueError):
        validate_waypoints([point(50.0, 5.0)])
    with pytest.raises(ValueError):
        validate_waypoints([point(5.0, -1.0)])


def test_ut07_clamp():
    assert clamp(10.0, -1.0, 1.0) == 1.0
    assert clamp(-10.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5
