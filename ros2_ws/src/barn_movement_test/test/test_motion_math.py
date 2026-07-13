"""Unit tests for planar movement helpers."""

import math

from barn_movement_test.motion_math import (
    heading_to_goal,
    normalize_angle,
    quaternion_to_yaw,
)


def test_normalize_angle_wraps_both_directions():
    assert math.isclose(normalize_angle(3.0 * math.pi), math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


def test_quaternion_to_yaw():
    half = math.pi / 4.0
    assert math.isclose(
        quaternion_to_yaw(0.0, 0.0, math.sin(half), math.cos(half)),
        math.pi / 2.0,
    )


def test_heading_to_goal():
    assert math.isclose(heading_to_goal(2.0, 3.0, 2.0, 8.0), math.pi / 2.0)
