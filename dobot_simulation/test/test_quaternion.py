import math

import pytest

from dobot_simulation.simulator import quaternion_to_euler_degrees


def test_identity_quaternion_is_zero_rpy():
    assert quaternion_to_euler_degrees(0.0, 0.0, 0.0, 1.0) == (0.0, 0.0, 0.0)


def test_quaternion_is_normalized_before_conversion():
    result = quaternion_to_euler_degrees(0.0, 0.0, 0.0, 2.0)
    assert result == (0.0, 0.0, 0.0)


def test_yaw_quarter_turn_is_reported_in_degrees():
    half = math.sqrt(0.5)
    roll, pitch, yaw = quaternion_to_euler_degrees(0.0, 0.0, half, half)
    assert roll == pytest.approx(0.0)
    assert pitch == pytest.approx(0.0)
    assert yaw == pytest.approx(90.0)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError, match="zero magnitude"):
        quaternion_to_euler_degrees(0.0, 0.0, 0.0, 0.0)


def test_non_finite_quaternion_is_rejected():
    with pytest.raises(ValueError, match="non-finite"):
        quaternion_to_euler_degrees(float("nan"), 0.0, 0.0, 1.0)
