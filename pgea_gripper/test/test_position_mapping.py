"""Unit tests for the PGEA per-finger position mapping."""
import math
import pytest

from pgea_gripper.pgea_gripper_node import (
    STROKE_M,
    m_to_permille,
    permille_to_m,
)


@pytest.mark.parametrize(
    ("position_m", "expected"),
    [
        (0.0, 0),
        (0.001, 50),
        (0.010, 500),
        (0.019, 950),
        (STROKE_M, 1000),
    ],
)
def test_m_to_permille(position_m, expected):
    assert m_to_permille(position_m) == expected


@pytest.mark.parametrize("position_m", [-0.001, STROKE_M + 0.001, math.nan, math.inf, -math.inf])
def test_m_to_permille_rejects_invalid_positions(position_m):
    with pytest.raises(ValueError):
        m_to_permille(position_m)


@pytest.mark.parametrize(
    ("position_pm", "expected"),
    [(0, 0.0), (500, 0.010), (1000, STROKE_M)],
)
def test_permille_to_m(position_pm, expected):
    assert permille_to_m(position_pm) == pytest.approx(expected)


def test_motion_trigger_registers_are_not_generic_writable():
    from pgea_gripper.pgea_app.register_map import WRITABLE_REGS

    # Motion must go through the serialized ROS action/service interfaces.
    assert 0x0100 not in WRITABLE_REGS
    assert 0x0103 not in WRITABLE_REGS
    assert 0x010A not in WRITABLE_REGS
