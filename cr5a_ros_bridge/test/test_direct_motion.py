import math

import pytest

from cr5a_ros_bridge.direct_motion import (
    joint_path_samples,
    movj_target,
    movl_path_samples,
    movl_target,
    quaternion_from_rpy_degrees,
    quaternion_angular_distance,
)


def test_movj_target_requires_six_finite_degree_values():
    assert movj_target([0, 10, -20, 30, 0, 90]) == [0, 10, -20, 30, 0, 90]
    with pytest.raises(ValueError, match="exactly 6"):
        movj_target([0, 1, 2])
    with pytest.raises(ValueError, match="finite"):
        movj_target([0, 0, 0, 0, float("nan"), 0])


def test_movl_target_requires_six_finite_pose_values():
    assert movl_target([100, 200, 300, 0, 90, 180]) == [100, 200, 300, 0, 90, 180]
    with pytest.raises(ValueError, match="finite"):
        movl_target([0, 0, 0, 0, float("inf"), 0])


def test_movj_path_samples_are_joint_space_and_include_target():
    samples = joint_path_samples([0.0] * 6, [0, 0, 0, 0, 0, 10], max_step_rad=math.radians(2))
    assert len(samples) == 5
    assert samples[0][5] == pytest.approx(math.radians(2))
    assert samples[-1][5] == pytest.approx(math.radians(10))


def test_movj_path_rejects_sample_overflow():
    with pytest.raises(ValueError, match="limit is 2"):
        joint_path_samples([0.0] * 6, [180, 0, 0, 0, 0, 0], max_step_rad=math.radians(1), max_samples=2)


def test_movl_path_is_linear_in_position_and_slerps_orientation():
    start_q = quaternion_from_rpy_degrees(0, 0, 0)
    samples = movl_path_samples(
        [0, 0, 0], start_q, [30, 0, 0, 0, 0, 90],
        max_linear_step_m=0.01,
        max_angular_step_rad=math.radians(45),
    )
    assert len(samples) == 3
    assert samples[0][0] == pytest.approx([0.01, 0, 0])
    assert samples[-1][0] == pytest.approx([0.03, 0, 0])
    assert quaternion_angular_distance(start_q, samples[-1][1]) == pytest.approx(math.pi / 2)


def test_movl_path_rejects_invalid_sample_configuration():
    with pytest.raises(ValueError, match="greater than zero"):
        movl_path_samples([0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 0, 0, 0], max_linear_step_m=0)
