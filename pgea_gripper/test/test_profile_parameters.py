import threading

from rclpy.parameter import Parameter

from pgea_gripper.pgea_gripper_node import PGEAGripperNode


def _node_stub():
    node = object.__new__(PGEAGripperNode)
    node._motion_state_lock = threading.Lock()
    node._active_motion = None
    node._force_percent = 60
    node._speed_percent = 50
    return node


def test_runtime_profile_parameters_update_values_used_by_position_moves():
    node = _node_stub()

    result = node._on_profile_parameters([
        Parameter("force_percent", value=42),
        Parameter("speed_percent", value=18),
    ])

    assert result.successful
    assert node._force_percent == 42
    assert node._speed_percent == 18


def test_runtime_profile_parameters_reject_out_of_range_values():
    node = _node_stub()

    result = node._on_profile_parameters([Parameter("force_percent", value=10)])

    assert not result.successful
    assert node._force_percent == 60


def test_runtime_profile_parameters_are_rejected_while_motion_is_active():
    node = _node_stub()
    node._active_motion = "position"

    result = node._on_profile_parameters([Parameter("speed_percent", value=22)])

    assert not result.successful
    assert "already active" in result.reason or "active" in result.reason
    assert node._speed_percent == 50
