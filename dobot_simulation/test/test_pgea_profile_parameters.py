import rclpy
from rclpy.parameter import Parameter

from dobot_simulation.pgea_simulation import PgeaGripperSimulation


def test_simulated_pgea_exposes_physical_profile_parameter_service():
    rclpy.init(args=None)
    node = PgeaGripperSimulation()
    try:
        service_names = {
            name for name, _service_types in node.get_service_names_and_types()
        }
        assert "/pgea_gripper_simulation/set_parameters_atomically" in service_names
        assert node._force_percent == 60
        assert node._speed_percent == 50
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_simulated_pgea_accepts_valid_force_and_speed_profile():
    rclpy.init(args=None)
    node = PgeaGripperSimulation()
    try:
        result = node._on_profile_parameters([
            Parameter("force_percent", value=42),
            Parameter("speed_percent", value=18),
        ])
        assert result.successful
        assert node._force_percent == 42
        assert node._speed_percent == 18
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_simulated_pgea_rejects_invalid_profile_values():
    rclpy.init(args=None)
    node = PgeaGripperSimulation()
    try:
        result = node._on_profile_parameters([
            Parameter("force_percent", value=10),
        ])
        assert not result.successful
        assert node._force_percent == 60
    finally:
        node.destroy_node()
        rclpy.shutdown()
