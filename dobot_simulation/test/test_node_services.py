import rclpy

from dobot_simulation.simulator import DobotSimulation
from dobot_simulation.pgea_simulation import PgeaGripperSimulation


def test_simulator_keeps_ros_parameter_services_registered():
    rclpy.init(args=None)
    node = DobotSimulation()
    try:
        service_names = {
            name for name, _service_types in node.get_service_names_and_types()
        }
        assert "/dobot_simulation/get_parameters" in service_names
        assert "/dobot_simulation/set_parameters" in service_names
        assert "/dobot_bringup_ros2/srv/RobotMode" in service_names
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pgea_simulation_publishes_both_finger_joint_states():
    rclpy.init(args=None)
    node = PgeaGripperSimulation()
    try:
        message = node._joint_state_message()
        assert message.name == ["left_finger_joint", "right_finger_joint"]
        assert list(message.position) == [0.0, 0.0]
        assert message.header.stamp.sec > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
