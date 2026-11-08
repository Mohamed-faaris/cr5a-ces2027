import rclpy

from dobot_simulation.simulator import DobotSimulation


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
