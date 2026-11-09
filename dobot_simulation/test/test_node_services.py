import rclpy

from dobot_simulation.simulator import DobotSimulation
from dobot_msgs_v4.srv import SetPayload


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
        assert "/dobot_bringup_ros2/srv/SetPayload" in service_names
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_simulator_accepts_and_stores_dobot_payload_configuration():
    rclpy.init(args=None)
    node = DobotSimulation()
    try:
        request = SetPayload.Request()
        request.load = 1.955
        request.x = -86.6
        request.y = 0.0
        request.z = -109.6
        response = node._handle_service("SetPayload", request, SetPayload.Response())
        assert response.res == 0
        assert node._payload_load_kg == 1.955
        assert node._payload_com_mm == (-86.6, 0.0, -109.6)
    finally:
        node.destroy_node()
        rclpy.shutdown()
