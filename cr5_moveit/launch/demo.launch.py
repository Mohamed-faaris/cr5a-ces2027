from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "cr5_robot", package_name="cr5_moveit"
    ).to_moveit_configs()
    launch_description = generate_demo_launch(moveit_config)

    # The fake ros2_control system supplies joint states. This adapter turns
    # those states and robot_state_publisher TF into the same Dobot telemetry
    # and dashboard service interfaces used by the physical bridge.
    launch_description.add_action(
        Node(
            package="dobot_simulation",
            executable="dobot_simulation",
            name="dobot_simulation",
            output="screen",
            parameters=[
                {
                    "base_frame": "dummy_link",
                    "tcp_link": "Link6",
                    "controller_action_name": "/cr5_group_controller/follow_joint_trajectory",
                }
            ],
        )
    )
    # Match the physical PGEA action and status-service contract in fake hardware.
    launch_description.add_action(
        Node(
            package="dobot_simulation",
            executable="pgea_gripper_simulation",
            name="pgea_gripper_simulation",
            output="screen",
        )
    )
    return launch_description
