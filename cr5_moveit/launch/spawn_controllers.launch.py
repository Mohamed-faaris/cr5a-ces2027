from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Only these controllers are backed by ros2_control. The PGEA
    # GripperCommand action is provided by pgea_gripper (physical) or
    # pgea_gripper_simulation (demo), so do not ask controller_manager to
    # spawn a second gripper action controller with the same action name.
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cr5_group_controller"],
        output="screen",
    )
    return LaunchDescription(
        [
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_controller],
                )
            ),
        ]
    )
