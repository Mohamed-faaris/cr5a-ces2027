from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "transport",
            default_value="bringup",
            description="Use the robot driver's RS485 services (bringup), standalone Dashboard, or serial adapter.",
        ),
        DeclareLaunchArgument("force_percent", default_value="60"),
        DeclareLaunchArgument("speed_percent", default_value="50"),
        DeclareLaunchArgument("auto_initialize", default_value="false"),
        Node(
            package="pgea_gripper",
            executable="pgea_gripper_node",
            name="pgea_gripper",
            output="screen",
            parameters=[{
                "transport": LaunchConfiguration("transport"),
                "force_percent": LaunchConfiguration("force_percent"),
                "speed_percent": LaunchConfiguration("speed_percent"),
                "auto_initialize": LaunchConfiguration("auto_initialize"),
            }],
        ),
    ])
