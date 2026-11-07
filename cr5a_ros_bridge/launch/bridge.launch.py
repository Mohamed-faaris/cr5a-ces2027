import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("cr5a_ros_bridge")
    config_file = os.path.join(share, "config", "bridge.yaml")
    rosbridge_share = get_package_share_directory("rosbridge_server")
    rosbridge_launch = os.path.join(
        rosbridge_share, "launch", "rosbridge_websocket_launch.xml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_rosbridge", default_value="true"),
        Node(
            package="cr5a_ros_bridge",
            executable="cr5a_ros_bridge",
            name="cr5a_ros_bridge",
            output="screen",
            additional_env={"CR5A_BRIDGE_CONFIG": config_file},
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            condition=IfCondition(LaunchConfiguration("start_rosbridge")),
            launch_arguments={
                "port": "9090",
                "address": "127.0.0.1",
            }.items(),
        ),
    ])
