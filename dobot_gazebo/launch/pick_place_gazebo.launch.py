"""Launch the PGEA CR5A in Gazebo Classic with ros2_control and MoveIt."""
from __future__ import annotations

import os
import re
import runpy
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    profile = LaunchConfiguration("shelf_profile").perform(context)
    object_kind = LaunchConfiguration("object_kind").perform(context)
    level = int(LaunchConfiguration("shelf_level").perform(context))
    world_path = Path("/tmp") / f"cr5a_pick_place_{profile}_{object_kind}.world"
    generator = Path(get_package_share_directory("dobot_gazebo")) / "scripts/generate_pick_place_world.py"
    generate_world = runpy.run_path(str(generator))["generate_world"]
    generate_world(profile, object_kind, world_path, level=level)

    gazebo_ros = get_package_share_directory("gazebo_ros")
    gazebo_launch = Path(gazebo_ros) / "launch" / "gazebo.launch.py"
    gazebo_plugin_dir = Path(get_package_share_directory("gazebo_ros2_control")).parent.parent / "lib"
    existing_plugins = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    plugin_path = os.pathsep.join(part for part in [existing_plugins, str(gazebo_plugin_dir)] if part)
    plugin_env = SetEnvironmentVariable(name="GAZEBO_PLUGIN_PATH", value=plugin_path)
    gazebo_port = LaunchConfiguration("gazebo_port").perform(context)
    master_env = SetEnvironmentVariable(name="GAZEBO_MASTER_URI", value=f"http://127.0.0.1:{gazebo_port}")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            "world": str(world_path),
            "verbose": "true",
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    urdf_path = get_package_share_path("dobot_rviz") / "urdf" / "cr5_pgea.urdf"
    controllers_path = get_package_share_path("cr5_moveit") / "config" / "ros2_controllers_pgea.yaml"
    robot_description_xml = urdf_path.read_text(encoding="utf-8")
    robot_description_xml, replacements = re.subn(
        r"<parameters>.*?</parameters>",
        f"<parameters>{controllers_path}</parameters>",
        robot_description_xml,
        count=1,
        flags=re.DOTALL,
    )
    if replacements != 1:
        raise RuntimeError("PGEA URDF is missing the gazebo_ros2_control parameters element")
    if "<robot_param>" not in robot_description_xml:
        robot_description_xml = robot_description_xml.replace(
            "<robot_param_node>",
            "<robot_param>robot_description</robot_param><robot_param_node>",
            1,
        )
    runtime_urdf = Path("/tmp/cr5a_pgea_gazebo.urdf")
    runtime_urdf.write_text(robot_description_xml, encoding="utf-8")
    runtime_sdf = Path("/tmp/cr5a_pgea_gazebo.sdf")
    converted = subprocess.run(
        ["gz", "sdf", "-p", str(runtime_urdf)],
        check=True, capture_output=True, text=True,
    )
    runtime_sdf.write_text(converted.stdout, encoding="utf-8")

    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description_xml, "use_sim_time": True}],
        remappings=[("/joint_states", "/rsp_joint_states")],
    )
    relay = Node(
        package="dobot_rviz", executable="joint_state_relay.py",
        name="joint_state_relay", output="screen", arguments=[str(urdf_path)],
        parameters=[{"use_sim_time": True}],
    )
    spawn_robot = Node(
        package="gazebo_ros", executable="spawn_entity.py", name="spawn_pgea_robot",
        output="screen", arguments=["-file", str(runtime_sdf), "-entity", "cr5_pgea"],
    )

    load_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"],
        output="screen",
    )
    load_arm = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "cr5_group_controller"],
        output="screen",
    )
    load_gripper = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "gripper_action_controller"],
        output="screen",
    )
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(get_package_share_path("cr5_moveit") / "launch" / "moveit_gazebo.launch.py")),
    )

    simulation_adapter = Node(
        package="dobot_simulation", executable="dobot_simulation", name="dobot_simulation",
        output="screen", parameters=[{
            "base_frame": "dummy_link", "tcp_link": "tcp",
            "controller_action_name": "/cr5_group_controller/follow_joint_trajectory",
            "use_sim_time": True,
        }],
    )

    chain_jsb = RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[load_jsb]))
    chain_arm = RegisterEventHandler(OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))
    chain_gripper = RegisterEventHandler(OnProcessExit(target_action=load_arm, on_exit=[load_gripper]))
    chain_moveit = RegisterEventHandler(OnProcessExit(target_action=load_gripper, on_exit=[moveit_launch]))
    return [plugin_env, master_env, gazebo, robot_state_publisher, relay, simulation_adapter, spawn_robot,
            chain_jsb, chain_arm, chain_gripper, chain_moveit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("shelf_profile", default_value="standard",
                              description="compact, standard, wide or deep shelf geometry"),
        DeclareLaunchArgument("object_kind", default_value="coke",
                              description="coke, pringles or wood_box"),
        DeclareLaunchArgument("shelf_level", default_value="1"),
        DeclareLaunchArgument("gui", default_value="false", description="Enable Gazebo GUI (requires a working display/GPU)"),
        DeclareLaunchArgument("gazebo_port", default_value="11346", description="Isolated Gazebo master TCP port"),
        OpaqueFunction(function=_setup),
    ])
