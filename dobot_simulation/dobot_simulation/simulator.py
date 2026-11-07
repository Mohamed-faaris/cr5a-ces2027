#!/usr/bin/env python3
"""Expose the Dobot CR5A driver-facing contracts over MoveIt fake hardware.

This node never connects to a physical robot. Joint positions come from the
fake ros2_control system; TCP coordinates are computed from the simulated TF
tree. Dashboard services update a small deterministic simulated state machine.
"""

import json
import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from dobot_msgs_v4.msg import RobotStatus, ToolVectorActual
from dobot_msgs_v4.srv import (
    ClearError,
    Continue,
    DisableRobot,
    EnableRobot,
    GetError,
    GetErrorID,
    Pause,
    PowerOn,
    ResetRobot,
    RobotMode,
    StartDrag,
    Stop,
    StopDrag,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_euler_degrees(x, y, z, w):
    """Convert a finite, non-zero quaternion to roll/pitch/yaw in degrees."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains a non-finite value")

    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError("quaternion has zero magnitude")
    x, y, z, w = (value / norm for value in values)

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class DobotSimulation(Node):
    """Dobot-compatible telemetry and dashboard services for fake hardware."""

    JOINT_NAMES = {f"joint{index}" for index in range(1, 7)}

    def __init__(self):
        super().__init__("dobot_simulation")
        self.declare_parameter("base_frame", "dummy_link")
        self.declare_parameter("tcp_link", "Link6")
        self.declare_parameter("joint_state_timeout_sec", 1.0)
        self.declare_parameter("joint_motion_threshold_rad", 0.0002)
        self.declare_parameter("idle_timeout_sec", 0.5)
        self.declare_parameter("simulated_error_code", 0)
        self.declare_parameter("controller_action_name", "/cr5_group_controller/follow_joint_trajectory")

        self._base_frame = self.get_parameter("base_frame").value
        self._tcp_link = self.get_parameter("tcp_link").value
        self._joint_timeout = float(self.get_parameter("joint_state_timeout_sec").value)
        self._motion_threshold = float(self.get_parameter("joint_motion_threshold_rad").value)
        self._idle_timeout = float(self.get_parameter("idle_timeout_sec").value)

        self._enabled = False
        self._paused = False
        self._dragging = False
        self._mode = 4  # Same RobotMode code used by the Dobot driver: disabled.
        self._has_joint_state = False
        self._last_joint_state_at = 0.0
        self._last_motion_at = 0.0
        self._joint_positions = {}
        self._active_error_code = max(0, int(self.get_parameter("simulated_error_code").value))
        self._errors = [self._active_error_code] if self._active_error_code else []
        if self._errors:
            self._mode = 9

        self._status_pub = self.create_publisher(
            RobotStatus, "/dobot_msgs_v4/msg/RobotStatus", 10
        )
        self._tcp_pub = self.create_publisher(
            ToolVectorActual, "/dobot_msgs_v4/msg/ToolVectorActual", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        action_name = str(self.get_parameter("controller_action_name").value).rstrip("/")
        self._cancel_client = self.create_client(
            CancelGoal, action_name + "/_action/cancel_goal"
        )

        prefix = "/dobot_bringup_ros2/srv/"
        service_types = {
            "RobotMode": RobotMode,
            "GetErrorID": GetErrorID,
            "GetError": GetError,
            "EnableRobot": EnableRobot,
            "DisableRobot": DisableRobot,
            "PowerOn": PowerOn,
            "Pause": Pause,
            "Continue": Continue,
            "Stop": Stop,
            "ResetRobot": ResetRobot,
            "StartDrag": StartDrag,
            "StopDrag": StopDrag,
            "ClearError": ClearError,
        }
        self._services = [
            self.create_service(
                service_type,
                prefix + name,
                lambda request, response, service_name=name: self._handle_service(
                    service_name, request, response
                ),
            )
            for name, service_type in service_types.items()
        ]

        self.create_timer(0.1, self._publish_telemetry)
        self.get_logger().info(
            "Simulation adapter ready; publishing Dobot telemetry from fake joint states and TF"
        )

    def _connected(self):
        return (
            self._has_joint_state
            and (time.monotonic() - self._last_joint_state_at) <= self._joint_timeout
        )

    def _cancel_active_trajectory(self):
        """Cancel all goals at the fake FollowJointTrajectory action server."""
        if not self._cancel_client.service_is_ready():
            return False
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        self._cancel_client.call_async(request)
        return True

    def _on_joint_state(self, message):
        if (
            len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
        ):
            return
        if not self.JOINT_NAMES.issubset(set(message.name)):
            return

        positions = {}
        try:
            for name, value in zip(message.name, message.position):
                value = float(value)
                if not math.isfinite(value):
                    return
                if name in self.JOINT_NAMES:
                    positions[name] = value
        except (TypeError, ValueError):
            return

        now = time.monotonic()
        previous = self._joint_positions
        moved = bool(previous) and any(
            abs(positions[name] - previous[name]) >= self._motion_threshold
            for name in self.JOINT_NAMES
        )
        self._joint_positions = positions
        self._has_joint_state = True
        self._last_joint_state_at = now

        if moved:
            self._last_motion_at = now
            if self._enabled and not self._paused and not self._dragging:
                self._mode = 7  # Running.
        elif (
            self._mode == 7
            and now - self._last_motion_at >= self._idle_timeout
        ):
            self._mode = 5 if self._enabled else 4

    def _sync_simulated_error(self):
        configured_code = max(0, int(self.get_parameter("simulated_error_code").value))
        if configured_code == self._active_error_code:
            return
        self._active_error_code = configured_code
        self._errors = [configured_code] if configured_code else []
        self._cancel_active_trajectory()
        self._enabled = False
        self._paused = False
        self._dragging = False
        self._mode = 9 if self._errors else 4

    def _publish_telemetry(self):
        self._sync_simulated_error()
        connected = self._connected()

        status = RobotStatus()
        status.is_connected = connected
        status.is_enable = self._enabled and self._mode == 5
        self._status_pub.publish(status)

        if not connected:
            if self._mode not in (9,):
                self._mode = 1  # Initializing / no fresh simulated joint state.
            return

        if self._mode == 1:
            self._mode = 5 if self._enabled else 4

        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._tcp_link, Time()
            )
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        try:
            rpy = quaternion_to_euler_degrees(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            xyz = (translation.x, translation.y, translation.z)
            if not all(math.isfinite(float(value)) for value in xyz):
                return
        except (TypeError, ValueError):
            return

        tcp = ToolVectorActual()
        # Dobot ToolVectorActual uses millimetres and degrees.
        tcp.x, tcp.y, tcp.z = (float(value) * 1000.0 for value in xyz)
        tcp.rx, tcp.ry, tcp.rz = rpy
        self._tcp_pub.publish(tcp)

    def _set_result(self, response, result=0):
        if hasattr(response, "res"):
            response.res = int(result)
        return response

    def _handle_service(self, name, request, response):
        del request

        if name == "RobotMode":
            if not self._connected():
                self._mode = 1
            response.robot_return = str(self._mode)
            return self._set_result(response)

        if name == "GetErrorID":
            response.robot_return = json.dumps(self._errors)
            return self._set_result(response)

        if name == "GetError":
            response.robot_return = json.dumps([
                {"id": code, "description": "Injected simulation fault"}
                for code in self._errors
            ])
            return self._set_result(response)

        if name == "ClearError":
            self.set_parameters([Parameter("simulated_error_code", value=0)])
            self._active_error_code = 0
            self._errors.clear()
            self._enabled = False
            self._mode = 4
            return self._set_result(response)

        if name == "PowerOn":
            if not self._connected():
                return self._set_result(response, -1)
            self._enabled = False
            self._paused = False
            self._dragging = False
            self._mode = 3
            return self._set_result(response)

        if name == "EnableRobot":
            if not self._connected() or self._errors:
                return self._set_result(response, -1)
            self._enabled = True
            self._paused = False
            self._dragging = False
            self._mode = 5
            return self._set_result(response)

        if name == "DisableRobot":
            self._cancel_active_trajectory()
            self._enabled = False
            self._paused = False
            self._dragging = False
            self._mode = 4
            response.robot_return = str(self._mode)
            return self._set_result(response)

        if name == "StartDrag":
            if not self._connected() or not self._enabled or self._errors:
                return self._set_result(response, -1)
            self._dragging = True
            self._paused = False
            self._mode = 6
            return self._set_result(response)

        if name == "StopDrag":
            self._dragging = False
            self._mode = 5 if self._enabled else 4
            return self._set_result(response)

        if name == "Pause":
            if not self._enabled or not self._connected():
                return self._set_result(response, -1)
            if self._mode == 7 and not self._cancel_active_trajectory():
                self.get_logger().warning(
                    "Pause requested while moving, but fake trajectory cancel service is unavailable"
                )
                return self._set_result(response, -1)
            self._paused = True
            self._dragging = False
            self._mode = 10
            return self._set_result(response)

        if name == "Continue":
            if not self._enabled or not self._connected():
                return self._set_result(response, -1)
            # A cancelled fake trajectory is not replayed; submit a fresh MoveIt plan.
            self._paused = False
            self._mode = 5
            return self._set_result(response)

        if name == "Stop":
            if self._mode == 7 and not self._cancel_active_trajectory():
                self.get_logger().warning(
                    "Stop requested while moving, but fake trajectory cancel service is unavailable"
                )
                return self._set_result(response, -1)
            self._paused = False
            self._dragging = False
            self._mode = 5 if self._enabled else 4
            return self._set_result(response)

        if name == "ResetRobot":
            self._cancel_active_trajectory()
            self.set_parameters([Parameter("simulated_error_code", value=0)])
            self._active_error_code = 0
            self._errors.clear()
            self._paused = False
            self._dragging = False
            self._enabled = False
            self._mode = 4
            return self._set_result(response)

        return self._set_result(response, -1)


def main(args=None):
    rclpy.init(args=args)
    node = DobotSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
