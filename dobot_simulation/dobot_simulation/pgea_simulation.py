"""Fake PGEA gripper action and status interfaces for the MoveIt demo stack.

This node is simulation-only. It never opens a robot or RS-485 connection.
The action name and GetStatus/Trigger services intentionally match pgea_gripper.
"""
from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from pgea_gripper_interfaces.srv import GetStatus
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState


STROKE_M = 0.020


class PgeaGripperSimulation(Node):
    """Deterministic fake PGEA matching the physical node's ROS contract."""

    def __init__(self):
        super().__init__("pgea_gripper_simulation")
        self.declare_parameter("stroke_m", STROKE_M)
        self.declare_parameter("motion_time_sec", 0.8)
        self._stroke = float(self.get_parameter("stroke_m").value)
        self._motion_time = float(self.get_parameter("motion_time_sec").value)
        if not math.isfinite(self._stroke) or self._stroke != STROKE_M:
            raise ValueError("PGEA simulation stroke_m must remain 0.020 m per finger")
        if not math.isfinite(self._motion_time) or self._motion_time <= 0:
            raise ValueError("motion_time_sec must be a finite positive value")

        self._lock = threading.RLock()
        self._position_m = 0.0
        self._busy = False
        self._initialized = True
        self._grip_state = 1
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._joint_state_timer = self.create_timer(0.05, self._publish_joint_state)
        self._action_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_action_controller/gripper_cmd",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._action_group,
        )
        self._status_service = self.create_service(
            GetStatus, "/pgea_gripper/get_status", self._get_status
        )
        self._initialize_service = self.create_service(
            Trigger, "/pgea_gripper/initialize", self._initialize
        )
        self._calibrate_service = self.create_service(
            Trigger, "/pgea_gripper/calibrate", self._calibrate
        )
        self.get_logger().info(
            "Fake PGEA gripper ready: 0 mm open, 20 mm closed per finger"
        )

    def _publish_joint_state(self):
        """Publish the actuated finger joint; robot_state_publisher applies URDF mimic."""
        with self._lock:
            position = self._position_m
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["left_finger_joint"]
        message.position = [position]
        message.velocity = [0.0]
        self._joint_state_pub.publish(message)

    def _snapshot(self):
        with self._lock:
            position = int(round(self._position_m / self._stroke * 1000))
            grip = 0 if self._busy else self._grip_state
            return {
                "init": 1 if self._initialized else 0,
                "init_text": "Initialized" if self._initialized else "Not initialized",
                "grip": grip,
                "grip_text": {
                    0: "Moving",
                    1: "Position reached",
                    2: "Object caught",
                    3: "Object dropped",
                }.get(grip, "Unknown"),
                "position": position,
                "current": 0,
                "error": 0,
                "temperature": None,
                "io_input": None,
                "io_output": None,
                "alarm": {"code": 0, "name": "No error", "remedy": "No alarm reported."},
                "simulation": True,
            }

    def _get_status(self, _request, response):
        response.success = True
        response.status_json = json.dumps(self._snapshot(), allow_nan=False)
        response.message = "simulated PGEA status"
        return response

    def _initialize(self, _request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "PGEA is moving; wait for the current action to finish"
                return response
            self._initialized = True
            self._grip_state = 1
        response.success = True
        response.message = "Simulated PGEA initialized"
        return response

    def _calibrate(self, _request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = "PGEA is moving; wait for the current action to finish"
                return response
            self._initialized = True
            self._position_m = 0.0
            self._grip_state = 1
        response.success = True
        response.message = "Simulated full calibration completed"
        return response

    def _goal(self, request):
        position = float(request.command.position)
        effort = float(request.command.max_effort)
        if not math.isfinite(position) or not 0.0 <= position <= self._stroke:
            return GoalResponse.REJECT
        # The physical PGEA driver only accepts zero max_effort until a
        # calibrated Newton-to-percent mapping exists.
        if not math.isfinite(effort) or effort != 0.0:
            return GoalResponse.REJECT
        with self._lock:
            if self._busy or not self._initialized:
                return GoalResponse.REJECT
            self._busy = True
            self._grip_state = 0
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = GripperCommand.Result()
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = False
        try:
            target = float(goal_handle.request.command.position)
            with self._lock:
                start = self._position_m
            duration = max(0.08, abs(target - start) / self._stroke * self._motion_time)
            started = time.monotonic()
            while True:
                if goal_handle.is_cancel_requested:
                    with self._lock:
                        result.position = self._position_m
                        self._grip_state = 1
                    result.reached_goal = False
                    goal_handle.canceled()
                    return result
                fraction = min(1.0, (time.monotonic() - started) / duration)
                position = start + (target - start) * fraction
                with self._lock:
                    self._position_m = position
                    self._grip_state = 0 if fraction < 1.0 else 1
                feedback = GripperCommand.Feedback()
                feedback.position = position
                feedback.effort = 0.0
                goal_handle.publish_feedback(feedback)
                if fraction >= 1.0:
                    break
                time.sleep(0.04)
            result.position = target
            result.reached_goal = True
            goal_handle.succeed()
            return result
        except Exception as exc:
            self.get_logger().error(f"Fake PGEA action failed: {exc}")
            with self._lock:
                result.position = self._position_m
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._busy = False
                if self._grip_state == 0:
                    self._grip_state = 1


def main(args=None):
    rclpy.init(args=args)
    node = PgeaGripperSimulation()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
