import concurrent.futures
import json
import threading
import time
import uuid
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from .json_utils import finite_float, finite_quaternion, finite_vector, json_safe
from rosidl_runtime_py.utilities import get_message, get_service

from rcl_interfaces.srv import (
    ListParameters, GetParameters, DescribeParameters,
    SetParameters, SetParametersAtomically,
)
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, PlanningSceneComponents, CollisionObject
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from moveit_msgs.srv import SetPlannerParams


def stamp():
    return datetime.now(timezone.utc).isoformat()


def message_dict(message, invalid_paths=None):
    payload = dict(message_to_ordereddict(message))
    return json_safe(payload, invalid_paths) if invalid_paths is not None else payload


class BridgeNode(Node):
    """ROS graph, topic, parameter, Dobot and MoveIt adapter."""

    def __init__(self, config):
        super().__init__("cr5a_ros_bridge")
        self.config = config
        self._lock = threading.RLock()
        self._service_clients = {}
        self._topic_subscriptions = {}
        self._topic_publishers = {}
        self._topic_cache = {}
        self._topic_seq = {}
        self._events = {}
        self._event_seq = {}
        self._non_finite_warnings = set()
        self.plans = {}
        self.action_handles = {}

        moveit = config.get("moveit", {})
        self.move_group_client = ActionClient(
            self, MoveGroup, moveit.get("move_group_action", "/move_action")
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory,
            moveit.get("execute_trajectory_action", "/execute_trajectory")
        )
        gripper = config.get("gripper", {})
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            gripper.get("action_name", "/gripper_action_controller/gripper_cmd"),
        )
        self._gripper_action_handle = None
        self.display_pub = self.create_publisher(
            DisplayTrajectory,
            moveit.get("display_trajectory_topic", "/display_planned_path"),
            10,
        )

        self._event_topic_map = {
            "/joint_states": "robot.joints",
            config.get("robot", {}).get("status_topic", "/dobot_msgs_v4/msg/RobotStatus"): "robot.status",
            config.get("robot", {}).get("tcp_topic", "/dobot_msgs_v4/msg/ToolVectorActual"): "robot.tcp",
            "/monitored_planning_scene": "moveit.scene",
            "/display_planned_path": "motion.plan_path",
        }
        self._error_poll_busy = False
        self._last_error_payload = None
        self._robot_mode_poll_busy = False
        self._last_robot_mode_payload = None
        self._last_graph_signature = None
        self._default_scene_seed_busy = False
        self._default_scene_seeded = False
        self._move_group_nodes = None
        self.create_timer(2.0, self._seed_default_scene)
        self.create_timer(1.0, self._poll_robot_mode)
        self.create_timer(2.0, self._poll_robot_errors)
        self.create_timer(5.0, self._poll_graph_changes)
        for entry in config.get("ros", {}).get("auto_subscribe", []):
            self.subscribe_topic(
                entry["topic"], entry.get("type"), entry.get("qos", {})
            )
        self.emit("bridge.health", {"state": "ready", "started_at": stamp()})

    def _seed_default_scene(self):
        """Seed the shared MoveIt world at bridge startup, independent of the web UI."""
        scene_config = self.config.get("scene", {})
        if scene_config.get("auto_load_defaults", True) is False:
            return
        if self._default_scene_seeded or self._default_scene_seed_busy:
            return
        moveit = self.config.get("moveit", {})
        get_name = moveit.get("get_planning_scene_service", "/get_planning_scene")
        apply_name = moveit.get("apply_planning_scene_service", "/apply_planning_scene")
        get_client = self._client(get_name, GetPlanningScene)
        apply_client = self._client(apply_name, ApplyPlanningScene)
        if not get_client.service_is_ready() or not apply_client.service_is_ready():
            return

        self._default_scene_seed_busy = True
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = get_client.call_async(request)

        def scene_loaded(done):
            try:
                response = done.result()
                existing = {
                    item.id for item in response.scene.world.collision_objects
                }
                frame = moveit.get("base_frame", "dummy_link")
                defaults = (
                    ("ground_plane", (10.0, 10.0, 0.05), (0.0, 0.0, -0.7679)),
                    ("base_pedestal", (0.15, 0.15, 0.7329), (0.0, 0.0, -0.37645)),
                    ("Box_0", (0.0405395, 0.0405395, 0.0405395), (0.0, 0.15, 0.05)),
                )
                missing = []
                for object_id, dimensions, position in defaults:
                    if object_id in existing:
                        continue
                    collision = CollisionObject()
                    collision.header.frame_id = frame
                    collision.id = object_id
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = list(dimensions)
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = position
                    pose.orientation.w = 1.0
                    collision.primitives = [primitive]
                    collision.primitive_poses = [pose]
                    collision.operation = CollisionObject.ADD
                    missing.append(collision)
                if not missing:
                    self._default_scene_seeded = True
                    self.get_logger().info("Default MoveIt scene already contains all configured objects.")
                    return
                apply_request = ApplyPlanningScene.Request()
                apply_request.scene = PlanningScene()
                apply_request.scene.is_diff = True
                apply_request.scene.world.collision_objects = missing
                applied = apply_client.call_async(apply_request)

                def scene_applied(result):
                    try:
                        if result.result().success:
                            self._default_scene_seeded = True
                            self.get_logger().info(
                                "Loaded default MoveIt collision objects at startup: "
                                + ", ".join(item.id for item in missing)
                            )
                            self.emit("moveit.scene.seeded", {"objects": [item.id for item in missing]})
                        else:
                            self.get_logger().warning("MoveIt rejected default scene initialization; will retry.")
                    except Exception as exc:
                        self.get_logger().warning(f"Default MoveIt scene initialization failed; will retry: {exc}")
                    finally:
                        self._default_scene_seed_busy = False

                applied.add_done_callback(scene_applied)
            except Exception as exc:
                self.get_logger().warning(f"Could not inspect MoveIt scene for startup objects; will retry: {exc}")
                self._default_scene_seed_busy = False

        future.add_done_callback(scene_loaded)

    def _warn_non_finite(self, source, paths):
        if not paths:
            return
        with self._lock:
            if source in self._non_finite_warnings:
                return
            self._non_finite_warnings.add(source)
        examples = ", ".join(paths[:5])
        suffix = " ..." if len(paths) > 5 else ""
        self.get_logger().warning(
            f"Non-finite numeric values received from {source}; exposed as null: {examples}{suffix}"
        )

    def emit(self, channel, data):
        invalid_paths = []
        safe_data = json_safe(data, invalid_paths)
        self._warn_non_finite("event:" + channel, invalid_paths)
        with self._lock:
            seq = self._event_seq.get(channel, 0) + 1
            self._event_seq[channel] = seq
            self._events[channel] = {
                "channel": channel, "sequence": seq,
                "timestamp": stamp(), "data": safe_data,
            }

    def event_snapshot(self, channel):
        with self._lock:
            event = self._events.get(channel)
            return dict(event) if event else None

    def topic_snapshot(self, topic):
        with self._lock:
            value = self._topic_cache.get(topic)
            return dict(value) if value else None

    def subscribe_topic(self, topic, type_name=None, qos_config=None):
        if topic in self._topic_subscriptions:
            return
        qos_config = qos_config or {}
        if not type_name:
            matches = dict(self.get_topic_names_and_types()).get(topic, [])
            if len(matches) != 1:
                raise ValueError(
                    f"Cannot resolve one message type for {topic}; found {matches}"
                )
            type_name = matches[0]
        msg_type = get_message(type_name)
        reliability = (
            ReliabilityPolicy.RELIABLE
            if qos_config.get("reliability", "reliable") == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
            if qos_config.get("durability", "volatile") == "transient_local"
            else DurabilityPolicy.VOLATILE
        )
        qos = QoSProfile(
            depth=int(qos_config.get("depth", 10)),
            reliability=reliability,
            durability=durability,
        )

        def callback(message, topic_name=topic, ros_type=type_name):
            invalid_paths = []
            payload = message_dict(message, invalid_paths)
            self._warn_non_finite(topic_name, invalid_paths)
            with self._lock:
                seq = self._topic_seq.get(topic_name, 0) + 1
                self._topic_seq[topic_name] = seq
                self._topic_cache[topic_name] = {
                    "topic": topic_name, "type": ros_type,
                    "sequence": seq, "timestamp": stamp(), "data": payload,
                }
            channel = self._event_topic_map.get(topic_name)
            if channel:
                self.emit(channel, payload)
            self.emit("topic:" + topic_name, payload)

        self._topic_subscriptions[topic] = self.create_subscription(
            msg_type, topic, callback, qos
        )
        self.get_logger().info(f"Auto-subscribed: {topic} [{type_name}]")

    def _poll_robot_mode(self):
        if self._robot_mode_poll_busy:
            return
        self._robot_mode_poll_busy = True
        try:
            future = self.robot_service("RobotMode")
        except Exception:
            self._robot_mode_poll_busy = False
            return

        def completed(done):
            try:
                response = done.result()
                invalid_paths = []
                payload = json_safe(
                    {"value": response.robot_return, "res": response.res},
                    invalid_paths,
                )
                self._warn_non_finite("robot.mode", invalid_paths)
                if payload != self._last_robot_mode_payload:
                    self._last_robot_mode_payload = payload
                    self.emit("robot.mode", payload)
            except Exception:
                pass
            finally:
                self._robot_mode_poll_busy = False

        future.add_done_callback(completed)

    def _poll_robot_errors(self):
        if self._error_poll_busy:
            return
        self._error_poll_busy = True
        try:
            future = self.robot_service("GetErrorID")
        except Exception:
            self._error_poll_busy = False
            return

        def completed(done):
            try:
                invalid_paths = []
                payload = message_dict(done.result(), invalid_paths)
                self._warn_non_finite("robot.errors", invalid_paths)
                if payload != self._last_error_payload:
                    self._last_error_payload = payload
                    self.emit("robot.errors", payload)
            except Exception:
                pass
            finally:
                self._error_poll_busy = False

        future.add_done_callback(completed)

    def _poll_graph_changes(self):
        try:
            graph = self.graph()
            move_group_nodes = tuple(sorted(
                item["fully_qualified_name"] for item in graph["nodes"]
                if item["name"] == "move_group" or item["name"].startswith("move_group_private_")
            ))
            if self._move_group_nodes is not None and move_group_nodes != self._move_group_nodes:
                self._default_scene_seeded = False
                self.get_logger().info("MoveGroup graph changed; will restore default collision objects when its scene service is ready.")
            self._move_group_nodes = move_group_nodes
            signature = json.dumps(graph, sort_keys=True, allow_nan=False)
            if signature != self._last_graph_signature:
                self._last_graph_signature = signature
                self.emit("ros.graph", graph)
        except Exception:
            pass

    def graph(self):
        nodes = []
        for name, namespace in self.get_node_names_and_namespaces():
            nodes.append({
                "name": name,
                "namespace": namespace,
                "fully_qualified_name": (
                    (namespace.rstrip("/") + "/" + name)
                    if namespace != "/" else "/" + name
                ),
            })
        services = self.get_service_names_and_types()
        service_map = dict(services)
        action_candidates = {}
        for service_name, service_types in services:
            for suffix, role in (
                ("/_action/send_goal", "send_goal"),
                ("/_action/get_result", "get_result"),
                ("/_action/cancel_goal", "cancel_goal"),
            ):
                if service_name.endswith(suffix) and service_types:
                    action_name = service_name[:-len(suffix)]
                    type_name = service_types[0]
                    if role == "send_goal" and type_name.endswith("_SendGoal"):
                        action_type = type_name[:-len("_SendGoal")]
                        action_candidates.setdefault(action_name, {})["type"] = action_type
                    action_candidates.setdefault(action_name, {})[role] = service_name
        actions = [
            {"name": name, **details}
            for name, details in sorted(action_candidates.items())
            if "send_goal" in details and "get_result" in details
        ]
        return {
            "nodes": nodes,
            "topics": [
                {"name": name, "types": types}
                for name, types in self.get_topic_names_and_types()
            ],
            "services": [
                {"name": name, "types": types}
                for name, types in services
            ],
            "actions": actions,
        }

    def _client(self, service_name, service_type):
        key = (service_name, service_type)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(service_type, service_name)
        return self._service_clients[key]

    def call_service(self, service_name, service_type, request):
        client = self._client(service_name, service_type)
        result = concurrent.futures.Future()
        timeout = float(self.config.get("ros", {}).get("service_timeout_sec", 3.0))
        if not client.wait_for_service(timeout_sec=min(timeout, 0.25)):
            result.set_exception(TimeoutError(f"Service unavailable: {service_name}"))
            return result
        ros_future = client.call_async(request)

        def complete(done):
            try:
                result.set_result(done.result())
            except Exception as exc:
                result.set_exception(exc)

        ros_future.add_done_callback(complete)
        return result

    def call_generic_service(self, service_name, type_name, payload):
        service_type = get_service(type_name)
        request = service_type.Request()
        set_message_fields(request, payload or {})
        return self.call_service(service_name, service_type, request)

    def publish_topic(self, topic, type_name, payload):
        msg_type = get_message(type_name)
        publisher = self._topic_publishers.get((topic, type_name))
        if publisher is None:
            publisher = self.create_publisher(msg_type, topic, 10)
            self._topic_publishers[(topic, type_name)] = publisher
        message = msg_type()
        set_message_fields(message, payload or {})
        publisher.publish(message)
        return {"published": True, "topic": topic, "type": type_name}

    def parameter_service_name(self, node_name, service_suffix):
        normalized = node_name if node_name.startswith("/") else "/" + node_name
        return normalized.rstrip("/") + "/" + service_suffix

    def parameter_client(self, node_name, service_suffix, service_type):
        service_name = self.parameter_service_name(node_name, service_suffix)
        return self._client(service_name, service_type)

    def list_parameter_names(self, node_name):
        request = ListParameters.Request()
        request.depth = 0
        service_name = self.parameter_service_name(node_name, "list_parameters")
        return self.call_service(service_name, ListParameters, request)

    def robot_service(self, name, payload=None):
        service_type = get_service("dobot_msgs_v4/srv/" + name)
        request = service_type.Request()
        set_message_fields(request, payload or {})
        prefix = self.config.get("robot", {}).get(
            "service_prefix", "/dobot_bringup_ros2/srv/"
        )
        return self.call_service(prefix + name, service_type, request)

    def set_planner_parameters(self, pipeline_id, group_name, planner_id, parameters):
        """Update planner-specific OMPL parameters through MoveIt's standard service."""
        request = SetPlannerParams.Request()
        request.pipeline_id = pipeline_id
        request.group = group_name
        request.planner_config = planner_id
        request.replace = False
        for key, value in parameters.items():
            request.params.keys.append(str(key))
            if isinstance(value, bool):
                request.params.values.append("true" if value else "false")
            else:
                request.params.values.append(str(value))
        service_name = self.config.get("moveit", {}).get(
            "set_planner_params_service", "/set_planner_params"
        )
        return self.call_service(service_name, SetPlannerParams, request)

    def make_move_group_goal(self, payload):
        from geometry_msgs.msg import PoseStamped
        from moveit_msgs.msg import (
            Constraints, JointConstraint, PositionConstraint,
            OrientationConstraint, BoundingVolume,
        )
        from shape_msgs.msg import SolidPrimitive

        moveit = self.config.get("moveit", {})
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = payload.get("group_name", moveit.get("group_name", "cr5_group"))
        # An empty start state is a diff against MoveIt's monitored current state.
        request.start_state.is_diff = True
        request.num_planning_attempts = int(payload.get("num_planning_attempts", 5))
        if not 1 <= request.num_planning_attempts <= 20:
            raise ValueError("num_planning_attempts must be between 1 and 20")
        request.allowed_planning_time = finite_float(
            payload.get("allowed_planning_time", 5.0), "allowed_planning_time"
        )
        if not 0 < request.allowed_planning_time <= 60:
            raise ValueError("allowed_planning_time must be greater than 0 and at most 60 seconds")
        velocity_scaling = finite_float(
            payload.get("max_velocity_scaling", 0.2), "max_velocity_scaling"
        )
        acceleration_scaling = finite_float(
            payload.get("max_acceleration_scaling", 0.2), "max_acceleration_scaling"
        )
        if not 0 < velocity_scaling <= 1:
            raise ValueError("max_velocity_scaling must be greater than 0 and at most 1")
        if not 0 < acceleration_scaling <= 1:
            raise ValueError("max_acceleration_scaling must be greater than 0 and at most 1")
        request.max_velocity_scaling_factor = velocity_scaling
        request.max_acceleration_scaling_factor = acceleration_scaling
        pipeline_id = payload.get("pipeline_id", "")
        if not isinstance(pipeline_id, str) or len(pipeline_id) > 128:
            raise ValueError("pipeline_id must be a string of at most 128 characters")
        request.pipeline_id = pipeline_id
        planner_id = payload.get("planner_id", "")
        if not isinstance(planner_id, str) or len(planner_id) > 128:
            raise ValueError("planner_id must be a string of at most 128 characters")
        request.planner_id = planner_id
        constraints = Constraints()

        target = payload.get("target", {})
        target_type = target.get("type")
        if target_type == "joints":
            names = target.get("joint_names", [])
            if (
                not isinstance(names, list) or not names
                or any(not isinstance(name, str) or not name for name in names)
                or len(set(names)) != len(names)
            ):
                raise ValueError("joint_names must be a non-empty list of unique joint names")
            values = finite_vector(
                target.get("positions_rad", []), len(names), "positions_rad"
            )
            tolerance = finite_float(target.get("tolerance_rad", 0.01), "tolerance_rad")
            if tolerance <= 0:
                raise ValueError("tolerance_rad must be greater than zero")
            for name, value in zip(names, values):
                item = JointConstraint()
                item.joint_name = name
                item.position = value
                item.tolerance_above = tolerance
                item.tolerance_below = tolerance
                item.weight = 1.0
                constraints.joint_constraints.append(item)
        elif target_type == "pose":
            frame = target.get("frame_id", moveit.get("base_frame", "dummy_link"))
            link = target.get("link_name", moveit.get("end_effector_link", "Link6"))
            position = finite_vector(target.get("position_m"), 3, "position_m")
            orientation = finite_quaternion(target.get("orientation_xyzw"), "orientation_xyzw")
            position_tolerance = finite_float(
                target.get("position_tolerance_m", 0.001), "position_tolerance_m"
            )
            if position_tolerance <= 0:
                raise ValueError("position_tolerance_m must be greater than zero")
            stamped = PoseStamped()
            stamped.header.frame_id = frame
            stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = position
            (
                stamped.pose.orientation.x, stamped.pose.orientation.y,
                stamped.pose.orientation.z, stamped.pose.orientation.w,
            ) = orientation

            region = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [position_tolerance]
            region.primitives.append(sphere)
            region.primitive_poses.append(stamped.pose)

            position_constraint = PositionConstraint()
            position_constraint.header = stamped.header
            position_constraint.link_name = link
            position_constraint.constraint_region = region
            position_constraint.weight = 1.0
            constraints.position_constraints.append(position_constraint)

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = stamped.header
            orientation_constraint.link_name = link
            orientation_constraint.orientation = stamped.pose.orientation
            tolerance = finite_float(
                target.get("orientation_tolerance_rad", 0.01), "orientation_tolerance_rad"
            )
            if tolerance <= 0:
                raise ValueError("orientation_tolerance_rad must be greater than zero")
            orientation_constraint.absolute_x_axis_tolerance = tolerance
            orientation_constraint.absolute_y_axis_tolerance = tolerance
            orientation_constraint.absolute_z_axis_tolerance = tolerance
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints.append(orientation_constraint)
        else:
            raise ValueError("target.type must be 'pose' or 'joints'")

        start_state = payload.get("start_state")
        if start_state is not None:
            if not isinstance(start_state, dict):
                raise ValueError("start_state must be an object")
            names = start_state.get("joint_names", [])
            if (not isinstance(names, list) or not names
                    or any(not isinstance(name, str) or not name for name in names)
                    or len(set(names)) != len(names)):
                raise ValueError("start_state.joint_names must be unique non-empty names")
            positions = finite_vector(start_state.get("positions_rad", []), len(names), "start_state.positions_rad")
            request.start_state.is_diff = True
            request.start_state.joint_state.name = names
            request.start_state.joint_state.position = positions

        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def send_gripper_goal(self, position_m, max_effort=0.0):
        """Send a PGEA position goal and return a concurrent Future.

        The PGEA effort register is percent, while GripperCommand.max_effort is
        Newtons. Without a calibrated conversion, only max_effort=0 is valid.
        """
        result_future = concurrent.futures.Future()
        if not self.gripper_client.wait_for_server(timeout_sec=0.5):
            result_future.set_exception(
                TimeoutError("PGEA gripper action server is unavailable")
            )
            return result_future

        goal = GripperCommand.Goal()
        goal.command.position = float(position_m)
        goal.command.max_effort = float(max_effort)

        def feedback(feedback_message):
            try:
                payload = message_dict(feedback_message.feedback)
                self.emit("gripper.feedback", payload)
            except Exception:
                pass

        sent = self.gripper_client.send_goal_async(
            goal, feedback_callback=feedback
        )

        def accepted(done):
            try:
                handle = done.result()
                if not handle.accepted:
                    raise RuntimeError(
                        "PGEA rejected the position goal (busy or not initialized)"
                    )
                self._gripper_action_handle = handle
                result = handle.get_result_async()

                def finished(result_done):
                    try:
                        wrapped = result_done.result()
                        result_future.set_result(wrapped.result)
                        self._gripper_action_handle = None
                    except Exception as exc:
                        self._gripper_action_handle = None
                        result_future.set_exception(exc)

                result.add_done_callback(finished)
            except Exception as exc:
                result_future.set_exception(exc)

        sent.add_done_callback(accepted)
        return result_future

    def cancel_gripper_goal(self):
        handle = self._gripper_action_handle
        if handle is None:
            raise KeyError("No active PGEA gripper action")
        result_future = concurrent.futures.Future()
        ros_future = handle.cancel_goal_async()

        def completed(done):
            try:
                result_future.set_result(done.result())
            except Exception as exc:
                result_future.set_exception(exc)

        ros_future.add_done_callback(completed)
        return result_future

    def send_plan_goal(self, plan_id, goal):
        result_future = concurrent.futures.Future()
        timeout = float(self.config.get("ros", {}).get("action_timeout_sec", 45.0))
        if not self.move_group_client.wait_for_server(timeout_sec=min(timeout, 1.0)):
            result_future.set_exception(TimeoutError("MoveIt /move_action is unavailable"))
            return result_future

        def feedback(feedback_message):
            try:
                invalid_paths = []
                feedback_data = message_dict(feedback_message.feedback, invalid_paths)
                self._warn_non_finite("motion.plan.feedback", invalid_paths)
                self.emit("motion.plan", {
                    "plan_id": plan_id,
                    "status": "PLANNING",
                    "feedback": feedback_data,
                })
            except Exception:
                pass

        sent = self.move_group_client.send_goal_async(goal, feedback_callback=feedback)

        def accepted(done):
            try:
                handle = done.result()
                if not handle.accepted:
                    raise RuntimeError("MoveIt rejected the planning goal")
                self.action_handles[plan_id] = handle
                result = handle.get_result_async()

                def finished(result_done):
                    try:
                        result_future.set_result(result_done.result().result)
                    except Exception as exc:
                        result_future.set_exception(exc)

                result.add_done_callback(finished)
            except Exception as exc:
                result_future.set_exception(exc)

        sent.add_done_callback(accepted)
        return result_future

    def send_execute_goal(self, plan_id, trajectory):
        result_future = concurrent.futures.Future()
        if not self.execute_client.wait_for_server(timeout_sec=1.0):
            result_future.set_exception(TimeoutError("MoveIt /execute_trajectory is unavailable"))
            return result_future
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        def feedback(feedback_message):
            try:
                invalid_paths = []
                feedback_data = message_dict(feedback_message.feedback, invalid_paths)
                self._warn_non_finite("motion.execution.feedback", invalid_paths)
                self.emit("motion.execution", {
                    "plan_id": plan_id,
                    "status": "EXECUTING",
                    "feedback": feedback_data,
                })
            except Exception:
                pass

        sent = self.execute_client.send_goal_async(
            goal, feedback_callback=feedback
        )

        def accepted(done):
            try:
                handle = done.result()
                if not handle.accepted:
                    raise RuntimeError("MoveIt rejected the execution goal")
                self.action_handles["execute:" + plan_id] = handle
                result = handle.get_result_async()

                def finished(result_done):
                    try:
                        result_future.set_result(result_done.result().result)
                    except Exception as exc:
                        result_future.set_exception(exc)

                result.add_done_callback(finished)
            except Exception as exc:
                result_future.set_exception(exc)

        sent.add_done_callback(accepted)
        return result_future

    def cancel_action(self, key):
        handle = self.action_handles.get(key)
        if handle is None:
            raise KeyError(f"No active action for {key}")
        return handle.cancel_goal_async()

    def publish_display_trajectory(self, trajectory, trajectory_start):
        display = DisplayTrajectory()
        display.trajectory_start = trajectory_start
        display.trajectory.append(trajectory)
        self.display_pub.publish(display)
