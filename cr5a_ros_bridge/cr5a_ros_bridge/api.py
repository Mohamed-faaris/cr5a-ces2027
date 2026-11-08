import asyncio
import copy
import json
import logging
import os
import time
from pathlib import Path
from typing import Optional
import uuid

import yaml
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.routing import APIRoute
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from .json_utils import finite_quaternion, finite_vector, json_safe, strict_json_loads
from rclpy.parameter import Parameter
from rcl_interfaces.srv import (
    ListParameters, GetParameters, DescribeParameters, SetParametersAtomically
)
from moveit_msgs.msg import (
    PlanningScene, PlanningSceneComponents, CollisionObject,
    AttachedCollisionObject,
)
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from pgea_gripper_interfaces.srv import GetStatus as PgeaGetStatus
from std_srvs.srv import Trigger


_json_logger = logging.getLogger("cr5a_ros_bridge.api")
_json_non_finite_warning_emitted = False


class SafeJSONResponse(JSONResponse):
    """JSON response that never emits non-standard NaN/Infinity tokens."""

    def render(self, content):
        global _json_non_finite_warning_emitted
        invalid_paths = []
        safe_content = json_safe(content, invalid_paths)
        if invalid_paths and not _json_non_finite_warning_emitted:
            _json_non_finite_warning_emitted = True
            examples = ", ".join(invalid_paths[:5])
            suffix = " ..." if len(invalid_paths) > 5 else ""
            _json_logger.warning(
                "Non-finite values in an API response were replaced with null: %s%s",
                examples, suffix,
            )
        return super().render(safe_content)


class PackageStaticFiles(StaticFiles):
    """Serve package assets installed with colcon --symlink-install."""

    def lookup_path(self, path):
        for directory in self.all_directories:
            root = os.path.abspath(directory)
            full_path = os.path.abspath(os.path.join(root, path))
            try:
                if os.path.commonpath((root, full_path)) != root:
                    continue
                stat_result = os.stat(full_path)
            except (FileNotFoundError, NotADirectoryError, ValueError):
                continue
            if not os.path.isfile(full_path):
                continue
            return full_path, stat_result
        return "", None


class StrictJSONRoute(APIRoute):
    """Reject non-standard and overflowed numbers in every JSON request body."""

    def get_route_handler(self):
        original_handler = super().get_route_handler()

        async def strict_handler(request):
            content_type = request.headers.get("content-type", "").split(";", 1)[0].strip().lower()
            if content_type == "application/json" or content_type.endswith("+json"):
                body = await request.body()
                if body:
                    try:
                        strict_json_loads(body)
                    except (ValueError, UnicodeDecodeError) as exc:
                        raise HTTPException(
                            status_code=400,
                            detail=f"Invalid JSON request body: {exc}",
                        ) from exc
            return await original_handler(request)

        return strict_handler


def _dict(message, invalid_paths=None):
    return json_safe(dict(message_to_ordereddict(message)), invalid_paths)


def _validate_finite_json(value, field_name):
    invalid_paths = []
    json_safe(value, invalid_paths)
    if invalid_paths:
        examples = ", ".join(invalid_paths[:8])
        suffix = " ..." if len(invalid_paths) > 8 else ""
        raise HTTPException(
            status_code=422,
            detail=f"{field_name} contains non-finite numbers at {examples}{suffix}",
        )


async def _await_ros(future, timeout=5.0):
    try:
        return await asyncio.wait_for(asyncio.wrap_future(future), timeout=timeout)
    except asyncio.TimeoutError as exc:
        raise HTTPException(status_code=504, detail="ROS operation timed out") from exc
    except TimeoutError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc


def _parameter_service_name(backend, node_name, suffix):
    return backend.parameter_service_name(node_name, suffix)


def _parameter_value_to_python(value):
    parameter_type = int(value.type)
    if parameter_type == 0:
        return None
    if parameter_type == 1:
        return bool(value.bool_value)
    if parameter_type == 2:
        return int(value.integer_value)
    if parameter_type == 3:
        return float(value.double_value)
    if parameter_type == 4:
        return str(value.string_value)
    if parameter_type == 5:
        return list(value.byte_array_value)
    if parameter_type == 6:
        return list(value.bool_array_value)
    if parameter_type == 7:
        return list(value.integer_array_value)
    if parameter_type == 8:
        return list(value.double_array_value)
    if parameter_type == 9:
        return list(value.string_array_value)
    return _dict(value)


async def _read_parameters(backend, node_name, names=None, describe=True):
    # Avoid an unnecessary ListParameters round-trip when the caller already
    # knows the exact parameters it needs (for example robot_description).
    if names:
        parameter_names = list(names)
    else:
        list_service = _parameter_service_name(backend, node_name, "list_parameters")
        list_request = ListParameters.Request()
        list_request.depth = 0
        listed = await _await_ros(
            backend.call_service(list_service, ListParameters, list_request)
        )
        parameter_names = list(listed.result.names)
    if not parameter_names:
        return {"parameters": {}, "descriptors": {}}

    get_service = _parameter_service_name(backend, node_name, "get_parameters")
    get_request = GetParameters.Request()
    get_request.names = parameter_names
    values = await _await_ros(
        backend.call_service(get_service, GetParameters, get_request)
    )
    output = {}
    for name, value in zip(parameter_names, values.values):
        output[name] = _parameter_value_to_python(value)

    descriptors = {}
    if describe:
        describe_service = _parameter_service_name(
            backend, node_name, "describe_parameters"
        )
        describe_request = DescribeParameters.Request()
        describe_request.names = parameter_names
        described = await _await_ros(
            backend.call_service(
                describe_service, DescribeParameters, describe_request
            )
        )
        descriptors = {
            name: _dict(value)
            for name, value in zip(parameter_names, described.descriptors)
        }
    return {"parameters": output, "descriptors": descriptors}


def create_app(backend, config):
    app = FastAPI(
        default_response_class=SafeJSONResponse,
        title="CR5A Robot API",
        version="0.1.0",
        description="Headless REST and WebSocket gateway for ROS 2 Humble, Dobot CR5A and MoveIt 2.",
    )
    app.router.route_class = StrictJSONRoute

    auth = config.get("auth", {})
    auth_enabled = bool(auth.get("enabled", False))
    token_name = auth.get("token_env", "CR5A_BRIDGE_TOKEN")
    token = os.getenv(token_name)

    if auth_enabled and not token:
        raise RuntimeError(
            f"Authentication is enabled but environment variable {token_name} is not set"
        )

    @app.middleware("http")
    async def authenticate(request: Request, call_next):
        is_public_mesh = request.url.path.startswith("/assets/dobot_rviz/meshes/")
        if auth_enabled and request.url.path != "/health" and not is_public_mesh:
            supplied = request.headers.get("Authorization", "")
            if supplied != "Bearer " + token:
                return JSONResponse(status_code=401, content={"detail": "Bearer token required"})

        return await call_next(request)

    try:
        from ament_index_python.packages import get_package_share_directory
        package_share = Path(get_package_share_directory("dobot_rviz"))
        app.mount(
            "/assets/dobot_rviz",
            PackageStaticFiles(directory=str(package_share), check_dir=False),
            name="dobot-rviz-assets",
        )
    except Exception as exc:
        _json_logger.warning("Dobot RViz package assets are unavailable: %s", exc)

    @app.get("/health")
    async def health():
        return {
            "status": "ok",
            "ros_node": backend.get_name(),
            "timestamp": time.time(),
        }

    @app.get("/api/v1/ros/graph")
    async def ros_graph():
        return backend.graph()

    @app.get("/api/v1/ros/topics")
    async def ros_topics():
        return backend.graph()["topics"]

    @app.get("/api/v1/ros/topics/latest")
    async def all_latest_topics():
        return {
            topic: backend.topic_snapshot(topic)
            for topic in list(backend._topic_cache.keys())
        }

    @app.get("/api/v1/ros/topics/{topic_name:path}/latest")
    async def latest_topic(topic_name: str):
        topic = "/" + topic_name.lstrip("/")
        value = backend.topic_snapshot(topic)
        if value is None:
            raise HTTPException(status_code=404, detail="No message received for topic")
        return value

    @app.post("/api/v1/ros/topics/publish")
    async def publish_topic(payload: dict):
        if not config.get("ros", {}).get("topic_publish_enabled", False):
            raise HTTPException(status_code=403, detail="Topic publishing is disabled by bridge safety configuration")
        required = ("topic", "type", "message")
        if any(key not in payload for key in required):
            raise HTTPException(status_code=422, detail=f"Required fields: {required}")
        _validate_finite_json(payload["message"], "message")
        try:
            return backend.publish_topic(
                payload["topic"], payload["type"], payload["message"]
            )
        except Exception as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/v1/ros/services/call")
    async def call_service(payload: dict):
        if not config.get("ros", {}).get("generic_service_calls_enabled", False):
            raise HTTPException(status_code=403, detail="Generic service calls are disabled")
        if not all(key in payload for key in ("service", "type")):
            raise HTTPException(status_code=422, detail="service and type are required")
        request_payload = payload.get("request", {})
        _validate_finite_json(request_payload, "request")
        future = backend.call_generic_service(
            payload["service"], payload["type"], request_payload
        )
        response = await _await_ros(
            future, float(config.get("ros", {}).get("service_timeout_sec", 3.0))
        )
        return {"service": payload["service"], "response": _dict(response)}

    @app.get("/api/v1/ros/parameters")
    async def get_all_parameters():
        graph = backend.graph()
        names = [
            node["fully_qualified_name"]
            for node in graph["nodes"]
            if node["fully_qualified_name"] != "/cr5a_ros_bridge"
        ]
        semaphore = asyncio.Semaphore(8)

        async def read_one(name):
            async with semaphore:
                try:
                    return name, await _read_parameters(backend, name)
                except HTTPException as exc:
                    return name, {"unavailable": exc.detail}
                except Exception as exc:
                    return name, {"unavailable": str(exc)}

        results = await asyncio.gather(*(read_one(name) for name in names))
        return {"nodes": {name: value for name, value in results}}

    @app.post("/api/v1/ros/parameters/get")
    async def get_node_parameters(payload: dict):
        node_name = payload.get("node")
        if not node_name:
            raise HTTPException(status_code=422, detail="node is required")
        return await _read_parameters(
            backend, node_name, payload.get("names"), payload.get("describe", True)
        )

    @app.post("/api/v1/ros/parameters/describe")
    async def describe_node_parameters(payload: dict):
        node_name = payload.get("node")
        if not node_name:
            raise HTTPException(status_code=422, detail="node is required")
        result = await _read_parameters(
            backend, node_name, payload.get("names"), describe=True
        )
        return {"node": node_name, "descriptors": result["descriptors"]}

    @app.post("/api/v1/ros/parameters/set")
    async def set_node_parameters(payload: dict):
        node_name = payload.get("node")
        values = payload.get("parameters")
        if not node_name or not isinstance(values, dict) or not values:
            raise HTTPException(status_code=422, detail="node and non-empty parameters object are required")
        _validate_finite_json(values, "parameters")
        service_name = _parameter_service_name(
            backend, node_name, "set_parameters_atomically"
        )
        request = SetParametersAtomically.Request()
        try:
            request.parameters = [
                Parameter(name=name, value=value).to_parameter_msg()
                for name, value in values.items()
            ]
        except Exception as exc:
            raise HTTPException(status_code=422, detail=f"Invalid parameter value: {exc}") from exc
        result = await _await_ros(
            backend.call_service(service_name, SetParametersAtomically, request)
        )
        return {"successful": result.result.successful, "reason": result.result.reason}

    @app.post("/api/v1/ros/parameters/load")
    async def load_parameters(payload: dict):
        document = payload.get("yaml")
        if document is not None:
            try:
                loaded = yaml.safe_load(document) or {}
            except yaml.YAMLError as exc:
                raise HTTPException(status_code=422, detail=f"Invalid YAML: {exc}") from exc
        elif payload.get("node") and isinstance(payload.get("parameters"), dict):
            loaded = {payload["node"]: payload["parameters"]}
        else:
            loaded = payload.get("nodes", payload)

        if not isinstance(loaded, dict) or not loaded:
            raise HTTPException(status_code=422, detail="Provide yaml, node+parameters, or a nodes mapping")

        results = {}
        for node_name, values in loaded.items():
            if not isinstance(values, dict):
                results[node_name] = {"successful": False, "reason": "Expected a parameter mapping"}
                continue
            values = {key: value for key, value in values.items() if not str(key).startswith("_")}
            try:
                _validate_finite_json(values, f"parameters for {node_name}")
            except HTTPException as exc:
                results[node_name] = {"successful": False, "reason": exc.detail}
                continue
            if not values:
                results[node_name] = {"successful": True, "reason": "No parameters to set"}
                continue
            service_name = _parameter_service_name(
                backend, node_name, "set_parameters_atomically"
            )
            request = SetParametersAtomically.Request()
            try:
                request.parameters = [
                    Parameter(name=name, value=value).to_parameter_msg()
                    for name, value in values.items()
                ]
                result = await _await_ros(
                    backend.call_service(service_name, SetParametersAtomically, request)
                )
                results[node_name] = {
                    "successful": result.result.successful,
                    "reason": result.result.reason,
                }
            except Exception as exc:
                results[node_name] = {"successful": False, "reason": str(exc)}
        return {"nodes": results, "atomicity": "per-node; not globally atomic"}

    @app.get("/api/v1/ros/parameters/dump")
    async def dump_parameters():
        graph = backend.graph()
        nodes = {}
        for item in graph["nodes"]:
            name = item["fully_qualified_name"]
            try:
                nodes[name] = (await _read_parameters(backend, name))["parameters"]
            except Exception as exc:
                nodes[name] = {"_unavailable": str(exc)}
        return {"format": "yaml", "yaml": yaml.safe_dump(nodes, sort_keys=True)}

    @app.get("/api/v1/robot/status")
    async def robot_status():
        robot_cfg = config.get("robot", {})
        status = backend.topic_snapshot(robot_cfg.get("status_topic", "/dobot_msgs_v4/msg/RobotStatus"))
        joints = backend.topic_snapshot("/joint_states")
        tcp = backend.topic_snapshot(robot_cfg.get("tcp_topic", "/dobot_msgs_v4/msg/ToolVectorActual"))
        mode = None
        try:
            response = await _await_ros(backend.robot_service("RobotMode"), 1.5)
            mode = {"value": response.robot_return, "res": response.res}
        except HTTPException:
            pass
        status_data = status["data"] if status else {}
        return {
            "connected": status_data.get("is_connected"),
            "enabled": status_data.get("is_enable"),
            "mode": mode,
            "joints": joints["data"] if joints else None,
            "tcp": tcp["data"] if tcp else None,
            "last_status_update": status.get("timestamp") if status else None,
            "simulation": bool(robot_cfg.get("simulation", False)),
        }

    @app.get("/api/v1/robot/errors")
    async def robot_errors(language: str = "English"):
        errors = {}
        for name, payload in (("GetErrorID", {}), ("GetError", {"language": language})):
            try:
                result = await _await_ros(backend.robot_service(name, payload))
                errors[name] = _dict(result)
            except HTTPException as exc:
                errors[name] = {"unavailable": exc.detail}
        return errors

    @app.post("/api/v1/robot/errors/clear")
    async def clear_robot_errors():
        if not config.get("robot", {}).get("allow_error_clear", False):
            raise HTTPException(status_code=403, detail="Error clearing is disabled by bridge safety configuration")
        result = await _await_ros(backend.robot_service("ClearError"))
        return _dict(result)

    async def read_gripper_status():
        gripper_cfg = config.get("gripper", {})
        service_name = gripper_cfg.get("status_service", "/pgea_gripper/get_status")
        try:
            response = await _await_ros(
                backend.call_service(service_name, PgeaGetStatus, PgeaGetStatus.Request()),
                float(gripper_cfg.get("status_timeout_sec", 2.0)),
            )
            if not response.success:
                return {
                    "connected": False,
                    "simulation": bool(config.get("robot", {}).get("simulation", False)),
                    "status": None,
                    "error": response.message or "PGEA status read failed",
                    "commands_enabled": bool(gripper_cfg.get("allow_commands", False)),
                }
            try:
                status = json.loads(response.status_json or "{}")
            except (TypeError, ValueError) as exc:
                raise HTTPException(status_code=502, detail=f"Invalid PGEA status response: {exc}") from exc
            return {
                "connected": True,
                "simulation": bool(config.get("robot", {}).get("simulation", False)),
                "status": status,
                "error": None,
                "message": response.message,
                "commands_enabled": bool(gripper_cfg.get("allow_commands", False)),
            }
        except HTTPException as exc:
            return {
                "connected": False,
                "simulation": bool(config.get("robot", {}).get("simulation", False)),
                "status": None,
                "error": exc.detail,
                "commands_enabled": bool(gripper_cfg.get("allow_commands", False)),
            }

    def require_gripper_commands():
        if not config.get("gripper", {}).get("allow_commands", False):
            raise HTTPException(
                status_code=403,
                detail="PGEA commands are disabled by bridge safety configuration",
            )

    async def require_connected_arm(enabled=False):
        status_topic = config.get("robot", {}).get(
            "status_topic", "/dobot_msgs_v4/msg/RobotStatus"
        )
        snapshot = backend.topic_snapshot(status_topic)
        data = snapshot.get("data", {}) if snapshot else {}
        if data.get("is_connected") is not True:
            raise HTTPException(status_code=409, detail="Robot connection is not confirmed")
        try:
            from datetime import datetime, timezone
            observed_at = datetime.fromisoformat(
                str(snapshot.get("timestamp", "")).replace("Z", "+00:00")
            )
            age = (datetime.now(timezone.utc) - observed_at).total_seconds()
        except (TypeError, ValueError):
            raise HTTPException(status_code=409, detail="Robot status timestamp is invalid")
        max_age = float(config.get("gripper", {}).get("max_robot_status_age_sec", 2.0))
        if age < -1.0 or age > max_age:
            raise HTTPException(status_code=409, detail="Robot status is stale; wait for fresh telemetry")
        mode = await _await_ros(backend.robot_service("RobotMode"), 1.5)
        mode_code = int(str(mode.robot_return))
        if int(mode.res) != 0 or mode_code not in (3, 4, 5):
            raise HTTPException(
                status_code=409,
                detail="PGEA operation is blocked while the robot controller is running, dragging, paused, or faulted",
            )
        if enabled and mode_code != 5:
            raise HTTPException(status_code=409, detail="Robot must be enabled and idle before moving the PGEA gripper")

    command_services = {
        "enable": "EnableRobot",
        "disable": "DisableRobot",
        "power-on": "PowerOn",
        "pause": "Pause",
        "resume": "Continue",
        "stop": "Stop",
        "reset": "ResetRobot",
        "start-drag": "StartDrag",
        "stop-drag": "StopDrag",
    }

    @app.post("/api/v1/robot/commands/{command}")
    async def robot_command(command: str, payload: Optional[dict] = None):
        if not config.get("robot", {}).get("allow_commands", False):
            raise HTTPException(status_code=403, detail="Robot commands are disabled by bridge safety configuration")
        service_name = command_services.get(command)
        if service_name is None:
            raise HTTPException(
                status_code=404,
                detail="Unsupported command. Emergency stop is deliberately not exposed through this network API.",
            )
        command_payload = payload or {}
        _validate_finite_json(command_payload, "command payload")
        result = await _await_ros(backend.robot_service(service_name, command_payload))
        data = _dict(result)
        backend.emit("robot.state", {"command": command, "response": data})
        return data

    async def get_scene():
        moveit = config.get("moveit", {})
        service_name = moveit.get("get_planning_scene_service", "/get_planning_scene")
        client = backend._client(service_name, GetPlanningScene)
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.TRANSFORMS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        result = await _await_ros(
            backend.call_service(service_name, GetPlanningScene, request)
        )
        return result.scene

    async def apply_scene(scene):
        service_name = config.get("moveit", {}).get(
            "apply_planning_scene_service", "/apply_planning_scene"
        )
        client = backend._client(service_name, ApplyPlanningScene)
        request = ApplyPlanningScene.Request()
        request.scene = scene
        result = await _await_ros(
            backend.call_service(service_name, ApplyPlanningScene, request)
        )
        if not result.success:
            raise HTTPException(status_code=409, detail="MoveIt rejected the planning-scene update")
        backend.emit("moveit.scene", {"updated": True, "timestamp": time.time()})

    def collision_object(payload, object_id=None):
        shape = payload.get("shape", "box").lower()
        frame = payload.get("frame_id", config.get("moveit", {}).get("base_frame", "dummy_link"))
        pose_values = finite_vector(payload.get("position_m", [0, 0, 0]), 3, "position_m")
        orientation = finite_quaternion(payload.get("orientation_xyzw", [0, 0, 0, 1]), "orientation_xyzw")
        obj = CollisionObject()
        obj.id = object_id or payload.get("id") or ("object_" + uuid.uuid4().hex[:10])
        obj.header.frame_id = frame
        obj.operation = CollisionObject.ADD
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, pose_values)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, orientation)
        primitive = SolidPrimitive()
        if shape == "box":
            dimensions = finite_vector(payload.get("dimensions_m"), 3, "dimensions_m")
            if any(value <= 0 for value in dimensions):
                raise ValueError("dimensions_m values must be greater than zero")
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = dimensions
        elif shape == "sphere":
            radius = finite_vector([payload.get("radius_m")], 1, "radius_m")[0]
            if radius <= 0:
                raise ValueError("radius_m must be greater than zero")
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [radius]
        elif shape == "cylinder":
            height, radius = finite_vector(
                [payload.get("height_m"), payload.get("radius_m")], 2, "cylinder dimensions"
            )
            if height <= 0 or radius <= 0:
                raise ValueError("height_m and radius_m must be greater than zero")
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [height, radius]
        else:
            raise ValueError("Supported shapes: box, sphere, cylinder")
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        return obj

    @app.get("/api/v1/moveit/scene")
    async def read_scene():
        return _dict(await get_scene())

    @app.get("/api/v1/moveit/scene/objects")
    async def scene_objects():
        scene = await get_scene()
        return {
            "world": [_dict(obj) for obj in scene.world.collision_objects],
            "attached": [_dict(obj) for obj in scene.robot_state.attached_collision_objects],
        }

    @app.post("/api/v1/moveit/scene/objects")
    async def add_scene_object(payload: dict):
        try:
            obj = collision_object(payload)
        except Exception as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        await apply_scene(scene)
        return {"id": obj.id, "created": True}

    @app.put("/api/v1/moveit/scene/objects/{object_id}")
    async def update_scene_object(object_id: str, payload: dict):
        try:
            obj = collision_object(payload, object_id)
        except Exception as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        await apply_scene(scene)
        return {"id": object_id, "updated": True}

    @app.delete("/api/v1/moveit/scene/objects/{object_id}")
    async def delete_scene_object(object_id: str):
        obj = CollisionObject()
        obj.id = object_id
        obj.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)
        await apply_scene(scene)
        return {"id": object_id, "removed": True}

    @app.post("/api/v1/moveit/scene/clear")
    async def clear_scene():
        current = await get_scene()
        scene = PlanningScene()
        scene.is_diff = True
        for existing in current.world.collision_objects:
            obj = CollisionObject()
            obj.id = existing.id
            obj.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(obj)
        await apply_scene(scene)
        return {"removed_world_objects": len(scene.world.collision_objects)}

    @app.post("/api/v1/moveit/scene/objects/{object_id}/attach")
    async def attach_scene_object(object_id: str, payload: dict):
        current = await get_scene()
        existing = next(
            (obj for obj in current.world.collision_objects if obj.id == object_id), None
        )
        if existing is None:
            raise HTTPException(status_code=404, detail="World collision object not found")
        link = payload.get("link_name", config.get("moveit", {}).get("end_effector_link", "Link6"))
        link_pose = payload.get("link_frame_pose")
        if not isinstance(link_pose, dict):
            raise HTTPException(
                status_code=422,
                detail="link_frame_pose is required; provide the object's pose relative to the attachment link",
            )
        try:
            link_position = finite_vector(link_pose.get("position_m"), 3, "link_frame_pose.position_m")
            link_orientation = finite_quaternion(link_pose.get("orientation_xyzw"), "link_frame_pose.orientation_xyzw")
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc
        if len(existing.primitives) != 1 or len(existing.primitive_poses) != 1 or existing.meshes:
            raise HTTPException(
                status_code=422,
                detail="Attach currently supports one bridge-created primitive object",
            )
        attached = AttachedCollisionObject()
        attached.link_name = link
        attached.touch_links = payload.get("touch_links", [])
        attached.object = copy.deepcopy(existing)
        attached.object.header.frame_id = link
        relative_pose = Pose()
        relative_pose.position.x, relative_pose.position.y, relative_pose.position.z = map(float, link_position)
        (
            relative_pose.orientation.x, relative_pose.orientation.y,
            relative_pose.orientation.z, relative_pose.orientation.w,
        ) = map(float, link_orientation)
        attached.object.primitive_poses = [relative_pose]
        attached.object.operation = CollisionObject.ADD
        remove = CollisionObject()
        remove.id = object_id
        remove.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(remove)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        await apply_scene(scene)
        return {"id": object_id, "attached_to": link}

    @app.post("/api/v1/moveit/scene/objects/{object_id}/detach")
    async def detach_scene_object(object_id: str, payload: dict):
        current = await get_scene()
        attached_existing = next(
            (item for item in current.robot_state.attached_collision_objects
             if item.object.id == object_id), None
        )
        if attached_existing is None:
            raise HTTPException(status_code=404, detail="Attached object not found")
        try:
            world_object = collision_object(payload, object_id)
        except Exception as exc:
            raise HTTPException(
                status_code=422,
                detail="Detach requires a world pose and the object's shape/dimensions: " + str(exc),
            ) from exc
        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = attached_existing.link_name
        remove_attached.object.id = object_id
        remove_attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(remove_attached)
        scene.world.collision_objects.append(world_object)
        await apply_scene(scene)
        return {"id": object_id, "detached": True}

    async def create_motion_plan_impl(payload: dict):
        plan_id = "plan_" + uuid.uuid4().hex[:12]
        backend.plans[plan_id] = {
            "id": plan_id, "status": "PLANNING", "created_at": time.time(),
            "trajectory": None, "trajectory_start": None,
        }
        backend.emit("motion.plan", {"plan_id": plan_id, "status": "PLANNING"})
        try:
            goal = backend.make_move_group_goal(payload)
            result = await _await_ros(
                backend.send_plan_goal(plan_id, goal),
                float(config.get("ros", {}).get("action_timeout_sec", 45.0)),
            )
        except HTTPException:
            backend.plans[plan_id]["status"] = "TIMED_OUT_OR_FAILED"
            backend.emit("motion.plan", {"plan_id": plan_id, "status": "TIMED_OUT_OR_FAILED"})
            raise
        except Exception as exc:
            backend.plans[plan_id]["status"] = "FAILED"
            backend.emit("motion.plan", {"plan_id": plan_id, "status": "FAILED", "error": str(exc)})
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        error_code = getattr(result.error_code, "val", None)
        succeeded = error_code == 1
        invalid_trajectory_fields = []
        trajectory_data = (
            _dict(result.planned_trajectory, invalid_trajectory_fields)
            if result.planned_trajectory is not None else None
        )
        trajectory_start_data = (
            _dict(result.trajectory_start, invalid_trajectory_fields)
            if result.trajectory_start is not None else None
        )
        planning_time = json_safe(getattr(result, "planning_time", None))

        # A planner success is not sufficient if its trajectory contains NaN,
        # Infinity, or is incomplete. Such a plan must never be executable.
        if succeeded and (
            result.planned_trajectory is None
            or result.trajectory_start is None
            or invalid_trajectory_fields
        ):
            reason = "MoveIt returned an incomplete or non-finite trajectory; the plan was rejected"
            backend.plans[plan_id].update({
                "status": "FAILED",
                "error_code": error_code,
                "planning_time": planning_time,
                "failure_reason": reason,
                "trajectory": None,
                "trajectory_start": None,
            })
            data = {
                "plan_id": plan_id,
                "status": "FAILED",
                "error_code": error_code,
                "planning_time": planning_time,
                "error": reason,
                "invalid_fields": invalid_trajectory_fields,
            }
            backend.emit("motion.plan", data)
            return data

        backend.plans[plan_id].update({
            "status": "SUCCEEDED" if succeeded else "FAILED",
            "error_code": error_code,
            "planning_time": planning_time,
            "trajectory": result.planned_trajectory,
            "trajectory_start": result.trajectory_start,
        })
        if succeeded:
            backend.publish_display_trajectory(
                result.planned_trajectory, result.trajectory_start
            )
        data = {
            "plan_id": plan_id,
            "status": backend.plans[plan_id]["status"],
            "error_code": error_code,
            "planning_time": planning_time,
            "trajectory": trajectory_data,
        }
        backend.emit("motion.plan", data)
        return data

    @app.post("/api/v1/motion-plans")
    async def create_motion_plan(payload: dict):
        return await create_motion_plan_impl(payload)

    @app.get("/api/v1/motion-plans")
    async def list_motion_plans():
        return [
            {key: value for key, value in item.items()
             if key not in ("trajectory", "trajectory_start")}
            for item in backend.plans.values()
        ]

    @app.get("/api/v1/motion-plans/{plan_id}")
    async def get_motion_plan(plan_id: str):
        plan = backend.plans.get(plan_id)
        if plan is None:
            raise HTTPException(status_code=404, detail="Plan not found")
        return {
            key: value for key, value in plan.items()
            if key not in ("trajectory", "trajectory_start")
        }

    @app.get("/api/v1/motion-plans/{plan_id}/trajectory")
    async def get_plan_trajectory(plan_id: str):
        plan = backend.plans.get(plan_id)
        if plan is None:
            raise HTTPException(status_code=404, detail="Plan not found")
        if plan.get("trajectory") is None:
            raise HTTPException(status_code=409, detail="Plan has no completed trajectory")
        return {
            "trajectory": _dict(plan["trajectory"]),
            "trajectory_start": _dict(plan["trajectory_start"]),
        }

    async def execute_motion_plan_impl(plan_id: str):
        if not config.get("moveit", {}).get("allow_execution", False):
            raise HTTPException(status_code=403, detail="Physical motion execution is disabled by bridge safety configuration")
        plan = backend.plans.get(plan_id)
        if plan is None or plan.get("status") != "SUCCEEDED":
            raise HTTPException(status_code=409, detail="Only a successful stored plan can be executed")
        status = backend.topic_snapshot(
            config.get("robot", {}).get("status_topic", "/dobot_msgs_v4/msg/RobotStatus")
        )
        if not status or not status.get("data", {}).get("is_connected"):
            raise HTTPException(status_code=409, detail="Robot connection is not confirmed")
        if not status.get("data", {}).get("is_enable"):
            raise HTTPException(status_code=409, detail="Robot is not enabled")

        # Never execute a plan against a different or stale robot start state.
        trajectory = plan.get("trajectory")
        trajectory_start = plan.get("trajectory_start")
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        planned_joint_state = getattr(trajectory_start, "joint_state", None)
        planned_names = list(getattr(joint_trajectory, "joint_names", []) or [])
        planned_state_names = list(getattr(planned_joint_state, "name", []) or [])
        planned_state_positions = list(getattr(planned_joint_state, "position", []) or [])
        if (
            not planned_names
            or any(not isinstance(name, str) or not name for name in planned_names)
            or len(set(planned_names)) != len(planned_names)
            or len(planned_state_names) != len(planned_state_positions)
            or len(set(planned_state_names)) != len(planned_state_names)
        ):
            raise HTTPException(
                status_code=409,
                detail="Stored plan has incomplete or duplicate joint-state data; create a new plan",
            )
        current_snapshot = backend.topic_snapshot("/joint_states")
        if not current_snapshot:
            raise HTTPException(status_code=409, detail="Cannot verify the robot start state; wait for joint telemetry and plan again")

        try:
            max_state_age = finite_vector(
                [config.get("moveit", {}).get("max_joint_state_age_sec", 1.0)],
                1, "max_joint_state_age_sec",
            )[0]
            if max_state_age <= 0:
                raise ValueError("max_joint_state_age_sec must be greater than zero")
            snapshot_time = current_snapshot.get("timestamp")
            from datetime import datetime
            observed_at = datetime.fromisoformat(
                str(snapshot_time).replace("Z", "+00:00")
            ).timestamp()
            telemetry_age = time.time() - observed_at
            if telemetry_age < -1.0:
                raise HTTPException(status_code=409, detail="Joint telemetry timestamp is in the future; verify the ROS clock and plan again")
            if telemetry_age > max_state_age:
                raise HTTPException(status_code=409, detail="Joint telemetry is stale; wait for fresh joint states and plan again")
        except (ValueError, OverflowError, OSError) as exc:
            raise HTTPException(status_code=409, detail="Joint telemetry timestamp or freshness configuration is invalid; plan again") from exc

        current_data = current_snapshot.get("data", {})
        current_names = list(current_data.get("name", []) or [])
        current_positions = list(current_data.get("position", []) or [])
        if (
            len(current_names) != len(current_positions)
            or any(not isinstance(name, str) or not name for name in current_names)
            or len(set(current_names)) != len(current_names)
        ):
            raise HTTPException(
                status_code=409,
                detail="Current joint telemetry has missing or duplicate joint positions; wait for a valid joint state and plan again",
            )
        try:
            current_by_name = {
                name: finite_vector([position], 1, f"current joint {name}")[0]
                for name, position in zip(current_names, current_positions)
            }
            planned_by_name = {
                name: finite_vector([position], 1, f"planned joint {name}")[0]
                for name, position in zip(planned_state_names, planned_state_positions)
            }
            tolerance = finite_vector(
                [config.get("moveit", {}).get("start_state_tolerance_rad", 0.03)],
                1, "start_state_tolerance_rad",
            )[0]
            if tolerance <= 0:
                raise ValueError("start_state_tolerance_rad must be greater than zero")
        except ValueError as exc:
            raise HTTPException(
                status_code=409,
                detail=f"Cannot verify joint state safely: {exc}. Refresh telemetry and plan again",
            ) from exc
        missing = [name for name in planned_names if name not in current_by_name or name not in planned_by_name]
        if missing:
            raise HTTPException(status_code=409, detail="Cannot compare planned and current joints: " + ", ".join(missing))
        max_delta = max(abs(current_by_name[name] - planned_by_name[name]) for name in planned_names)
        if not (max_delta >= 0 and max_delta < float("inf")):
            raise HTTPException(status_code=409, detail="Joint-state comparison produced a non-finite result; refresh telemetry and plan again")
        if max_delta > tolerance:
            raise HTTPException(
                status_code=409,
                detail=f"Robot state changed after planning (max joint delta {max_delta:.4f} rad > {tolerance:.4f} rad); create a new plan",
            )

        backend.emit("motion.execution", {"plan_id": plan_id, "status": "EXECUTING"})
        try:
            result = await _await_ros(
                backend.send_execute_goal(plan_id, plan["trajectory"]),
                float(config.get("ros", {}).get("action_timeout_sec", 45.0)),
            )
        except HTTPException:
            backend.emit("motion.execution", {"plan_id": plan_id, "status": "FAILED"})
            raise
        error_code = getattr(result.error_code, "val", None)
        state = "SUCCEEDED" if error_code == 1 else "FAILED"
        backend.emit("motion.execution", {
            "plan_id": plan_id, "status": state, "error_code": error_code
        })
        return {"plan_id": plan_id, "status": state, "error_code": error_code}

    @app.post("/api/v1/motion-plans/{plan_id}/execute")
    async def execute_motion_plan(plan_id: str):
        return await execute_motion_plan_impl(plan_id)

    @app.post("/api/v1/motion-plans/{plan_id}/cancel")
    async def cancel_motion_plan(plan_id: str):
        try:
            result = await _await_ros(backend.cancel_action(plan_id), 3.0)
            return _dict(result)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post("/api/v1/motion-plans/{plan_id}/cancel-execution")
    async def cancel_motion_execution(plan_id: str):
        if not config.get("moveit", {}).get("allow_execution", False):
            raise HTTPException(status_code=403, detail="Motion execution control is disabled by bridge safety configuration")
        try:
            result = await _await_ros(backend.cancel_action("execute:" + plan_id), 3.0)
            return _dict(result)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    async def websocket_request(method: str, params: dict):
        if method == "gripper.get_status":
            return await read_gripper_status()

        if method in ("gripper.initialize", "gripper.calibrate"):
            require_gripper_commands()
            await require_connected_arm(enabled=False)
            service_key = "initialize_service" if method == "gripper.initialize" else "calibrate_service"
            service_name = config.get("gripper", {}).get(
                service_key,
                "/pgea_gripper/initialize" if method == "gripper.initialize" else "/pgea_gripper/calibrate",
            )
            result = await _await_ros(
                backend.call_service(service_name, Trigger, Trigger.Request()),
                float(config.get("gripper", {}).get("lifecycle_timeout_sec", 70.0)),
            )
            data = {
                "success": bool(result.success),
                "message": str(result.message),
                "operation": "initialize" if method == "gripper.initialize" else "calibrate",
            }
            backend.emit("gripper.status", data)
            if not result.success:
                raise HTTPException(status_code=409, detail=result.message or "PGEA lifecycle operation failed")
            return data

        if method == "gripper.move":
            require_gripper_commands()
            await require_connected_arm(enabled=True)
            status = await read_gripper_status()
            if not status.get("connected"):
                raise HTTPException(status_code=503, detail=status.get("error") or "PGEA is not connected")
            if int((status.get("status") or {}).get("init", 0)) != 1:
                raise HTTPException(status_code=409, detail="PGEA is not initialized; initialize it before moving")

            position = finite_vector([params.get("position_m")], 1, "position_m")[0]
            if not 0.0 <= position <= 0.020:
                raise HTTPException(status_code=422, detail="PGEA position must be between 0 and 0.020 m per finger")
            max_effort = finite_vector([params.get("max_effort", 0.0)], 1, "max_effort")[0]
            if max_effort != 0.0:
                raise HTTPException(
                    status_code=422,
                    detail="PGEA max_effort must remain 0 until a calibrated Newton-to-percent force mapping exists",
                )
            result = await _await_ros(
                backend.send_gripper_goal(position, max_effort),
                float(config.get("gripper", {}).get("action_timeout_sec", 25.0)),
            )
            data = _dict(result)
            data["status"] = "SUCCEEDED" if bool(getattr(result, "reached_goal", False)) else "FAILED"
            backend.emit("gripper.result", data)
            if data["status"] != "SUCCEEDED":
                raise HTTPException(status_code=409, detail="PGEA did not confirm that it reached the requested position")
            return data

        if method == "gripper.stop":
            require_gripper_commands()
            try:
                result = await _await_ros(backend.cancel_gripper_goal(), 3.0)
                data = _dict(result)
                backend.emit("gripper.result", {"status": "CANCEL_REQUESTED", "response": data})
                return data
            except KeyError as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc

        if method == "robot.get_snapshot":
            robot_cfg = config.get("robot", {})
            status = backend.topic_snapshot(robot_cfg.get(
                "status_topic", "/dobot_msgs_v4/msg/RobotStatus"
            ))
            joints = backend.topic_snapshot("/joint_states")
            tcp = backend.topic_snapshot(robot_cfg.get(
                "tcp_topic", "/dobot_msgs_v4/msg/ToolVectorActual"
            ))
            mode = backend.event_snapshot("robot.mode")
            try:
                mode_response = await _await_ros(backend.robot_service("RobotMode"), 1.5)
                mode_data = {
                    "value": str(mode_response.robot_return),
                    "res": int(mode_response.res),
                }
                backend.emit("robot.mode", mode_data)
                mode = {"data": mode_data}
            except HTTPException:
                pass
            errors = backend.event_snapshot("robot.errors")
            status_data = status.get("data", {}) if status else {}
            return {
                "connected": status_data.get("is_connected"),
                "enabled": status_data.get("is_enable"),
                "mode": mode.get("data") if mode else None,
                "errors": errors.get("data") if errors else None,
                "joints": joints.get("data") if joints else None,
                "tcp": tcp.get("data") if tcp else None,
                "timestamps": {
                    "status": status.get("timestamp") if status else None,
                    "joints": joints.get("timestamp") if joints else None,
                    "tcp": tcp.get("timestamp") if tcp else None,
                },
                "simulation": bool(robot_cfg.get("simulation", False)),
                "capabilities": {
                    "robot_commands_enabled": bool(config.get("robot", {}).get("allow_commands", False)),
                    "error_clear_enabled": bool(config.get("robot", {}).get("allow_error_clear", False)),
                    "motion_execution_enabled": bool(config.get("moveit", {}).get("allow_execution", False)),
                    "gripper_commands_enabled": bool(config.get("gripper", {}).get("allow_commands", False)),
                },
            }

        if method == "robot.get_model":
            descriptions = {}
            for node_name in ("/robot_state_publisher", "/move_group"):
                try:
                    result = await _read_parameters(
                        backend, node_name,
                        ["robot_description", "robot_description_semantic"],
                        describe=False,
                    )
                    descriptions.update(result["parameters"])
                except Exception:
                    continue
            urdf = descriptions.get("robot_description")
            if not urdf:
                raise HTTPException(
                    status_code=503,
                    detail="robot_description is unavailable; start robot_state_publisher or MoveGroup",
                )
            return {
                "urdf": urdf,
                "srdf": descriptions.get("robot_description_semantic"),
                "base_frame": config.get("moveit", {}).get("base_frame", "dummy_link"),
                "end_effector_link": config.get("moveit", {}).get("end_effector_link", "Link6"),
                "group_name": config.get("moveit", {}).get("group_name", "cr5_group"),
            }

        if method == "robot.get_joint_limits":
            try:
                from ament_index_python.packages import get_package_share_directory
                path = Path(get_package_share_directory("cr5_moveit")) / "config" / "joint_limits.yaml"
                with path.open("r", encoding="utf-8") as stream:
                    return yaml.safe_load(stream) or {}
            except Exception as exc:
                raise HTTPException(status_code=503, detail=f"Joint limits unavailable: {exc}") from exc

        if method == "robot.get_errors":
            result = await _await_ros(backend.robot_service("GetErrorID"), 2.0)
            data = _dict(result)
            backend.emit("robot.errors", data)
            return data

        if method == "robot.get_error_details":
            language = str(params.get("language", "English"))
            result = await _await_ros(
                backend.robot_service("GetError", {"language": language}), 3.0
            )
            return _dict(result)

        if method == "robot.command":
            if not config.get("robot", {}).get("allow_commands", False):
                raise HTTPException(status_code=403, detail="Robot commands are disabled by bridge safety configuration")
            command = str(params.get("command", ""))
            service_name = command_services.get(command)
            if service_name is None:
                raise HTTPException(
                    status_code=404,
                    detail="Unsupported command. Emergency stop is not exposed through this network API.",
                )
            result = await _await_ros(
                backend.robot_service(service_name, params.get("payload", {}))
            )
            data = _dict(result)
            backend.emit("robot.state", {"command": command, "response": data})
            return data

        if method == "robot.clear_errors":
            if not config.get("robot", {}).get("allow_error_clear", False):
                raise HTTPException(status_code=403, detail="Error clearing is disabled by bridge safety configuration")
            result = await _await_ros(backend.robot_service("ClearError"))
            data = _dict(result)
            backend.emit("robot.state", {"command": "clear-errors", "response": data})
            return data

        if method == "moveit.get_scene":
            return _dict(await get_scene())

        if method in ("scene.add", "scene.update"):
            object_id = params.get("id") if method == "scene.update" else None
            try:
                obj = collision_object(params, object_id)
            except Exception as exc:
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(obj)
            await apply_scene(scene)
            return {"id": obj.id, "updated": method == "scene.update"}

        if method == "scene.remove":
            object_id = params.get("id")
            if not object_id:
                raise HTTPException(status_code=422, detail="id is required")
            obj = CollisionObject()
            obj.id = str(object_id)
            obj.operation = CollisionObject.REMOVE
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(obj)
            await apply_scene(scene)
            return {"id": object_id, "removed": True}

        if method == "scene.clear":
            current = await get_scene()
            scene = PlanningScene()
            scene.is_diff = True
            for existing in current.world.collision_objects:
                obj = CollisionObject()
                obj.id = existing.id
                obj.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(obj)
            await apply_scene(scene)
            return {"removed_world_objects": len(scene.world.collision_objects)}

        if method == "motion.plan":
            return await create_motion_plan_impl(params)

        if method == "motion.get_plan":
            plan_id = params.get("plan_id")
            plan = backend.plans.get(plan_id)
            if plan is None:
                raise HTTPException(status_code=404, detail="Plan not found")
            result = {
                key: value for key, value in plan.items()
                if key not in ("trajectory", "trajectory_start")
            }
            if plan.get("trajectory") is not None:
                result["trajectory"] = _dict(plan["trajectory"])
                result["trajectory_start"] = _dict(plan["trajectory_start"])
            return result

        if method == "motion.execute":
            return await execute_motion_plan_impl(str(params.get("plan_id", "")))

        if method in ("motion.cancel", "motion.cancel_execution"):
            plan_id = str(params.get("plan_id", ""))
            key = "execute:" + plan_id if method == "motion.cancel_execution" else plan_id
            if method == "motion.cancel_execution" and not config.get("moveit", {}).get("allow_execution", False):
                raise HTTPException(status_code=403, detail="Motion execution control is disabled by bridge safety configuration")
            try:
                result = await _await_ros(backend.cancel_action(key), 3.0)
                return _dict(result)
            except KeyError as exc:
                raise HTTPException(status_code=404, detail=str(exc)) from exc

        raise HTTPException(status_code=404, detail=f"Unknown WebSocket request method: {method}")

    @app.websocket("/ws/v1")
    async def websocket_events(websocket: WebSocket):
        if auth_enabled and websocket.query_params.get("token") != token:
            await websocket.close(code=4401)
            return
        await websocket.accept()
        channels = set()
        sent_sequences = {}
        send_lock = asyncio.Lock()

        async def safe_send(payload):
            safe_payload = json_safe(payload)
            async with send_lock:
                await websocket.send_json(safe_payload)

        async def respond(request_id, method, params):
            try:
                data = await websocket_request(method, params)
                response = {"op": "response", "id": request_id, "method": method, "ok": True, "data": data}
            except HTTPException as exc:
                response = {
                    "op": "response", "id": request_id, "method": method, "ok": False,
                    "error": {"code": exc.status_code, "message": str(exc.detail)},
                }
            except Exception as exc:
                response = {
                    "op": "response", "id": request_id, "method": method, "ok": False,
                    "error": {"code": 500, "message": str(exc)},
                }
            try:
                await safe_send(response)
            except (WebSocketDisconnect, RuntimeError):
                return

        await safe_send({
            "op": "hello", "api_version": "v1",
            "capabilities": ["request-response", "subscriptions"],
            "available_channels": [
                "robot.status", "robot.mode", "robot.joints", "robot.tcp",
                "robot.errors", "robot.state", "moveit.scene", "motion.plan",
                "motion.execution", "bridge.health", "ros.graph",
            ],
        })
        try:
            while True:
                try:
                    message = await asyncio.wait_for(websocket.receive(), timeout=0.05)
                    # Inspect the raw ASGI message: some older Uvicorn/wsproto
                    # combinations omit the optional disconnect code. Starlette's
                    # receive_json() assumes that key exists and can raise KeyError.
                    if message.get("type") == "websocket.disconnect":
                        return
                    if message.get("type") != "websocket.receive":
                        continue
                    try:
                        raw_data = message.get("text")
                        if raw_data is None:
                            raw_data = (message.get("bytes") or b"").decode("utf-8")
                        incoming = strict_json_loads(raw_data)
                    except (TypeError, ValueError, UnicodeDecodeError):
                        await safe_send({
                            "op": "error", "error": {"code": 400, "message": "Invalid JSON message"}
                        })
                        continue
                    if not isinstance(incoming, dict):
                        continue
                    invalid_numeric_paths = []
                    json_safe(incoming, invalid_numeric_paths)
                    if invalid_numeric_paths:
                        await safe_send({
                            "op": "error",
                            "error": {
                                "code": 400,
                                "message": "Non-finite numeric values are not allowed",
                                "fields": invalid_numeric_paths[:8],
                            },
                        })
                        continue
                    operation = incoming.get("op")
                    if operation in ("subscribe", "unsubscribe"):
                        requested_channels = incoming.get("channels", [])
                        if (
                            not isinstance(requested_channels, list)
                            or any(not isinstance(channel, str) or not channel for channel in requested_channels)
                        ):
                            await safe_send({
                                "op": "error",
                                "error": {"code": 422, "message": "channels must be a list of non-empty strings"},
                            })
                            continue
                        if operation == "subscribe":
                            for channel in requested_channels:
                                channels.add(channel)
                                if channel.startswith("topic:"):
                                    topic = channel[len("topic:"):]
                                    try:
                                        backend.subscribe_topic(topic)
                                    except Exception as exc:
                                        await safe_send({
                                            "op": "subscription_error", "channel": channel,
                                            "detail": str(exc),
                                        })
                            await safe_send({"op": "subscribed", "channels": sorted(channels)})
                        else:
                            for channel in requested_channels:
                                channels.discard(channel)
                            await safe_send({"op": "unsubscribed", "channels": sorted(channels)})
                    elif operation == "ping":
                        await safe_send({"op": "pong", "timestamp": time.time()})
                    elif operation == "request":
                        request_id = str(incoming.get("id") or uuid.uuid4().hex)
                        method = str(incoming.get("method") or "")
                        params = incoming.get("params") or {}
                        if not method or not isinstance(params, dict):
                            await safe_send({
                                "op": "response", "id": request_id, "method": method,
                                "ok": False,
                                "error": {"code": 422, "message": "method and object params are required"},
                            })
                        else:
                            asyncio.create_task(respond(request_id, method, params))
                except asyncio.TimeoutError:
                    pass

                for channel in tuple(channels):
                    event = (
                        backend.topic_snapshot(channel[len("topic:"):])
                        if channel.startswith("topic:")
                        else backend.event_snapshot(channel)
                    )
                    if event and event.get("sequence") != sent_sequences.get(channel):
                        sent_sequences[channel] = event["sequence"]
                        await safe_send({
                            "op": "event", "channel": channel,
                            "sequence": event["sequence"],
                            "timestamp": event["timestamp"], "data": event["data"],
                        })
        except WebSocketDisconnect:
            return

    return app
