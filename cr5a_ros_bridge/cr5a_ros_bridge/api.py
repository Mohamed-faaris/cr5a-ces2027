import asyncio
import copy
import hashlib
import json
import logging
import math
import os
import re
import time
from pathlib import Path
from typing import Optional
from datetime import datetime, timezone
import uuid

import yaml
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.routing import APIRoute
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from .json_utils import finite_quaternion, finite_vector, json_safe, strict_json_loads
from .gripper_profiles import (
    DEFAULT_GRIPPER_PROFILES,
    GripperProfileStore,
    resolve_profiles_path,
)
from .direct_motion import (JOINT_NAMES, joint_path_samples, movj_target, movl_path_samples, movl_target)
from .pick_place import (
    SHELF_PRESETS, build_task_waypoints, generate_grasp_candidates,
    generate_shelf_scene, normalize_object, quaternion_from_rpy,
    pose_relative_to, pose_compose,
)
from rclpy.parameter import Parameter
from rcl_interfaces.srv import (
    ListParameters, GetParameters, DescribeParameters, SetParametersAtomically
)
from moveit_msgs.msg import (
    PlanningScene, PlanningSceneComponents, CollisionObject,
    AttachedCollisionObject, AllowedCollisionEntry,
)
from moveit_msgs.srv import (
    GetPlanningScene, ApplyPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity,
)
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from rosidl_runtime_py.convert import message_to_ordereddict
from pgea_gripper_interfaces.srv import GetStatus as PgeaGetStatus
from std_srvs.srv import Trigger



def _read_proc_bytes(path):
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
        if value == "max":
            return None
        return int(value)
    except (OSError, ValueError):
        return None


def system_resource_snapshot():
    """Return resource limits visible to the bridge process (including cgroups)."""
    cpu_count = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)
    cpu_quota = None
    try:
        quota, period = Path("/sys/fs/cgroup/cpu.max").read_text().split()[:2]
        if quota != "max":
            cpu_quota = max(1, int(float(quota) / int(period) + 0.999))
    except (OSError, ValueError, IndexError, ZeroDivisionError):
        pass
    if cpu_quota is not None:
        cpu_count = min(cpu_count, cpu_quota)
    cpu_count = max(1, cpu_count)

    memory = {}
    try:
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            key, raw = line.split(":", 1)
            fields = raw.strip().split()
            if fields:
                memory[key] = int(fields[0]) * (1024 if len(fields) > 1 and fields[1] == "kB" else 1)
    except (OSError, ValueError):
        pass

    total_bytes = memory.get("MemTotal", 0)
    available_bytes = memory.get("MemAvailable", memory.get("MemFree", 0))
    cgroup_limit = _read_proc_bytes("/sys/fs/cgroup/memory.max")
    cgroup_current = _read_proc_bytes("/sys/fs/cgroup/memory.current")
    if cgroup_limit is not None and cgroup_limit > 0:
        total_bytes = min(total_bytes, cgroup_limit) if total_bytes else cgroup_limit
        if cgroup_current is not None:
            available_bytes = min(available_bytes, max(0, cgroup_limit - cgroup_current))

    swap_total = memory.get("SwapTotal", 0)
    swap_free = memory.get("SwapFree", 0)
    gib = 1024 ** 3
    total_gib = total_bytes / gib if total_bytes else 0
    available_gib = available_bytes / gib if available_bytes else 0

    # Keep parallel planning bounded: at most four workers and roughly one
    # worker per 1.5 GiB of currently available memory.
    max_threads = max(1, min(4, cpu_count // 2 or 1, int(total_gib // 2) or 1))
    recommended_threads = max(
        1, min(max_threads, cpu_count // 3 or 1, int(available_gib / 1.5) or 1)
    )
    swap_used_ratio = (swap_total - swap_free) / swap_total if swap_total else 0.0
    memory_pressure = (
        available_bytes < total_bytes * 0.25 if total_bytes else False
    ) or swap_used_ratio >= 0.5

    return {
        "logical_cpus": cpu_count,
        "memory_total_bytes": total_bytes,
        "memory_available_bytes": available_bytes,
        "swap_total_bytes": swap_total,
        "swap_free_bytes": swap_free,
        "max_planning_threads": max_threads,
        "recommended_planning_threads": recommended_threads,
        "memory_pressure": memory_pressure,
    }




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

    gripper_cfg = config.get("gripper", {})
    try:
        profile_store = GripperProfileStore(
            resolve_profiles_path(gripper_cfg),
            gripper_cfg.get("profiles") or DEFAULT_GRIPPER_PROFILES,
            gripper_cfg.get("default_profile_id", "balanced"),
        )
    except ValueError as exc:
        raise RuntimeError(f"Invalid PGEA gripper profile configuration: {exc}") from exc
    gripper_runtime = {"profile_id": None}

    async def apply_gripper_profile(profile):
        node_name = gripper_cfg.get("node_name", "/pgea_gripper")
        service_name = _parameter_service_name(
            backend, node_name, "set_parameters_atomically"
        )
        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter(name="force_percent", value=int(profile["force_percent"])).to_parameter_msg(),
            Parameter(name="speed_percent", value=int(profile["speed_percent"])).to_parameter_msg(),
        ]
        response = await _await_ros(
            backend.call_service(service_name, SetParametersAtomically, request),
            float(gripper_cfg.get("profile_apply_timeout_sec", 3.0)),
        )
        if not response.result.successful:
            raise HTTPException(
                status_code=409,
                detail=response.result.reason or "PGEA rejected the selected force/speed settings",
            )
        gripper_runtime["profile_id"] = profile["id"]

    async def ensure_active_gripper_profile():
        profile = profile_store.active()
        if gripper_runtime["profile_id"] != profile["id"]:
            await apply_gripper_profile(profile)
        return profile

    def gripper_profile_snapshot():
        return profile_store.snapshot(
            runtime_synced=gripper_runtime["profile_id"] == profile_store.active_profile_id
        )

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

    @app.get("/api/v1/system/resources")
    async def system_resources():
        return system_resource_snapshot()

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

    direct_motion_lock = asyncio.Lock()
    direct_movement_enabled = bool(
        config.get("robot", {}).get("allow_commands", False)
        and config.get("robot", {}).get("allow_direct_movement", False)
    )
    direct_motion_state = {
        "phase": "recovery_required" if direct_movement_enabled else "idle",
        "last_command": None,
        "detail": (
            "Bridge restarted while direct movement is enabled. Visually verify the robot is stopped, "
            "then confirm idle before issuing a direct move."
            if direct_movement_enabled else None
        ),
        "updated_at": datetime.now(timezone.utc).isoformat(),
    }

    def _set_direct_motion_state(phase, detail=None, command=None):
        direct_motion_state.update({
            "phase": phase,
            "detail": detail,
            "last_command": command if command is not None else direct_motion_state.get("last_command"),
            "updated_at": datetime.now(timezone.utc).isoformat(),
        })
        backend.emit("robot.direct.status", dict(direct_motion_state))

    def _parse_dobot_mode_value(value):
        text = str(value or "").strip()
        match = re.fullmatch(r"\{\s*(-?\d+)\s*\}", text)
        if match:
            return int(match.group(1))
        if re.fullmatch(r"-?\d+", text):
            return int(text)
        raise HTTPException(
            status_code=409,
            detail=f"Controller returned an invalid RobotMode value: {text!r}",
        )

    async def _read_dobot_mode():
        response = await _await_ros(backend.robot_service("RobotMode"), 1.5)
        if int(getattr(response, "res", -1)) != 0:
            raise HTTPException(status_code=409, detail="Controller RobotMode query failed")
        return _parse_dobot_mode_value(response.robot_return)

    async def _wait_for_direct_motion_completion(start_joint_names, start_positions, accepted_mode):
        direct_cfg = config.get("direct_movement", {})
        start_timeout = float(direct_cfg.get("start_timeout_sec", 3.0))
        motion_timeout = float(direct_cfg.get("motion_timeout_sec", 180.0))
        poll_interval = float(direct_cfg.get("poll_interval_sec", 0.15))
        stable_delta = math.radians(float(direct_cfg.get("stable_joint_delta_deg", 0.15)))
        min_motion_delta = math.radians(float(direct_cfg.get("min_motion_delta_deg", 0.1)))
        idle_confirmations = int(direct_cfg.get("idle_confirmations", 3))
        if (
            not (0.1 <= start_timeout <= motion_timeout <= 900)
            or not (0.05 <= poll_interval <= 1.0)
            or not (0 < stable_delta <= math.radians(2))
            or not (0 < min_motion_delta <= stable_delta)
            or not (2 <= idle_confirmations <= 10)
        ):
            raise HTTPException(status_code=503, detail="Invalid direct_movement completion-monitor configuration")

        started_at = time.monotonic()
        saw_motion = accepted_mode in (6, 7, 8, 11)
        previous_positions = list(start_positions)
        idle_count = 0
        _set_direct_motion_state(
            "awaiting_motion",
            "Dobot accepted the command; waiting for motion to start and then return to idle.",
        )

        while True:
            mode = await _read_dobot_mode()
            if mode in (6, 7, 8, 11):
                saw_motion = True
                idle_count = 0
                _set_direct_motion_state("moving", f"Controller mode {mode}; motion is active.")
            elif mode == 5:
                snapshot = backend.topic_snapshot("/joint_states")
                _direct_freshness(snapshot, "Joint state")
                data = snapshot.get("data") or {}
                names = list(data.get("name") or [])
                positions = list(data.get("position") or [])
                if len(names) != len(positions) or len(set(names)) != len(names):
                    raise HTTPException(status_code=409, detail="Joint telemetry became invalid while waiting for completion")
                by_name = dict(zip(names, positions))
                if any(name not in by_name for name in start_joint_names):
                    raise HTTPException(status_code=409, detail="Joint telemetry lost a required joint while moving")
                current_positions = [float(by_name[name]) for name in start_joint_names]
                if any(not math.isfinite(value) for value in current_positions):
                    raise HTTPException(status_code=409, detail="Joint telemetry contains non-finite positions")
                if max(abs(a - b) for a, b in zip(current_positions, start_positions)) >= min_motion_delta:
                    saw_motion = True

                if saw_motion:
                    max_delta = max(
                        abs(a - b) for a, b in zip(current_positions, previous_positions)
                    )
                    idle_count = idle_count + 1 if max_delta <= stable_delta else 0
                    if idle_count >= idle_confirmations:
                        scene = await get_scene()
                        final_state, _, _, _ = await _direct_current_state(scene)
                        valid, contacts = await _direct_state_validity(final_state)
                        if not valid:
                            raise HTTPException(
                                status_code=409,
                                detail={
                                    "message": "Motion ended in a state MoveIt considers invalid",
                                    "contacts": contacts,
                                },
                            )
                        return {
                            "completed": True,
                            "final_mode": mode,
                            "stable_samples": idle_count,
                            "message": "Dobot returned to enabled/idle and joint telemetry stabilized.",
                        }
                elif time.monotonic() - started_at >= start_timeout:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "Dobot accepted the command but no running mode or joint movement was observed. "
                            "The outcome is uncertain; direct movement is locked pending manual idle confirmation."
                        ),
                    )
                previous_positions = current_positions
            else:
                raise HTTPException(
                    status_code=409,
                    detail=(
                        f"Controller entered mode {mode} while direct movement was in progress. "
                        "Motion completion is uncertain; direct movement is locked."
                    ),
                )

            elapsed = time.monotonic() - started_at
            if elapsed >= motion_timeout:
                raise HTTPException(
                    status_code=409,
                    detail=(
                        f"Direct movement did not return to idle within {motion_timeout:.0f} seconds. "
                        "Motion completion is uncertain; direct movement is locked."
                    ),
                )
            await asyncio.sleep(poll_interval)

    def _direct_joint_names():
        names = config.get("moveit", {}).get("joint_names", list(JOINT_NAMES))
        if (
            not isinstance(names, list)
            or len(names) != 6
            or any(not isinstance(name, str) or not name for name in names)
            or len(set(names)) != 6
        ):
            raise HTTPException(status_code=503, detail="Direct movement requires six unique MoveIt joint names")
        return names

    def _direct_joint_position_limits(joint_names):
        try:
            from ament_index_python.packages import get_package_share_directory
            limits_path = Path(get_package_share_directory("cr5_moveit")) / "config" / "joint_limits.yaml"
            with limits_path.open("r", encoding="utf-8") as stream:
                configured = yaml.safe_load(stream) or {}
            entries = configured.get("joint_limits", {})
        except Exception as exc:
            raise HTTPException(
                status_code=503,
                detail=f"Direct movement cannot load CR5A MoveIt joint limits: {exc}",
            ) from exc

        result = {}
        for name in joint_names:
            entry = entries.get(name) or {}
            if entry.get("has_position_limits") is not True:
                raise HTTPException(
                    status_code=503,
                    detail=f"Direct movement requires configured position limits for {name}",
                )
            try:
                lower = float(entry["min_position"])
                upper = float(entry["max_position"])
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                raise HTTPException(
                    status_code=503,
                    detail=f"Invalid configured position limits for {name}",
                ) from exc
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise HTTPException(
                    status_code=503,
                    detail=f"Invalid configured position limits for {name}",
                )
            result[name] = (lower, upper)
        return result

    def _validate_direct_joint_positions(names, positions, limits, label):
        if len(names) != len(positions):
            raise ValueError(f"{label} does not contain a complete joint position vector")
        by_name = dict(zip(names, positions))
        for name, (lower, upper) in limits.items():
            value = by_name.get(name)
            if value is None or not math.isfinite(float(value)):
                raise ValueError(f"{label} is missing a finite position for {name}")
            if value < lower or value > upper:
                raise ValueError(
                    f"{label} puts {name} at {math.degrees(value):.2f} degrees, "
                    f"outside the configured range [{math.degrees(lower):.2f}, "
                    f"{math.degrees(upper):.2f}] degrees"
                )

    def _direct_freshness(snapshot, label):
        if not snapshot or not snapshot.get("timestamp"):
            raise HTTPException(status_code=409, detail=f"{label} telemetry is unavailable")
        try:
            observed = datetime.fromisoformat(
                str(snapshot["timestamp"]).replace("Z", "+00:00")
            ).timestamp()
            age = time.time() - observed
            max_age = float(config.get("moveit", {}).get("max_joint_state_age_sec", 1.0))
            if not (0 <= age <= max_age):
                raise HTTPException(
                    status_code=409,
                    detail=f"{label} telemetry is stale or future-dated; wait for fresh telemetry",
                )
        except (ValueError, TypeError, OverflowError, OSError) as exc:
            raise HTTPException(status_code=409, detail=f"{label} telemetry timestamp is invalid") from exc

    async def _direct_current_state(scene):
        robot_cfg = config.get("robot", {})
        status = backend.topic_snapshot(robot_cfg.get(
            "status_topic", "/dobot_msgs_v4/msg/RobotStatus"
        ))
        if not status or status.get("data", {}).get("is_connected") is not True:
            raise HTTPException(status_code=409, detail="Robot connection is not confirmed")
        if status.get("data", {}).get("is_enable") is not True:
            raise HTTPException(status_code=409, detail="Robot is not enabled")
        _direct_freshness(status, "Robot status")

        mode = await _await_ros(backend.robot_service("RobotMode"), 1.5)
        if int(getattr(mode, "res", 0)) != 0:
            raise HTTPException(status_code=409, detail="Robot mode could not be verified")
        try:
            mode_value = _parse_dobot_mode_value(mode.robot_return)
        except HTTPException as exc:
            raise HTTPException(status_code=409, detail="Robot mode could not be verified") from exc
        if mode_value != 5:
            raise HTTPException(
                status_code=409,
                detail=f"Direct movement requires enabled/idle mode 5; current mode is {mode_value}",
            )

        try:
            errors = await _await_ros(backend.robot_service("GetErrorID"), 1.5)
            raw_errors = str(getattr(errors, "robot_return", "")).strip()
            if not raw_errors:
                raise HTTPException(status_code=409, detail="Controller error state could not be verified")
            if int(getattr(errors, "res", 0)) != 0:
                raise HTTPException(status_code=409, detail="Controller error state could not be verified")
            try:
                error_payload = json.loads(raw_errors) if raw_errors else []
            except (TypeError, ValueError):
                error_payload = raw_errors
            def contains_active_error(value):
                if isinstance(value, bool) or value is None:
                    return False
                if isinstance(value, (int, float)):
                    return value != 0
                if isinstance(value, (list, tuple)):
                    return any(contains_active_error(item) for item in value)
                if isinstance(value, dict):
                    return any(contains_active_error(item) for item in value.values())
                if isinstance(value, str):
                    text = value.strip()
                    if not text or text == "[]" or text.lower() in ("none", "no error"):
                        return False
                    try:
                        return contains_active_error(json.loads(text))
                    except (TypeError, ValueError):
                        numbers = re.findall(r"-?\d+(?:\.\d+)?", text)
                        return any(float(item) != 0 for item in numbers) if numbers else True
                return False
            if contains_active_error(error_payload):
                raise HTTPException(status_code=409, detail="Controller reports active error IDs; clear and verify errors before moving")
        except HTTPException:
            raise

        snapshot = backend.topic_snapshot("/joint_states")
        _direct_freshness(snapshot, "Joint state")
        names = list((snapshot.get("data") or {}).get("name") or [])
        positions = list((snapshot.get("data") or {}).get("position") or [])
        if len(names) != len(positions) or len(set(names)) != len(names):
            raise HTTPException(status_code=409, detail="Joint telemetry has missing or duplicate positions")
        try:
            from .json_utils import finite_vector as _finite_vector
            by_name = {
                name: _finite_vector([position], 1, f"joint {name}")[0]
                for name, position in zip(names, positions)
                if isinstance(name, str) and name
            }
            joint_names = _direct_joint_names()
            missing = [name for name in joint_names if name not in by_name]
            if missing:
                raise ValueError("Missing joints: " + ", ".join(missing))
            values = [by_name[name] for name in joint_names]
            joint_limits = _direct_joint_position_limits(joint_names)
            _validate_direct_joint_positions(
                joint_names, values, joint_limits, "Current joint state"
            )
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=f"Cannot use current joint telemetry: {exc}") from exc

        state = RobotState()
        state.joint_state.name = joint_names
        state.joint_state.position = values
        state.is_diff = False
        # Preserve attached payloads in every validity check.
        state.attached_collision_objects = copy.deepcopy(
            list(getattr(getattr(scene, "robot_state", None), "attached_collision_objects", []) or [])
        )
        return state, joint_names, values, joint_limits

    async def _direct_state_validity(state):
        moveit = config.get("moveit", {})
        service_name = moveit.get("check_state_validity_service", "/check_state_validity")
        request = GetStateValidity.Request()
        request.robot_state = state
        request.group_name = moveit.get("group_name", "cr5_group")
        response = await _await_ros(
            backend.call_service(service_name, GetStateValidity, request),
            float(config.get("direct_movement", {}).get("service_timeout_sec", 3.0)),
        )
        contacts = [_dict(item) for item in list(response.contacts)[:5]]
        return bool(response.valid), contacts

    def _movl_controller_frame():
        direct_cfg = config.get("direct_movement", {})
        user_index = direct_cfg.get("controller_user")
        tool_index = direct_cfg.get("controller_tool")
        confirmed = direct_cfg.get("controller_pose_frame_confirmed", False)
        if (
            not confirmed
            or isinstance(user_index, bool) or not isinstance(user_index, int) or user_index < 0
            or isinstance(tool_index, bool) or not isinstance(tool_index, int) or tool_index < 0
        ):
            raise HTTPException(
                status_code=409,
                detail=(
                    "MovL requires verified controller user/tool frame alignment and explicit "
                    "non-negative controller_user/controller_tool indices in bridge.yaml"
                ),
            )
        return user_index, tool_index

    def _public_direct_check(result):
        return {
            key: value for key, value in result.items()
            if not key.startswith("_")
        }

    def _direct_scene_fingerprint(scene):
        signature = {
            "world": _dict(scene.world),
            "attached": [
                _dict(item) for item in scene.robot_state.attached_collision_objects
            ],
            "allowed_collision_matrix": _dict(scene.allowed_collision_matrix),
        }
        encoded = json.dumps(signature, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(encoded.encode("utf-8")).hexdigest()

    async def check_direct_movement_impl(payload):
        movement_type = str(payload.get("type", "")).lower()
        try:
            if movement_type == "movj":
                target = movj_target(payload.get("target_joints_deg"))
            elif movement_type == "movl":
                target = movl_target(payload.get("target_pose_mm_deg"))
            else:
                raise ValueError("type must be 'movj' or 'movl'")
        except ValueError as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

        # A planning-scene fetch is mandatory: fail closed if MoveIt cannot
        # provide its current world and attached-object state.
        current_scene = await get_scene()
        scene_fingerprint = _direct_scene_fingerprint(current_scene)
        current_state, joint_names, current_positions, joint_limits = await _direct_current_state(current_scene)
        is_valid, contacts = await _direct_state_validity(current_state)
        if not is_valid:
            return {
                "type": movement_type, "collision_free": False, "samples_checked": 0,
                "reason": "Current robot state is already invalid or in collision",
                "contacts": contacts,
            }

        moveit = config.get("moveit", {})
        group_name = moveit.get("group_name", "cr5_group")
        base_frame = moveit.get("base_frame", "dummy_link")
        end_effector = moveit.get("end_effector_link", "Link6")
        direct_cfg = config.get("direct_movement", {})
        max_samples = int(direct_cfg.get("max_samples", 200))
        service_timeout = float(direct_cfg.get("service_timeout_sec", 3.0))
        checked = 0

        if movement_type == "movj":
            target_radians = [math.radians(value) for value in target]
            try:
                _validate_direct_joint_positions(
                    joint_names, target_radians, joint_limits, "MovJ target"
                )
                samples = joint_path_samples(
                    current_positions,
                    target,
                    math.radians(float(direct_cfg.get("joint_step_deg", 2.0))),
                    max_samples,
                )
            except ValueError as exc:
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            for index, positions in enumerate(samples, start=1):
                candidate = copy.deepcopy(current_state)
                candidate.joint_state.position = positions
                valid, contacts = await _direct_state_validity(candidate)
                checked += 1
                if not valid:
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": "MoveIt reports a collision or invalid joint state",
                        "contacts": contacts,
                    }
        else:
            _movl_controller_frame()
            # The Dobot driver accepts raw controller user/tool coordinates;
            # the configured indices must have been checked against MoveIt's
            # base frame and end-effector/TCP before this path is checked.
            fk_name = moveit.get("compute_fk_service", "/compute_fk")
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = base_frame
            fk_request.fk_link_names = [end_effector]
            fk_request.robot_state = current_state
            fk_response = await _await_ros(
                backend.call_service(fk_name, GetPositionFK, fk_request), service_timeout
            )
            if getattr(fk_response.error_code, "val", None) != 1 or not fk_response.pose_stamped:
                raise HTTPException(status_code=409, detail="MoveIt could not determine the current end-effector pose")
            start_pose = fk_response.pose_stamped[0].pose
            start_position = [
                start_pose.position.x, start_pose.position.y, start_pose.position.z
            ]
            start_orientation = [
                start_pose.orientation.x, start_pose.orientation.y,
                start_pose.orientation.z, start_pose.orientation.w,
            ]
            try:
                samples = movl_path_samples(
                    start_position, start_orientation, target,
                    float(direct_cfg.get("cartesian_step_mm", 10.0)) / 1000.0,
                    math.radians(float(direct_cfg.get("angular_step_deg", 2.0))),
                    max_samples,
                )
            except ValueError as exc:
                raise HTTPException(status_code=422, detail=str(exc)) from exc

            try:
                max_ik_joint_step = math.radians(float(direct_cfg.get("max_joint_step_deg", 12.0)))
            except (TypeError, ValueError, OverflowError) as exc:
                raise HTTPException(status_code=503, detail="Invalid max_joint_step_deg configuration") from exc
            if not math.isfinite(max_ik_joint_step) or not (0 < max_ik_joint_step <= math.pi):
                raise HTTPException(status_code=503, detail="max_joint_step_deg must be greater than zero and at most 180")

            previous_ik_positions = dict(zip(joint_names, current_positions))
            seed_state = current_state
            for index, (position, orientation) in enumerate(samples, start=1):
                ik_request = GetPositionIK.Request()
                ik = ik_request.ik_request
                ik.group_name = group_name
                ik.robot_state = seed_state
                ik.avoid_collisions = True
                ik.ik_link_name = end_effector
                ik.pose_stamped = PoseStamped()
                ik.pose_stamped.header.frame_id = base_frame
                ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y, ik.pose_stamped.pose.position.z = position
                (
                    ik.pose_stamped.pose.orientation.x,
                    ik.pose_stamped.pose.orientation.y,
                    ik.pose_stamped.pose.orientation.z,
                    ik.pose_stamped.pose.orientation.w,
                ) = orientation
                timeout_sec = float(direct_cfg.get("ik_timeout_sec", 0.1))
                ik.timeout.sec = int(timeout_sec)
                ik.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1_000_000_000)
                ik_response = await _await_ros(
                    backend.call_service(
                        moveit.get("compute_ik_service", "/compute_ik"),
                        GetPositionIK, ik_request,
                    ),
                    service_timeout,
                )
                if getattr(ik_response.error_code, "val", None) != 1:
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": "MoveIt could not solve IK for an intermediate Cartesian pose",
                        "ik_error_code": getattr(ik_response.error_code, "val", None),
                        "contacts": [],
                    }
                candidate = ik_response.solution
                candidate.attached_collision_objects = copy.deepcopy(current_state.attached_collision_objects)
                try:
                    _validate_direct_joint_positions(
                        list(candidate.joint_state.name),
                        list(candidate.joint_state.position),
                        joint_limits,
                        "MovL IK solution",
                    )
                except ValueError as exc:
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": str(exc), "contacts": [],
                    }
                candidate_by_name = dict(zip(
                    candidate.joint_state.name, candidate.joint_state.position
                ))
                missing_step_names = [name for name in joint_names if name not in candidate_by_name]
                if missing_step_names:
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": "MovL IK solution is missing joints: " + ", ".join(missing_step_names),
                        "contacts": [],
                    }
                max_step = max(
                    abs(float(candidate_by_name[name]) - previous_ik_positions[name])
                    for name in joint_names
                )
                if max_step > max_ik_joint_step:
                    worst_joint = max(
                        joint_names,
                        key=lambda name: abs(float(candidate_by_name[name]) - previous_ik_positions[name]),
                    )
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": (
                            f"MovL IK branch jump: {worst_joint} changed by "
                            f"{math.degrees(max_step):.2f} degrees between Cartesian samples "
                            f"(limit {math.degrees(max_ik_joint_step):.2f} degrees)"
                        ),
                        "contacts": [],
                    }
                valid, contacts = await _direct_state_validity(candidate)
                checked += 1
                if not valid:
                    return {
                        "type": movement_type, "collision_free": False,
                        "samples_checked": checked, "failed_sample": index,
                        "reason": "MoveIt reports a collision or invalid IK state",
                        "contacts": contacts,
                    }
                previous_ik_positions = {
                    name: float(candidate_by_name[name]) for name in joint_names
                }
                seed_state = candidate

        return {
            "type": movement_type,
            "collision_free": True,
            "samples_checked": checked,
            "joint_names": joint_names,
            "_scene_fingerprint": scene_fingerprint,
            "_start_joint_positions_rad": current_positions,
            "message": (
                "All sampled robot states passed MoveIt collision/state validation. "
                "This is a discrete pre-check, not a safety-rated guarantee."
            ),
        }

    async def _execute_direct_movement_body(payload):
        robot_cfg = config.get("robot", {})
        if not robot_cfg.get("allow_commands", False) or not robot_cfg.get("allow_direct_movement", False):
            raise HTTPException(
                status_code=403,
                detail=(
                    "Direct movement requires both robot.allow_commands and "
                    "robot.allow_direct_movement in bridge safety configuration"
                ),
            )
        movement_type = str(payload.get("type", "")).lower()
        direct_cfg = config.get("direct_movement", {})
        if movement_type == "movl":
            try:
                controller_user, controller_tool = _movl_controller_frame()
            except HTTPException as exc:
                raise HTTPException(status_code=403, detail=exc.detail) from exc
        checked = await check_direct_movement_impl(payload)
        if not checked.get("collision_free"):
            raise HTTPException(
                status_code=409,
                detail={
                    "message": checked.get("reason", "Path validation failed"),
                    "collision_check": checked,
                },
            )

        # The path check can involve many MoveIt service round trips. Reject
        # execution if the robot start state or collision scene changed while
        # those samples were being checked.
        latest_scene = await get_scene()
        if _direct_scene_fingerprint(latest_scene) != checked.get("_scene_fingerprint"):
            raise HTTPException(
                status_code=409,
                detail="MoveIt collision scene changed during path validation; check the path again",
            )
        latest_state, latest_names, latest_positions, _ = await _direct_current_state(latest_scene)
        checked_names = list(checked.get("joint_names") or [])
        checked_start = list(checked.get("_start_joint_positions_rad") or [])
        if (
            latest_names != checked_names
            or len(latest_positions) != len(checked_start)
        ):
            raise HTTPException(
                status_code=409,
                detail="Robot joint set changed during path validation; check the path again",
            )
        try:
            start_tolerance = finite_vector(
                [config.get("moveit", {}).get("start_state_tolerance_rad", 0.03)],
                1, "start_state_tolerance_rad",
            )[0]
        except ValueError as exc:
            raise HTTPException(status_code=503, detail="Invalid start-state tolerance configuration") from exc
        if start_tolerance <= 0:
            raise HTTPException(status_code=503, detail="start_state_tolerance_rad must be greater than zero")
        max_start_delta = max(
            abs(float(current) - float(previous))
            for current, previous in zip(latest_positions, checked_start)
        )
        if max_start_delta > start_tolerance:
            raise HTTPException(
                status_code=409,
                detail=(
                    f"Robot moved during path validation (max joint delta {max_start_delta:.4f} rad "
                    f"> {start_tolerance:.4f} rad); check the path again"
                ),
            )
        current_valid, current_contacts = await _direct_state_validity(latest_state)
        if not current_valid:
            raise HTTPException(
                status_code=409,
                detail={
                    "message": "Robot current state became invalid after path validation",
                    "contacts": current_contacts,
                },
            )

        values = (
            movj_target(payload.get("target_joints_deg"))
            if movement_type == "movj"
            else movl_target(payload.get("target_pose_mm_deg"))
        )
        service_name = "MovJ" if movement_type == "movj" else "MovL"
        # Dobot parser maps mode=true to joint={...} and false to pose={...}.
        request_payload = {
            "mode": movement_type == "movj",
            "a": values[0], "b": values[1], "c": values[2],
            "d": values[3], "e": values[4], "f": values[5],
            "param_value": (
                ["user=" + str(controller_user),
                 "tool=" + str(controller_tool)]
                if movement_type == "movl" else []
            ),
        }
        _set_direct_motion_state(
            "dispatching",
            "Collision check passed; sending the direct command to the Dobot controller.",
            movement_type,
        )
        response = await _await_ros(
            backend.robot_service(service_name, request_payload),
            float(config.get("ros", {}).get("action_timeout_sec", 45.0)),
        )
        result = _dict(response)
        if int(result.get("res", -1)) != 0:
            raise HTTPException(
                status_code=409,
                detail={
                    "message": "Dobot rejected or could not confirm the direct movement command; direct movement is locked for manual recovery.",
                    "controller_response": result,
                },
            )
        try:
            accepted_mode = _parse_dobot_mode_value(result.get("robot_return"))
        except HTTPException:
            accepted_mode = None
        completion = await _wait_for_direct_motion_completion(
            list(checked.get("joint_names") or []),
            list(checked.get("_start_joint_positions_rad") or []),
            accepted_mode,
        )
        public_check = _public_direct_check(checked)
        backend.emit("robot.state", {
            "command": "direct-" + movement_type,
            "response": result,
            "collision_check": public_check,
        })
        return {
            "type": movement_type,
            "collision_check": public_check,
            "robot_response": result,
            "completion": completion,
        }

    async def execute_direct_movement_impl(payload):
        if direct_motion_state.get("phase") != "idle":
            raise HTTPException(
                status_code=409,
                detail={
                    "message": "A prior direct movement is still active or its completion is uncertain.",
                    "direct_motion": dict(direct_motion_state),
                },
            )
        if direct_motion_lock.locked():
            raise HTTPException(status_code=409, detail="Another direct movement request is already being processed")
        async with direct_motion_lock:
            if direct_motion_state.get("phase") != "idle":
                raise HTTPException(
                    status_code=409,
                    detail={
                        "message": "A prior direct movement is still active or its completion is uncertain.",
                        "direct_motion": dict(direct_motion_state),
                    },
                )
            command = str(payload.get("type", "")).lower()
            _set_direct_motion_state("checking", "Validating the requested path.", command)
            try:
                result = await _execute_direct_movement_body(payload)
            except BaseException as exc:
                phase = direct_motion_state.get("phase")
                if phase in ("dispatching", "awaiting_motion", "moving"):
                    detail = str(getattr(exc, "detail", exc))
                    _set_direct_motion_state(
                        "recovery_required",
                        "The command may have reached the controller, but completion was not confirmed. "
                        + (detail if isinstance(detail, str) else json.dumps(detail, default=str)),
                        command,
                    )
                else:
                    _set_direct_motion_state("idle", str(getattr(exc, "detail", exc)), command)
                raise
            _set_direct_motion_state(
                "idle",
                "Controller returned to idle and stable joint telemetry confirmed completion.",
                command,
            )
            return result

    async def direct_motion_status_impl():
        return dict(direct_motion_state)

    async def recover_direct_movement_impl(payload):
        if payload.get("confirmed_idle") is not True:
            raise HTTPException(
                status_code=422,
                detail="Explicit confirmed_idle=true is required after visually verifying the robot is stopped.",
            )
        if direct_motion_state.get("phase") != "recovery_required":
            return {"recovered": False, "direct_motion": dict(direct_motion_state)}
        if direct_motion_lock.locked():
            raise HTTPException(status_code=409, detail="Direct movement monitoring is still active")
        async with direct_motion_lock:
            if direct_motion_state.get("phase") != "recovery_required":
                return {"recovered": False, "direct_motion": dict(direct_motion_state)}
            try:
                scene = await get_scene()
                current_state, _, _, _ = await _direct_current_state(scene)
                valid, contacts = await _direct_state_validity(current_state)
                if not valid:
                    raise HTTPException(
                        status_code=409,
                        detail={"message": "Current robot state is invalid or in collision", "contacts": contacts},
                    )
            except HTTPException:
                raise
            _set_direct_motion_state(
                "idle",
                "Operator confirmed the robot was stopped; fresh controller idle state and MoveIt validity were verified.",
            )
            return {"recovered": True, "direct_motion": dict(direct_motion_state)}

    @app.get("/api/v1/robot/direct-movement/status")
    async def direct_motion_status():
        return await direct_motion_status_impl()

    @app.post("/api/v1/robot/direct-movement/recover")
    async def recover_direct_movement(payload: dict):
        return await recover_direct_movement_impl(payload)

    @app.post("/api/v1/robot/direct-movement/check")
    async def check_direct_movement(payload: dict):
        if direct_motion_state.get("phase") != "idle" or direct_motion_lock.locked():
            raise HTTPException(status_code=409, detail="Direct path checks are blocked while a move is active or requires recovery")
        return _public_direct_check(await check_direct_movement_impl(payload))

    @app.post("/api/v1/robot/direct-movement/execute")
    async def execute_direct_movement(payload: dict):
        return await execute_direct_movement_impl(payload)

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

    async def create_motion_plan_impl(payload: dict, goal_mutator=None):
        plan_id = "plan_" + uuid.uuid4().hex[:12]
        backend.plans[plan_id] = {
            "id": plan_id, "status": "PLANNING", "created_at": time.time(),
            "trajectory": None, "trajectory_start": None,
        }
        backend.emit("motion.plan", {"plan_id": plan_id, "status": "PLANNING"})
        try:
            planner_parameters = payload.get("planner_parameters", {})
            if not isinstance(planner_parameters, dict):
                raise ValueError("planner_parameters must be a JSON object")
            if len(planner_parameters) > 24:
                raise ValueError("planner_parameters may contain at most 24 entries")
            if planner_parameters:
                pipeline_id = payload.get("pipeline_id", "ompl") or "ompl"
                planner_id = payload.get("planner_id", "")
                group_name = payload.get("group_name", config.get("moveit", {}).get("group_name", "cr5_group"))
                if pipeline_id != "ompl":
                    raise ValueError("planner_parameters are currently supported for the OMPL pipeline only")
                if not planner_id:
                    raise ValueError("planner_id is required when planner_parameters are supplied")
                for key, value in planner_parameters.items():
                    if (
                        not isinstance(key, str) or not key
                        or len(key) > 64
                        or not all(char.isalnum() or char == "_" for char in key)
                    ):
                        raise ValueError(f"Invalid planner parameter name: {key!r}")
                    if isinstance(value, bool):
                        continue
                    if isinstance(value, (int, float)):
                        finite_vector([value], 1, f"planner_parameters.{key}")
                        continue
                    if isinstance(value, str) and len(value) <= 64:
                        continue
                    raise ValueError(
                        f"planner_parameters.{key} must be a finite number, boolean, or short string"
                    )
                await _await_ros(
                    backend.set_planner_parameters(
                        pipeline_id, group_name, planner_id, planner_parameters
                    ),
                    3.0,
                )

            goal = backend.make_move_group_goal(payload)
            if goal_mutator is not None:
                goal_mutator(goal)
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

    def _pick_place_scene_payload(payload: dict):
        obj = normalize_object(payload.get("object", {}))
        frame_id = str(payload.get("frame_id", obj["frame_id"]))
        shelf_spec = payload.get("shelf", "standard")
        platform = payload.get("platform", {"position_m": [0.45, 0.0, 0.0], "dimensions_m": [0.30, 0.24, 0.02]})
        clearance = float(payload.get("clearance_m", 0.012))
        level = payload.get("level", 0)
        scene_data = generate_shelf_scene(
            shelf_spec, frame_id=frame_id, level=level,
            clearance_m=clearance, object_dimensions_m=obj["dimensions_m"],
            platform=platform,
        )
        if not scene_data["fit"]["valid"]:
            raise ValueError("Object does not fit selected shelf bay: " + "; ".join(scene_data["fit"]["reasons"]))
        grasp = generate_grasp_candidates(
            obj,
            gripper_yaw_deg=payload.get("gripper_yaw_deg"),
            approach_angle_deg=payload.get("approach_angle_deg", 0.0),
            approach_distance_m=payload.get("approach_distance_m", 0.08),
            tcp_offset_m=payload.get("tcp_offset_m", [0.0, 0.0, 0.0]),
        )[0]
        waypoints = build_task_waypoints(
            obj, grasp, scene_data["place_surface_center_m"],
            lift_distance_m=payload.get("lift_distance_m", 0.10),
        )
        return obj, scene_data, grasp, waypoints, frame_id

    async def prepare_pick_place_scene(payload: dict):
        try:
            obj, scene_data, grasp, waypoints, frame_id = _pick_place_scene_payload(payload)
            frame_id = str(payload.get("frame_id", config.get("moveit", {}).get("base_frame", "dummy_link")))
            additions = []
            for item in scene_data["collision_objects"]:
                item = dict(item)
                item["id"] = "pick_place_" + item["id"]
                additions.append(collision_object(item))
            object_pose = obj["centroid_m"]
            object_orientation = quaternion_from_rpy(*obj["rpy_rad"])
            object_payload = {
                "id": "pick_place_object", "shape": obj["shape"], "frame_id": frame_id,
                "position_m": object_pose, "orientation_xyzw": object_orientation,
            }
            if obj["shape"] == "cylinder":
                object_payload["height_m"] = obj["dimensions_m"][2]
                object_payload["radius_m"] = max(obj["dimensions_m"][0:2]) / 2
            elif obj["shape"] == "sphere":
                object_payload["radius_m"] = obj["dimensions_m"][0] / 2
            else:
                object_payload["dimensions_m"] = obj["dimensions_m"]
            additions.append(collision_object(object_payload))
            current = await get_scene()
            diff = PlanningScene()
            diff.is_diff = True
            for existing in current.world.collision_objects:
                if existing.id.startswith("pick_place_"):
                    remove = CollisionObject()
                    remove.id = existing.id
                    remove.operation = CollisionObject.REMOVE
                    diff.world.collision_objects.append(remove)
            diff.world.collision_objects.extend(additions)
            await apply_scene(diff)
            return {
                "status": "SCENE_READY", "frame_id": frame_id,
                "scene_object_ids": [item.id for item in additions],
                "object": obj, "shelf": scene_data["shelf"],
                "fit": scene_data["fit"], "grasp": grasp,
                "waypoints": waypoints,
                "place_surface_center_m": scene_data["place_surface_center_m"],
                "gripper_nominal_opening_m": 0.04,
                "gripper_width_feasible": max(obj["dimensions_m"][0:2]) <= 0.04,
            }
        except Exception as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    async def plan_pick_place_task(payload: dict):
        prepared = await prepare_pick_place_scene(payload)
        if not prepared["gripper_width_feasible"] and payload.get("allow_infeasible_gripper", False) is not True:
            return {**prepared, "status": "BLOCKED_GRIPPER_WIDTH",
                    "error": "Object exceeds the configured PGEA 40 mm nominal opening; change end-effector or provide a validated grasp configuration."}
        obj = prepared["object"]
        waypoints = prepared["waypoints"]
        frame_id = prepared["frame_id"]
        link_name = config.get("moveit", {}).get("end_effector_link", "Link6")
        object_id = "pick_place_object"
        touch_links = payload.get("touch_links", ["Link6", "gripper_body", "left_finger", "right_finger"])
        if not isinstance(touch_links, list) or any(not isinstance(item, str) or not item for item in touch_links):
            raise HTTPException(status_code=422, detail="touch_links must be a list of link names")

        def allow_object_contact(goal):
            matrix = goal.planning_options.planning_scene_diff.allowed_collision_matrix
            names = [object_id] + touch_links
            matrix.entry_names = names
            for row in range(len(names)):
                entry = AllowedCollisionEntry()
                entry.enabled = [row != col and (row == 0 or col == 0) for col in range(len(names))]
                matrix.entry_values.append(entry)

        def attach_object(goal):
            scene_diff = goal.planning_options.planning_scene_diff
            remove = CollisionObject()
            remove.id = object_id
            remove.operation = CollisionObject.REMOVE
            scene_diff.world.collision_objects.append(remove)
            attached = AttachedCollisionObject()
            attached.link_name = link_name
            attached.touch_links = touch_links
            object_orientation = quaternion_from_rpy(*obj["rpy_rad"])
            relative = pose_relative_to(
                waypoints[1]["position_m"], waypoints[1]["orientation_xyzw"],
                obj["centroid_m"], object_orientation,
            )
            attached.object = collision_object({
                "id": object_id, "shape": obj["shape"], "frame_id": link_name,
                "position_m": relative["position_m"], "orientation_xyzw": relative["orientation_xyzw"],
                **({"dimensions_m": obj["dimensions_m"]} if obj["shape"] == "box" else
                   {"height_m": obj["dimensions_m"][2], "radius_m": max(obj["dimensions_m"][0:2]) / 2} if obj["shape"] == "cylinder" else
                   {"radius_m": obj["dimensions_m"][0] / 2}),
            })
            attached.object.header.frame_id = link_name
            scene_diff.robot_state.attached_collision_objects.append(attached)
            scene_diff.robot_state.is_diff = True

        def detach_and_place(goal):
            scene_diff = goal.planning_options.planning_scene_diff
            attached = AttachedCollisionObject()
            attached.link_name = link_name
            attached.object.id = object_id
            attached.object.operation = CollisionObject.REMOVE
            scene_diff.robot_state.attached_collision_objects.append(attached)
            scene_diff.robot_state.is_diff = True
            place = waypoints[4]
            object_orientation = quaternion_from_rpy(*obj["rpy_rad"])
            relative = pose_relative_to(
                waypoints[1]["position_m"], waypoints[1]["orientation_xyzw"],
                obj["centroid_m"], object_orientation,
            )
            world_pose = pose_compose(
                place["position_m"], place["orientation_xyzw"],
                relative["position_m"], relative["orientation_xyzw"],
            )
            object_payload = {
                "id": object_id, "shape": obj["shape"], "frame_id": frame_id,
                "position_m": world_pose["position_m"],
                "orientation_xyzw": world_pose["orientation_xyzw"],
            }
            if obj["shape"] == "box":
                object_payload["dimensions_m"] = obj["dimensions_m"]
            elif obj["shape"] == "cylinder":
                object_payload["height_m"] = obj["dimensions_m"][2]
                object_payload["radius_m"] = max(obj["dimensions_m"][0:2]) / 2
            else:
                object_payload["radius_m"] = obj["dimensions_m"][0] / 2
            scene_diff.world.collision_objects.append(collision_object(object_payload))

        plans = []
        start_state = None
        for index, waypoint in enumerate(waypoints):
            stage_payload = {
                "group_name": payload.get("group_name", config.get("moveit", {}).get("group_name", "cr5_group")),
                "target": {"type": "pose", "frame_id": frame_id, "link_name": link_name,
                    "position_m": waypoint["position_m"], "orientation_xyzw": waypoint["orientation_xyzw"],
                    "position_tolerance_m": payload.get("position_tolerance_m", 0.003),
                    "orientation_tolerance_rad": payload.get("orientation_tolerance_rad", 0.03)},
                "allowed_planning_time": payload.get("allowed_planning_time", 5.0),
                "max_velocity_scaling": payload.get("max_velocity_scaling", 0.15),
                "max_acceleration_scaling": payload.get("max_acceleration_scaling", 0.15),
                "pipeline_id": payload.get("pipeline_id", "ompl"),
                "planner_id": payload.get("planner_id", ""),
            }
            if start_state:
                stage_payload["start_state"] = start_state
            mutator = None
            if waypoint["name"] == "grasp":
                mutator = allow_object_contact
            elif waypoint["name"] in ("lift", "preplace", "place"):
                mutator = attach_object
            elif waypoint["name"] == "retreat":
                mutator = detach_and_place
            result = await create_motion_plan_impl(stage_payload, mutator)
            plans.append({"name": waypoint["name"], **result})
            if result.get("status") != "SUCCEEDED":
                return {**prepared, "status": "PARTIAL_FAILED", "plans": plans,
                        "failed_stage": waypoint["name"], "error_code": result.get("error_code")}
            trajectory = result.get("trajectory") or {}
            joint = trajectory.get("joint_trajectory") or {}
            points = joint.get("points") or []
            if not points:
                return {**prepared, "status": "PARTIAL_FAILED", "plans": plans,
                        "failed_stage": waypoint["name"], "error": "MoveIt returned no joint trajectory points"}
            start_state = {"joint_names": joint.get("joint_names", []),
                           "positions_rad": points[-1].get("positions", [])}
        return {**prepared, "status": "PLANNED", "plans": plans,
                "planning_time_total": sum(float(item.get("planning_time") or 0.0) for item in plans)}

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
            from datetime import datetime, timezone
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
        if method == "gripper.get_profiles":
            try:
                await ensure_active_gripper_profile()
            except HTTPException as exc:
                snapshot = gripper_profile_snapshot()
                snapshot["runtime_error"] = exc.detail
                return snapshot
            return gripper_profile_snapshot()

        if method == "gripper.save_profile":
            try:
                profile = profile_store.upsert(params.get("profile", {}))
            except ValueError as exc:
                raise HTTPException(status_code=422, detail=str(exc)) from exc
            runtime_error = None
            if profile["id"] == profile_store.active_profile_id:
                gripper_runtime["profile_id"] = None
                try:
                    await apply_gripper_profile(profile)
                except HTTPException as exc:
                    runtime_error = exc.detail
            snapshot = gripper_profile_snapshot()
            snapshot["saved_profile"] = profile
            snapshot["runtime_error"] = runtime_error
            return snapshot

        if method == "gripper.delete_profile":
            profile_id = str(params.get("profile_id") or "")
            try:
                profile_store.delete(profile_id)
            except KeyError as exc:
                raise HTTPException(status_code=404, detail=str(exc)) from exc
            except ValueError as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
            return gripper_profile_snapshot()

        if method == "gripper.apply_profile":
            require_gripper_commands()
            profile_id = str(params.get("profile_id") or "")
            profile = profile_store.get(profile_id)
            if profile is None:
                raise HTTPException(status_code=404, detail="Unknown gripper profile")
            previous_profile = profile_store.active()
            await apply_gripper_profile(profile)
            try:
                profile_store.set_active(profile_id)
            except ValueError as exc:
                try:
                    await apply_gripper_profile(previous_profile)
                except HTTPException:
                    gripper_runtime["profile_id"] = None
                raise HTTPException(status_code=500, detail=f"Profile was applied but could not be persisted: {exc}") from exc
            backend.emit("gripper.profile", {
                "active_profile_id": profile_id,
                "force_percent": profile["force_percent"],
                "speed_percent": profile["speed_percent"],
            })
            return gripper_profile_snapshot()

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
            await ensure_active_gripper_profile()

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
                    "direct_movement_enabled": bool(
                        config.get("robot", {}).get("allow_commands", False)
                        and config.get("robot", {}).get("allow_direct_movement", False)
                    ),
                    "movl_frame_confirmed": bool(
                        config.get("direct_movement", {}).get("controller_pose_frame_confirmed", False)
                        and isinstance(config.get("direct_movement", {}).get("controller_user"), int)
                        and not isinstance(config.get("direct_movement", {}).get("controller_user"), bool)
                        and config.get("direct_movement", {}).get("controller_user") >= 0
                        and isinstance(config.get("direct_movement", {}).get("controller_tool"), int)
                        and not isinstance(config.get("direct_movement", {}).get("controller_tool"), bool)
                        and config.get("direct_movement", {}).get("controller_tool") >= 0
                    ),
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
            pgea_urdf = None
            try:
                from ament_index_python.packages import get_package_share_directory
                pgea_path = Path(get_package_share_directory("dobot_rviz")) / "urdf" / "cr5_pgea.urdf"
                if pgea_path.is_file():
                    pgea_urdf = pgea_path.read_text(encoding="utf-8")
            except Exception:
                # PGEA geometry is an optional visual overlay; the active arm URDF
                # remains authoritative when this package/variant is unavailable.
                pass
            return {
                "urdf": urdf,
                "pgea_urdf": pgea_urdf,
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

        if method == "robot.direct.check":
            if direct_motion_state.get("phase") != "idle" or direct_motion_lock.locked():
                raise HTTPException(status_code=409, detail="Direct path checks are blocked while a move is active or requires recovery")
            return _public_direct_check(await check_direct_movement_impl(params))

        if method == "robot.direct.execute":
            return await execute_direct_movement_impl(params)

        if method == "robot.direct.status":
            return await direct_motion_status_impl()

        if method == "robot.direct.recover":
            return await recover_direct_movement_impl(params)

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

        if method == "pick_place.prepare":
            return await prepare_pick_place_scene(params)

        if method == "pick_place.plan":
            return await plan_pick_place_task(params)

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
