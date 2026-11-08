"""Regression tests for the public CR5A bridge HTTP APIs.

These tests use a fake ROS backend: they exercise FastAPI routing, validation,
serialization and response contracts without requiring a running ROS graph.
"""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters

from cr5a_ros_bridge.api import create_app
from cr5a_ros_bridge.ros_backend import BridgeNode


class FakeBackend:
    def __init__(self):
        self._topic_cache = {}

    def get_name(self):
        return "cr5a_ros_bridge"

    def graph(self):
        return {
            "nodes": [
                {"name": "robot_state_publisher", "namespace": "/",
                 "fully_qualified_name": "/robot_state_publisher"}
            ],
            "topics": [
                {"name": "/joint_states", "types": ["sensor_msgs/msg/JointState"]}
            ],
            "services": [
                {"name": "/robot_state_publisher/get_parameters",
                 "types": ["rcl_interfaces/srv/GetParameters"]}
            ],
        }

    def topic_snapshot(self, name):
        return self._topic_cache.get(name)

    def event_snapshot(self, name):
        return None

    def parameter_service_name(self, node_name, suffix):
        return f"{node_name}/{suffix}"

    def call_service(self, service_name, service_type, request):
        future = Future()
        if service_type is GetParameters:
            response = GetParameters.Response()
            value = ParameterValue()
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = "<robot name='cr5a'/>"
            response.values = [value for _ in request.names]
            future.set_result(response)
        else:
            future.set_exception(RuntimeError(f"Unexpected service: {service_name}"))
        return future

    def call_generic_service(self, service_name, service_type, request):
        return self.call_service(service_name, object, request)

    def publish_topic(self, topic, message_type, message):
        return {"topic": topic, "published": True}


@pytest.fixture()
def client():
    backend = FakeBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {
            "topic_publish_enabled": False,
            "generic_service_calls_enabled": False,
        },
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "tcp_topic": "/dobot_msgs_v4/msg/ToolVectorActual",
        },
        "moveit": {"allow_execution": False},
    }
    with TestClient(create_app(backend, config)) as test_client:
        yield test_client


def test_dobot_mesh_assets_are_served_from_package_share(client):
    response = client.get("/assets/dobot_rviz/meshes/cr5/J1.STL")

    assert response.status_code == 200
    assert len(response.content) > 1024


def test_pgea_mesh_assets_are_served_from_package_share(client):
    response = client.get("/assets/dobot_rviz/meshes/pgea/PGEA-100-40-W.STL")

    assert response.status_code == 200
    assert len(response.content) > 1024
def test_set_planner_parameters_builds_moveit_service_request():
    captured = {}
    backend = SimpleNamespace(
        config={"moveit": {"set_planner_params_service": "/set_planner_params"}},
        call_service=lambda name, service_type, request: captured.update({
            "name": name, "service_type": service_type, "request": request
        }),
    )

    BridgeNode.set_planner_parameters(
        backend, "ompl", "cr5_group", "RRTConnect", {"range": 0.2, "enabled": True}
    )

    request = captured["request"]
    assert captured["name"] == "/set_planner_params"
    assert request.pipeline_id == "ompl"
    assert request.group == "cr5_group"
    assert request.planner_config == "RRTConnect"
    assert request.replace is False
    assert request.params.keys == ["range", "enabled"]
    assert request.params.values == ["0.2", "true"]


def test_move_group_request_forwards_pipeline_and_planner_id():
    backend = SimpleNamespace(config={"moveit": {"group_name": "cr5_group"}})

    goal = BridgeNode.make_move_group_goal(backend, {
        "pipeline_id": "ompl",
        "planner_id": "RRTstar",
        "num_planning_attempts": 3,
        "allowed_planning_time": 4.0,
        "target": {
            "type": "joints",
            "joint_names": ["joint1"],
            "positions_rad": [0.25],
        },
    })

    assert goal.request.pipeline_id == "ompl"
    assert goal.request.planner_id == "RRTstar"
    assert goal.request.num_planning_attempts == 3
    assert goal.request.allowed_planning_time == 4.0


def test_system_resources_reports_safe_planning_thread_recommendation(client):
    response = client.get("/api/v1/system/resources")

    assert response.status_code == 200
    body = response.json()
    assert body["logical_cpus"] >= 1
    assert body["memory_total_bytes"] >= 0
    assert body["memory_available_bytes"] >= 0
    assert 1 <= body["recommended_planning_threads"] <= body["max_planning_threads"] <= 4
    assert isinstance(body["memory_pressure"], bool)


def test_health_returns_bridge_identity_and_timestamp(client):
    response = client.get("/health")

    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    assert body["ros_node"] == "cr5a_ros_bridge"
    assert isinstance(body["timestamp"], (int, float))


def test_ros_graph_exposes_nodes_topics_and_services(client):
    response = client.get("/api/v1/ros/graph")

    assert response.status_code == 200
    body = response.json()
    assert body["nodes"][0]["fully_qualified_name"] == "/robot_state_publisher"
    assert body["topics"][0]["name"] == "/joint_states"
    assert body["services"][0]["name"].endswith("/get_parameters")


def test_ros_topics_returns_topic_list(client):
    response = client.get("/api/v1/ros/topics")

    assert response.status_code == 200
    assert response.json()[0]["name"] == "/joint_states"


def test_parameter_get_returns_string_parameter(client):
    response = client.post(
        "/api/v1/ros/parameters/get",
        json={
            "node": "/robot_state_publisher",
            "names": ["robot_description"],
            "describe": False,
        },
    )

    assert response.status_code == 200
    assert response.json()["parameters"]["robot_description"] == "<robot name='cr5a'/>"


def test_parameter_get_requires_node(client):
    response = client.post("/api/v1/ros/parameters/get", json={"names": []})

    assert response.status_code == 422
    assert response.json()["detail"] == "node is required"


def test_generic_service_calls_are_disabled_by_default(client):
    response = client.post(
        "/api/v1/ros/services/call",
        json={"service": "/some_service", "type": "example/srv/Example", "request": {}},
    )

    assert response.status_code == 403
    assert "disabled" in response.json()["detail"]


def test_topic_publish_is_disabled_by_default(client):
    response = client.post(
        "/api/v1/ros/topics/publish",
        json={"topic": "/joint_states", "type": "std_msgs/msg/String", "message": {"data": "x"}},
    )

    assert response.status_code == 403
    assert "disabled" in response.json()["detail"]


def test_non_finite_json_request_is_rejected(client):
    response = client.post(
        "/api/v1/ros/topics/publish",
        data=b'{"topic":"/x","type":"std_msgs/msg/String","message":{"value":NaN}}',
        headers={"Content-Type": "application/json"},
    )

    assert response.status_code == 400
    assert "Invalid JSON request body" in response.json()["detail"]

class FakeGripperBackend(FakeBackend):
    def __init__(self, mode="5", allow_commands=True, initialized=True):
        super().__init__()
        from datetime import datetime, timezone
        self.mode = mode
        self.sent_gripper_goals = []
        self.parameter_updates = []
        self._topic_cache["/dobot_msgs_v4/msg/RobotStatus"] = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "data": {"is_connected": True, "is_enable": mode == "5"},
        }
        self.gripper_status = {
            "init": 1 if initialized else 0,
            "init_text": "Initialized" if initialized else "Not initialized",
            "grip": 1,
            "grip_text": "Position reached",
            "position": 0,
            "current": 0,
            "error": 0,
            "alarm": {"code": 0, "name": "No error", "remedy": ""},
        }

    def call_service(self, service_name, service_type, request):
        from pgea_gripper_interfaces.srv import GetStatus as PgeaGetStatus
        from std_srvs.srv import Trigger
        import json
        future = Future()
        if service_type is PgeaGetStatus:
            response = PgeaGetStatus.Response()
            response.success = True
            response.status_json = json.dumps(self.gripper_status)
            response.message = "fake status"
            future.set_result(response)
        elif service_type is Trigger:
            response = Trigger.Response()
            response.success = True
            response.message = "fake lifecycle completed"
            future.set_result(response)
        else:
            from rcl_interfaces.srv import SetParametersAtomically
            if service_type is SetParametersAtomically:
                response = SetParametersAtomically.Response()
                response.result.successful = True
                response.result.reason = ""
                self.parameter_updates.append({
                    parameter.name: parameter.value.integer_value
                    for parameter in request.parameters
                })
                future.set_result(response)
            else:
                future.set_exception(RuntimeError(f"Unexpected service: {service_name}"))
        return future

    def robot_service(self, name, payload=None):
        from types import SimpleNamespace
        future = Future()
        future.set_result(SimpleNamespace(robot_return=self.mode, res=0))
        return future

    def send_gripper_goal(self, position_m, max_effort=0.0):
        from control_msgs.action import GripperCommand
        self.sent_gripper_goals.append((position_m, max_effort))
        future = Future()
        result = GripperCommand.Result()
        result.position = position_m
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = True
        future.set_result(result)
        return future

    def emit(self, channel, data):
        pass

    def cancel_gripper_goal(self):
        future = Future()
        future.set_result(object())
        return future


def _gripper_ws_request(client, method, params=None):
    with client.websocket_connect("/ws/v1") as websocket:
        hello = websocket.receive_json()
        assert hello["op"] == "hello"
        websocket.send_json({
            "op": "request", "id": "gripper-test",
            "method": method, "params": params or {},
        })
        while True:
            response = websocket.receive_json()
            if response.get("op") == "response":
                return response


def test_gripper_status_is_available_through_websocket():
    backend = FakeGripperBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(client, "gripper.get_status")

    assert response["ok"] is True
    assert response["data"]["connected"] is True
    assert response["data"]["status"]["init"] == 1


def test_gripper_move_rejects_positions_outside_pgea_stroke():
    backend = FakeGripperBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.021, "max_effort": 0.0}
        )

    assert response["ok"] is False
    assert response["error"]["code"] == 422
    assert backend.sent_gripper_goals == []


def test_gripper_move_requires_zero_max_effort_without_calibration():
    backend = FakeGripperBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.01, "max_effort": 2.0}
        )

    assert response["ok"] is False
    assert response["error"]["code"] == 422
    assert backend.sent_gripper_goals == []


def test_gripper_move_requires_enabled_idle_robot_and_successful_action():
    backend = FakeGripperBackend(mode="4")
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        disabled = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.01, "max_effort": 0.0}
        )

    assert disabled["ok"] is False
    assert disabled["error"]["code"] == 409
    assert backend.sent_gripper_goals == []

    backend.mode = "5"
    with TestClient(create_app(backend, config)) as client:
        moved = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.01, "max_effort": 0.0}
        )

    assert moved["ok"] is True
    assert moved["data"]["reached_goal"] is True
    assert moved["data"]["status"] == "SUCCEEDED"
    assert backend.sent_gripper_goals == [(0.01, 0.0)]

def test_gripper_move_requires_initialization():
    backend = FakeGripperBackend(initialized=False)
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.01, "max_effort": 0.0}
        )

    assert response["ok"] is False
    assert response["error"]["code"] == 409
    assert "not initialized" in response["error"]["message"]
    assert backend.sent_gripper_goals == []


def test_gripper_commands_are_separately_safety_gated():
    backend = FakeGripperBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": False},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(
            client, "gripper.move", {"position_m": 0.01, "max_effort": 0.0}
        )

    assert response["ok"] is False
    assert response["error"]["code"] == 403
    assert backend.sent_gripper_goals == []


def test_gripper_initialize_uses_dedicated_trigger_service():
    backend = FakeGripperBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {"allow_commands": True},
    }
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(client, "gripper.initialize")

    assert response["ok"] is True
    assert response["data"]["success"] is True
    assert response["data"]["operation"] == "initialize"


def _profile_test_config(tmp_path):
    return {
        "auth": {"enabled": False},
        "ros": {"topic_publish_enabled": False, "generic_service_calls_enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {"allow_execution": False},
        "gripper": {
            "allow_commands": True,
            "profiles_file": str(tmp_path / "persistent" / "gripper_profiles.yaml"),
            "profiles": [
                {"id": "gentle", "name": "Gentle", "force_percent": 30, "speed_percent": 25},
                {"id": "balanced", "name": "Balanced", "force_percent": 60, "speed_percent": 50},
                {"id": "firm", "name": "Firm", "force_percent": 80, "speed_percent": 30},
            ],
        },
    }


def test_gripper_profiles_apply_and_persist_active_profile(tmp_path):
    backend = FakeGripperBackend()
    config = _profile_test_config(tmp_path)
    with TestClient(create_app(backend, config)) as client:
        initial = _gripper_ws_request(client, "gripper.get_profiles")
        applied = _gripper_ws_request(client, "gripper.apply_profile", {"profile_id": "gentle"})

    assert initial["ok"] is True
    assert initial["data"]["active_profile_id"] == "balanced"
    assert applied["ok"] is True
    assert applied["data"]["active_profile_id"] == "gentle"
    assert applied["data"]["runtime_synced"] is True
    assert backend.parameter_updates[-1] == {"force_percent": 30, "speed_percent": 25}
    import yaml
    saved = yaml.safe_load((tmp_path / "persistent" / "gripper_profiles.yaml").read_text())
    assert saved["active_profile_id"] == "gentle"


def test_gripper_profiles_can_be_created_and_invalid_values_are_rejected(tmp_path):
    backend = FakeGripperBackend()
    config = _profile_test_config(tmp_path)
    with TestClient(create_app(backend, config)) as client:
        created = _gripper_ws_request(client, "gripper.save_profile", {
            "profile": {"name": "Soft insert", "force_percent": 24, "speed_percent": 15}
        })
        invalid = _gripper_ws_request(client, "gripper.save_profile", {
            "profile": {"name": "Unsafe", "force_percent": 10, "speed_percent": 15}
        })

    assert created["ok"] is True
    profile = created["data"]["saved_profile"]
    assert profile["name"] == "Soft insert"
    assert 20 <= profile["force_percent"] <= 100
    assert invalid["ok"] is False
    assert invalid["error"]["code"] == 422


def test_gripper_profiles_cannot_delete_the_active_profile(tmp_path):
    backend = FakeGripperBackend()
    config = _profile_test_config(tmp_path)
    with TestClient(create_app(backend, config)) as client:
        response = _gripper_ws_request(client, "gripper.delete_profile", {"profile_id": "balanced"})

    assert response["ok"] is False
    assert response["error"]["code"] == 409

def test_direct_movement_execution_keeps_robot_command_safety_gate(client):
    response = client.post(
        "/api/v1/robot/direct-movement/execute",
        json={"type": "movj", "target_joints_deg": [0, 0, 0, 0, 0, 0]},
    )

    assert response.status_code == 403
    assert "allow_commands" in response.json()["detail"]


def test_direct_movement_check_rejects_unknown_move_type_before_ros_calls(client):
    response = client.post(
        "/api/v1/robot/direct-movement/check",
        json={"type": "movc", "target": []},
    )

    assert response.status_code == 422
    assert "movj" in response.json()["detail"]


class DirectMovementBackend(FakeBackend):
    def __init__(self):
        from datetime import datetime, timezone
        self._topic_cache = {
            "/dobot_msgs_v4/msg/RobotStatus": {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "data": {"is_connected": True, "is_enable": True},
            },
            "/joint_states": {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "data": {
                    "name": [f"joint{i}" for i in range(1, 7)],
                    "position": [0.0] * 6,
                },
            },
        }
        self.validity_states = []
        self.ik_poses = []
        self.robot_calls = []
        self.events = []
        self.valid = True
        self.scene_fetch_count = 0
        self.change_scene_after_first = False
        self.joint_reads = 0
        self.drift_after_first_joint_read = False

    def topic_snapshot(self, topic):
        snapshot = super().topic_snapshot(topic)
        if topic == "/joint_states":
            self.joint_reads += 1
            if self.drift_after_first_joint_read and self.joint_reads > 1 and snapshot:
                import copy
                snapshot = copy.deepcopy(snapshot)
                snapshot["data"]["position"][0] = 0.1
        return snapshot

    def _client(self, service_name, service_type):
        return object()

    def robot_service(self, name, payload=None):
        self.robot_calls.append((name, payload or {}))
        future = Future()
        if name == "RobotMode":
            future.set_result(SimpleNamespace(robot_return="5", res=0))
        elif name == "GetErrorID":
            future.set_result(SimpleNamespace(robot_return="[]", res=0))
        elif name == "MovJ":
            from dobot_msgs_v4.srv import MovJ
            response = MovJ.Response()
            response.robot_return = "0"
            response.res = 0
            future.set_result(response)
        elif name == "MovL":
            from dobot_msgs_v4.srv import MovL
            response = MovL.Response()
            response.robot_return = "0"
            response.res = 0
            future.set_result(response)
        else:
            future.set_exception(RuntimeError(f"Unexpected Dobot service: {name}"))
        return future

    def emit(self, channel, data):
        self.events.append((channel, data))

    def call_service(self, service_name, service_type, request):
        from moveit_msgs.srv import GetPlanningScene, GetStateValidity, GetPositionFK, GetPositionIK
        from moveit_msgs.msg import PlanningScene
        from geometry_msgs.msg import PoseStamped
        future = Future()
        if service_type is GetPlanningScene:
            from moveit_msgs.msg import CollisionObject
            self.scene_fetch_count += 1
            response = GetPlanningScene.Response()
            response.scene = PlanningScene()
            if self.change_scene_after_first and self.scene_fetch_count > 1:
                obstacle = CollisionObject()
                obstacle.id = "added-during-check"
                response.scene.world.collision_objects.append(obstacle)
            future.set_result(response)
        elif service_type is GetStateValidity:
            self.validity_states.append(list(request.robot_state.joint_state.position))
            response = GetStateValidity.Response()
            response.valid = self.valid
            future.set_result(response)
        elif service_type is GetPositionFK:
            response = GetPositionFK.Response()
            response.error_code.val = 1
            pose = PoseStamped()
            pose.pose.position.z = 0.4
            pose.pose.orientation.w = 1.0
            response.pose_stamped = [pose]
            future.set_result(response)
        elif service_type is GetPositionIK:
            request_pose = request.ik_request.pose_stamped
            self.ik_poses.append((
                request_pose.header.frame_id,
                request_pose.pose.position.x,
                request_pose.pose.position.y,
                request_pose.pose.position.z,
            ))
            response = GetPositionIK.Response()
            response.error_code.val = 1
            response.solution = request.ik_request.robot_state
            future.set_result(response)
        else:
            future.set_exception(RuntimeError(f"Unexpected MoveIt service: {service_name}"))
        return future


def test_direct_movj_collision_check_samples_start_and_target_states():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"service_timeout_sec": 3.0},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": False,
        },
        "moveit": {
            "group_name": "cr5_group",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
        },
        "direct_movement": {"joint_step_deg": 2.0, "max_samples": 200},
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/check",
            json={"type": "movj", "target_joints_deg": [4, 0, 0, 0, 0, 0]},
        )

    assert response.status_code == 200
    assert response.json()["collision_free"] is True
    assert response.json()["samples_checked"] == 2
    assert len(backend.validity_states) == 3  # current state plus two path samples
    assert backend.validity_states[0] == [0.0] * 6
    assert backend.validity_states[-1][0] == pytest.approx(4 * 3.141592653589793 / 180)



def test_direct_movj_execution_rechecks_path_then_calls_dobot_service():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"service_timeout_sec": 3.0, "action_timeout_sec": 45.0},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": True,
            "allow_direct_movement": True,
        },
        "moveit": {
            "group_name": "cr5_group",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
        },
        "direct_movement": {"joint_step_deg": 2.0, "max_samples": 200},
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/execute",
            json={"type": "movj", "target_joints_deg": [4, 0, 0, 0, 0, 0]},
        )

    assert response.status_code == 200
    assert response.json()["robot_response"]["res"] == 0
    movement_calls = [(name, payload) for name, payload in backend.robot_calls if name == "MovJ"]
    assert len(movement_calls) == 1
    payload = movement_calls[0][1]
    assert payload["mode"] is True
    assert [payload[key] for key in ("a", "b", "c", "d", "e", "f")] == [4, 0, 0, 0, 0, 0]


def test_direct_movj_execution_never_calls_controller_when_start_state_is_invalid():
    backend = DirectMovementBackend()
    backend.valid = False
    config = {
        "auth": {"enabled": False},
        "ros": {"service_timeout_sec": 3.0},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": True,
            "allow_direct_movement": True,
        },
        "moveit": {
            "group_name": "cr5_group",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
        },
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/execute",
            json={"type": "movj", "target_joints_deg": [4, 0, 0, 0, 0, 0]},
        )

    assert response.status_code == 409
    assert not any(name == "MovJ" for name, _ in backend.robot_calls)



def test_direct_movl_collision_check_interpolates_pose_and_solves_sequential_ik():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"service_timeout_sec": 3.0},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": False,
        },
        "moveit": {
            "group_name": "cr5_group",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "base_frame": "dummy_link",
            "end_effector_link": "Link6",
            "get_planning_scene_service": "/get_planning_scene",
        },
        "direct_movement": {
            "controller_pose_frame_confirmed": True,
            "controller_user": 0,
            "controller_tool": 0,
            "cartesian_step_mm": 10.0,
            "angular_step_deg": 10.0,
            "max_samples": 20,
        },
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/check",
            json={"type": "movl", "target_pose_mm_deg": [20, 0, 400, 0, 0, 0]},
        )

    assert response.status_code == 200
    assert response.json()["collision_free"] is True
    assert response.json()["samples_checked"] == 2
    assert backend.ik_poses == [
        ("dummy_link", pytest.approx(0.01), pytest.approx(0), pytest.approx(0.4)),
        ("dummy_link", pytest.approx(0.02), pytest.approx(0), pytest.approx(0.4)),
    ]
    assert len(backend.validity_states) == 3



def test_movl_check_requires_explicit_controller_user_and_tool_indices():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": False,
        },
        "moveit": {
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "base_frame": "dummy_link",
            "end_effector_link": "Link6",
        },
        "direct_movement": {"controller_pose_frame_confirmed": True},
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/check",
            json={"type": "movl", "target_pose_mm_deg": [20, 0, 400, 0, 0, 0]},
        )

    assert response.status_code == 409
    assert "controller_user/controller_tool" in response.json()["detail"]
    assert not backend.ik_poses


def test_direct_movl_execution_uses_verified_user_tool_indices():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "ros": {"service_timeout_sec": 3.0, "action_timeout_sec": 45.0},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": True,
            "allow_direct_movement": True,
        },
        "moveit": {
            "group_name": "cr5_group",
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "base_frame": "dummy_link",
            "end_effector_link": "Link6",
        },
        "direct_movement": {
            "controller_pose_frame_confirmed": True,
            "controller_user": 2,
            "controller_tool": 3,
            "cartesian_step_mm": 10.0,
            "angular_step_deg": 10.0,
            "max_samples": 20,
        },
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/execute",
            json={"type": "movl", "target_pose_mm_deg": [20, 0, 400, 0, 0, 0]},
        )

    assert response.status_code == 200
    movement_calls = [(name, payload) for name, payload in backend.robot_calls if name == "MovL"]
    assert len(movement_calls) == 1
    payload = movement_calls[0][1]
    assert payload["mode"] is False
    assert [payload[key] for key in ("a", "b", "c", "d", "e", "f")] == [20, 0, 400, 0, 0, 0]
    assert payload["param_value"] == ["user=2", "tool=3"]



def test_direct_movj_rejects_target_outside_configured_joint_limits():
    backend = DirectMovementBackend()
    config = {
        "auth": {"enabled": False},
        "robot": {"status_topic": "/dobot_msgs_v4/msg/RobotStatus"},
        "moveit": {
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
        },
        "direct_movement": {"joint_step_deg": 2.0, "max_samples": 200},
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/check",
            json={"type": "movj", "target_joints_deg": [0, 0, 180, 0, 0, 0]},
        )

    assert response.status_code == 422
    assert "joint3" in response.json()["detail"]
    assert backend.validity_states == [[0.0] * 6]


def test_direct_movj_execution_rejects_collision_scene_change_before_service_call():
    backend = DirectMovementBackend()
    backend.change_scene_after_first = True
    config = {
        "auth": {"enabled": False},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": True,
            "allow_direct_movement": True,
        },
        "moveit": {
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
        },
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/execute",
            json={"type": "movj", "target_joints_deg": [4, 0, 0, 0, 0, 0]},
        )

    assert response.status_code == 409
    assert "scene changed" in response.json()["detail"]
    assert not any(name == "MovJ" for name, _ in backend.robot_calls)


def test_direct_movj_execution_rejects_robot_drift_during_collision_check():
    backend = DirectMovementBackend()
    backend.drift_after_first_joint_read = True
    config = {
        "auth": {"enabled": False},
        "robot": {
            "status_topic": "/dobot_msgs_v4/msg/RobotStatus",
            "allow_commands": True,
            "allow_direct_movement": True,
        },
        "moveit": {
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "get_planning_scene_service": "/get_planning_scene",
            "start_state_tolerance_rad": 0.03,
        },
    }
    with TestClient(create_app(backend, config)) as test_client:
        response = test_client.post(
            "/api/v1/robot/direct-movement/execute",
            json={"type": "movj", "target_joints_deg": [4, 0, 0, 0, 0, 0]},
        )

    assert response.status_code == 409
    assert "moved during path validation" in response.json()["detail"]
    assert not any(name == "MovJ" for name, _ in backend.robot_calls)
