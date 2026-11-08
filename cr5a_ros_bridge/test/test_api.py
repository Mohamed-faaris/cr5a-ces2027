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
