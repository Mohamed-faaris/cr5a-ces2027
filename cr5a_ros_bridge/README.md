# CR5A ROS Bridge

Headless REST + WebSocket gateway for the ROS 2 Humble Dobot CR5A and MoveIt 2 stack in this repository.

- REST API: robot status/commands, ROS graph, topics, parameters, planning scenes and motion plans.
- Application WebSocket: `/ws/v1` for live robot, scene and motion events.
- Optional raw rosbridge: `ws://127.0.0.1:9090` for the standard rosbridge JSON protocol.
- No UI is included.

## Requirements

Use the same ROS 2 Humble environment as the CR5A driver and MoveIt packages. Build this package in the workspace that contains `dobot_msgs_v4`, `cr5_moveit` and the official driver packages.

Install the bridge runtime dependencies:

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite python3-yaml python3-fastapi python3-uvicorn python3-wsproto
```

MoveIt 2, `dobot_msgs_v4`, the CR5 description and the Dobot driver must already be built and sourced in the same ROS environment.

### Python 3.10 WebSocket backend

The API explicitly selects Uvicorn's `wsproto` backend. This avoids older
`websockets` releases that pass the removed `loop=` argument to
`asyncio.Lock` on Python 3.10. Install `python3-wsproto` as shown above.
Do not work around this by monkey-patching `asyncio` or editing files under
`/usr/lib/python3/dist-packages`.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -i -y
colcon build --packages-up-to cr5a_ros_bridge
source install/setup.bash
```

## Launch

Start the Dobot driver and MoveIt using your existing bringup first. Then:

```bash
ros2 launch cr5a_ros_bridge bridge.launch.py
```

The launch starts the custom API, `rosapi`, and the standard rosbridge WebSocket server. To skip raw rosbridge:

```bash
ros2 launch cr5a_ros_bridge bridge.launch.py start_rosbridge:=false
```

Defaults are loopback-only:

- REST: `http://127.0.0.1:8000`
- Application WSS (plain ws on localhost): `ws://127.0.0.1:8000/ws/v1`
- Raw rosbridge: `ws://127.0.0.1:9090`

The application WSS route is WebSocket-capable, but TLS is not terminated by this package. For remote access, put the API behind an authenticated TLS reverse proxy and restrict the ROS domain/network with firewall rules. Do not change the bind address to `0.0.0.0` without that protection.

## REST endpoints

### Health and ROS graph

- `GET /health`
- `GET /api/v1/ros/graph`
- `GET /api/v1/ros/topics`
- `GET /api/v1/ros/topics/latest`
- `GET /api/v1/ros/topics/{topic_name}/latest`
- `POST /api/v1/ros/topics/publish`
- `POST /api/v1/ros/services/call`

Generic service calls and topic publishing are disabled by default. Keep these safety gates disabled unless the API is protected and the effects of the requested ROS operation are understood.

Topic publish body:

```json
{
  "topic": "/example_topic",
  "type": "std_msgs/msg/String",
  "message": {"data": "hello"}
}
```

Generic service body:

```json
{
  "service": "/dobot_bringup_ros2/srv/RobotMode",
  "type": "dobot_msgs_v4/srv/RobotMode",
  "request": {}
}
```

### Parameters

- `GET /api/v1/ros/parameters` — enumerate parameters on discoverable nodes.
- `POST /api/v1/ros/parameters/get` — body: `{"node":"/move_group","names":["planning_pipelines"],"describe":true}`.
- `POST /api/v1/ros/parameters/describe` — return parameter descriptors for selected or all parameters on one node.
- `POST /api/v1/ros/parameters/set` — body: `{"node":"/move_group","parameters":{"some_parameter":true}}`; uses atomic set.
- `GET /api/v1/ros/parameters/dump` — returns a YAML document in JSON.
- `POST /api/v1/ros/parameters/load` — accept a YAML document or a `node` + `parameters` mapping. Updates are atomic per node, not globally atomic.

Nodes that do not expose parameter services are returned with an `unavailable` reason. Parameter changes can be rejected by the target node.

### Robot

- `GET /api/v1/robot/status`
- `GET /api/v1/robot/errors`
- `POST /api/v1/robot/errors/clear`
- `POST /api/v1/robot/commands/enable`
- `POST /api/v1/robot/commands/disable`
- `POST /api/v1/robot/commands/power-on`
- `POST /api/v1/robot/commands/pause`
- `POST /api/v1/robot/commands/resume`
- `POST /api/v1/robot/commands/stop`
- `POST /api/v1/robot/commands/reset`

Emergency-stop is deliberately not exposed as an ordinary HTTP command. The robot's safety-rated physical emergency-stop circuit remains independent.

### MoveIt scene

- `GET /api/v1/moveit/scene`
- `GET /api/v1/moveit/scene/objects`
- `POST /api/v1/moveit/scene/objects`
- `PUT /api/v1/moveit/scene/objects/{id}`
- `DELETE /api/v1/moveit/scene/objects/{id}`
- `POST /api/v1/moveit/scene/objects/{id}/attach`
- `POST /api/v1/moveit/scene/objects/{id}/detach`
- `POST /api/v1/moveit/scene/clear`

Supported primitive geometry: box, sphere and cylinder. Mesh insertion is not implemented in this first version.

Example box:

```json
{
  "id": "fixture_01",
  "frame_id": "dummy_link",
  "shape": "box",
  "dimensions_m": [0.10, 0.10, 0.20],
  "position_m": [0.40, 0.10, 0.20],
  "orientation_xyzw": [0, 0, 0, 1]
}
```

Attach requires an explicit `link_frame_pose` containing the object's pose relative to the attachment link; the API does not guess this transform. The initial implementation supports one primitive per attached object. Detach requires a new world pose and shape description in the request body. The caller must provide a correct world-frame pose.

### Motion planning

- `POST /api/v1/motion-plans` — sends a MoveIt `/move_action` goal with `plan_only=true`.
- `GET /api/v1/motion-plans`
- `GET /api/v1/motion-plans/{id}`
- `GET /api/v1/motion-plans/{id}/trajectory`
- `POST /api/v1/motion-plans/{id}/execute`
- `POST /api/v1/motion-plans/{id}/cancel`
- `POST /api/v1/motion-plans/{id}/cancel-execution`

Plan request supports a joint target:

```json
{
  "group_name": "cr5_group",
  "target": {
    "type": "joints",
    "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "positions_rad": [0.0, 0.4, -1.0, 0.0, 1.2, 0.0]
  },
  "allowed_planning_time": 5.0,
  "max_velocity_scaling": 0.2,
  "max_acceleration_scaling": 0.2,
  "planner_id": ""
}
```

Or a pose target:

```json
{
  "group_name": "cr5_group",
  "target": {
    "type": "pose",
    "frame_id": "dummy_link",
    "link_name": "Link6",
    "position_m": [0.40, 0.10, 0.30],
    "orientation_xyzw": [0, 0, 0, 1],
    "position_tolerance_m": 0.001,
    "orientation_tolerance_rad": 0.01
  },
  "allowed_planning_time": 5.0,
  "max_velocity_scaling": 0.2,
  "max_acceleration_scaling": 0.2,
  "planner_id": ""
}
```

Planning does not move the robot. Successful trajectories are published to `/display_planned_path` and stored in memory under a plan ID. Execution is a separate, explicit request through MoveIt's `/execute_trajectory` action. The API currently requires the latest Dobot status sample to report connected and enabled before execution. Plan storage is in-memory and is cleared when the bridge restarts.

## WebSocket events

Connect to `ws://127.0.0.1:8000/ws/v1` and send:

```json
{
  "op": "subscribe",
  "channels": [
    "robot.status",
    "robot.mode",
    "robot.joints",
    "robot.tcp",
    "robot.errors",
    "robot.state",
    "moveit.scene",
    "motion.plan",
    "motion.execution",
    "topic:/tf"
  ]
}
```

Send `{"op":"unsubscribe","channels":["robot.joints"]}` to stop receiving a channel. Send `{"op":"ping"}` for a pong response. Arbitrary topic subscription is available with `topic:/fully/qualified/topic`; the topic type must be discoverable or be present in auto-subscribe configuration.

The application WebSocket also supports request/response RPC. Each request has a caller-generated ID and receives a matching response:

```json
{"op":"request","id":"req_001","method":"robot.get_snapshot","params":{}}
```

Available methods: `robot.get_snapshot`, `robot.get_model`, `robot.get_joint_limits`, `robot.get_errors`, `robot.get_error_details`, `robot.command`, `robot.clear_errors`, `moveit.get_scene`, `scene.add`, `scene.update`, `scene.remove`, `scene.clear`, `motion.plan`, `motion.get_plan`, `motion.execute`, `motion.cancel` and `motion.cancel_execution`.

Mutating requests use the same safety gates as their REST equivalents. CR5 mesh assets are served under `/assets/dobot_rviz/meshes/`; the URDF, scene, live state, planning and commands are requested through the application WebSocket.

## Authentication

Authentication is disabled by default because both HTTP and WebSocket listeners bind to loopback. To enable a bearer token:

1. Set `auth.enabled: true` in `config/bridge.yaml`.
2. Export `CR5A_BRIDGE_TOKEN` (or the configured `token_env`).
3. Send `Authorization: Bearer <token>` on REST requests.
4. Connect to WSS with `?token=<token>`.

The token must be injected through the environment or deployment secret manager; never commit it. For production, terminate TLS at a reverse proxy and use a stronger identity/authorization layer with separate observer/operator/admin roles.

## First-run validation

1. Confirm `/dobot_bringup_ros2/srv/RobotMode` and `/dobot_bringup_ros2/srv/GetErrorID` exist.
2. Confirm the configured status and TCP topics exist and have the message types listed in `config/bridge.yaml`.
3. Confirm MoveIt exposes `/move_action`, `/execute_trajectory`, `/get_planning_scene` and `/apply_planning_scene`.
4. Test graph, parameter reads, robot status and scene reads first.
5. Add a harmless collision object in a test scene and verify it appears in MoveIt.
6. Request a plan with a conservative target and inspect the trajectory. Do not execute until the planning scene, start state, collision environment, controller and physical workspace have been verified.

This package has not yet been colcon-built or tested against a live CR5A. Treat all motion endpoints as unvalidated until the first-run checklist and supervised hardware tests have passed.

## Execute-time state validation

Before allowing an execution request, the bridge compares the current `/joint_states`
positions against the stored plan's start state for every planned joint. It rejects
missing or stale telemetry and rejects plans whose largest joint delta exceeds
`moveit.start_state_tolerance_rad` (default `0.03` rad). The telemetry age limit is
`moveit.max_joint_state_age_sec` (default `1.0` second). These checks apply to both
the REST compatibility route and the WebSocket request method.
