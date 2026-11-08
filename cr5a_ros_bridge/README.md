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

The visualizer includes reusable planner profiles. Built-in profiles cover conservative and balanced OMPL/RRTConnect planning, RRTstar path-quality experiments, PRMstar roadmaps, Pilz PTP/LIN motions, CHOMP refinement, and a higher-throughput OMPL configuration. Custom profiles are stored in the current browser's local storage; import/export is available as JSON.

Each plan request accepts `pipeline_id`, `planner_id`, `allowed_planning_time`, `num_planning_attempts`, `max_velocity_scaling`, and `max_acceleration_scaling`. The bridge forwards the pipeline and planner IDs through `moveit_msgs/MotionPlanRequest`. For OMPL, a profile can include a `planner_parameters` object; the bridge applies these through MoveIt's `/set_planner_params` service before submitting the request. This service must be available from the running MoveGroup process. These service updates merge into the live planner configuration and persist until overwritten or MoveGroup is restarted; switching to a profile with no overrides does not clear them.

OMPL `max_planning_threads` is a planner-manager startup setting, not a per-request field. The UI reads CPU affinity, cgroup CPU/memory limits, available RAM, and swap pressure from the bridge process and suggests a bounded thread count (maximum four). The selected value can be copied as a YAML snippet for `cr5_moveit/config/ompl_planning.yaml`; restart MoveGroup after changing it. A profile selection does not silently change the currently running MoveGroup thread pool.

The bridge exposes `GET /api/v1/system/resources` for the visible resource snapshot used by the UI.

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
  "pipeline_id": "ompl",
  "planner_id": "RRTConnect",
  "allowed_planning_time": 5.0,
  "num_planning_attempts": 5,
  "max_velocity_scaling": 0.2,
  "max_acceleration_scaling": 0.2
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
  "pipeline_id": "ompl",
  "planner_id": "RRTConnect",
  "allowed_planning_time": 5.0,
  "num_planning_attempts": 5,
  "max_velocity_scaling": 0.2,
  "max_acceleration_scaling": 0.2
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

Available methods: `robot.get_snapshot`, `robot.get_model`, `robot.get_joint_limits`, `robot.get_errors`, `robot.get_error_details`, `robot.command`, `robot.clear_errors`, `moveit.get_scene`, `scene.add`, `scene.update`, `scene.remove`, `scene.clear`, `motion.plan`, `motion.get_plan`, `motion.execute`, `motion.cancel`, `motion.cancel_execution`, `robot.direct.check` and `robot.direct.execute`.

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


### PGEA force and speed profiles

The existing gripper controls include editable profiles for the PGEA's two percentage settings:

- `force_percent`: 20–100, written to PGEA register `0x0101`.
- `speed_percent`: 1–100, written to PGEA register `0x0104`.

Built-in starting profiles are `Gentle` (30% force / 25% speed), `Balanced` (60% / 50%) and `Firm` (80% / 30%). Treat these as starting points, not validated material-specific grasp settings. Test with a light, non-fragile object and a clear finger travel area before using a new combination.

User edits and the active profile are persisted by the bridge as YAML outside the repository. By default the file is `~/.config/cr5a_ros_bridge/gripper_profiles.yaml`. The path can be changed with `CR5A_GRIPPER_PROFILES_FILE` or the bridge config's `gripper.profiles_file`. When the bridge runs inside a container, mount the chosen host directory into the container; otherwise profiles can be lost when the container is recreated. The file is created with owner-only permissions.

Applying a profile atomically updates the live `/pgea_gripper` ROS parameters. The gripper node validates the ranges and rejects parameter changes while a position or force-grip action is active. The bridge applies the active profile again before the next standard position move if the node restarted or the runtime settings are out of sync.

These settings affect the standard `GripperCommand` position action used by Move, Open and Close. The separate `grip_until_force` action accepts force and speed on each goal and is not changed by the standard position profile. Force remains a PGEA percentage; it is not a calibrated value in Newtons.


The profile UI communicates through these `/ws/v1` request methods:

- `gripper.get_profiles` — list profiles, active profile, storage path and runtime sync status.
- `gripper.save_profile` — create or update a profile using `{profile:{id?,name,force_percent,speed_percent}}`.
- `gripper.apply_profile` — apply and persist `{profile_id}` as the active profile.
- `gripper.delete_profile` — remove `{profile_id}`; deleting the active profile is rejected.
## Collision-checked direct Dobot movement

The visualizer exposes a separate **Direct movement** panel for `MovJ` and
`MovL`; these controls are not planner profiles and do not create MoveIt plans.
The UI first calls `robot.direct.check` (REST:
`POST /api/v1/robot/direct-movement/check`). The explicit `Move` action calls
`robot.direct.execute` (REST:
`POST /api/v1/robot/direct-movement/execute`), which repeats the full check
immediately before sending the Dobot service request.

- **MovJ** input is six controller joint angles in degrees (`joint1` through
  `joint6`). The bridge converts these to radians for MoveIt, samples the
  joint-space segment, and checks the current state and each sample with
  `/check_state_validity`. The Dobot `MovJ` service receives the original
  six degree values (`mode=true`). Both the current and target joint values are checked against the position limits in `cr5_moveit/config/joint_limits.yaml`.
- **MovL** input is `[x,y,z,rx,ry,rz]` in mm/degrees. The bridge obtains the
  current end-effector pose with `/compute_fk`, linearly interpolates position,
  slerps orientation, solves sequential IK with `/compute_ik`, and validates
  every solved state with `/check_state_validity`. It then calls Dobot `MovL`
  with `mode=false`.

**MovL frame setup is intentionally locked by default.** The Dobot driver
forwards MovL coordinates in the controller's active user/tool coordinate
system, while the pre-check uses the MoveIt base frame and end-effector link.
Before using MovL, verify the active controller user/tool transforms and TCP
against `moveit.base_frame` / `moveit.end_effector_link`. Then set
`direct_movement.controller_pose_frame_confirmed: true` and set the explicit
`controller_user` and `controller_tool` indices in `bridge.yaml`. Those indices
are passed to the Dobot command as `user=N` and `tool=N`. Do not enable this
setting by guesswork.

Both check and execute require fresh joint/status telemetry, `RobotMode` 5
(enabled/idle), no active controller error IDs, an available MoveIt planning
scene, and a valid starting state. Execution additionally requires both
`robot.allow_commands: true` and the separate `robot.allow_direct_movement: true`;
both gates default to false in the hardware configuration. Before sending a
controller command, the bridge fetches the scene again, rejects any changed
collision geometry/allowed-collision matrix, verifies the robot has not drifted
beyond `moveit.start_state_tolerance_rad`, and validates the latest state again.
The simulation profile keeps direct movement disabled because its fake driver
does not implement the Dobot MovJ/MovL services.

The collision check is discrete sampling of MoveIt states, not validation of
the exact trajectory generated inside the Dobot controller. It cannot guarantee
physical collision avoidance, certify safety, or account for obstacles that
move after validation. Keep the workspace clear and retain the robot/controller
safety functions; validate on simulation and then with supervised, low-speed
hardware tests before normal operation.
