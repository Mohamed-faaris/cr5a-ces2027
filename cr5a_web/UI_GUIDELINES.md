# CR5A Web UI contracts

This file records frontend-specific assumptions that are easy to break or costly to rediscover. Repository-wide bridge, ROS, and safety contracts belong in the root `AGENTS.md`.

## Robot and operation status

- `src/lib/status.ts` is the canonical status classifier. Use `deriveRobotStatus()` instead of reimplementing controller-mode or error precedence in a page.
- Use `StatusBadge` from `src/components/status-badge.tsx`. The `status-*` color tokens live in `src/index.css`; when adding a status, update `RobotStatusKey`, `ROBOT_STATUS`, and the CSS mapping together.
- Current colors: error = red; idle = green; paused = amber; moving = blue; planning = purple; connecting/initializing = cyan; disabled/disconnected/unknown = zinc.
- Use `operationStatus()` for plan/execution outcomes. An operation result is not the robot's live status and must not overwrite it.
- Drag mode is reported as RobotMode `6` (Backdrive); `start-drag`/`stop-drag` are commands, not confirmation. Wait for RobotMode telemetry before treating the toggle as changed.
- Diagnostics uses `RobotStatus.is_enable` for its single Enable/Disable button. Do not infer the enabled state from a command response; wait for telemetry confirmation, and block enabling while controller errors are active. Gate Enable by RobotMode 3/4, Disable by mode 5, Drag by mode 5/6, Pause by mode 5/7, and Play by mode 10; do not treat `is_enable=false` during Running, Backdrive, or Paused as permission to enable.
- `robot.get_errors` returns `GetErrorID`; `robot.get_error_details` calls `GetError` with a language argument. Keep error IDs and descriptions distinct.

## Socket and live state

- `SocketProvider` starts the shared connection implemented by `src/lib/socket-client.ts`. It owns subscriptions, request correlation, reconnects, and initial snapshot/model/scene loading.
- Initial `robot.get_model` loading retries transient `robot_description`-unavailable responses while `robot_state_publisher` and MoveGroup initialize; preserve this bounded retry when changing initial state loading.
- Use `useRobotSocket().request()` for bridge requests and `useRobotStore` for shared live state. Do not create page-specific WebSockets or a second source of truth for telemetry.
- The socket client updates the store from `robot.*`, `motion.*`, and `moveit.scene` events. A request acknowledgement is not a substitute for the resulting telemetry/event.
- Dynamic topic subscriptions must go through `useRobotSocket().subscribeChannels()`; it reference-counts page subscriptions and restores them after reconnect. Topic snapshots are stored in `useRobotStore.topicMessages`. Do not open a page-specific WebSocket or subscribe directly from a page.
- Endpoint, environment overrides, and the distinction from raw rosbridge are documented in the root `AGENTS.md`.

## Motion UI

- Diagnostics drag controls map to `robot.command` values `start-drag`/`stop-drag`, backed by Dobot `StartDrag`/`StopDrag` services; they share the existing `allow_commands` gate. RobotMode 6 is Backdrive, not a separate bridge capability.
- `ToolVectorActual` TCP values are controller-reported and do not establish the MoveIt `model.base_frame` / `model.end_effector_link` pose; a manual TCP target must not silently assume these frames and tool offsets match.
- `motion.plan` responses include `error_code`; retain and show it on failure rather than replacing it with a generic toast.

- Keep live joint/TCP telemetry distinct from editable target values. Editing a target is not a robot command.
- Planning and execution are separate operations. Do not wire a Plan interaction to execution or present a successful plan as proof that motion occurred.
- Manual TCP targets are MoveIt poses for `model.end_effector_link` in `model.base_frame`; controller `ToolVectorActual` may use a different tool/user frame or TCP offset. Preserve the UI warning and `planErrorDetail()` mapping for MoveIt error codes.
- Joint-state positions are radians. Keep them in radians when passed to the URDF model; convert only at a display/input boundary.

## PGEA gripper UI

- `src/components/gripper/use-gripper-controller.ts` owns the shared `gripper.get_status` polling and gripper actions. `gripper-panels.tsx` provides the diagnostics and position-control panels. Do not create a second WebSocket or generic ROS service/topic-write path.
- Diagnostics embeds `GripperDiagnosticsCard` for connection, initialization, position, current, alarm, Initialize, and Full calibration. Visualizer embeds `GripperPositionCard` in its Controls column for the 0..20 mm/finger slider, Move, Open, Close, and Stop.
- `/gripper` remains a direct-link full view composing both cards; it is intentionally omitted from the primary app navigation. Keep the in-card links to this route when changing the split layout.
- The gripper's `position` feedback is 0..1000 per mille and maps to 0..20 mm **per finger**. Keep target inputs in millimetres at the UI boundary and send metres to `gripper.move`.
- Position motion requires a connected, enabled, idle arm (RobotMode 5), a connected initialized PGEA, no active arm error, and `gripper_commands_enabled`. Initialization/calibration require fresh connected arm telemetry but do not require the arm motors to be enabled.
- Stop cancels the active `GripperCommand` action; it is not a safety-rated protective stop. Keep physical E-stop messaging visible.
- Do not map Newtons into PGEA force-percent registers. Send `max_effort=0` until a calibrated force conversion exists; use the gripper node's configured force percentage.
- Full calibration moves through the full finger stroke; retain an explicit confirmation warning that the fingers and workpiece area must be clear.

## 3D viewer assumptions

- Reuse `src/components/robot/robot-viewport.tsx`; it is the existing URDF, MoveIt scene, and trajectory renderer.
- The bridge supplies `urdf`, optional `pgea_urdf`, `mesh_prefix`, and `mesh_base_url`. `pgea_urdf` is the clean PGEA visual model also used by MoveIt; it must not contain simulator plugins or ros2_control. The viewport may parse it as the rendered model while continuing to use `urdf`/MoveIt metadata for planning frames. The viewer replaces the URDF mesh prefix with the supplied mesh base URL before loading. Do not substitute workstation-specific mesh paths.
- The viewer displays geometry in metres and rotates its scene root by -90° around X. Preserve this coordinate conversion when changing model or scene transforms.
- Collision objects are drawn only when `header.frame_id` is empty or equals `model.base_frame`. The viewer does not perform TF transforms; objects in other frames remain in the scene response but are not rendered.
- The current scene renderer handles BOX, SPHERE, and CYLINDER primitives only. Other primitive types are not visualized.
- Trajectory preview temporarily changes the loaded URDF's joint values to sample end-effector positions, then restores the live joint values. Preserve that restoration when modifying the preview.
## Direct Dobot movement

- The visualizer Direct movement panel uses the shared `useRobotSocket().request()` path (`robot.direct.check` / `robot.direct.execute`); do not create another WebSocket. Keep this panel separate from planner profiles and from `motion.plan` / `motion.execute`.
- Move is only offered after a successful check, but the bridge repeats the check during execution. Require connected + enabled + RobotMode 5, fresh joint/status timestamps, no active controller errors, and the `direct_movement_enabled` capability.
- MovJ UI fields are controller degrees. The bridge converts to radians only for MoveIt collision checking. MovL fields are mm/degrees in MoveIt `base_frame` for `end_effector_link`; the bridge blocks MovL until the Dobot user/tool indices and frame alignment are explicitly configured.
- The bridge validates MovJ/IK joint values against `cr5_moveit/config/joint_limits.yaml`, re-fetches the scene before command, and rejects scene changes or robot start-state drift beyond `moveit.start_state_tolerance_rad`.
- The bridge serializes direct moves until controller idle and stable telemetry are confirmed; uncertain command outcomes and bridge restarts remain locked until explicit operator recovery after visually confirming the robot is stopped.
- MovL IK samples are checked for large joint-branch jumps using the configured maximum joint-step bound.
- A successful sampled validity check is not proof of collision-free execution: the Dobot controller interpolates its own path and moving obstacles can invalidate the check. Preserve this warning.
