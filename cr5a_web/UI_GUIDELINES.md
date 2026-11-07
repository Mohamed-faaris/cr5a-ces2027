# CR5A Web UI contracts

This file records frontend-specific assumptions that are easy to break or costly to rediscover. Repository-wide bridge, ROS, and safety contracts belong in the root `AGENTS.md`.

## Robot and operation status

- `src/lib/status.ts` is the canonical status classifier. Use `deriveRobotStatus()` instead of reimplementing controller-mode or error precedence in a page.
- Use `StatusBadge` from `src/components/status-badge.tsx`. The `status-*` color tokens live in `src/index.css`; when adding a status, update `RobotStatusKey`, `ROBOT_STATUS`, and the CSS mapping together.
- Current colors: error = red; idle = green; paused = amber; moving = blue; planning = purple; connecting/initializing = cyan; disabled/disconnected/unknown = zinc.
- Use `operationStatus()` for plan/execution outcomes. An operation result is not the robot's live status and must not overwrite it.

## Socket and live state

- `SocketProvider` starts the shared connection implemented by `src/lib/socket-client.ts`. It owns subscriptions, request correlation, reconnects, and initial snapshot/model/scene loading.
- Use `useRobotSocket().request()` for bridge requests and `useRobotStore` for shared live state. Do not create page-specific WebSockets or a second source of truth for telemetry.
- The socket client updates the store from `robot.*`, `motion.*`, and `moveit.scene` events. A request acknowledgement is not a substitute for the resulting telemetry/event.
- Endpoint, environment overrides, and the distinction from raw rosbridge are documented in the root `AGENTS.md`.

## Motion UI

- Diagnostics drag controls map to `robot.command` values `start-drag`/`stop-drag`, backed by Dobot `StartDrag`/`StopDrag` services; they share the existing `allow_commands` gate. RobotMode 6 is Backdrive, not a separate bridge capability.
- `ToolVectorActual` TCP values are controller-reported and do not establish the MoveIt `model.base_frame` / `model.end_effector_link` pose; a manual TCP target must not silently assume these frames and tool offsets match.
- `motion.plan` responses include `error_code`; retain and show it on failure rather than replacing it with a generic toast.

- Keep live joint/TCP telemetry distinct from editable target values. Editing a target is not a robot command.
- Planning and execution are separate operations. Do not wire a Plan interaction to execution or present a successful plan as proof that motion occurred.
- Joint-state positions are radians. Keep them in radians when passed to the URDF model; convert only at a display/input boundary.

## 3D viewer assumptions

- Reuse `src/components/robot/robot-viewport.tsx`; it is the existing URDF, MoveIt scene, and trajectory renderer.
- The bridge supplies `urdf`, `mesh_prefix`, and `mesh_base_url`. The viewer replaces the URDF mesh prefix with the supplied mesh base URL before loading. Do not substitute workstation-specific mesh paths.
- The viewer displays geometry in metres and rotates its scene root by -90° around X. Preserve this coordinate conversion when changing model or scene transforms.
- Collision objects are drawn only when `header.frame_id` is empty or equals `model.base_frame`. The viewer does not perform TF transforms; objects in other frames remain in the scene response but are not rendered.
- The current scene renderer handles BOX, SPHERE, and CYLINDER primitives only. Other primitive types are not visualized.
- Trajectory preview temporarily changes the loaded URDF's joint values to sample end-effector positions, then restores the live joint values. Preserve that restoration when modifying the preview.