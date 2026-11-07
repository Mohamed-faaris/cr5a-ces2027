# CR5A Web Console

Two-page React + TypeScript operations console for the Dobot CR5A.

- /diagnostics: RobotMode, status, normal Stop/Pause/Resume commands, error clearing and copyable telemetry.
- /visualizer: browser CR5 URDF viewer, MoveIt collision objects, TCP target inputs, joint sliders, plan preview and separately gated execution.
- Dynamic robot state, model descriptions, MoveIt scene snapshots, planning and commands use the application WebSocket at /ws/v1.
- HTTP is used only for static STL mesh assets. Vite proxies these files to the bridge during development.

## Run

Start the Dobot driver, robot_state_publisher, MoveIt and cr5a_ros_bridge first.

```bash
cd cr5a_web
pnpm install
pnpm dev
```

Open http://127.0.0.1:5173.

The bridge defaults to loopback and has robot commands, error clearing and physical MoveIt execution disabled. Enable those gates explicitly in cr5a_ros_bridge/config/bridge.yaml only after reviewing the safety and deployment setup. Planning remains separate from execution.

Copy .env.example to .env.local only when overriding the bridge URL or using an authentication token. Do not commit tokens.

## UI development

Read [UI_GUIDELINES.md](./UI_GUIDELINES.md) for frontend-specific status derivation, socket/store ownership, motion-state separation, and URDF/scene-frame assumptions.