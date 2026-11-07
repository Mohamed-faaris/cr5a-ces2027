# AGENTS.md

Repository-specific contracts; verify these before changing related code.

- **Keep this file high-signal:** record only non-obvious repository-specific facts, hidden dependencies, interface contracts, and failure-prone assumptions. Do not pad it with generic programming or tooling advice; update it when the codebase establishes a new fact an agent would otherwise need to rediscover.

- **Robot selection:** `dobot_bringup_v4/launch/dobot_bringup_ros2.launch.py` uses 1-based `current_robot` to select `node_info[current_robot - 1]` in `config/param.json`. Overrides are case-sensitive: `IP_address` and `DOBOT_TYPE`.
- **MoveIt/bridge interface:** bridge defaults are group `cr5_group`, base `dummy_link`, TCP link `Link6`, planning action `/move_action`, execution action `/execute_trajectory`. Keep `cr5a_ros_bridge/config/bridge.yaml` aligned with the MoveIt SRDF/launch configuration.
- **Two WebSockets, different protocols:** the web console uses the bridge application protocol at `/ws/v1` on port `8000`. Raw rosbridge is the separate standard protocol on port `9090`; do not point the UI at it. Frontend overrides are `VITE_BRIDGE_WS_URL` and `VITE_BRIDGE_TOKEN`.
- **Safety gates are intentional:** `allow_execution`, `allow_commands`, `allow_error_clear`, `generic_service_calls_enabled`, and `topic_publish_enabled` default to `false`. The bridge does not expose the safety-rated E-stop. Do not weaken these defaults as a side effect of feature work.
- **Launches are not interchangeable:** `cr5_moveit demo.launch.py` uses fake hardware. `dobot_gazebo.launch.py` spawns the model and relays joint states, but does not connect MoveIt execution to Gazebo. Physical operation uses `dobot_bringup_ros2.launch.py` plus `cr5_moveit dobot_moveit.launch.py`; never run fake and physical stacks together in one ROS domain.
- **tmux logging is part of the profiles:** demo and hardware panes start through `.tmuxp/log-pane.sh`. Preserve that wrapper when editing pane commands; logs go to `/root/cr5a_ws/logs/tmux/<session-timestamp>/`.
- **Runtime assumptions:** this checkout is built in the Humble container at `/root/cr5a_ws`; the host's Jazzy environment is not the build environment.
