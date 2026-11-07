# tmux session scripts

Run these scripts **inside the ROS 2 Humble container**. They use the session
name to select the matching `.tmuxp/<session-name>.yaml` profile.

From the repository root:

```bash
bash scripts/kill-session.sh cr5a-demo
bash scripts/restart-session.sh cr5a-demo
```

Known profiles:

- `cr5a-demo` — MoveIt fake-hardware demo, simulation bridge, frontend, and monitors.
- `cr5a-hardware` — physical Dobot driver, MoveIt, bridge, and monitor.

Restart stops the existing session when it is running, then loads the matching
tmuxp profile. If the session is not running, it simply loads that profile.
The profile filename and its `session_name` field must match the supplied name.

**Hardware safety:** stopping or restarting `cr5a-hardware` terminates the
physical driver and MoveIt processes. The scripts require interactive
confirmation that the robot is stationary and safe before doing so.

The scripts are intentionally invoked with `bash`, so they work even when
Git has checked them out without executable file permissions.
