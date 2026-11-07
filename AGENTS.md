# AGENTS.md

- Inspect the relevant source, launch files, configs, and package manifests before changing anything. Do not guess APIs or package names.
- Run ROS work inside the Ubuntu 22.04 / ROS 2 Humble container; do not mix it with host Jazzy. Workspace: `/root/cr5a_ws`.
- Build from the workspace root with `colcon build --symlink-install --packages-up-to <package>`; source `install/setup.bash` after building.
- Keep changes small and consistent with existing package patterns. Avoid unrelated refactors and new dependencies.
- Test the affected package or workflow when practical. State clearly what was tested and what was not.
- Keep documentation aligned with the actual repository paths, commands, and launch behavior.
- Treat fake hardware, Gazebo, and the physical CR5A as separate workflows unless source code proves they are connected. Never assume a demo launch controls the real robot.
- Do not enable or issue physical robot motion commands unless explicitly requested. Preserve safety defaults and require the physical E-stop to remain available.
- Keep terminal output and logs concise. tmux pane logs are stored under `/root/cr5a_ws/logs/tmux/`.
- Do not commit generated `build/`, `install/`, or `log/` workspace artifacts.
