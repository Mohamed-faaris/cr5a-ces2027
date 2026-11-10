#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /root/cr5a_ws/install/setup.bash
PY_MINOR=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
VISION_SITE="${HOME}/.venvs/cr5a-vision/lib/python${PY_MINOR}/site-packages"
if [[ -d "${VISION_SITE}" ]]; then
  export PYTHONPATH="${VISION_SITE}:${PYTHONPATH:-}"
fi
exec ros2 launch cr5a_vision vision.launch.py "$@"
