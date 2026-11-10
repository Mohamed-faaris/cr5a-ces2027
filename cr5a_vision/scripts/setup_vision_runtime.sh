#!/usr/bin/env bash
set -euo pipefail
ROOT="${HOME}/.venvs/cr5a-vision"
if [[ ! -x "${ROOT}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${ROOT}"
fi
"${ROOT}/bin/python" -m pip install --upgrade pip
"${ROOT}/bin/python" -m pip install -r "$(dirname "$(readlink -f "$0")")/../requirements-yolo.txt"
PY_MINOR=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
echo
printf 'Vision runtime installed in %s\n' "${ROOT}"
printf 'For ROS 2 Humble launches, export this path before ros2 launch:\n'
printf 'export PYTHONPATH="%s/lib/python%s/site-packages:${PYTHONPATH:-}"\n' "${ROOT}" "${PY_MINOR}"
