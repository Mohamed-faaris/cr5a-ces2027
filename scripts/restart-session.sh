#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=session-common.sh
source "$SCRIPT_DIR/session-common.sh"

if [[ $# -ne 1 ]]; then
  printf 'Usage: bash scripts/restart-session.sh <session-name>\n' >&2
  printf 'Example: bash scripts/restart-session.sh cr5a-demo\n' >&2
  exit 2
fi

session_name="$1"
cr5_validate_session_name "$session_name"

# Killing the session that owns this script would terminate the restart before
# tmuxp can recreate it. Require an external terminal in that case.
if [[ -n "${TMUX:-}" ]]; then
  current_session="$(tmux display-message -p '#S' 2>/dev/null || true)"
  if [[ "$current_session" == "$session_name" ]]; then
    printf 'Run restart-session.sh from a terminal outside tmux session %s.\n' "$session_name" >&2
    exit 1
  fi
fi

command -v tmux >/dev/null 2>&1 || {
  printf 'tmux is not installed or not on PATH.\n' >&2
  exit 127
}
command -v tmuxp >/dev/null 2>&1 || {
  printf 'tmuxp is not installed or not on PATH.\n' >&2
  exit 127
}

profile="$REPO_ROOT/.tmuxp/$session_name.yaml"
if [[ ! -f "$profile" ]]; then
  printf 'No tmuxp profile for session: %s\n' "$session_name" >&2
  printf 'Expected profile: %s\n' "$profile" >&2
  exit 2
fi

# The session name is the profile filename and must match its declared name.
if ! grep -Fqx "session_name: $session_name" "$profile"; then
  printf 'Profile session_name does not match: %s\n' "$profile" >&2
  exit 2
fi

# Profiles are authored to start from the ROS workspace root.
WORKSPACE_ROOT="$(cd -- "$REPO_ROOT/../.." && pwd)"
cd "$WORKSPACE_ROOT"

cr5_kill_session "$session_name"
exec tmuxp load "$profile"
