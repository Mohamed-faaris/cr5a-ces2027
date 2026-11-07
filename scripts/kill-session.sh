#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=session-common.sh
source "$SCRIPT_DIR/session-common.sh"

if [[ $# -ne 1 ]]; then
  printf 'Usage: bash scripts/kill-session.sh <session-name>\n' >&2
  printf 'Example: bash scripts/kill-session.sh cr5a-demo\n' >&2
  exit 2
fi

cr5_validate_session_name "$1"
command -v tmux >/dev/null 2>&1 || {
  printf 'tmux is not installed or not on PATH.\n' >&2
  exit 127
}

cr5_kill_session "$1"
