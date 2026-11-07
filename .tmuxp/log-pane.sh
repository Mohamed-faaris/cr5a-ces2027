#!/usr/bin/env bash
# Start a timestamped tmux pane log, then replace this process with the command.
# Usage: bash .tmuxp/log-pane.sh <log-name> <command> [args...]

set -euo pipefail

log_name="${1:?Usage: log-pane.sh <log-name> <command> [args...]}"
shift

pane_id="${TMUX_PANE:?This helper must run inside a tmux pane}"
session_created="$(tmux display-message -p -t "$pane_id" '#{session_created}')"
session_stamp="$(date -d "@${session_created}" +%Y%m%d-%H%M%S)"
log_dir="/root/cr5a_ws/logs/tmux/${session_stamp}"
mkdir -p "$log_dir"

log_file="${log_dir}/${log_name}.log"
tmux pipe-pane -o -t "$pane_id" "cat >> '${log_file}'"

printf 'Logging pane output to %s\n' "$log_file"
exec "$@"
