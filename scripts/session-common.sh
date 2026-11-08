#!/usr/bin/env bash
# Shared helpers for the CR5A tmux session scripts.
# This file is sourced by kill-session.sh and restart-session.sh.

cr5_validate_session_name() {
  local name="${1:-}"
  if [[ ! "$name" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    printf 'Invalid session name: %s\n' "$name" >&2
    printf 'Use the exact tmux session name, for example: cr5a-demo\n' >&2
    return 2
  fi
}

cr5_confirm_hardware_stop() {
  local name="$1"
  [[ "$name" == "cr5a-hardware" ]] || return 0

  if [[ ! -t 0 ]]; then
    printf 'Refusing to stop cr5a-hardware without an interactive safety confirmation.\n' >&2
    return 1
  fi

  local answer
  read -r -p 'This stops the physical driver and MoveIt. Confirm the robot is stationary and safe [y/N]: ' answer
  case "$answer" in
    y|Y|yes|YES) return 0 ;;
    *) printf 'Cancelled.\n' >&2; return 1 ;;
  esac
}

cr5_kill_session() {
  local name="$1"
  cr5_confirm_hardware_stop "$name" || return $?

  if tmux has-session -t "=$name" 2>/dev/null; then
    tmux kill-session -t "=$name"
    printf 'Stopped tmux session: %s\n' "$name"
  else
    printf 'tmux session is not running: %s\n' "$name"
  fi
}
