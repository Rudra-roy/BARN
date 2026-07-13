#!/usr/bin/env bash
# Fail fast before a benchmark campaign can attach to stale ROS/Gazebo state.

set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "error: ROS 2 Jazzy is not the active distribution." >&2
  exit 1
fi

if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && \
      ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
  echo "error: FASTRTPS_DEFAULT_PROFILES_FILE does not name a readable file." >&2
  exit 1
fi

if pgrep -af '^gz sim( |$)' >/dev/null; then
  echo "error: a stale Gazebo Sim process is already running:" >&2
  pgrep -af '^gz sim( |$)' >&2
  echo "       Stop it before starting a campaign (pkill -TERM -x gz)." >&2
  exit 1
fi

echo "[preflight_barn_campaign] Jazzy environment is clean; no stale Gazebo process."
