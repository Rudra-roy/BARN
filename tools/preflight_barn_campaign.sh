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

# A leaked /clock bridge is worse than a leaked Gazebo: it keeps publishing
# SIMULATION TIME from a world that no longer exists, so every node in the next
# trial reads corrupted time. The symptom is not a crash -- it is the evaluator
# timing the reset teleport and ~30 s appearing on every trial, which reads as
# "the planner got slower". Refuse to start rather than record poisoned results.
if pgrep -af 'parameter_bridge /clock@' >/dev/null; then
  echo "error: a stale Gazebo /clock bridge is still running:" >&2
  pgrep -af 'parameter_bridge /clock@' >&2
  echo "       It will feed stale sim time to every trial. Kill it by PID first." >&2
  exit 1
fi

if pgrep -af '^gz sim( |$)' >/dev/null; then
  echo "error: a stale Gazebo Sim process is already running:" >&2
  pgrep -af '^gz sim( |$)' >&2
  echo "       Stop it before starting a campaign (pkill -TERM -x gz)." >&2
  exit 1
fi

echo "[preflight_barn_campaign] Jazzy environment is clean; no stale Gazebo process."
