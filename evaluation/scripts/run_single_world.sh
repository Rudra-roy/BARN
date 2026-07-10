#!/usr/bin/env bash
#
# Run ONE BARN world under the pinned evaluator with a selected navigation mode.
# The evaluator owns Gazebo, the Jackal spawn, collision/goal/timeout checks,
# and result logging; we only pass the world index and the navigation mode
# (via BARN_MODE, which the launch patch reads).
#
# Usage:  run_single_world.sh <world_idx> [mode] [trial]
#           mode  = classical | e2e_rl | hybrid   (default: classical)
# Requires the workspace overlay to be sourced first:
#           source ros2_ws/install/setup.bash
#
# Env overrides:
#   BARN_EVAL_LAUNCH_PKG   (default: jackal_helper)
#   BARN_EVAL_LAUNCH_FILE  (default: BARN_runner.launch.py)
#   BARN_RESULTS_DIR       (default: results/<mode>/adhoc)
#   BARN_OUT_FILE          (default: <results dir>/raw_results.txt)
#   BARN_GUI, BARN_RVIZ    (default: false)

set -euo pipefail

WORLD_IDX="${1:?usage: run_single_world.sh <world_idx> [mode] [trial]}"
MODE="${2:-classical}"
TRIAL="${3:-1}"

LAUNCH_PKG="${BARN_EVAL_LAUNCH_PKG:-jackal_helper}"
LAUNCH_FILE="${BARN_EVAL_LAUNCH_FILE:-BARN_runner.launch.py}"
RESULTS_DIR="${BARN_RESULTS_DIR:-results/${MODE}/adhoc}"
OUT_FILE="${BARN_OUT_FILE:-${RESULTS_DIR}/raw_results.txt}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: 'ros2' not found. Source ROS 2 and the workspace overlay first." >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT_FILE")"
export BARN_MODE="$MODE"

echo "[run_single_world] world=${WORLD_IDX} mode=${MODE} trial=${TRIAL} out=${OUT_FILE}"

ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
  world_idx:="$WORLD_IDX" \
  out_file:="$OUT_FILE" \
  gui:="${BARN_GUI:-false}" \
  rviz:="${BARN_RVIZ:-false}"
