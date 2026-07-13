#!/usr/bin/env bash
#
# Run ONE BARN world with a selected evaluator algorithm.
# The evaluator owns Gazebo, the Jackal spawn, collision/goal/timeout checks,
# and result logging; we pass the world index and algo_type launch argument.
#
# Usage:  run_single_world.sh <world_idx> [algo_type] [trial]
#           algo_type = builtin or a future dispatcher name (default: builtin)
# Requires the workspace overlay to be sourced first:
#           source ros2_ws/install/setup.bash
#
# Env overrides:
#   BARN_EVAL_LAUNCH_PKG   (default: jackal_helper)
#   BARN_EVAL_LAUNCH_FILE  (default: BARN_runner.launch.py)
#   BARN_RESULTS_DIR       (default: results/<mode>/adhoc)
#   BARN_OUT_FILE          (default: <results dir>/raw_results.txt)
#   BARN_GUI, BARN_RVIZ, BARN_PLANNER_RVIZ (default: false)

set -euo pipefail

WORLD_IDX="${1:?usage: run_single_world.sh <world_idx> [algo_type] [trial]}"
ALGO_TYPE="${2:-builtin}"
TRIAL="${3:-1}"

LAUNCH_PKG="${BARN_EVAL_LAUNCH_PKG:-jackal_helper}"
LAUNCH_FILE="${BARN_EVAL_LAUNCH_FILE:-BARN_runner.launch.py}"
RESULTS_DIR="${BARN_RESULTS_DIR:-results/${ALGO_TYPE}/adhoc}"
OUT_FILE="${BARN_OUT_FILE:-${RESULTS_DIR}/raw_results.txt}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: 'ros2' not found. Source ROS 2 and the workspace overlay first." >&2
  exit 1
fi

"${REPO_ROOT}/tools/preflight_barn_campaign.sh"

mkdir -p "$(dirname "$OUT_FILE")"
echo "[run_single_world] world=${WORLD_IDX} algo=${ALGO_TYPE} trial=${TRIAL} out=${OUT_FILE}"

ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
  algo_type:="$ALGO_TYPE" \
  world_idx:="$WORLD_IDX" \
  out_file:="$OUT_FILE" \
  gui:="${BARN_GUI:-false}" \
  rviz:="${BARN_RVIZ:-false}" \
  planner_rviz:="${BARN_PLANNER_RVIZ:-false}"
