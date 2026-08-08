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

# Wall-clock cap on one trial.
#
# The evaluator's own 100 s limit is in SIMULATION time and only starts once the
# robot moves, so it cannot bound this. If Gazebo fails to come up -- a segfault
# on startup, a permission error in the Clearpath generator, a controller that
# never spawns -- `ros2 launch` waits forever and takes the whole 500-trial
# campaign with it, silently, at whichever trial it happened to be on.
#
# A healthy trial is about 45 s of wall clock and a full sim-timeout trial about
# 90 s, so 300 s is far outside the normal range: reaching it means the trial is
# broken, not slow. The trial is then abandoned WITHOUT a result line, which the
# metric reports surface as "Test on world_N not finished (k/10)" -- visible in
# the report rather than hidden in a log.
TRIAL_TIMEOUT_S="${BARN_TRIAL_TIMEOUT_S:-300}"

# Killing `ros2 launch` does NOT reap its children. Whatever it started keeps
# running and gets re-parented to init -- parameter_bridge, twist_mux, the
# controllers, Gazebo -- still holding topics on this ROS_DOMAIN_ID.
#
# The worst offender is the /clock bridge. A leaked
# `parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` keeps
# publishing simulation time from a world that no longer exists, so every node
# in every LATER trial reads corrupted time. Measured effect: the evaluator's
# timer starts during the reset teleport (logging the robot at the spawn origin
# rather than the start pose) and roughly 30 s is added to every trial, in a way
# that looks exactly like the navigation stack getting slower. One survivor
# poisoned two entire 12-trial batches before it was noticed, at 140% CPU.
#
# So reap by PID, matching on this workspace's own paths and the evaluator's
# signature remap, and never with a bare `pkill <node name>` -- that ignores
# ROS_DOMAIN_ID and would kill a colleague's nodes on the same machine.
reap_trial_processes() {
  local signal="$1" pid cmdline
  for pid in $(pgrep -f "jackal_helper|${REPO_ROOT}|^gz sim( |$)|/sensors/lidar2d_0/scan:=/front/scan|parameter_bridge /clock@" 2>/dev/null || true); do
    [ "$pid" = "$$" ] && continue
    [ -r "/proc/${pid}/cmdline" ] || continue
    cmdline=$(tr '\0' ' ' < "/proc/${pid}/cmdline")
    case "$cmdline" in
      *run_single_world.sh*|*run_barn2026_public_suite.sh*|*run_tuning_batch.sh*|*run_minor_worlds.sh*|*pgrep*) continue ;;
    esac
    kill "-${signal}" "$pid" 2>/dev/null || true
  done
}

set +e
timeout --signal=INT --kill-after=45 "$TRIAL_TIMEOUT_S" \
  ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
    algo_type:="$ALGO_TYPE" \
    world_idx:="$WORLD_IDX" \
    out_file:="$OUT_FILE" \
    gui:="${BARN_GUI:-false}" \
    rviz:="${BARN_RVIZ:-true}" \
    planner_rviz:="${BARN_PLANNER_RVIZ:-false}"
launch_status=$?
set -e

# Reap UNCONDITIONALLY, not only on the timeout path. A trial interrupted any
# other way -- Ctrl-C, a killed parent, a crash, a batch driver stopped
# mid-flight -- leaks the same children, and the next trial then fails for
# reasons that belong to this one. Cheap when there is nothing to kill.
reap_trial_processes TERM
sleep 4
reap_trial_processes KILL
sleep 1

if [ "$launch_status" -eq 124 ] || [ "$launch_status" -eq 137 ]; then
  echo "[run_single_world] TIMEOUT after ${TRIAL_TIMEOUT_S}s:" \
       "world=${WORLD_IDX} trial=${TRIAL} produced no result." >&2
elif [ "$launch_status" -ne 0 ]; then
  # A nonzero exit with a result already written is normal (a node reporting a
  # failed trial). Report it and carry on; the campaign is judged on the result
  # file, not on launch exit codes.
  echo "[run_single_world] launch exited ${launch_status} for" \
       "world=${WORLD_IDX} trial=${TRIAL}." >&2
fi

exit 0
