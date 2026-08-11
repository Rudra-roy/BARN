#!/usr/bin/env bash
#
# Run N trials of one world with a live telemetry recorder attached, to answer
# the question a result line cannot: WHAT is limiting the robot's speed?
#
# The evaluator reports only "success, 22.1 s". That is enough to rank a config
# and useless for deciding what to change. This attaches
# tools/record_recovery_trace.py, which samples /barn/cmd_desired,
# /barn/cmd_safe, /barn/pose and the shield's diagnostics at 20 Hz, so the loss
# can be attributed:
#
#   desired_v already low            -> the MPC / local planner is the limiter
#   desired_v high, safe_v scaled    -> barn_safety's shield is the limiter
#
# One trial answers it; three guard against reading a single unlucky run, and
# on a bimodal world (66) they may catch both the fast and the stalled mode.
#
# Usage:  instrument_trial.sh <world_idx> [n_trials] [tag]
#
# Results land in evaluation/tuning/world_<idx>/<tag>/ :
#   traces/trial_N.jsonl   20 Hz telemetry, one JSON object per sample
#   raw_results.txt        evaluator result lines, same format as a normal batch
#   logs/trial_N.log       full launch output
#   classical_mpc.yaml     the config that produced it

set -uo pipefail

WORLD="${1:?usage: instrument_trial.sh <world_idx> [n_trials] [tag]}"
NTRIALS="${2:-3}"
TAG="${3:-instrumented}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUTDIR="${SCRIPT_DIR}/world_${WORLD}/${TAG}"
OUT_FILE="${OUTDIR}/raw_results.txt"

mkdir -p "${OUTDIR}/traces" "${OUTDIR}/logs"
cp "${REPO_ROOT}/ros2_ws/src/barn_bringup/config/classical_mpc.yaml" "${OUTDIR}/" 2>/dev/null || true
: > "$OUT_FILE"

echo "[instrument] world=${WORLD} trials=${NTRIALS} tag=${TAG}"
echo "[instrument] out=${OUTDIR}"

for t in $(seq 1 "$NTRIALS"); do
  echo "[instrument] === world ${WORLD} trial ${t}/${NTRIALS} === $(date +%T)"

  # Start the recorder BEFORE the stack: it tolerates topics that do not exist
  # yet, and starting late would miss the launch transient where the robot is
  # deciding whether to move at all.
  python3 "${REPO_ROOT}/tools/record_recovery_trace.py" \
    -o "${OUTDIR}/traces/trial_${t}.jsonl" --rate 20 \
    > "${OUTDIR}/logs/recorder_${t}.log" 2>&1 &
  recorder_pid=$!
  sleep 2

  BARN_GUI=false BARN_RVIZ=false BARN_PLANNER_RVIZ=false \
  BARN_OUT_FILE="$OUT_FILE" BARN_RESULTS_DIR="$OUTDIR" \
    "${REPO_ROOT}/evaluation/scripts/run_single_world.sh" "$WORLD" classical_mpc "$t" \
    > "${OUTDIR}/logs/trial_${t}.log" 2>&1

  # run_single_world.sh's reaper matches on REPO_ROOT and so may have taken the
  # recorder with it already; either way every row is flushed as it is written,
  # so the trace is complete regardless of how it dies.
  kill "$recorder_pid" 2>/dev/null || true
  wait "$recorder_pid" 2>/dev/null || true

  rows=$(wc -l < "${OUTDIR}/traces/trial_${t}.jsonl" 2>/dev/null || echo 0)
  echo "[instrument]   result: $(tail -1 "$OUT_FILE" 2>/dev/null) | ${rows} trace rows"
done

echo "[instrument] done $(date +%T)"
