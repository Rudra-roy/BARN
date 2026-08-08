#!/usr/bin/env bash
#
# Run ONE world, N trials, headless, capturing everything needed to decide
# whether a parameter change helped: the evaluator result lines, the full node
# log of every trial, and a snapshot of the config that produced them.
#
# This is a TUNING harness, not a benchmark. It deliberately runs a single
# world so a change can be judged in ~10 minutes instead of an hour, and it
# snapshots the config next to the results because a tuning number without the
# parameters that produced it is worthless three iterations later.
#
# Usage:  run_tuning_batch.sh <world_idx> <n_trials> <tag>
#
# Results land in evaluation/tuning/world_<idx>/<tag>/ :
#   raw_results.txt      evaluator lines: world success collided timeout AT score
#   classical_mpc.yaml   the exact config used
#   logs/trial_N.log     full launch output (recovery events, MPC warnings)
#   summary.txt          computed after the batch

set -uo pipefail

WORLD="${1:?usage: run_tuning_batch.sh <world_idx> <n_trials> <tag>}"
NTRIALS="${2:?usage: run_tuning_batch.sh <world_idx> <n_trials> <tag>}"
TAG="${3:?usage: run_tuning_batch.sh <world_idx> <n_trials> <tag>}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUTDIR="${SCRIPT_DIR}/world_${WORLD}/${TAG}"
OUT_FILE="${OUTDIR}/raw_results.txt"

mkdir -p "${OUTDIR}/logs"
: > "$OUT_FILE"
cp "${REPO_ROOT}/ros2_ws/src/barn_bringup/config/classical_mpc.yaml" "${OUTDIR}/"

echo "[tuning] world=${WORLD} trials=${NTRIALS} tag=${TAG}"
echo "[tuning] out=${OUTDIR}"

for t in $(seq 1 "$NTRIALS"); do
  echo "[tuning] === world ${WORLD} trial ${t}/${NTRIALS} (${TAG}) === $(date -u +%H:%M:%S)"
  # rviz defaults to TRUE in run_single_world.sh -- headless means saying so.
  BARN_GUI=false BARN_RVIZ=false BARN_PLANNER_RVIZ=false \
  BARN_OUT_FILE="$OUT_FILE" BARN_RESULTS_DIR="$OUTDIR" \
    "${REPO_ROOT}/evaluation/scripts/run_single_world.sh" \
      "$WORLD" classical_mpc "$t" \
      > "${OUTDIR}/logs/trial_${t}.log" 2>&1
  tail -n 1 "$OUT_FILE" 2>/dev/null | sed 's/^/[tuning]   result: /'
  # Let the DDS discovery of the previous trial's nodes actually expire before
  # the next one starts; a shorter gap makes trial N+1 fail for reasons that
  # belong to trial N.
  sleep 8
done

python3 "${SCRIPT_DIR}/summarize_batch.py" "$OUTDIR" | tee "${OUTDIR}/summary.txt"
