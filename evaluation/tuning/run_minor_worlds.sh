#!/usr/bin/env bash
#
# Run the Tier-2 "minor tuning needed" worlds, 10 trials each, headless.
#
# 10 trials per world matches the campaign's own trials-per-world, so each row
# is directly comparable to the campaign result for that world (subject to the
# headless-vs-RViz caveat recorded in RESULTS.md).
#
# One world at a time, sequential. Roughly 14-16 min per world, so ~4 h total.
# A world that fails outright does not stop the sweep -- the aggregate report
# shows it as missing rather than silently omitting it.
#
# Usage:  run_minor_worlds.sh [trials] [tag]

set -uo pipefail

TRIALS="${1:-10}"
TAG="${2:-minor_10}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Tier 2 from the 430-trial campaign analysis: 80-99% success with isolated
# timeouts, or 100% success with AT/OT in the 2.0-3.4 band.
WORLDS=(78 114 120 126 150 162 168 174 180 186 198 216 222 234 288)

echo "[minor] ${#WORLDS[@]} worlds x ${TRIALS} trials, tag=${TAG}"
echo "[minor] active config deltas vs pristine baseline:"
diff "${SCRIPT_DIR}/baseline_config/classical_mpc.yaml" \
     "${SCRIPT_DIR}/../../ros2_ws/src/barn_bringup/config/classical_mpc.yaml" \
  | grep -E '^[<>]' | grep -vE '^[<>][[:space:]]*#' | sed 's/^/[minor]   /'
echo

start_all=$(date +%s)
for w in "${WORLDS[@]}"; do
  start=$(date +%s)
  echo "[minor] >>> world ${w} starting $(date -u +%H:%M:%S)"
  "${SCRIPT_DIR}/run_tuning_batch.sh" "$w" "$TRIALS" "$TAG" >/dev/null 2>&1
  mins=$(( ($(date +%s) - start) / 60 ))
  line=$(grep -E '^(trials|MEAN SCORE)' "${SCRIPT_DIR}/world_${w}/${TAG}/summary.txt" 2>/dev/null | tr '\n' ' ')
  echo "[minor] <<< world ${w} done in ${mins}m :: ${line:-NO SUMMARY}"
done
echo "[minor] sweep finished in $(( ($(date +%s) - start_all) / 60 )) min"

python3 "${SCRIPT_DIR}/summarize_minor.py" "$TAG" \
  | tee "${SCRIPT_DIR}/minor_sweep_${TAG}.txt"
