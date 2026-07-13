#!/usr/bin/env bash
#
# Fast development sweep: one trial per world in evaluation/suites/dev_worlds.txt.
# Use during iteration; it is NOT the scored campaign (see run_barn2026_public_suite.sh).
#
# Usage:  run_dev_suite.sh [algo_type]     (default: builtin)

set -euo pipefail

MODE="${1:-builtin}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUITE_FILE="${SCRIPT_DIR}/../suites/dev_worlds.txt"
RESULTS_DIR="${BARN_RESULTS_DIR:-results/${MODE}/dev}"
OUT_FILE="${RESULTS_DIR}/raw_results.txt"

mkdir -p "$RESULTS_DIR"
: > "$OUT_FILE"  # start a fresh dev out_file (dev sweeps are disposable)

while read -r world_idx; do
  [[ -z "$world_idx" || "$world_idx" == \#* ]] && continue
  echo "=== dev world ${world_idx} (${MODE}) ==="
  BARN_RESULTS_DIR="$RESULTS_DIR" BARN_OUT_FILE="$OUT_FILE" \
    "${SCRIPT_DIR}/run_single_world.sh" "$world_idx" "$MODE" 1
  sleep 5
done < "$SUITE_FILE"

echo
echo "[run_dev_suite] published-rule report:"
python3 "${SCRIPT_DIR}/../metrics/barn2026_metric.py" --out_path "$OUT_FILE" || true
