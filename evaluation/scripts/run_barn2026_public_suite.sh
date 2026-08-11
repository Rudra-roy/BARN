#!/usr/bin/env bash
#
# The scored public campaign: the 50 evenly spaced public worlds
# [0, 6, 12, ..., 294] x 10 trials = 500 trials.
#
# This intentionally REPLACES the upstream test.sh, whose loop `for i in {7..49}`
# starts at world 42 and skips worlds 0/6/12/18/24/30/36 (see
# docs/benchmark/metric_notes.md). Do NOT use the upstream test.sh for a fresh
# full campaign.
#
# Usage:  run_barn2026_public_suite.sh [algo_type]     (default: builtin)
# No per-world tuning during the campaign.

set -euo pipefail

MODE="${1:-builtin}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIALS="${BARN_TRIALS_PER_WORLD:-10}"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# MUST be absolute. The evaluator writes to
# os.path.join(get_pkg_src_path()/res, out_file): an absolute out_file is returned
# unchanged by os.path.join, but a RELATIVE one is nested under the evaluator's
# res/ directory, whose subdirectories it never creates. Every write then fails
# and the result is lost SILENTLY -- the campaign runs to completion, every trial
# logs "Navigation succeeded", and raw_results.txt stays empty. Cost 42 trials
# before it was spotted. evaluation/tuning/run_tuning_batch.sh always passed an
# absolute path, which is why per-world batches recorded fine and only the suite
# lost data.
RESULTS_DIR="${BARN_RESULTS_DIR:-${REPO_ROOT}/results/${MODE}/public_$(printf '%(%Y%m%d_%H%M%S)T' -1)}"
OUT_FILE="${RESULTS_DIR}/raw_results.txt"

mkdir -p "$RESULTS_DIR"
: > "$OUT_FILE"

# Capture reproducibility metadata before the first run.
"${SCRIPT_DIR}/capture_manifest.sh" "$RESULTS_DIR" "$MODE" || true

for i in $(seq 0 49); do
  world_idx=$((i * 6))
  for trial in $(seq 1 "$TRIALS"); do
    echo "=== world ${world_idx}, trial ${trial}/${TRIALS} (${MODE}) ==="
    BARN_RESULTS_DIR="$RESULTS_DIR" BARN_OUT_FILE="$OUT_FILE" \
      "${SCRIPT_DIR}/run_single_world.sh" "$world_idx" "$MODE" "$trial"
    sleep 10
  done
done

echo
echo "###################  REPORTS  ###################"
echo "[A] Published BARN 2026 rule (research metric):"
python3 "${SCRIPT_DIR}/../metrics/barn2026_metric.py" --out_path "$OUT_FILE" \
  | tee "${RESULTS_DIR}/report_barn2026_rules.txt"
echo
echo "[B] Upstream-compatible report (evaluator debugging only):"
python3 "${SCRIPT_DIR}/../metrics/upstream_compat_metric.py" --out_path "$OUT_FILE" \
  | tee "${RESULTS_DIR}/report_upstream_master.txt"

echo
echo "[run_barn2026_public_suite] raw results + reports in ${RESULTS_DIR}"
