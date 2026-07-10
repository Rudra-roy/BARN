#!/usr/bin/env bash
#
# Capture reproducibility metadata for a benchmark campaign into
# <results_dir>/manifest.json plus a few plain-text state files. The raw
# evaluator output remains the source of truth; this records the context needed
# to reproduce and compare campaigns.
#
# Usage:  capture_manifest.sh <results_dir> [mode]

set -euo pipefail

RESULTS_DIR="${1:?usage: capture_manifest.sh <results_dir> [mode]}"
MODE="${2:-unknown}"
mkdir -p "$RESULTS_DIR"

repo_sha="$(git rev-parse HEAD 2>/dev/null || echo unknown)"
repo_dirty="$(test -n "$(git status --porcelain 2>/dev/null)" && echo true || echo false)"
eval_dir="ros2_ws/src/barn_eval"
eval_sha="$(git -C "$eval_dir" rev-parse HEAD 2>/dev/null || echo not-present)"
date_iso="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# JSON manifest (kept out of .gitignore so it is committed with the campaign).
cat > "${RESULTS_DIR}/manifest.json" <<JSON
{
  "date_utc": "${date_iso}",
  "mode": "${MODE}",
  "repo_commit": "${repo_sha}",
  "repo_dirty": ${repo_dirty},
  "evaluator_commit": "${eval_sha}",
  "ros_distro": "${ROS_DISTRO:-unknown}",
  "hostname": "$(hostname)"
}
JSON

# Verbose plain-text state (git-ignored heavy files stay out of the repo).
{ uname -a; echo; lscpu 2>/dev/null || true; echo; free -h 2>/dev/null || true; } \
  > "${RESULTS_DIR}/system_info.txt" 2>/dev/null || true
( command -v ros2 >/dev/null 2>&1 && ros2 pkg list | sort > "${RESULTS_DIR}/ros_packages.txt" ) || true
( command -v apt >/dev/null 2>&1 && apt list --installed 2>/dev/null > "${RESULTS_DIR}/apt_packages.txt" ) || true
git status --short > "${RESULTS_DIR}/git_state.txt" 2>/dev/null || true

echo "[capture_manifest] wrote ${RESULTS_DIR}/manifest.json (repo ${repo_sha}, eval ${eval_sha})"
