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

# A campaign's whole claim to reproducibility is this block, so a git command
# that FAILS must never be recorded as a fact. `repo_dirty` used to be
# `test -n "$(git status --porcelain 2>/dev/null)" && echo true || echo false`,
# which reports a CLEAN tree whenever git errors out -- the one case where the
# tree's state is unknown. That is not a conservative default, it is a false
# statement in the provenance record of a scored benchmark.
#
# It happens easily: run the campaign as a different user from the one that owns
# the checkout (root inside a container, say) and git refuses the repository for
# "dubious ownership". Every command here fails, and the old code produced
# `"repo_commit": "unknown", "repo_dirty": false` -- unknown commit, certified
# clean. Now a failure is reported as `null`, which no reader can mistake for
# either state.
if ! git rev-parse HEAD >/dev/null 2>&1; then
  echo "warning: git cannot read this repository (wrong user? dubious ownership?)." >&2
  echo "         The manifest will record null provenance." >&2
  repo_sha="unknown"
  repo_dirty="null"
else
  repo_sha="$(git rev-parse HEAD)"
  repo_dirty="$(test -n "$(git status --porcelain)" && echo true || echo false)"
fi
eval_dir="ros2_ws/src/The-Barn-Challenge-Ros2"
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
