#!/usr/bin/env bash
#
# Fetch, pin, and patch the BARN 2026 ROS 2 evaluator into the workspace.
# The evaluator is NOT vendored in this repo (it is git-ignored under
# ros2_ws/src/barn_eval). Only the pinned commit (patches/pinned_commit.txt) and
# the launch-hook patch (patches/barn_eval_launch_navigation_stack.patch) are
# tracked here.
#
# Idempotent: safe to re-run. Does NOT modify the evaluator's collision/goal/
# timeout/metric logic — only its navigation-stack launch hook.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

EVAL_REPO="${BARN_EVAL_REPO:-https://github.com/Saadmaghani/The-Barn-Challenge-Ros2.git}"
EVAL_BRANCH="${BARN_EVAL_BRANCH:-master}"
DEST="ros2_ws/src/barn_eval"
PIN_FILE="patches/pinned_commit.txt"
PATCH_FILE="patches/barn_eval_launch_navigation_stack.patch"

# --- 1. clone (or reuse) --------------------------------------------------
if [[ -d "$DEST/.git" ]]; then
  echo "[setup_barn_eval] evaluator already present at ${DEST} (reusing)"
else
  echo "[setup_barn_eval] cloning ${EVAL_REPO} (${EVAL_BRANCH}) -> ${DEST}"
  git clone --branch "$EVAL_BRANCH" "$EVAL_REPO" "$DEST"
fi

# --- 2. pin ---------------------------------------------------------------
PIN=""
if [[ -f "$PIN_FILE" ]]; then
  PIN="$(grep -vE '^\s*(#|$)' "$PIN_FILE" | head -n1 | tr -d '[:space:]' || true)"
fi
if [[ -n "$PIN" && "$PIN" != "REPLACE_WITH_PINNED_COMMIT" ]]; then
  echo "[setup_barn_eval] checking out pinned commit ${PIN}"
  git -C "$DEST" fetch --all --quiet || true
  git -C "$DEST" checkout --quiet "$PIN"
else
  ACTUAL="$(git -C "$DEST" rev-parse HEAD)"
  echo "[setup_barn_eval] WARNING: no commit pinned. Currently at ${ACTUAL} on ${EVAL_BRANCH}."
  echo "[setup_barn_eval] Record it for reproducibility:"
  echo "                    echo ${ACTUAL} > ${PIN_FILE}"
fi

# --- 3. apply the launch-hook patch --------------------------------------
if git -C "$DEST" apply --reverse --check "../../../${PATCH_FILE}" >/dev/null 2>&1; then
  echo "[setup_barn_eval] launch patch already applied"
elif git -C "$DEST" apply --3way "../../../${PATCH_FILE}" >/dev/null 2>&1; then
  echo "[setup_barn_eval] applied ${PATCH_FILE}"
else
  cat >&2 <<'MSG'
[setup_barn_eval] NOTE: could not auto-apply the launch patch.
  The upstream evaluator has no tagged release, so the patch context may not
  match your pinned revision. Apply the change by hand, then regenerate the
  patch (see patches/README.md):

    1. Edit the evaluator's launch_navigation_stack() to IncludeLaunchDescription
       barn_bringup/launch/barn_navigation.launch.py with mode:=$BARN_MODE.
    2. From ros2_ws/src/barn_eval:  git diff > ../../../patches/barn_eval_launch_navigation_stack.patch
    3. git restore .   (keep the evaluator clean; the patch is the source of truth)
MSG
fi

echo
echo "[setup_barn_eval] next:"
echo "  bash tools/setup_workspace.sh    # rosdep install + colcon build"
