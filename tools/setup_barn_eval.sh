#!/usr/bin/env bash
#
# Clone the official BARN 2026 ROS 2 evaluator into this colcon workspace and
# add the algo_type dispatcher. The builtin branch remains stock, the movement
# test remains available, and classical_mpc launches this workspace's stack.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

EVAL_REPO="${BARN_EVAL_REPO:-https://github.com/Saadmaghani/The-Barn-Challenge-Ros2.git}"
DEST="ros2_ws/src/The-Barn-Challenge-Ros2"
RUNNER="$DEST/jackal_helper/launch/BARN_runner.launch.py"

if [[ -d "$DEST/.git" ]]; then
  echo "[setup_barn_eval] official evaluator already present at ${DEST}"
else
  echo "[setup_barn_eval] cloning official evaluator -> ${DEST}"
  git clone "$EVAL_REPO" "$DEST"
fi

python3 tools/configure_barn_eval_dispatch.py "$RUNNER"
echo "[setup_barn_eval] builtin keeps the stock jackal_helper Nav2 baseline"
echo "[setup_barn_eval] movement_and_odom_test enables the local wiring test"
echo "[setup_barn_eval] classical_mpc enables mapping + lattice planning + MPC + safety"

echo
echo "[setup_barn_eval] ready. Next:"
echo "  bash tools/setup_workspace.sh"
