#!/usr/bin/env bash
#
# Format / lint the source we own (never the external evaluator checkout).
#   --check   report only, non-zero exit on any diff/violation (for CI/pre-push)
# Default: format C++ in place and lint Python.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

CHECK=0
[[ "${1:-}" == "--check" ]] && CHECK=1

# C++ files under ros2_ws/src, excluding the external evaluator checkout.
mapfile -t CPP < <(find ros2_ws/src -path ros2_ws/src/The-Barn-Challenge-Ros2 -prune -o \
  \( -name '*.cpp' -o -name '*.hpp' \) -print)

if command -v clang-format >/dev/null 2>&1 && ((${#CPP[@]})); then
  if ((CHECK)); then
    echo "[format] clang-format --dry-run --Werror on ${#CPP[@]} files"
    clang-format --dry-run --Werror "${CPP[@]}"
  else
    echo "[format] clang-format -i on ${#CPP[@]} files"
    clang-format -i "${CPP[@]}"
  fi
else
  echo "[format] clang-format not found or no C++ files; skipping C++"
fi

# Python: flake8 the two ament_python packages and the learning code.
PY_PATHS=(ros2_ws/src/barn_rl_runtime ros2_ws/src/barn_hybrid learning evaluation tools)
if command -v flake8 >/dev/null 2>&1; then
  echo "[format] flake8 ${PY_PATHS[*]}"
  flake8 "${PY_PATHS[@]}" || { ((CHECK)) && exit 1 || true; }
else
  echo "[format] flake8 not found; skipping Python lint"
fi

echo "[format] done"
