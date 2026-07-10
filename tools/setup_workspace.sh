#!/usr/bin/env bash
#
# Resolve dependencies and build the workspace. Run inside the ROS 2 Jazzy
# distrobox after tools/setup_barn_eval.sh.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${REPO_ROOT}/ros2_ws"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "error: ROS 2 is not sourced. Run: source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

echo "[setup_workspace] rosdep install (${ROS_DISTRO})"
if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${ROS_DISTRO}" || {
    echo "[setup_workspace] rosdep reported issues; continuing to build." >&2
  }
else
  echo "[setup_workspace] rosdep not found; skipping dependency resolution." >&2
fi

echo "[setup_workspace] colcon build --symlink-install"
( cd "$WS" && colcon build --symlink-install )

echo
echo "[setup_workspace] done. Source the overlay:"
echo "  source ${WS}/install/setup.bash"
