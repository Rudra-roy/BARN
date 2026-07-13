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

if [[ "$ROS_DISTRO" != "jazzy" ]]; then
  echo "error: this workspace requires ROS 2 Jazzy; found ROS_DISTRO=${ROS_DISTRO}." >&2
  echo "       Run: source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

# Remove ROS paths inherited from a differently sourced host environment.
source "${REPO_ROOT}/infra/env/barn_jazzy.env"

echo "[setup_workspace] rosdep install (${ROS_DISTRO})"
if ! command -v rosdep >/dev/null 2>&1; then
  echo "error: rosdep not found; install ros-dev-tools before continuing." >&2
  exit 1
fi
rosdep install --from-paths "${WS}/src" --ignore-src -y --rosdistro "${ROS_DISTRO}"

echo "[setup_workspace] colcon build --symlink-install (Release)"
( cd "$WS" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release )

echo
echo "[setup_workspace] done. Source the overlay:"
echo "  source ${WS}/install/setup.bash"
