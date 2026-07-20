#!/usr/bin/env bash
# Install the DynaBARN dynamic-obstacle worlds into the BARN ROS2 evaluator and
# print the environment/run commands needed to load the DynamicObstacle plugin.
set -euo pipefail

# Resolve the repo root from this script's location (tools/ lives at repo root).
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"

SRC_DIR="${REPO_ROOT}/worlds/DynaBARN"
DST_DIR="${REPO_ROOT}/ros2_ws/src/The-Barn-Challenge-Ros2/jackal_helper/worlds/DynaBARN"
PLUGIN_DIR="${REPO_ROOT}/simulation/barn_dynamic_obstacle/build"

if [ ! -d "${SRC_DIR}" ]; then
  echo "ERROR: source worlds dir not found: ${SRC_DIR}" >&2
  exit 1
fi

shopt -s nullglob
worlds=("${SRC_DIR}"/*.world)
if [ ${#worlds[@]} -eq 0 ]; then
  echo "ERROR: no *.world files in ${SRC_DIR}" >&2
  exit 1
fi

mkdir -p "${DST_DIR}"
for w in "${worlds[@]}"; do
  cp -v "${w}" "${DST_DIR}/"
done

echo
echo "Installed ${#worlds[@]} world(s) into:"
echo "  ${DST_DIR}"
echo
echo "The evaluator maps world_idx 300..359 -> DynaBARN/world_{idx-300}.world."
echo "world_0/1/2 correspond to world_idx 300/301/302."
echo
echo "Before launching, export the plugin path so gz-sim can load DynamicObstacle:"
echo
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=${PLUGIN_DIR}"
echo
echo "Example run command:"
echo
echo "  ros2 launch jackal_helper BARN_runner.launch.py algo_type:=classical_mpc world_idx:=300 gui:=true planner_rviz:=true"
echo
