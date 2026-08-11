#!/usr/bin/env bash
#
# Run a command inside the barn-jazzy box with the environment the box's own
# .bashrc would produce.
#
# Why this exists: `distrobox enter` inherits the HOST shell's environment,
# which on this machine carries ROS Humble (AMENT_PREFIX_PATH, PYTHONPATH,
# LD_LIBRARY_PATH entries under /opt/ros/humble). The box's ~/.bashrc scrubs
# those before sourcing Jazzy -- but .bashrc only runs for INTERACTIVE shells,
# so `distrobox enter -- bash -c ...` skips it entirely and
# infra/env/barn_jazzy.env then refuses to load ("non-Jazzy ROS path").
#
# Rather than depend on shell interactivity, this replicates the scrub
# explicitly. Note it PRESERVES the non-Humble parts of LD_LIBRARY_PATH, which
# `docker exec` does not have at all -- that difference is why trials driven
# through `docker exec` failed to bring up Gazebo's renderer while the same
# scripts work through `distrobox enter`.
#
# Usage (from the host):
#   distrobox enter barn-jazzy -- bash /run/host/home/mt-labpc/BARN/evaluation/tuning/in_box.sh <cmd> [args...]

# No `set -u`: ROS's own setup.bash files read unset variables
# (AMENT_TRACE_SETUP_FILES and friends) and abort under nounset.
set -o pipefail

# --- scrub, exactly as the box's .bashrc does -------------------------------
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH ROS_LOCALHOST_ONLY
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH COLCON_CURRENT_PREFIX
for _v in PATH LD_LIBRARY_PATH PKG_CONFIG_PATH; do
  _clean="$(printf '%s' "${!_v:-}" | tr ':' '\n' \
    | grep -v -e '^/opt/ros/humble' -e '^/home/mt-labpc/ros2_ws' | paste -sd: -)"
  export "$_v"="$_clean"
done
unset _v _clean

REPO="${BARN_REPO:-/run/host/home/mt-labpc/BARN}"
cd "$REPO" || { echo "in_box: cannot cd to $REPO" >&2; exit 1; }

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO/infra/env/barn_jazzy.env" >/dev/null || {
  echo "in_box: barn_jazzy.env refused to load" >&2; exit 1; }
# shellcheck disable=SC1091
source "$REPO/ros2_ws/install/setup.bash"

if [ "${1:-}" = "--check" ]; then
  echo "ROS_DISTRO=$ROS_DISTRO"
  echo "non-jazzy AMENT entries: $(echo "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | grep -vc jazzy)"
  echo "DISPLAY=${DISPLAY:-unset}"
  echo "LD_LIBRARY_PATH entries: $(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -c .)"
  exit 0
fi

cmd="${1:?usage: in_box.sh <script-relative-to-evaluation/tuning> [args...]}"
shift
# A bare name is a script in evaluation/tuning/; anything containing a slash is
# repo-relative, so the campaign suite can use this same scrubbed environment.
case "$cmd" in
  */*) exec "$REPO/$cmd" "$@" ;;
  *)   exec "$REPO/evaluation/tuning/$cmd" "$@" ;;
esac
