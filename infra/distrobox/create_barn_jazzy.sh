#!/usr/bin/env bash
#
# Create a dedicated Ubuntu 24.04 distrobox for BARN development, with its own
# HOME so the Jazzy environment does not contaminate the host ROS setup.
# You likely already have such a container; this is provided for reproducibility.
#
# Env: BARN_BOX_NAME (default barn-jazzy), BARN_BOX_HOME (default ~/distrobox-homes/barn-jazzy)
# Pass --nvidia to enable host NVIDIA integration for the Gazebo GUI ONLY.
# Do NOT let simulator GPU access make the final policy GPU-dependent.

set -euo pipefail

BOX_NAME="${BARN_BOX_NAME:-barn-jazzy}"
BOX_HOME="${BARN_BOX_HOME:-${HOME}/distrobox-homes/barn-jazzy}"
IMAGE="docker.io/library/ubuntu:24.04"

NVIDIA_FLAG=()
[[ "${1:-}" == "--nvidia" ]] && NVIDIA_FLAG=(--nvidia)

if ! command -v distrobox >/dev/null 2>&1; then
  echo "error: distrobox not found on the host." >&2
  exit 1
fi

mkdir -p "$BOX_HOME"

echo "[create_barn_jazzy] creating '${BOX_NAME}' (home ${BOX_HOME})"
distrobox create \
  --name "$BOX_NAME" \
  --image "$IMAGE" \
  --home "$BOX_HOME" \
  "${NVIDIA_FLAG[@]}"

cat <<EOF

[create_barn_jazzy] created. Next:
  distrobox enter ${BOX_NAME}
  bash infra/distrobox/setup_jazzy.sh     # install ROS 2 Jazzy inside the box
EOF
