#!/usr/bin/env bash
#
# Install ROS 2 Jazzy inside the Ubuntu 24.04 distrobox. Run this INSIDE the
# container (distrobox enter barn-jazzy). Idempotent-ish; safe to re-run.

set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "24.04" ]]; then
  echo "warning: expected Ubuntu 24.04 (Noble) for ROS 2 Jazzy." >&2
fi

echo "[setup_jazzy] locale"
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[setup_jazzy] universe + base tools"
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt install -y curl git gnupg lsb-release build-essential

echo "[setup_jazzy] ROS 2 apt source"
ROS_APT_SOURCE_VERSION="$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F\" '{print $4}')"
echo "[setup_jazzy] ros-apt-source ${ROS_APT_SOURCE_VERSION}"
codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${codename}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

echo "[setup_jazzy] ros-jazzy-desktop + dev tools"
sudo apt install -y ros-jazzy-desktop ros-dev-tools

echo "[setup_jazzy] rosdep"
sudo rosdep init 2>/dev/null || true
rosdep update

if ! grep -q '/opt/ros/jazzy/setup.bash' "${HOME}/.bashrc" 2>/dev/null; then
  echo 'source /opt/ros/jazzy/setup.bash' >> "${HOME}/.bashrc"
fi

echo "[setup_jazzy] done. Open a new shell (or 'source /opt/ros/jazzy/setup.bash'), then:"
echo "  bash tools/setup_barn_eval.sh && bash tools/setup_workspace.sh"
