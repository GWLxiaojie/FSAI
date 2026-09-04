#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

echo "Jazzy compile guard. Runtime E2E is Humble-only."
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed. Skipping on this host."
  exit 0
fi
# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths "$repo_root/simulator/src/fsai_sim_core" \
  --build-base "$repo_root/build/jazzy" \
  --install-base "$repo_root/install/jazzy"
