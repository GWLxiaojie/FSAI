#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
ros_setup="${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"

usage() {
  printf 'Usage: %s [--help]\n' "${0##*/}"
}

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "$#" -ne 0 ]]; then
  usage >&2
  exit 2
fi

# shellcheck disable=SC1090
source "$ros_setup"
exec colcon build \
  --base-paths "$repo_root/simulator/src" \
  --build-base "$repo_root/build" \
  --install-base "$repo_root/install" \
  --log-base "$repo_root/log" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
