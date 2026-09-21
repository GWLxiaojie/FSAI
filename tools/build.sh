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

# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup "$ros_setup"
fsai_use_local_dependencies "$repo_root"
# Header-heavy upstream libraries can exhaust memory at CPU-count parallelism.
export MAKEFLAGS="${MAKEFLAGS:--j${FSAI_BUILD_JOBS:-2}}"
exec colcon --log-base "$repo_root/log" build \
  --base-paths "$repo_root/simulator/src" \
  --build-base "$repo_root/build" \
  --install-base "$repo_root/install" \
  --packages-up-to fsai_bringup \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "-DCMAKE_PROJECT_INCLUDE=$repo_root/cmake/eufs_compatibility.cmake"
