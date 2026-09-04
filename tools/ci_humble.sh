#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

"$repo_root/tools/bootstrap_ubuntu.sh"
"$repo_root/tools/prepare_eufs_checkout.sh" "$repo_root/simulator/src/eufs_sim2"
# shellcheck disable=SC1090
source "${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"
"$repo_root/tools/build.sh"
"$repo_root/tools/test.sh"
