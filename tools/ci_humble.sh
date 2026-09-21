#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

"$repo_root/tools/bootstrap_ubuntu.sh"
"$repo_root/tools/prepare_eufs_checkout.sh" "$repo_root/simulator/src/eufs_sim2"
"$repo_root/tools/tests/test_ci_contract.sh"
"$repo_root/tools/tests/test_entry_points.sh"
python3 -m unittest discover -s "$repo_root/tools/tests" -p 'test_*.py'
"$repo_root/tools/build.sh"
"$repo_root/tools/test.sh"
# Real cross-process DDS/CLI acceptance in its own localhost-only domain.
# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup "${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"
fsai_use_local_dependencies "$repo_root"
fsai_source_setup "$repo_root/install/setup.bash"
python3 -B "$repo_root/tools/tests/ros_control_smoke.py"
