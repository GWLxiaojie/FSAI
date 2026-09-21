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
fsai_source_setup "${FSAI_WORKSPACE_SETUP:-$repo_root/install/setup.bash}"
# Integration tests publish commands and reset services. Keep them off the
# team's normal DDS domain and off the physical network.
export ROS_DOMAIN_ID="${FSAI_TEST_DOMAIN_ID:-171}"
export ROS_LOCALHOST_ONLY=1

set +e
colcon --log-base "$repo_root/log" test \
  --base-paths "$repo_root/simulator/src" \
  --build-base "$repo_root/build" \
  --install-base "$repo_root/install" \
  --packages-up-to fsai_bringup \
  --executor sequential \
  --return-code-on-test-failure
test_status=$?
colcon --log-base "$repo_root/log" test-result --verbose --test-result-base "$repo_root/build"
result_status=$?
set -e

if [[ "$test_status" -ne 0 ]]; then
  exit "$test_status"
fi
exit "$result_status"
