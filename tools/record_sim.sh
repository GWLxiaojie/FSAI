#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
if [[ "$#" != 1 || "$1" == --help ]]; then
  printf 'Usage: %s OUTPUT_DIRECTORY (Ctrl-C finalizes the MCAP recording)\n' "${0##*/}"
  [[ "${1:-}" == --help ]] && exit 0
  exit 2
fi
if [[ -e "$1" ]]; then
  printf 'record_sim: output already exists; choose a new directory: %s\n' "$1" >&2
  exit 1
fi
# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup "${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"
fsai_use_local_dependencies "$repo_root"
fsai_source_setup "${FSAI_WORKSPACE_SETUP:-$repo_root/install/setup.bash}"
exec ros2 bag record --storage mcap --use-sim-time --output "$1" \
  /clock /tf /tf_static /fsai/actuation_command /fsai/actuator_state \
  /sim/state/as_state /sim/state/mission /ground_truth/odom \
  /ground_truth/forces /ground_truth/track_markers \
  /sensors/imu /sensors/oss /sensors/gnss /sensors/wheel_speeds \
  /sensors/camera/cones /sensors/lidar/cones
