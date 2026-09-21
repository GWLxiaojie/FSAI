#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
ros_setup="${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"

usage() {
  printf 'Usage: %s --scenario SCENARIO [--vehicle VEHICLE] [--track TRACK] [--run-mode MODE] [--max-steps N] [--visualize]\n' "${0##*/}"
}

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  usage
  exit 0
fi

vehicle=""
track=""
scenario=""
launch_options=()
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --vehicle)
      [[ "$#" -ge 2 && -n "$2" && "$2" != --* ]] || { usage >&2; exit 2; }
      vehicle="$2"
      shift 2
      ;;
    --track)
      [[ "$#" -ge 2 && -n "$2" && "$2" != --* ]] || { usage >&2; exit 2; }
      track="$2"
      shift 2
      ;;
    --scenario)
      [[ "$#" -ge 2 && -n "$2" && "$2" != --* ]] || { usage >&2; exit 2; }
      scenario="$2"
      shift 2
      ;;
    --run-mode)
      [[ "$#" -ge 2 && ( "$2" == realtime || "$2" == as_fast_as_possible ) ]] || { usage >&2; exit 2; }
      launch_options+=("run_mode:=$2")
      shift 2
      ;;
    --max-steps)
      [[ "$#" -ge 2 && "$2" =~ ^[0-9]+$ ]] || { usage >&2; exit 2; }
      launch_options+=("max_steps:=$2")
      shift 2
      ;;
    --visualize)
      launch_options+=("visualize:=true")
      shift
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$scenario" ]]; then
  usage >&2
  exit 2
fi

resources=()
if [[ -n "$vehicle" ]]; then resources+=("vehicle:=$vehicle"); fi
if [[ -n "$track" ]]; then resources+=("track:=$track"); fi

cd "$repo_root"
# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup "$ros_setup"
fsai_use_local_dependencies "$repo_root"
fsai_source_setup "${FSAI_WORKSPACE_SETUP:-$repo_root/install/setup.bash}"
exec ros2 launch fsai_bringup simulator.launch.py \
  "${resources[@]}" \
  "scenario:=$scenario" \
  "${launch_options[@]}"
