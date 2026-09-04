#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
ros_setup="${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"

usage() {
  printf 'Usage: %s --vehicle VEHICLE --track TRACK --scenario SCENARIO\n' "${0##*/}"
}

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  usage
  exit 0
fi

vehicle=""
track=""
scenario=""
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
    *)
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$vehicle" || -z "$track" || -z "$scenario" ]]; then
  usage >&2
  exit 2
fi

cd "$repo_root"
# shellcheck disable=SC1090
source "$ros_setup"
exec ros2 launch fsai_bringup simulator.launch.py \
  "vehicle:=$vehicle" \
  "track:=$track" \
  "scenario:=$scenario"
