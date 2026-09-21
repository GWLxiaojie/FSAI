#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
mode=realtime
visualize=true
report=""
usage() {
  printf 'Usage: %s [--fast] [--no-gui] [--visualize] [--report NEW_FILE.json]\n' "${0##*/}"
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --help) usage; exit 0 ;;
    --fast) mode=as_fast_as_possible; visualize=false; shift ;;
    --no-gui) visualize=false; shift ;;
    --visualize) visualize=true; shift ;;
    --report)
      [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { usage >&2; exit 2; }
      report="$2"; shift 2 ;;
    *) usage >&2; exit 2 ;;
  esac
done
if [[ -z "$report" ]]; then
  report="$repo_root/artifacts/official-laps-$(date -u +%Y%m%dT%H%M%S)-$$.json"
fi
if [[ -e "$report" ]]; then
  printf 'Report already exists; choose a new path: %s\n' "$report" >&2
  exit 1
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-183}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || { printf 'ROS_DOMAIN_ID must be numeric\n' >&2; exit 2; }
mkdir -p "$repo_root/.dependencies/official"
exec 9>"$repo_root/.dependencies/official/run-domain-$ROS_DOMAIN_ID.lock"
flock -n 9 || { printf 'An official run is already active in DDS domain %s\n' "$ROS_DOMAIN_ID" >&2; exit 1; }
python3 "$script_dir/official_assets.py"
# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup "${FSAI_ROS_SETUP:-/opt/ros/humble/setup.bash}"
fsai_use_local_dependencies "$repo_root"
"$script_dir/build.sh"
fsai_source_setup "$repo_root/install/setup.bash"
generated="$repo_root/.dependencies/official/generated"
printf 'Official FS-AI Sprint: 10 laps at 2.5 m/s (~49 simulated minutes).\n'
printf 'Mode: %s; DDS domain: %s; report: %s\n' "$mode" "$ROS_DOMAIN_ID" "$report"
if [[ "$visualize" == true ]]; then
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi
ros2 launch fsai_bringup simulator.launch.py \
  scenario:=official_ten_laps vehicle:=ads_dv \
  "track:=$generated/track" "run_mode:=$mode" "visualize:=$visualize" \
  "vehicle_description:=$generated/ads_dv_official.urdf" \
  "rviz_config:=$repo_root/simulator/src/fsai_bringup/config/official_laps.rviz" \
  "report_path:=$report"
python3 "$script_dir/check_lap_report.py" "$report"
