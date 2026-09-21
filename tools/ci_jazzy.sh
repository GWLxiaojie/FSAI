#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

echo "Jazzy compile guard. Runtime E2E is Humble-only."
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required for this compile guard." >&2
  exit 1
fi
# shellcheck source=ros_env.sh
source "$script_dir/ros_env.sh"
fsai_source_setup /opt/ros/jazzy/setup.bash
colcon --log-base "$repo_root/log/jazzy" build \
  --base-paths "$repo_root/simulator/src/fsai_sim_core" \
  --build-base "$repo_root/build/jazzy" \
  --install-base "$repo_root/install/jazzy" \
  --cmake-args "-DCMAKE_PROJECT_INCLUDE=$repo_root/cmake/eufs_compatibility.cmake"
colcon --log-base "$repo_root/log/jazzy" test \
  --base-paths "$repo_root/simulator/src/fsai_sim_core" \
  --build-base "$repo_root/build/jazzy" \
  --install-base "$repo_root/install/jazzy" \
  --return-code-on-test-failure
colcon --log-base "$repo_root/log/jazzy" test-result \
  --test-result-base "$repo_root/build/jazzy" --verbose
