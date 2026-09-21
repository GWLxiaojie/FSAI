#!/usr/bin/env bash
# ROS-generated setup scripts intentionally read optional, unset variables.
# Keep strict mode in our callers, but not while sourcing those scripts.
fsai_source_setup() {
  local setup_file="$1"
  local setup_status=0
  if [[ ! -r "$setup_file" ]]; then
    printf 'FSAI: setup file is missing: %s\n' "$setup_file" >&2
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$setup_file" || setup_status=$?
  set -u
  return "$setup_status"
}

# Optional project-local apt extraction used on hosts without non-interactive
# sudo. Normal bootstrap installs the same dependencies through rosdep instead.
fsai_use_local_dependencies() {
  local dependency_prefix="$1/.dependencies/root/opt/ros/humble"
  if [[ -d "$dependency_prefix/share/ament_index" ]]; then
    export CMAKE_PREFIX_PATH="$dependency_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    export AMENT_PREFIX_PATH="$dependency_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
    export LD_LIBRARY_PATH="$dependency_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
}
