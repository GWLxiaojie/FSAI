#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bootstrap="$repo_root/tools/bootstrap_ubuntu.sh"
build="$repo_root/tools/build.sh"
test_command="$repo_root/tools/test.sh"
run_sim="$repo_root/tools/run_sim.sh"
tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

expect_status_and_output() {
  local expected_status="$1"
  local expected_text="$2"
  shift 2

  local output status
  set +e
  output="$("$@" 2>&1)"
  status=$?
  set -e

  [[ "$status" -eq "$expected_status" ]] \
    || fail "expected exit $expected_status, got $status: $output"
  [[ "$output" == *"$expected_text"* ]] \
    || fail "expected output containing '$expected_text', got: $output"
}

make_fake_commands() {
  local fake_bin="$1"
  mkdir -p "$fake_bin"

  for command in sudo apt-get curl vcs rosdep dpkg-query; do
    printf '%s\n' \
      '#!/usr/bin/env bash' \
      'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
      'for argument in "$@"; do' \
      '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
      'done' \
      'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
      >"$fake_bin/$command"
    chmod +x "$fake_bin/$command"
  done

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'if [[ "${1:-}" == "test" && "${FSAI_FAKE_COLCON_TEST_STATUS:-0}" != "0" ]]; then' \
    '  exit "$FSAI_FAKE_COLCON_TEST_STATUS"' \
    'fi' \
    >"$fake_bin/colcon"
  chmod +x "$fake_bin/colcon"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    >"$fake_bin/ros2"
  chmod +x "$fake_bin/ros2"
}

assert_log_equals() {
  local expected="$1"
  local actual
  actual="$(cat "$FSAI_COMMAND_LOG")"
  [[ "$actual" == "$expected" ]] \
    || fail "unexpected command log:\n$actual\nexpected:\n$expected"
}

for command in "$bootstrap" "$build" "$test_command" "$run_sim"; do
  (
    cd "$tmp_dir"
    expect_status_and_output 0 'Usage:' "$command" --help
  )
  expect_status_and_output 2 'Usage:' "$command" --unknown
done

fake_bin="$tmp_dir/fake-bin"
command_log="$tmp_dir/commands.log"
ros_setup="$tmp_dir/setup.bash"
make_fake_commands "$fake_bin"
printf 'printf "setup\\n" >>"$FSAI_COMMAND_LOG"\n' >"$ros_setup"
chmod +x "$ros_setup"

touch "$command_log"
export FSAI_COMMAND_LOG="$command_log"
(
  cd "$tmp_dir"
  expect_status_and_output 1 'requires Ubuntu 22.04' env \
    PATH="$fake_bin:$PATH" \
    FSAI_COMMAND_LOG="$command_log" \
    "$bootstrap"
)
[[ ! -s "$command_log" ]] || fail 'bootstrap invoked a mutating command on a non-Ubuntu host'

: >"$command_log"
(
  cd "$tmp_dir"
  env \
    PATH="$fake_bin:$PATH" \
    FSAI_COMMAND_LOG="$command_log" \
    FSAI_ROS_SETUP="$ros_setup" \
    "$build"
)
assert_log_equals "setup
colcon build --base-paths $repo_root/simulator/src --build-base $repo_root/build --install-base $repo_root/install --log-base $repo_root/log --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

: >"$command_log"
set +e
(
  cd "$tmp_dir"
  env \
    PATH="$fake_bin:$PATH" \
    FSAI_COMMAND_LOG="$command_log" \
    FSAI_ROS_SETUP="$ros_setup" \
    FSAI_FAKE_COLCON_TEST_STATUS=17 \
    "$test_command"
)
test_status=$?
set -e
[[ "$test_status" -eq 17 ]] || fail "test command should preserve colcon test status, got $test_status"
assert_log_equals "setup
colcon test --base-paths $repo_root/simulator/src --build-base $repo_root/build --install-base $repo_root/install --log-base $repo_root/log
colcon test-result --verbose --test-result-base $repo_root/build"

: >"$command_log"
(
  cd "$tmp_dir"
  env \
    PATH="$fake_bin:$PATH" \
    FSAI_COMMAND_LOG="$command_log" \
    FSAI_ROS_SETUP="$ros_setup" \
    "$run_sim" \
    --vehicle fsai_vehicle \
    --track skidpad \
    --scenario dry_run
)
assert_log_equals "setup
ros2 launch fsai_bringup simulator.launch.py vehicle:=fsai_vehicle track:=skidpad scenario:=dry_run"

expect_status_and_output 2 'Usage:' "$run_sim" --vehicle fsai_vehicle --track skidpad

printf 'PASS: entry point contracts\n'
