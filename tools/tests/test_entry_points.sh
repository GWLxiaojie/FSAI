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

make_bootstrap_fakes() {
  local fake_bin="$1"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'if [[ "$1" == "dpkg" && "$2" == "--install" ]]; then' \
    '  : >"$FSAI_ROS_KEYRING"' \
    '  printf '\''deb https://packages.ros.org/ros2/ubuntu jammy main\n'\'' >"$FSAI_APT_SOURCES_DIR/ros2.list"' \
    'fi' \
    'if [[ "$1" == "rosdep" && "$2" == "init" ]]; then' \
    '  : >"$FSAI_ROSDEP_SOURCES"' \
    'fi' \
    >"$fake_bin/sudo"
  chmod +x "$fake_bin/sudo"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'if [[ "${FSAI_BOOTSTRAP_REPEAT:-false}" == true ]]; then' \
    '  printf '\''installed\n'\''' \
    'else' \
    '  exit 1' \
    'fi' \
    >"$fake_bin/dpkg-query"
  chmod +x "$fake_bin/dpkg-query"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'output=""' \
    'while [[ "$#" -gt 0 ]]; do' \
    '  if [[ "$1" == "--output" ]]; then output="$2"; shift 2; continue; fi' \
    '  shift' \
    'done' \
    'if [[ -n "$output" ]]; then' \
    '  printf '\''fake ros2 apt source package\n'\'' >"$output"' \
    'else' \
    '  printf '\''  "tag_name": "1.2.3",\n'\''' \
    'fi' \
    >"$fake_bin/curl"
  chmod +x "$fake_bin/curl"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'fixture="$FSAI_BOOTSTRAP_TMP/ros2-apt-source.fixture"' \
    ': >"$fixture"' \
    'printf '\''%s\n'\'' "$fixture"' \
    >"$fake_bin/mktemp"
  chmod +x "$fake_bin/mktemp"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'src_dir="${@: -1}"' \
    'for repository in eufs_sim2 vehicle_models state_lib map_lib eufs_msgs eufs_gmock_matchers eufs_logger open_car_dynamics; do' \
    '  mkdir -p "$src_dir/$repository/.git"' \
    'done' \
    >"$fake_bin/vcs"
  chmod +x "$fake_bin/vcs"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    >"$fake_bin/rosdep"
  chmod +x "$fake_bin/rosdep"

  printf '%s\n' \
    '#!/usr/bin/env bash' \
    'printf '\''%q'\'' "$(basename "$0")" >>"$FSAI_COMMAND_LOG"' \
    'for argument in "$@"; do' \
    '  printf '\'' %q'\'' "$argument" >>"$FSAI_COMMAND_LOG"' \
    'done' \
    'printf '\''\n'\'' >>"$FSAI_COMMAND_LOG"' \
    'checkout=""' \
    'if [[ "${1:-}" == "-C" ]]; then checkout="$2"; shift 2; fi' \
    'case "${1:-}" in' \
    '  status) exit 0 ;;' \
    '  rev-parse)' \
    '    if [[ "${2:-}" == "--is-inside-work-tree" ]]; then printf '\''true\n'\''; exit 0; fi' \
    '    case "${checkout##*/}" in' \
    '      eufs_sim2) printf '\''9f5df79a03725ea7d10542fc2ce8224d90836560\n'\'' ;;' \
    '      vehicle_models) printf '\''3508bec2c3d77e0ff16f08794675d4f7b52479b7\n'\'' ;;' \
    '      state_lib) printf '\''ec83a141f188e8a4c39a381f4666485d8cc83e20\n'\'' ;;' \
    '      map_lib) printf '\''1919b36062850c9ba4553d1833a9b517c61c2e86\n'\'' ;;' \
    '      eufs_msgs) printf '\''9e918686c9e9292c613f321e6fd85e3a5d87cd87\n'\'' ;;' \
    '      eufs_gmock_matchers) printf '\''7ef83d030746c6a31bcf4f888d4121fcf4b7e8a9\n'\'' ;;' \
    '      eufs_logger) printf '\''375ea1d8f8885af66809129e444624ba13353fa7\n'\'' ;;' \
    '      open_car_dynamics) printf '\''94f8fb187fb0ed22bba1d809bd74f66d1ff75af4\n'\'' ;;' \
    '    esac' \
    '    ;;' \
    '  remote)' \
    '    case "${2:-}" in' \
    '      "") if [[ -f "$FSAI_GIT_PREPARED" ]]; then printf '\''upstream\n'\''; else printf '\''origin\n'\''; fi ;;' \
    '      get-url)' \
    '        if [[ "${3:-}" == "--push" ]]; then printf '\''DISABLED\n'\''; else printf '\''https://gitlab.com/eufs/public/eufs_sim2.git\n'\''; fi' \
    '        ;;' \
    '      rename) : >"$FSAI_GIT_PREPARED" ;;' \
    '      set-url) exit 0 ;;' \
    '    esac' \
    '    ;;' \
    'esac' \
    >"$fake_bin/git"
  chmod +x "$fake_bin/git"
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
    TMPDIR="$tmp_dir" \
    FSAI_COMMAND_LOG="$command_log" \
    "$bootstrap"
)
[[ ! -s "$command_log" ]] || fail 'bootstrap invoked a mutating command on a non-Ubuntu host'

ubuntu_os_release="$tmp_dir/ubuntu-os-release"
system_root="$tmp_dir/system"
ros_keyring="$system_root/usr/share/keyrings/ros2-archive-keyring.gpg"
apt_sources_dir="$system_root/etc/apt/sources.list.d"
rosdep_sources="$system_root/etc/ros/rosdep/sources.list.d/20-default.list"
simulator_src="$tmp_dir/simulator-src"
git_prepared="$tmp_dir/git-prepared"
printf 'ID=ubuntu\nVERSION_ID="22.04"\n' >"$ubuntu_os_release"
mkdir -p "$(dirname "$ros_keyring")" "$apt_sources_dir" "$(dirname "$rosdep_sources")" "$simulator_src"
make_bootstrap_fakes "$fake_bin"

: >"$command_log"
(
  cd "$tmp_dir"
  env \
    PATH="$fake_bin:$PATH" \
    TMPDIR="$tmp_dir" \
    FSAI_COMMAND_LOG="$command_log" \
    FSAI_OS_RELEASE_FILE="$ubuntu_os_release" \
    FSAI_ROS_KEYRING="$ros_keyring" \
    FSAI_APT_SOURCES_DIR="$apt_sources_dir" \
    FSAI_ROSDEP_SOURCES="$rosdep_sources" \
    FSAI_SIMULATOR_SRC="$simulator_src" \
    FSAI_BOOTSTRAP_TMP="$tmp_dir" \
    FSAI_GIT_PREPARED="$git_prepared" \
    "$bootstrap"
)
assert_log_equals "sudo apt-get update
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} software-properties-common
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} curl
sudo apt-get install --yes software-properties-common curl
sudo add-apt-repository --yes universe
sudo apt-get update
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} ros2-apt-source
curl --fail --silent --show-error https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest
mktemp $tmp_dir/ros2-apt-source.XXXXXX
curl --fail --silent --show-error --location --output $tmp_dir/ros2-apt-source.fixture https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.3/ros2-apt-source_1.2.3.jammy_all.deb
sudo dpkg --install $tmp_dir/ros2-apt-source.fixture
sudo apt-get update
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} ros-humble-desktop
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-vcstool
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-rosdep
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-colcon-common-extensions
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} clang-tidy
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} lcov
sudo apt-get install --yes ros-humble-desktop python3-vcstool python3-rosdep python3-colcon-common-extensions clang-tidy lcov
vcs import --skip-existing $simulator_src
git -C $simulator_src/eufs_sim2 status --porcelain
git -C $simulator_src/eufs_sim2 rev-parse HEAD
git -C $simulator_src/vehicle_models status --porcelain
git -C $simulator_src/vehicle_models rev-parse HEAD
git -C $simulator_src/state_lib status --porcelain
git -C $simulator_src/state_lib rev-parse HEAD
git -C $simulator_src/map_lib status --porcelain
git -C $simulator_src/map_lib rev-parse HEAD
git -C $simulator_src/eufs_msgs status --porcelain
git -C $simulator_src/eufs_msgs rev-parse HEAD
git -C $simulator_src/eufs_gmock_matchers status --porcelain
git -C $simulator_src/eufs_gmock_matchers rev-parse HEAD
git -C $simulator_src/eufs_logger status --porcelain
git -C $simulator_src/eufs_logger rev-parse HEAD
git -C $simulator_src/open_car_dynamics status --porcelain
git -C $simulator_src/open_car_dynamics rev-parse HEAD
git -C $simulator_src/eufs_sim2 rev-parse --is-inside-work-tree
git -C $simulator_src/eufs_sim2 rev-parse --is-inside-work-tree
git -C $simulator_src/eufs_sim2 status --porcelain
git -C $simulator_src/eufs_sim2 rev-parse HEAD
git -C $simulator_src/eufs_sim2 remote
git -C $simulator_src/eufs_sim2 remote get-url origin
git -C $simulator_src/eufs_sim2 remote rename origin upstream
git -C $simulator_src/eufs_sim2 remote set-url --push upstream DISABLED
sudo rosdep init
rosdep update
rosdep install --from-paths $simulator_src --ignore-src --recursive --yes"

: >"$command_log"
(
  cd "$tmp_dir"
  env \
    PATH="$fake_bin:$PATH" \
    FSAI_COMMAND_LOG="$command_log" \
    FSAI_OS_RELEASE_FILE="$ubuntu_os_release" \
    FSAI_ROS_KEYRING="$ros_keyring" \
    FSAI_APT_SOURCES_DIR="$apt_sources_dir" \
    FSAI_ROSDEP_SOURCES="$rosdep_sources" \
    FSAI_SIMULATOR_SRC="$simulator_src" \
    FSAI_BOOTSTRAP_TMP="$tmp_dir" \
    FSAI_BOOTSTRAP_REPEAT=true \
    FSAI_GIT_PREPARED="$git_prepared" \
    "$bootstrap"
)
assert_log_equals "sudo apt-get update
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} software-properties-common
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} curl
sudo add-apt-repository --yes universe
sudo apt-get update
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} ros2-apt-source
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} ros-humble-desktop
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-vcstool
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-rosdep
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} python3-colcon-common-extensions
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} clang-tidy
dpkg-query --show --showformat=\\$\\{db:Status-Status\\} lcov
vcs import --skip-existing $simulator_src
git -C $simulator_src/eufs_sim2 status --porcelain
git -C $simulator_src/eufs_sim2 rev-parse HEAD
git -C $simulator_src/vehicle_models status --porcelain
git -C $simulator_src/vehicle_models rev-parse HEAD
git -C $simulator_src/state_lib status --porcelain
git -C $simulator_src/state_lib rev-parse HEAD
git -C $simulator_src/map_lib status --porcelain
git -C $simulator_src/map_lib rev-parse HEAD
git -C $simulator_src/eufs_msgs status --porcelain
git -C $simulator_src/eufs_msgs rev-parse HEAD
git -C $simulator_src/eufs_gmock_matchers status --porcelain
git -C $simulator_src/eufs_gmock_matchers rev-parse HEAD
git -C $simulator_src/eufs_logger status --porcelain
git -C $simulator_src/eufs_logger rev-parse HEAD
git -C $simulator_src/open_car_dynamics status --porcelain
git -C $simulator_src/open_car_dynamics rev-parse HEAD
git -C $simulator_src/eufs_sim2 rev-parse --is-inside-work-tree
git -C $simulator_src/eufs_sim2 rev-parse --is-inside-work-tree
git -C $simulator_src/eufs_sim2 status --porcelain
git -C $simulator_src/eufs_sim2 rev-parse HEAD
git -C $simulator_src/eufs_sim2 remote
git -C $simulator_src/eufs_sim2 remote get-url upstream
git -C $simulator_src/eufs_sim2 remote get-url --push --all upstream
git -C $simulator_src/eufs_sim2 remote get-url --push --all upstream
rosdep update
rosdep install --from-paths $simulator_src --ignore-src --recursive --yes"

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
