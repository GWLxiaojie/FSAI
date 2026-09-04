#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
src_dir="$repo_root/simulator/src"
repos_file="$repo_root/simulator/fsai_sim.repos"
lock_file="$repo_root/simulator/dependencies.lock.yaml"
ros_source_package="ros2-apt-source"
rosdep_sources="/etc/ros/rosdep/sources.list.d/20-default.list"

usage() {
  printf 'Usage: %s [--help]\n' "${0##*/}"
}

fail() {
  printf 'bootstrap_ubuntu: %s\n' "$*" >&2
  exit 1
}

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  usage
  exit 0
fi
if [[ "$#" -ne 0 ]]; then
  usage >&2
  exit 2
fi

if [[ ! -r /etc/os-release ]]; then
  fail 'requires Ubuntu 22.04'
fi

# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  fail 'requires Ubuntu 22.04'
fi

package_is_installed() {
  dpkg-query --show --showformat='${db:Status-Status}' "$1" 2>/dev/null \
    | grep -qx 'installed'
}

ensure_apt_packages() {
  local missing=()
  local package
  for package in "$@"; do
    if ! package_is_installed "$package"; then
      missing+=("$package")
    fi
  done

  if [[ "${#missing[@]}" -gt 0 ]]; then
    sudo apt-get install --yes "${missing[@]}"
  fi
}

ros_source_is_configured() {
  grep --recursive --quiet \
    --include='*.list' \
    --include='*.sources' \
    'packages.ros.org/ros2/ubuntu' \
    /etc/apt/sources.list.d
}

ros_key_is_configured() {
  [[ -f /usr/share/keyrings/ros-archive-keyring.gpg ]] \
    || [[ -f /usr/share/keyrings/ros2-archive-keyring.gpg ]]
}

sudo apt-get update
ensure_apt_packages software-properties-common curl
sudo add-apt-repository --yes universe
sudo apt-get update

if ! ros_source_is_configured || ! ros_key_is_configured \
  || ! package_is_installed "$ros_source_package"; then
  ros_apt_source_version="$(
    curl --fail --silent --show-error \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | awk -F'"' '/"tag_name"/ { print $4; exit }'
  )"
  [[ -n "$ros_apt_source_version" ]] || fail 'could not determine ros2-apt-source version'

  ros_source_deb="$(mktemp "${TMPDIR:-/tmp}/ros2-apt-source.XXXXXX.deb")"
  trap 'rm -f "$ros_source_deb"' EXIT
  curl --fail --silent --show-error --location \
    --output "$ros_source_deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.jammy_all.deb"
  sudo dpkg --install "$ros_source_deb"
  ros_source_is_configured || fail 'ros2-apt-source did not configure a ROS 2 apt source'
  ros_key_is_configured || fail 'ros2-apt-source did not install a ROS 2 apt keyring'
  sudo apt-get update
fi

ensure_apt_packages \
  ros-humble-desktop \
  python3-vcstool \
  python3-rosdep \
  python3-colcon-common-extensions \
  clang-tidy \
  lcov

mkdir -p "$src_dir"
vcs import --skip-existing "$src_dir" < "$repos_file"

verify_locked_checkouts() {
  local name revision checkout
  while IFS=$'\t' read -r name revision; do
    checkout="$src_dir/$name"
    [[ -d "$checkout/.git" ]] || fail "missing imported repository: $name"
    [[ -z "$(git -C "$checkout" status --porcelain)" ]] \
      || fail "imported repository is dirty: $name"
    [[ "$(git -C "$checkout" rev-parse HEAD)" == "$revision" ]] \
      || fail "imported repository is not at locked revision: $name"
  done < <(
    awk '
      /^dependencies:/ { in_dependencies = 1; next }
      in_dependencies && /^  [[:alnum:]_]+:$/ { name = $1; sub(/:$/, "", name); next }
      in_dependencies && /^    revision: / { print name "\t" $2 }
    ' "$lock_file"
  )
}

verify_locked_checkouts
"$script_dir/prepare_eufs_checkout.sh" "$src_dir/eufs_sim2"

if [[ ! -f "$rosdep_sources" ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths "$src_dir" --ignore-src --recursive --yes
