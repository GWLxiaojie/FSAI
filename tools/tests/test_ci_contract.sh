#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

grep -q "ubuntu-22.04" "$repo_root/.github/workflows/humble.yml" \
  || fail "humble workflow must use ubuntu-22.04"
grep -q "ROS_DISTRO: humble" "$repo_root/.github/workflows/humble.yml" \
  || fail "humble workflow must set ROS_DISTRO humble"
grep -q "ubuntu-24.04" "$repo_root/.github/workflows/jazzy.yml" \
  || fail "jazzy workflow must use ubuntu-24.04"
grep -q "ROS_DISTRO: jazzy" "$repo_root/.github/workflows/jazzy.yml" \
  || fail "jazzy workflow must set ROS_DISTRO jazzy"
grep -q "contents: read" "$repo_root/.github/workflows/humble.yml" \
  || fail "humble workflow must be read-only"
grep -q "prepare_eufs_checkout.sh" "$repo_root/tools/ci_humble.sh" \
  || fail "humble CI must call prepare_eufs_checkout.sh"

printf 'PASS: CI contract\n'
