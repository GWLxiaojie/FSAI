#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
manifest="$project_root/simulator/fsai_sim.repos"
guard="$project_root/tools/prepare_eufs_checkout.sh"
expected_url="https://gitlab.com/eufs/public/eufs_sim2.git"
expected_revision="9f5df79a03725ea7d10542fc2ce8224d90836560"

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

expect_failure() {
  if "$@"; then
    fail "expected failure: $*"
  fi
}

vcs validate < "$manifest"

python3 - "$manifest" <<'PY'
import sys
import yaml

expected = {
    "eufs_sim2": ("https://gitlab.com/eufs/public/eufs_sim2.git", "9f5df79a03725ea7d10542fc2ce8224d90836560"),
    "vehicle_models": ("https://gitlab.com/eufs/public/vehicle_models.git", "3508bec2c3d77e0ff16f08794675d4f7b52479b7"),
    "state_lib": ("https://gitlab.com/eufs/public/state_lib.git", "ec83a141f188e8a4c39a381f4666485d8cc83e20"),
    "map_lib": ("https://gitlab.com/eufs/public/map_lib.git", "1919b36062850c9ba4553d1833a9b517c61c2e86"),
    "eufs_msgs": ("https://gitlab.com/eufs/public/eufs_msgs.git", "9e918686c9e9292c613f321e6fd85e3a5d87cd87"),
    "eufs_gmock_matchers": ("https://gitlab.com/eufs/public/eufs-gmock-matchers.git", "7ef83d030746c6a31bcf4f888d4121fcf4b7e8a9"),
    "eufs_logger": ("https://gitlab.com/eufs/public/eufs-logger.git", "375ea1d8f8885af66809129e444624ba13353fa7"),
    "open_car_dynamics": ("https://github.com/TUMFTM/Open-Car-Dynamics.git", "94f8fb187fb0ed22bba1d809bd74f66d1ff75af4"),
}
with open(sys.argv[1], encoding="utf-8") as manifest_file:
    manifest = yaml.safe_load(manifest_file)
repositories = manifest.get("repositories") if isinstance(manifest, dict) else None
if not isinstance(repositories, dict) or set(repositories) != set(expected):
    raise SystemExit("manifest repositories do not exactly match the dependency contract")
for name, (url, revision) in expected.items():
    repository = repositories[name]
    if repository != {"type": "git", "url": url, "version": revision}:
        raise SystemExit(f"invalid manifest entry for {name}: {repository!r}")
PY

test -x "$guard"
for ignored_path in \
  simulator/src/eufs_sim2/placeholder \
  simulator/src/vehicle_models/placeholder \
  simulator/src/state_lib/placeholder \
  simulator/src/map_lib/placeholder \
  simulator/src/eufs_msgs/placeholder \
  simulator/src/eufs_gmock_matchers/placeholder \
  simulator/src/eufs_logger/placeholder \
  simulator/src/open_car_dynamics/placeholder \
  build/placeholder \
  install/placeholder \
  log/placeholder \
  artifacts/placeholder \
  recording.mcap; do
  git -C "$project_root" check-ignore -q "$ignored_path" || fail "not ignored: $ignored_path"
done

test_dir="$(mktemp -d "${TMPDIR:-/tmp}/fsai-dependency-contract.XXXXXX")"
trap 'rm -rf "$test_dir"' EXIT

git clone --quiet --no-checkout "$expected_url" "$test_dir/source"
git -C "$test_dir/source" checkout --quiet "$expected_revision"

"$guard" "$test_dir/source"
test "$(git -C "$test_dir/source" remote)" = "upstream" || fail "valid checkout retained an unexpected remote"
test "$(git -C "$test_dir/source" remote get-url upstream)" = "$expected_url" || fail "valid checkout changed fetch URL"
test "$(git -C "$test_dir/source" remote get-url --push upstream)" = "DISABLED" || fail "valid checkout permits pushes"
"$guard" "$test_dir/source"

expect_failure "$guard"
expect_failure "$guard" "$test_dir/source" extra

git clone --quiet "$test_dir/source" "$test_dir/wrong-url"
git -C "$test_dir/wrong-url" remote set-url origin https://example.invalid/eufs_sim2.git
expect_failure "$guard" "$test_dir/wrong-url"
test "$(git -C "$test_dir/wrong-url" remote)" = "origin" || fail "wrong URL renamed origin"

git clone --quiet "$test_dir/source" "$test_dir/wrong-revision"
git -C "$test_dir/wrong-revision" remote set-url origin "$expected_url"
git -C "$test_dir/wrong-revision" -c user.name='Contract Test' -c user.email='contract@example.invalid' commit --allow-empty --quiet -m 'wrong revision fixture'
expect_failure "$guard" "$test_dir/wrong-revision"
test "$(git -C "$test_dir/wrong-revision" remote)" = "origin" || fail "wrong revision renamed origin"

git clone --quiet "$test_dir/source" "$test_dir/unprepared-upstream"
git -C "$test_dir/unprepared-upstream" remote set-url origin "$expected_url"
git -C "$test_dir/unprepared-upstream" remote rename origin upstream
expect_failure "$guard" "$test_dir/unprepared-upstream"
test "$(git -C "$test_dir/unprepared-upstream" remote get-url --push upstream)" = "$expected_url" || fail "unprepared upstream changed unexpectedly"

git clone --quiet "$test_dir/source" "$test_dir/multiple-push-urls"
git -C "$test_dir/multiple-push-urls" remote set-url origin "$expected_url"
git -C "$test_dir/multiple-push-urls" remote rename origin upstream
git -C "$test_dir/multiple-push-urls" remote set-url --push upstream DISABLED
git -C "$test_dir/multiple-push-urls" remote set-url --add --push upstream https://example.invalid/live-push.git
expect_failure "$guard" "$test_dir/multiple-push-urls"
test "$(git -C "$test_dir/multiple-push-urls" remote get-url --push --all upstream | wc -l | tr -d ' ')" = "2" || fail "multiple push URLs changed unexpectedly"

git clone --quiet "$test_dir/source" "$test_dir/dirty"
git -C "$test_dir/dirty" remote set-url origin "$expected_url"
touch "$test_dir/dirty/uncommitted-change"
expect_failure "$guard" "$test_dir/dirty"
test "$(git -C "$test_dir/dirty" remote)" = "origin" || fail "dirty checkout renamed origin"

printf 'dependency contract passed\n'
