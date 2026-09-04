#!/usr/bin/env bash
set -euo pipefail

expected_url="https://gitlab.com/eufs/public/eufs_sim2.git"
expected_revision="9f5df79a03725ea7d10542fc2ce8224d90836560"

fail() {
  printf 'prepare_eufs_checkout: %s\n' "$*" >&2
  exit 1
}

if [[ "$#" -ne 1 ]]; then
  fail "usage: $0 CHECKOUT_PATH"
fi

checkout_path="$1"
git -C "$checkout_path" rev-parse --is-inside-work-tree >/dev/null 2>&1 \
  || fail "not a Git checkout: $checkout_path"
test "$(git -C "$checkout_path" rev-parse --is-inside-work-tree)" = "true" \
  || fail "not a Git work tree: $checkout_path"

test -z "$(git -C "$checkout_path" status --porcelain)" \
  || fail "checkout has uncommitted changes"
test "$(git -C "$checkout_path" rev-parse HEAD)" = "$expected_revision" \
  || fail "HEAD is not the required EUFS revision"

remotes="$(git -C "$checkout_path" remote)"
if test "$remotes" = "upstream"; then
  test "$(git -C "$checkout_path" remote get-url upstream)" = "$expected_url" \
    || fail "upstream fetch URL is not the required EUFS URL"
  test "$(git -C "$checkout_path" remote get-url --push upstream)" = "DISABLED" \
    || fail "upstream remote has not been prepared for read-only use"
  exit 0
fi

test "$remotes" = "origin" || fail "checkout must have only an origin remote before preparation"
test "$(git -C "$checkout_path" remote get-url origin)" = "$expected_url" \
  || fail "origin fetch URL is not the required EUFS URL"

git -C "$checkout_path" remote rename origin upstream
git -C "$checkout_path" remote set-url --push upstream DISABLED
