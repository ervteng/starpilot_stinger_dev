#!/usr/bin/env bash
# Run StarPilot python tests on x86 against a git ref, using the env from setup_x86.sh.
# Usage: [STINGER_TOOLCHAIN=sg|old] run_tests.sh [ref] [pytest args...]
#   ref defaults to sg-stinger-combined (old: archive/stinger-combined-final); tests default to the Hyundai car tests + controls tests.
#   Test paths are relative to the build tree, so they follow its layout: old selfdrive/controls/tests/...,
#   sg openpilot/selfdrive/controls/tests/...; opendbc_repo/opendbc/car/hyundai/tests is the same in both.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/toolchain.sh"

[ -e "$W/.git" ] || { echo "no x86 worktree at $W; run setup_x86.sh first (STINGER_TOOLCHAIN=$TC)" >&2; exit 1; }
REF=${1:-$TC_DEFAULT_REF}; shift || true
rev=$(git -C "$W" rev-parse --verify -q "$REF^{commit}" || git -C "$W" rev-parse "origin/$REF")
# setup_x86.sh replaces the repo's aarch64 prebuilt binaries with x86 builds and hides them with skip-worktree. When the
# target ref changes any tracked .so/.a (e.g. libparams_c.so after a params_keys.h change), a plain checkout refuses
# or brings back an aarch64 copy, and our x86 copy would be stale anyway. So: put the repo copies back, check out,
# delete those files (SCons won't rebuild a target replaced behind its back) and rebuild the native targets.
changed=$(git -C "$W" diff --name-only HEAD "$rev")
skipped=$(git -C "$W" ls-files -v | awk '/^S /{print $2}')
rebuild=$( (comm -12 <(sort <<<"$skipped") <(sort <<<"$changed"); grep -E '\.(so|a)$' <<<"$changed" || true) | sort -u | sed '/^$/d')
if [ -n "$rebuild" ]; then
  echo "[run_tests:$TC] $REF changes native binaries; rebuilding for x86:" $rebuild >&2
  in_head=$(while read -r f; do git -C "$W" cat-file -e "HEAD:$f" 2>/dev/null && echo "$f"; done <<<"$rebuild")
  # shellcheck disable=SC2086
  [ -n "$in_head" ] && git -C "$W" update-index --no-skip-worktree $in_head && git -C "$W" checkout -q HEAD -- $in_head
fi
git -C "$W" checkout -q --detach "$rev"
echo "[run_tests:$TC] $REF -> $(git -C "$W" log -1 --format='%h %s')"
if [ -n "$rebuild" ]; then
  tc_rebuild_natives $rebuild
fi
tc_restore_acados_libs

cd "$W"
export PYTHONPATH=$TC_PYTHONPATH
TESTS=("$@")
[ ${#TESTS[@]} -eq 0 ] && TESTS=("${TC_TESTS[@]}")
# 8 xdist workers unless the caller passes its own -n (e.g. -n 0: pytest-subtests with enum/CAR params cannot cross xdist workers in sg)
NJ=(-n 8)
for a in "${TESTS[@]}"; do case "$a" in -n|-n[0-9]*|--numprocesses|--numprocesses=*) NJ=();; esac; done
exec "$PY" -m pytest -q "${NJ[@]}" -p no:cacheprovider "${TESTS[@]}"
