#!/usr/bin/env bash
# Run StarPilot python tests on x86 against a git ref, using the env from setup_x86.sh.
# Usage: [STINGER_TOOLCHAIN=old|sg] run_tests.sh [ref] [pytest args...]
#   ref defaults to stinger-radar-lfa (sg: upstream/SecretGoodStarPilot); tests default to the Hyundai car tests + controls tests.
#   Test paths are relative to the build tree, so they follow its layout: old selfdrive/controls/tests/...,
#   sg openpilot/selfdrive/controls/tests/...; opendbc_repo/opendbc/car/hyundai/tests is the same in both.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/toolchain.sh"

[ -e "$W/.git" ] || { echo "no x86 worktree at $W; run setup_x86.sh first (STINGER_TOOLCHAIN=$TC)" >&2; exit 1; }
REF=${1:-$TC_DEFAULT_REF}; shift || true
rev=$(git -C "$W" rev-parse --verify -q "$REF^{commit}" || git -C "$W" rev-parse "origin/$REF")
git -C "$W" checkout -q --detach "$rev"
echo "[run_tests:$TC] $REF -> $(git -C "$W" log -1 --format='%h %s')"
tc_restore_acados_libs

cd "$W"
export PYTHONPATH=$TC_PYTHONPATH
TESTS=("$@")
[ ${#TESTS[@]} -eq 0 ] && TESTS=("${TC_TESTS[@]}")
# 8 xdist workers unless the caller passes its own -n (e.g. -n 0: pytest-subtests with enum/CAR params cannot cross xdist workers in sg)
NJ=(-n 8)
for a in "${TESTS[@]}"; do case "$a" in -n|-n[0-9]*|--numprocesses|--numprocesses=*) NJ=();; esac; done
exec "$PY" -m pytest -q "${NJ[@]}" -p no:cacheprovider "${TESTS[@]}"
