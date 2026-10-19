#!/usr/bin/env bash
# Run StarPilot python tests on x86 against a git ref, using the env from setup_x86.sh.
# Usage: run_tests.sh [ref] [pytest args...]
#   ref defaults to stinger-radar-lfa; tests default to the Hyundai car tests + controls tests.
set -euo pipefail
DEV=${STINGER_DEV_HOME:-/home/user/starpilot-dev}
W=$DEV/x86
REF=${1:-stinger-radar-lfa}; shift || true

[ -e "$W/.git" ] || { echo "no x86 worktree at $W; run setup_x86.sh first" >&2; exit 1; }
rev=$(git -C "$W" rev-parse --verify -q "$REF^{commit}" || git -C "$W" rev-parse "origin/$REF")
git -C "$W" checkout -q --detach "$rev"
echo "[run_tests] $REF -> $(git -C "$W" log -1 --format='%h %s')"

cd "$W"
export PYTHONPATH=$W:$W/opendbc_repo
TESTS=("$@")
[ ${#TESTS[@]} -eq 0 ] && TESTS=(opendbc_repo/opendbc/car/hyundai/tests selfdrive/controls/tests)
exec "$DEV/venv/bin/python" -m pytest -q -n 8 -p no:cacheprovider "${TESTS[@]}"
