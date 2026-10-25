#!/usr/bin/env bash
# Run a python script (or -m module) with the selected toolchain's venv: STINGER_TOOLCHAIN=old|sg run_py.sh analysis/foo.py args...
HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/toolchain.sh"
[ -x "$PY" ] || { echo "no venv at $VENV; run setup_x86.sh (STINGER_TOOLCHAIN=$TC) first" >&2; exit 1; }
exec "$PY" "$@"
