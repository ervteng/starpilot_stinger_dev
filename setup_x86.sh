#!/usr/bin/env bash
# Build an x86 Python environment that can run StarPilot's car and controls tests.
#
# The StarPilot tree ships prebuilt aarch64 (comma device) binaries, so this script builds the
# few native modules the tests import (msgq, params, pandad, long MPC, ...) for x86
# in a separate git worktree, leaving the main checkout untouched. Safe to re-run.
#
# Two independent toolchains, picked with STINGER_TOOLCHAIN (see toolchain.sh):
#   old (default)  daily StarPilot layout  -> $STINGER_DEV_HOME/x86     + $STINGER_DEV_HOME/venv   (pinned from requirements.txt)
#   sg             SecretGoodStarPilot     -> $STINGER_DEV_HOME/x86-sg  + $STINGER_DEV_HOME/venv-sg
#                  (venv from SG's own uv.lock/pyproject via `uv sync`, plus the requirements.txt extras it lacks)
#
# Env:
#   STINGER_DEV_HOME  where the venvs, x86 worktrees and logs live (default ~/stinger/starpilot-dev)
#   STINGER_TOOLCHAIN old|sg (default old)
#   STARPILOT_DIR     StarPilot checkout (default: ~/stinger/starpilot if present,
#                     otherwise a fresh clone of ervteng/StarPilot into $STINGER_DEV_HOME/StarPilot)
#   STARPILOT_REF     ref to check out in a new worktree (default stinger-radar-lfa; sg: upstream/SecretGoodStarPilot)
#   STARPILOT_BUILD   override the worktree path
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
. "$HERE/toolchain.sh"
REF=$TC_REF
mkdir -p "$DEV/logs"

log() { echo "[setup_x86:$TC] $*"; }

# 1. System headers for msgq (zmq, OpenCL) and transformations (Eigen). Old layout only: SG gets zmq/capnp/acados/json11 from comma-deps wheels.
if [ "$LAYOUT" = old ]; then
  pkgs=()
  [ -f /usr/include/zmq.h ] || pkgs+=(libzmq3-dev)
  [ -f /usr/include/CL/cl.h ] || pkgs+=(opencl-headers ocl-icd-opencl-dev)
  [ -d /usr/include/eigen3 ] || pkgs+=(libeigen3-dev)
  if [ ${#pkgs[@]} -gt 0 ]; then
    log "apt installing ${pkgs[*]}"
    SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO=sudo
    $SUDO apt-get update -qq
    DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y -qq "${pkgs[@]}"
  fi
fi

# 2. StarPilot checkout.
if [ -z "${STARPILOT_DIR:-}" ]; then
  for d in "$HOME/stinger/starpilot"; do
    if git -C "$d" rev-parse --git-dir >/dev/null 2>&1; then STARPILOT_DIR=$d; break; fi
  done
fi
if [ -z "${STARPILOT_DIR:-}" ]; then
  STARPILOT_DIR=$DEV/StarPilot
  if [ ! -d "$STARPILOT_DIR/.git" ]; then
    log "cloning ervteng/StarPilot into $STARPILOT_DIR"
    GIT_LFS_SKIP_SMUDGE=1 git clone --filter=blob:none --no-single-branch https://github.com/ervteng/StarPilot "$STARPILOT_DIR"
  fi
fi
log "StarPilot checkout: $STARPILOT_DIR"
if [ "$LAYOUT" = sg ] && [[ "$REF" == upstream/* ]]; then
  git -C "$STARPILOT_DIR" fetch -q upstream "${REF#upstream/}" 2>/dev/null || true
else
  git -C "$STARPILOT_DIR" fetch -q origin "$REF" 2>/dev/null || true
fi

# 3. x86 worktree (created before the sg venv, which installs editable packages from it).
if [ ! -e "$W/.git" ]; then
  log "adding worktree $W at $REF"
  rev=$(git -C "$STARPILOT_DIR" rev-parse --verify -q "origin/$REF" || git -C "$STARPILOT_DIR" rev-parse "$REF")
  git -C "$STARPILOT_DIR" worktree add -q --detach "$W" "$rev"
fi
# the worktree's own layout is now known
if [ -f "$W/openpilot/cereal/log.capnp" ]; then
  [ "$LAYOUT" = sg ] || { echo "$W holds the sg layout but STINGER_TOOLCHAIN=$TC; unset STARPILOT_BUILD or use the matching toolchain" >&2; exit 1; }
elif [ "$LAYOUT" = sg ]; then
  echo "$W is not a SecretGoodStarPilot tree (no openpilot/cereal/log.capnp)" >&2; exit 1
fi

# 4. Python venv (3.12 to match the device).
if [ ! -x "$PY" ]; then
  log "creating venv $VENV"
  uv venv -q -p 3.12 "$VENV"
fi
if [ "$LAYOUT" = old ]; then
  uv pip install -q -p "$PY" -r "$HERE/requirements.txt"
else
  # SG's own lock (pytest<9, numpy 2.5, comma-deps wheels incl. acados; editable opendbc/panda/rednose/openpilot from $W) ...
  (cd "$W" && UV_PROJECT_ENVIRONMENT="$VENV" uv sync -q --frozen --extra dev --python-preference system -p 3.12)
  # ... plus the SCons-only comma-deps (ncurses, libusb) the SConstruct imports, at the lock's versions ...
  for pkg in comma-deps-ncurses comma-deps-libusb; do
    ver=$(awk -v n="$pkg" '$0=="name = \"" n "\""{getline; gsub(/[^0-9a-z.]/,"",$3); print $3; exit}' "$W/uv.lock")
    uv pip install -q -p "$PY" "$pkg${ver:+==$ver}"
  done
  # ... plus whatever requirements.txt lists that the venv lacks (casadi for long_mpc codegen, hypothesis, parameterized, ...).
  # Packages SG already pins are skipped so requirements.txt cannot downgrade/upgrade them.
  extras=$(mktemp)
  "$PY" - "$HERE/requirements.txt" "$extras" <<'PYEOF'
import re, sys
from importlib import metadata
have = {re.sub(r"[-_.]+", "-", d.metadata["Name"]).lower() for d in metadata.distributions()}
keep = [l for l in open(sys.argv[1]) if l.strip() and re.sub(r"[-_.]+", "-", re.split(r"[=<>~ ]", l.strip())[0]).lower() not in have]
open(sys.argv[2], "w").writelines(keep)
PYEOF
  [ -s "$extras" ] && uv pip install -q -p "$PY" -r "$extras"
  rm -f "$extras"
fi

# 5. Native builds.
if file "$W/msgq_repo/msgq/ipc_pyx.so" 2>/dev/null | grep -q x86-64 && file "$TC_MPC_SO" 2>/dev/null | grep -q x86-64; then
  log "x86 builds already present"
else
  log "building native modules for x86 (a few minutes)"
  (cd "$W" && PATH="$VENV/bin:$PATH" scons -j"$(nproc)" --minimal "${TC_TARGETS[@]}" > "$DEV/${W##*/}_build.log" 2>&1) \
    || { tail -20 "$DEV/${W##*/}_build.log"; exit 1; }
fi
# Ubuntu 26.04's glibc refuses libraries that request an executable stack (the old tree's bundled libblasfeo does).
if [ -f "$TC_BLASFEO" ] && readelf -lW "$TC_BLASFEO" 2>/dev/null | grep GNU_STACK | grep -q 'RWE'; then
  log "clearing execstack on $TC_BLASFEO"
  patchelf --clear-execstack "$TC_BLASFEO"
fi
# The builds overwrite tracked aarch64 binaries; hide them from git so run_tests.sh can switch refs.
modified=$(git -C "$W" ls-files -m)
[ -n "$modified" ] && git -C "$W" update-index --skip-worktree $modified

log "done. Try: STINGER_TOOLCHAIN=$TC $HERE/run_tests.sh $REF ${TC_TESTS[0]}"
