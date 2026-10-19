#!/usr/bin/env bash
# Build an x86 Python environment that can run StarPilot's car and controls tests.
#
# The StarPilot tree ships prebuilt aarch64 (comma device) binaries, so this script builds the
# few native modules the tests import (msgq, params, transformations, pandad, long MPC) for x86
# in a separate git worktree, leaving the main checkout untouched. Safe to re-run.
#
# Env:
#   STINGER_DEV_HOME  where the venv, x86 worktree and logs live (default /home/user/starpilot-dev)
#   STARPILOT_DIR     StarPilot checkout (default: first of /home/user/starpilot, /home/user/ervteng/starpilot,
#                     otherwise a fresh clone of ervteng/StarPilot into $STINGER_DEV_HOME/StarPilot)
#   STARPILOT_REF     ref to check out in the x86 worktree (default stinger-radar-lfa)
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
DEV=${STINGER_DEV_HOME:-/home/user/starpilot-dev}
REF=${STARPILOT_REF:-stinger-radar-lfa}
mkdir -p "$DEV/logs"

log() { echo "[setup_x86] $*"; }

# 1. System headers for msgq (zmq, OpenCL) and transformations (Eigen).
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

# 2. StarPilot checkout.
if [ -z "${STARPILOT_DIR:-}" ]; then
  for d in /home/user/starpilot /home/user/ervteng/starpilot; do
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
git -C "$STARPILOT_DIR" fetch -q origin "$REF" 2>/dev/null || true

# 3. Python venv (3.12 to match the device), pinned from requirements.txt.
if [ ! -x "$DEV/venv/bin/python" ]; then
  log "creating venv"
  uv venv -q -p 3.12 "$DEV/venv"
fi
uv pip install -q -p "$DEV/venv/bin/python" -r "$HERE/requirements.txt"

# 4. x86 worktree with native builds.
W=$DEV/x86
if [ ! -e "$W/.git" ]; then
  log "adding worktree $W at $REF"
  rev=$(git -C "$STARPILOT_DIR" rev-parse --verify -q "origin/$REF" || git -C "$STARPILOT_DIR" rev-parse "$REF")
  git -C "$STARPILOT_DIR" worktree add -q --detach "$W" "$rev"
fi
targets=(
  common/params_pyx.so
  common/transformations/transformations.so
  msgq_repo/msgq/ipc_pyx.so
  msgq_repo/msgq/visionipc/visionipc_pyx.so
  selfdrive/pandad/pandad_api_impl.so
  selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so
)
if file "$W/msgq_repo/msgq/ipc_pyx.so" 2>/dev/null | grep -q x86-64 && \
   file "$W/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so" | grep -q x86-64; then
  log "x86 builds already present"
else
  log "building native modules for x86 (a few minutes)"
  (cd "$W" && PATH="$DEV/venv/bin:$PATH" scons -j"$(nproc)" --minimal "${targets[@]}" > "$DEV/x86_build.log" 2>&1) \
    || { tail -20 "$DEV/x86_build.log"; exit 1; }
fi
# The builds overwrite tracked aarch64 binaries; hide them from git so run_tests.sh can switch refs.
modified=$(git -C "$W" ls-files -m)
[ -n "$modified" ] && git -C "$W" update-index --skip-worktree $modified

log "done. Try: $HERE/run_tests.sh $REF opendbc_repo/opendbc/car/hyundai/tests -k stinger"
