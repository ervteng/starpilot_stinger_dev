#!/usr/bin/env bash
# Toolchain selector, sourced by setup_x86.sh / run_tests.sh / run_py.sh. Python twin: analysis/toolchain.py (keep in sync).
#
#   STINGER_TOOLCHAIN=old|sg   old (default): the daily StarPilot layout (code at the repo root), built in $DEV/x86 with $DEV/venv
#                              sg: the SecretGoodStarPilot layout (code under openpilot/), built in $DEV/x86-sg with $DEV/venv-sg
#   STARPILOT_BUILD            override the build dir (default $DEV/x86 or $DEV/x86-sg)
#   STARPILOT_DIR              StarPilot clone the worktree is created from (setup_x86.sh) / schema fallback
#   STARPILOT_REF              ref to check out in a NEW worktree (default per toolchain below)
#
# Sets: DEV TC TC_DEFAULT_REF (ref run_tests.sh uses when none is given) TC_REF (setup_x86.sh: STARPILOT_REF or the default) (old|sg) W (build dir) VENV PY (venv python) LAYOUT (old|sg, auto-detected from the build tree:
# openpilot/cereal/log.capnp present => sg) TC_REF TC_PYTHONPATH TC_TESTS TC_TARGETS (scons targets, relative to W).
DEV=${STINGER_DEV_HOME:-$HOME/stinger/starpilot-dev}
TC=${STINGER_TOOLCHAIN:-old}
case "$TC" in
  old) _tc_dir=x86;    _tc_venv=venv;    TC_DEFAULT_REF=stinger-radar-lfa ;;
  sg)  _tc_dir=x86-sg; _tc_venv=venv-sg; TC_DEFAULT_REF=upstream/SecretGoodStarPilot ;;
  *) echo "STINGER_TOOLCHAIN must be old or sg (got '$TC')" >&2; return 2 2>/dev/null || exit 2 ;;
esac
TC_REF=${STARPILOT_REF:-$TC_DEFAULT_REF}
W=${STARPILOT_BUILD:-$DEV/$_tc_dir}
VENV=$DEV/$_tc_venv
PY=$VENV/bin/python
# (the old layout also has an openpilot/ dir of symlinks to the root dirs, so key on the cereal schema, which only sg keeps there)
if [ -f "$W/openpilot/cereal/log.capnp" ]; then LAYOUT=sg
elif [ -d "$W/selfdrive" ]; then LAYOUT=old
else LAYOUT=$TC; fi   # build dir not created yet (setup_x86.sh): the toolchain's own layout

if [ "$LAYOUT" = sg ]; then
  # SG's own dev-mode install (pyproject [tool.hatch...] dev-mode-dirs) puts the repo root and msgq_repo on the path; opendbc,
  # rednose, panda are editable installs of the sibling dirs.
  TC_PYTHONPATH=$W:$W/msgq_repo:$W/opendbc_repo:$W/rednose_repo:$W/panda
  TC_TESTS=(openpilot/selfdrive/controls/tests opendbc_repo/opendbc/car/hyundai/tests)
  TC_TARGETS=(
    openpilot/common/libparams_c.so
    msgq_repo/msgq/ipc_pyx.so
    msgq_repo/msgq/visionipc/visionipc_pyx.so
    opendbc_repo/opendbc/can/_raw_value.so
    openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so
  )
  _tc_mpc=$W/openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code
  _tc_blasfeo=$_tc_mpc/libblasfeo.so
else
  TC_PYTHONPATH=$W:$W/opendbc_repo
  TC_TESTS=(opendbc_repo/opendbc/car/hyundai/tests selfdrive/controls/tests)
  TC_TARGETS=(
    common/params_pyx.so
    common/transformations/transformations.so
    msgq_repo/msgq/ipc_pyx.so
    msgq_repo/msgq/visionipc/visionipc_pyx.so
    selfdrive/pandad/pandad_api_impl.so
    selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/acados_ocp_solver_pyx.so
  )
  _tc_mpc=$W/selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code
  _tc_blasfeo=$W/third_party/acados/x86_64/lib/libblasfeo.so
fi
TC_MPC_SO=$_tc_mpc/acados_ocp_solver_pyx.so
TC_BLASFEO=$_tc_blasfeo

# SG tracks aarch64 copies of the acados runtime libs next to the MPC solver. A checkout of a ref that changes them
# puts the aarch64 files back (skip-worktree doesn't stop that) while SCons still considers its x86 copies current,
# so the solver can't load libacados.so. Re-copy the x86 libs from the acados wheel, as the SConscript's Copy step does.
tc_restore_acados_libs() {
  [ "$TC" = sg ] || return 0
  local lib dir=${TC_MPC_SO%/*} src
  src=$("$PY" -c 'import acados; print(acados.LIB_DIR)' 2>/dev/null) || return 0
  for lib in libacados.so libblasfeo.so libhpipm.so libqpOASES_e.so.3.1; do
    if [ -f "$src/$lib" ] && [ -f "$dir/$lib" ] && ! file "$dir/$lib" | grep -q x86-64; then
      echo "[toolchain:sg] restoring x86 $lib" >&2
      cp "$src/$lib" "$dir/$lib"
      git -C "$W" update-index --skip-worktree "${dir#$W/}/$lib" 2>/dev/null || true
    fi
  done
}
