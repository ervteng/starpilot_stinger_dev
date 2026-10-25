"""Toolchain selector (python twin of ../toolchain.sh; keep the two in sync).

  STINGER_TOOLCHAIN=old|sg  (default old)
    old: the daily StarPilot layout (code at the repo root), build tree $STINGER_DEV_HOME/x86, venv $STINGER_DEV_HOME/venv
    sg : SecretGoodStarPilot (code under openpilot/),        build tree $STINGER_DEV_HOME/x86-sg, venv $STINGER_DEV_HOME/venv-sg
  STARPILOT_BUILD overrides the build tree. The layout is auto-detected from it (openpilot/cereal/log.capnp present => sg).

Scripts that only read logs need nothing from here besides logs.py (which uses it for the cereal schema and opendbc path). Scripts
that import planner / MPC internals call `setup_paths()` and then import through `layout_import()` or branch on `LAYOUT`; features
that exist only in the old layout call `require_old("what")`, which exits with a clear message under sg.
Run such scripts with the toolchain's venv python: `STINGER_TOOLCHAIN=sg ./run_py.sh analysis/x.py ...`.
"""
import os
import sys

TOOLCHAIN = os.environ.get("STINGER_TOOLCHAIN", "old")
if TOOLCHAIN not in ("old", "sg"):
  sys.exit(f"STINGER_TOOLCHAIN must be old or sg (got '{TOOLCHAIN}')")

# Same base-folder rule as logs.py used before the toolchain split: parent of $STARPILOT_DIR, else ~/stinger.
BASE = os.path.dirname(os.path.abspath(os.environ["STARPILOT_DIR"])) if os.environ.get("STARPILOT_DIR") \
  else os.path.expanduser("~/stinger")
DEV = os.environ.get("STINGER_DEV_HOME", os.path.join(BASE, "starpilot-dev"))

_DIRS = {"old": ("x86", "venv"), "sg": ("x86-sg", "venv-sg")}
BUILD = os.environ.get("STARPILOT_BUILD", os.path.join(DEV, _DIRS[TOOLCHAIN][0]))
VENV = os.path.join(DEV, _DIRS[TOOLCHAIN][1])
PYTHON = os.path.join(VENV, "bin", "python")


def detect_layout(tree: str, default: str = "old") -> str:
  """'sg' if the tree has the code under openpilot/, 'old' if at the root, else `default` (tree missing / not built yet).

  The old layout also has an openpilot/ dir, but of symlinks to the root dirs: key on the cereal schema, which only sg keeps there."""
  if os.path.isfile(os.path.join(tree, "openpilot", "cereal", "log.capnp")):
    return "sg"
  if os.path.isdir(os.path.join(tree, "selfdrive")):
    return "old"
  return default


LAYOUT = detect_layout(BUILD, TOOLCHAIN)
SG = LAYOUT == "sg"


def code_paths(tree: str = None) -> list[str]:
  """sys.path entries that make `import opendbc...`, `import cereal...` (old) / `import openpilot.cereal...` (sg) work for a tree."""
  tree = tree or BUILD
  if detect_layout(tree, LAYOUT) == "sg":
    # SG's pyproject dev-mode-dirs = [".", "msgq_repo"]; opendbc / rednose / panda are editable installs of the sibling dirs
    return [tree] + [os.path.join(tree, d) for d in ("msgq_repo", "opendbc_repo", "rednose_repo", "panda")]
  return [tree, os.path.join(tree, "opendbc_repo")]


def setup_paths(tree: str = None, extra: tuple = ()) -> str:
  """Put the tree's code on sys.path (front, same order as the old per-script loops: tree, opendbc_repo, then `extra`)."""
  tree = tree or BUILD
  for p in (*code_paths(tree), *extra):
    if p not in sys.path:
      sys.path.insert(0, p)
  return tree


def cereal_dir(tree: str = None) -> str:
  tree = tree or BUILD
  return os.path.join(tree, "openpilot", "cereal") if detect_layout(tree, LAYOUT) == "sg" else os.path.join(tree, "cereal")


def have_cereal(tree: str) -> bool:
  return bool(tree) and os.path.exists(os.path.join(cereal_dir(tree), "log.capnp"))


def require_old(what: str):
  """Exit with a clear message when a feature that only exists in the old layout is used under the sg toolchain."""
  if SG:
    sys.exit(f"{what} is not available with STINGER_TOOLCHAIN=sg (SecretGoodStarPilot at {BUILD}): it needs something the new layout no "
             f"longer has (Testing Grounds / longitudinal_vehicle_tunes / starpilot_variables). Run it with STINGER_TOOLCHAIN=old.")


def require_sg_port(what: str, missing: str):
  """Exit under sg for a tool that has not been adapted to the new planner/MPC APIs; `missing` names the missing piece."""
  if SG:
    sys.exit(f"{what} is not ported to STINGER_TOOLCHAIN=sg yet: {missing}. Run it with STINGER_TOOLCHAIN=old.")


# ------------------------------------------------------------------ log schema
# Drive logs were written by the OLD build, and the sg cereal schema is not wire-compatible with them (Event union members were
# renamed / reused: starpilotPlan @113 is customReserved6 there, liveParameters is vehicleParameters, RadarState.LeadData.status is
# present, ...). So logs are always parsed with the old-layout schema. Under sg that schema cannot be loaded as is next to
# openpilot.cereal (capnp aborts with "Duplicate ID"), so a copy with every explicit capnp ID changed is used; IDs are not on the wire.
_REMAP_XOR = 0x00C0FFEE00C0FFEE


def log_schema_source() -> str:
  """Old-layout cereal dir (log.capnp, custom.capnp, legacy.capnp, car.capnp, include/c++.capnp) the sg toolchain reads logs with."""
  cands = [os.environ.get("STINGER_LOG_SCHEMA"), os.path.join(DEV, "x86", "cereal"), os.path.join(BASE, "starpilot", "cereal"),
           os.path.join(BASE, "wt", "stinger-combined", "cereal"), os.path.join(os.environ.get("STARPILOT_DIR", ""), "cereal")]
  for d in cands:
    if d and os.path.exists(os.path.join(d, "log.capnp")) and os.path.exists(os.path.join(d, "include", "c++.capnp")):
      return d
  raise RuntimeError("no old-layout cereal dir to read drive logs with under STINGER_TOOLCHAIN=sg; set STINGER_LOG_SCHEMA=<dir with "
                     "log.capnp> (e.g. $STINGER_DEV_HOME/x86/cereal)")


def remapped_log_schema_dir() -> str:
  """Cache dir holding the ID-remapped copy of log_schema_source() (rebuilt when the source files change)."""
  import hashlib
  import re
  src = log_schema_source()
  files = ("log.capnp", "custom.capnp", "legacy.capnp", "car.capnp", os.path.join("include", "c++.capnp"))
  text = {f: open(os.path.join(src, f)).read() for f in files if os.path.exists(os.path.join(src, f))}
  key = hashlib.sha1("".join(text[f] for f in sorted(text)).encode()).hexdigest()[:12]
  out = os.path.join(DEV, "schema-old-remap", key)
  if not os.path.exists(os.path.join(out, "log.capnp")):
    tmp = out + f".tmp{os.getpid()}"
    os.makedirs(os.path.join(tmp, "include"), exist_ok=True)
    for f, t in text.items():
      with open(os.path.join(tmp, f), "w") as fh:
        fh.write(re.sub(r"@0x([0-9a-f]{16})", lambda m: "@0x%016x" % (int(m.group(1), 16) ^ _REMAP_XOR), t))
    try:
      os.rename(tmp, out)
    except OSError:    # another process won the race
      import shutil
      shutil.rmtree(tmp, ignore_errors=True)
  return out
