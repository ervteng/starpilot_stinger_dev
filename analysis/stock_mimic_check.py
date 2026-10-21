#!/usr/bin/env python3
"""How closely the Stinger stock-like longitudinal tune reproduces the factory SCC, on stock-ACC rlogs.

Drives update_kia_stinger_longitudinal_tuning (from --starpilot's opendbc) at 100 Hz, once per carState event, with
accel_cmd = the factory SCC12 aReqRaw, long_active = SCC12 ACCMode == 1 with no gas/brake pedal, state pid (stopping
while SCC12 StopReq == 1), lead = SCC11 ACC_ObjStatus / ACC_ObjDist, v_ego = carState.vEgo. The output is compared
with what the factory SCC sent at the same time: SCC12 aReqValue and SCC14 JerkUpperLimit / JerkLowerLimit /
ComfortBandUpper / ComfortBandLower. "today" is what openpilot sends without the tune (aReqValue = aReqRaw, jerk
3.0 / 5.0, bands 0).

Samples with StopReq set are left out by default (the tune passes them through untouched, so they say nothing about
it); --include-stopping keeps them. Per group: aReqValue |error| median / p95, share of samples whose jerk limit is
within 0.15 of the factory's, comfort-band mean |error| (m/s^2).

Usage: stock_mimic_check.py [ROUTE ...] [--starpilot DIR] [--include-stopping]
"""
import argparse
import sys

import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short, sorted_events

DEFAULT_ROUTES = ["0000001d--2ead59d2dd", "00000028--297af64c9c", "00000029--b3c2c13819", "00000034--5f8605e608"]
JERK_TOL = 0.15
TODAY = dict(jerk_upper=3.0, jerk_lower=5.0)
SPEED_BINS = ((0, 5), (5, 15), (15, 25), (25, 99))
LEAD_BINS = (("no lead", lambda r: r[:, 1] == 0), ("lead <60 m", lambda r: (r[:, 1] != 0) & (r[:, 2] < 60)),
             ("lead >=60 m", lambda r: (r[:, 1] != 0) & (r[:, 2] >= 60)))
# columns: v, lead_visible, lead_dist, stop_req, | tune: a, ju, jl, cbu, cbl | factory: a, a_raw, ju, jl, cbu, cbl
COLS = 4 + 5 + 6


def scan(arg):
  path, include_stopping = arg
  from opendbc.can import CANParser  # imported before the first log read, so logs.py reuses its schema
  from opendbc.car import structs
  from opendbc.car.hyundai.carcontroller import KiaStingerLongitudinalTuningState, update_kia_stinger_longitudinal_tuning

  LongCtrlState = structs.CarControl.Actuators.LongControlState
  p = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0), ("SCC14", 0)], 0)
  state = KiaStingerLongitudinalTuningState()
  seen = set(); rows = []
  for e in sorted_events(path):
    w = e.which()
    if w == "can":
      frames = [(c.address, bytes(c.dat), c.src) for c in e.can]
      seen.update(a for a, _, s in frames if s == 0 and a in (905, 1056, 1057))
      p.update([(e.logMonoTime, frames)])
    elif w == "carState" and len(seen) == 3:
      cs = e.carState
      s11, s12, s14 = p.vl["SCC11"], p.vl["SCC12"], p.vl["SCC14"]
      stop = s12["StopReq"] == 1
      active = s12["ACCMode"] == 1 and not (cs.gasPressed or cs.brakePressed)
      lead = bool(s11["ACC_ObjStatus"])
      dist = float(s11["ACC_ObjDist"])
      lcs = LongCtrlState.stopping if stop else LongCtrlState.pid
      update_kia_stinger_longitudinal_tuning(state, float(s12["aReqRaw"]), cs.vEgo, lcs, active, False, lead, dist)
      if active and (include_stopping or not stop):
        rows.append((cs.vEgo, lead, dist, stop, state.actual_accel, state.jerk_upper, state.jerk_lower,
                     state.comfort_band_upper, state.comfort_band_lower, s12["aReqValue"], s12["aReqRaw"],
                     s14["JerkUpperLimit"], s14["JerkLowerLimit"], s14["ComfortBandUpper"], s14["ComfortBandLower"]))
  return np.array(rows, dtype=float).reshape(-1, COLS)


def metrics(r):
  """(a_med, a_p95, ju_match, jl_match, cbu_mae, cbl_mae) for the tune and for today's openpilot."""
  fa, fju, fjl, fcbu, fcbl = r[:, 9], r[:, 11], r[:, 12], r[:, 13], r[:, 14]
  def one(a, ju, jl, cbu, cbl):
    err = np.abs(a - fa)
    return (np.median(err), np.percentile(err, 95), 100 * np.mean(np.abs(ju - fju) <= JERK_TOL),
            100 * np.mean(np.abs(jl - fjl) <= JERK_TOL), np.mean(np.abs(cbu - fcbu)), np.mean(np.abs(cbl - fcbl)))
  z = np.zeros(len(r))
  return one(r[:, 4], r[:, 5], r[:, 6], r[:, 7], r[:, 8]), \
      one(r[:, 10], z + TODAY["jerk_upper"], z + TODAY["jerk_lower"], z, z)


def print_table(title, groups):
  print(f"\n== {title}")
  print(f"{'group':14s} {'n':>8s} | {'aReqValue |err| med / p95':>26s} | {'jerkU match %':>14s} | {'jerkL match %':>14s} | {'band U MAE':>13s} | {'band L MAE':>13s}")
  print(f"{'':14s} {'':>8s} | {'tune':>12s} {'today':>13s} | {'tune':>6s} {'today':>7s} | {'tune':>6s} {'today':>7s} | {'tune':>6s} {'today':>6s} | {'tune':>6s} {'today':>6s}")
  for name, r in groups:
    if not len(r):
      print(f"{name:14s} {0:8d}"); continue
    t, o = metrics(r)
    print(f"{name:14s} {len(r):8d} | {t[0]:5.3f}/{t[1]:5.3f} {o[0]:6.3f}/{o[1]:6.3f} | {t[2]:6.1f} {o[2]:7.1f} | {t[3]:6.1f} {o[3]:7.1f} |"
          f" {t[4]:6.3f} {o[4]:6.3f} | {t[5]:6.3f} {o[5]:6.3f}")


def report(title, rows):
  groups = [("overall", rows)]
  groups += [(f"{lo}-{hi} m/s" if hi < 99 else f"{lo}+ m/s", rows[(rows[:, 0] >= lo) & (rows[:, 0] < hi)]) for lo, hi in SPEED_BINS]
  groups += [(name, rows[f(rows)]) for name, f in LEAD_BINS]
  print_table(title, groups)


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="*", default=DEFAULT_ROUTES, help="stock-ACC drives with rlogs (default: %(default)s)")
  p.add_argument("--starpilot", default="/home/user/wt/long-tune", help="checkout with the tune (default: %(default)s)")
  p.add_argument("--include-stopping", action="store_true", help="keep samples where SCC12 StopReq is set")
  args = p.parse_args()
  print(f"tune from {add_starpilot_path(args.starpilot)}")
  per_route = []
  for route in args.routes:
    files = segment_files(route, "rlog")
    if not files:
      print(f"{short(route)}: no rlogs"); continue
    rows = np.concatenate(pmap(scan, [(f, args.include_stopping) for f in files]))
    per_route.append((short(route), rows))
    print(f"{short(route)}: {len(files)} rlogs, {len(rows)} stock-ACC active samples ({len(rows) / 100:.0f} s)")
  if not per_route:
    return 1
  allrows = np.concatenate([r for _, r in per_route])
  report(f"all routes ({len(allrows)} samples)", allrows)
  for name, rows in per_route:
    print_table(f"{name} overall", [("overall", rows)])


if __name__ == "__main__":
  sys.exit(main())
