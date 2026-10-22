#!/usr/bin/env python3
"""How fast the accel command changes while following a lead: factory SCC vs openpilot longitudinal.

Basis for two planner entries of the stock-like Stinger tunes: get_far_follow_output_slew_rates (the planner's far-follow
output slew, see get_vehicle_far_follow_slew_target) and get_lead_follow_jerk_scale. The rate is the change of the
accel command over 0.2 s (m/s^3), split into rising and falling, as percentiles (50 / 90 / 95 / 99):
  stock      stock-ACC rlogs: SCC12 aReqRaw (the factory planner's target) and aReqValue (after its jerk limits), with the
             lead from SCC11 (ACC_ObjStatus / ACC_ObjDist / ACC_ObjRelSpd), on enabled, no-pedal, non-StopReq samples
  openpilot  openpilot-long rlogs: carControl actuators.accel (what the planner asked for) with radarState leadOne
Groups: "far follow" = the slew's own conditions (lead >= max(25 m, 1.35 s * v), headway >= 1.35 s, TTC >= 8 s, v >= 10
m/s, lateral offset <= 1.5 m where known); "any lead" = a lead closer than 60 m. A slew rate only makes sense if it
is not slower than what openpilot does today in the same situations (release) and is backed by the stock numbers (brake).

Usage: follow_rates.py [--stock ROUTE ...] [--openpilot ROUTE ...] [--cache FILE.npz] [--starpilot DIR]
"""
import argparse
import sys

import numpy as np

from logs import TUNE_DIR, add_starpilot_path, pmap, segment_files, short, sorted_events
import stock_mimic_check as S

STOCK = S.DEFAULT_ROUTES
OPENPILOT = ["0000000d--5e0d1d244b", "0000000e--5ebf4636a1"]
WINDOW = 0.2
MIN_SPEED, MIN_DIST, MIN_HEADWAY, MIN_TTC, MAX_LAT = 10.0, 25.0, 1.35, 8.0, 1.5
PCT = (50, 90, 95, 99)


def far_follow(v, dist, closing, lead):
  ttc = np.where(closing > 0.1, dist / np.maximum(closing, 0.1), np.inf)
  return lead & (v >= MIN_SPEED) & (dist >= np.maximum(MIN_DIST, MIN_HEADWAY * v)) & (dist / np.maximum(v, 1e-3) >= MIN_HEADWAY) & (ttc >= MIN_TTC)


def rates(x, step, mask):
  """Rates over `step` samples where the mask holds at both ends; returns (rising, falling) magnitudes."""
  r = (x[step:] - x[:-step]) / WINDOW
  m = mask[step:] & mask[:-step]
  r = r[m]
  return r[r > 0.01], -r[r < -0.01]


def stock_rows(routes, cache):
  out = []
  for route in routes:
    rows = S.load_route(route, cache)
    if rows is None:
      continue
    r = rows[rows[:, S.NEW] == 1]  # one row per SCC12 (50 Hz)
    ok = (r[:, S.MODE] == 1) & (r[:, S.PEDAL] == 0) & (r[:, S.STOP] == 0)
    lead = r[:, S.OBJSTATUS] != 0
    ff = far_follow(r[:, S.V], r[:, S.OBJDIST], -r[:, S.OBJRELSPD], lead) & ok
    near = lead & (r[:, S.OBJDIST] < 60) & ok
    out.append((r, ff, near))
  return out


def scan(path):
  """(t, accel, longActive, v, lead, dRel, vLead, yRel) rows at carControl rate."""
  v = 0.0; lead = (0.0, 0.0, 0.0, 0.0); rows = []
  for e in sorted_events(path):
    w = e.which()
    if w == "carState":
      v = e.carState.vEgo
    elif w == "radarState":
      l = e.radarState.leadOne; lead = (float(l.status), l.dRel, l.vLead, l.yRel)
    elif w == "carControl":
      cc = e.carControl
      rows.append((e.logMonoTime / 1e9, cc.actuators.accel, float(cc.longActive and not cc.cruiseControl.override), v, *lead))
  return rows


def show(name, cmd, step, mask):
  up, dn = rates(cmd, step, mask)
  f = lambda x: " / ".join(f"{np.percentile(x, p):5.2f}" for p in PCT) if len(x) else "n/a"
  print(f"   {name:28s} rising {f(up):28s} (n {len(up):6d}) | falling {f(dn):28s} (n {len(dn):6d})")


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--stock", nargs="*", default=STOCK, help="stock-ACC drives (default: %(default)s)")
  p.add_argument("--openpilot", nargs="*", default=OPENPILOT, help="openpilot-long drives (default: %(default)s)")
  p.add_argument("--cache", help="npz cache of the stock samples (see stock_mimic_check.py)")
  p.add_argument("--starpilot", default=TUNE_DIR)
  args = p.parse_args()
  add_starpilot_path(args.starpilot)
  print(f"percentiles {'/'.join(map(str, PCT))} of the accel change over {WINDOW} s, m/s^3")
  print("\n== factory SCC (stock-ACC drives)")
  sr = stock_rows(args.stock, args.cache)
  step = int(round(WINDOW / 0.02))
  for label, col in (("aReqRaw (factory target)", S.RAW), ("aReqValue (after jerk limits)", S.VALUE)):
    for gname, gi in (("far follow", 1), ("any lead <60 m", 2)):
      rise = []; fall = []
      for r, ff, near in sr:
        m = (ff, near)[gi - 1]
        u, d = rates(r[:, col], step, m); rise.append(u); fall.append(d)
      u, d = np.concatenate(rise), np.concatenate(fall)
      fm = lambda x: " / ".join(f"{np.percentile(x, q):5.2f}" for q in PCT)
      print(f"   {label + ' ' + gname:44s} rising {fm(u)} (n {len(u)}) | falling {fm(d)} (n {len(d)})")
  print("\n== openpilot longitudinal (actuators.accel)")
  step = int(round(WINDOW / 0.01))
  for route in args.openpilot:
    files = segment_files(route, "rlog")
    rows = np.array(sorted(r for rs in pmap(scan, files) for r in rs))
    if not len(rows):
      continue
    t, a, act, v, st, d, vl, y = rows.T
    # hold to 100 Hz (carControl is 100 Hz in rlogs)
    ok = (act == 1) & (np.diff(t, append=t[-1]) < 0.05)
    lead = (st == 1) & (np.abs(y) <= MAX_LAT)
    ff = far_follow(v, d, v - vl, lead) & ok
    near = (st == 1) & (d < 60) & ok
    print(f" {short(route)} ({len(rows) / 100:.0f} s)")
    show("far follow", a, step, ff)
    show("any lead <60 m", a, step, near)


if __name__ == "__main__":
  sys.exit(main())
