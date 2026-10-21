#!/usr/bin/env python3
"""Compare how the car responds when the lead brakes, between groups of drives (e.g. stock ACC vs openpilot).

An event starts when cruise is engaged (stock SCC or openpilot), no pedals, ego above --min-speed and radard's lead
aLeadK drops below --start, and lasts while aLeadK stays below --end. Per group: lead vs ego peak decel, how much
the time gap shrank, and how many events stayed in the coast range (ego above -0.6 m/s^2, throttle-only on the
Stinger), binned by how hard the lead braked.

Usage: lead_braking.py --group NAME ROUTE [ROUTE ...] [--group NAME ROUTE ...] [--min-speed 13] [--start -0.4]
"""
import argparse
import sys

import numpy as np

from logs import events, pmap, segment_files

LEAD_BINS = ((-0.8, -0.4), (-1.2, -0.8), (-2.0, -1.2), (-9.0, -2.0))


def load(path):
  rows = []; st = {}
  for e in events(path):
    w = e.which()
    if w == "carState":
      cs = e.carState
      st.update(v=cs.vEgo, a=cs.aEgo, pedal=cs.brakePressed or cs.gasPressed, cruise=cs.cruiseState.enabled)
    elif w == "radarState" and "v" in st:
      l = e.radarState.leadOne
      rows.append((e.logMonoTime / 1e9, st["v"], st["a"], st["pedal"], st["cruise"], l.status, l.dRel, l.vLead, l.aLeadK))
  return sorted(rows)


def find(rows, min_speed, start, end):
  out = []; i = 0
  while i < len(rows):
    t, v, a, pedal, cruise, ls, d, vl, al = rows[i]
    if cruise and not pedal and v > min_speed and ls and al < start:
      j = i
      while j < len(rows) and rows[j][5] and rows[j][8] < end and not rows[j][3]:
        j += 1
      ep = rows[i:j]
      if len(ep) >= 14 and ep[-1][0] - ep[0][0] >= 0.7:
        out.append((min(r[8] for r in ep), min(r[2] for r in ep), ep[0][6] / ep[0][1], min(r[6] / max(r[1], 1) for r in ep)))
      i = j + 1
    else:
      i += 1
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--group", nargs="+", action="append", metavar=("NAME", "ROUTE"), required=True)
  p.add_argument("--min-speed", type=float, default=13.0, help="m/s (default 13 = 30 mph)")
  p.add_argument("--start", type=float, default=-0.4, help="lead aLeadK that starts an event")
  p.add_argument("--end", type=float, default=-0.15, help="lead aLeadK that ends it")
  args = p.parse_args()
  for name, *routes in args.group:
    if not routes:
      sys.exit(f"group {name} has no routes")
    files = [f for r in routes for f in segment_files(r, "qlog")]
    ev = [e for rows in pmap(load, files) for e in find(rows, args.min_speed, args.start, args.end)]
    print(f"== {name}: {len(ev)} lead-braking events in {len(files)} qlogs")
    if not ev:
      continue
    a = np.array(ev)
    print(f"   lead peak median {np.median(a[:, 0]):+.2f} | ego peak median {np.median(a[:, 1]):+.2f} | ego/lead ratio "
          f"{np.median(a[:, 1] / a[:, 0]):.2f} | gap {np.median(a[:, 2]):.2f}s -> min {np.median(a[:, 3]):.2f}s | "
          f"coast only (> -0.6): {int((a[:, 1] > -0.6).sum())}/{len(ev)}")
    for lo, hi in LEAD_BINS:
      q = (a[:, 0] >= lo) & (a[:, 0] < hi)
      if q.sum():
        print(f"     lead {lo:+.1f}..{hi:+.1f}: n={q.sum():3d} ego peak {np.median(a[q, 1]):+.2f}  gap shrink {np.median(a[q, 2] - a[q, 3]):.2f}s")


if __name__ == "__main__":
  main()
