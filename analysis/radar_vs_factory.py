#!/usr/bin/env python3
"""Stinger radar decoder vs the factory radar's own lead (SCC11), on stock ACC rlogs.

Runs the StarPilot checkout's RadarInterface over the raw bus-1 tracks, matches the nearest in-lane track to the
SCC11 lead (ACC_ObjDist / ACC_ObjRelSpd), and reports distance, relative speed and lateral differences by range
and ego speed. SCC11 only comes from the radar under stock ACC; with openpilot longitudinal the radar's bus-0
output is silenced. Pass --starpilot to compare decoder versions (e.g. before and after a fix).

Usage: radar_vs_factory.py ROUTE [ROUTE ...] [--starpilot DIR] [--fine]
"""
import argparse

import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short, sorted_events


def scan(path):
  from opendbc.can import CANParser  # before the first log read, so logs.py reuses its schema
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.radar_interface import RadarInterface
  from opendbc.car.hyundai.values import CAR

  evs = sorted_events(path)
  fp = gen_empty_fingerprint()
  for e in evs[:3000]:
    if e.which() == "can":
      for c in e.can:
        if c.src < 3:
          fp[c.src][c.address] = len(c.dat)
  ri = RadarInterface(CarInterface.get_params(CAR.KIA_STINGER_2022, fp, [], True, False, False, None))
  p = CANParser("hyundai_kia_generic", [("SCC11", 0)], 0)
  out = []; v = 0.0; pts = {}
  for e in evs:
    w = e.which()
    if w == "carState":
      v = e.carState.vEgo
    elif w == "can":
      frames = [(c.address, bytes(c.dat), c.src) for c in e.can]
      p.update([(e.logMonoTime, frames)])
      rr = ri.update([(e.logMonoTime, frames)])
      if rr is not None:
        pts = {pt.trackId: pt for pt in rr.points}
      s = p.vl["SCC11"]
      if p.can_valid and s["ACC_ObjStatus"] and 2 < s["ACC_ObjDist"] < 150 and pts:
        cands = [pt for pt in pts.values() if abs(pt.yRel) < 3]
        if cands:
          best = min(cands, key=lambda pt: abs(pt.dRel - s["ACC_ObjDist"]))
          if abs(best.dRel - s["ACC_ObjDist"]) < 6:
            out.append((s["ACC_ObjDist"], best.dRel - s["ACC_ObjDist"], best.vRel - s["ACC_ObjRelSpd"], v,
                        best.yRel + s["ACC_ObjLatPos"]))
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--starpilot", help="checkout whose decoder to run (default: auto-detect)")
  p.add_argument("--fine", action="store_true", help="also print the distance difference in 2.5 m bins")
  args = p.parse_args()
  print(f"decoder from {add_starpilot_path(args.starpilot)}")
  for route in args.routes:
    rows = np.array([r for out in pmap(scan, segment_files(route, "rlog")) for r in out])
    print(f"== {short(route)}: {len(rows)} samples where the factory lead matches a decoded track")
    if not len(rows):
      continue
    iqr = lambda x: f"{np.median(x):+.2f} [{np.percentile(x, 25):+.2f},{np.percentile(x, 75):+.2f}]"
    for lo, hi in ((0, 15), (15, 30), (30, 50), (50, 80), (80, 150)):
      s = rows[(rows[:, 0] >= lo) & (rows[:, 0] < hi)]
      if len(s) > 20:
        print(f"  {lo:3d}-{hi:<3d} m n={len(s):6d}  distance (decoder-factory) {iqr(s[:, 1])} m  vRel {np.median(s[:, 2]):+.2f} m/s  "
              f"lateral {np.median(s[:, 4]):+.2f} m")
    for lo, hi in ((0, 0.01), (0.01, 1), (1, 10), (10, 20), (20, 35)):
      s = rows[(rows[:, 3] >= lo) & (rows[:, 3] < hi)]
      if len(s) > 20:
        print(f"  ego {lo:5.2f}-{hi:<5.2f} m/s n={len(s):6d}  vRel (decoder-factory) {iqr(s[:, 2])} m/s")
    if args.fine:
      cells = [f"{lo:5.1f}m {np.median(rows[m, 1]):+5.2f}" for lo in np.arange(2.5, 100, 2.5)
               if (m := (rows[:, 0] >= lo) & (rows[:, 0] < lo + 2.5)).sum() > 100]
      for i in range(0, len(cells), 4):
        print("   " + "   ".join(cells[i:i + 4]))


if __name__ == "__main__":
  main()
