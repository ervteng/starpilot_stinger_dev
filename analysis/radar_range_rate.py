#!/usr/bin/env python3
"""Check the Stinger decoder's relative speed against the radar's own range rate (works on any rlog).

For each decoded track, compare the mean decoder vRel over ~1 s with the slope of its distance over the same
window. This needs no factory lead, so it also works with openpilot longitudinal. Off-centre objects read a lower
range rate purely from geometry, so by default only near-centre tracks (|lateral| < 1.2 m, 8-40 m) count.

Usage: radar_range_rate.py ROUTE [ROUTE ...] [--starpilot DIR] [--max-lateral 1.2]
"""
import argparse
from collections import defaultdict

import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short, sorted_events


def scan(job):
  path, max_lat = job
  from opendbc.can import CANParser  # before the first log read, so logs.py reuses its schema
  from opendbc.car.hyundai.radar_interface import Stinger2022ObjectDecoder
  p = CANParser("hyundai_kia_generic", [("WHL_SPD11", 0)], 0)
  dec = Stinger2022ObjectDecoder(); hist = defaultdict(list); out = []
  for e in sorted_events(path):
    if e.which() != "can":
      continue
    frames = [(c.address, bytes(c.dat), c.src) for c in e.can]
    p.update([(e.logMonoTime, frames)])
    w = p.vl["WHL_SPD11"]
    v = 0.25 * sum(w[k] for k in ("WHL_SPD_FL", "WHL_SPD_FR", "WHL_SPD_RL", "WHL_SPD_RR")) / 3.6
    for addr, dat, bus in frames:
      if bus != 1:
        continue
      o = dec.update(e.logMonoTime, addr, dat, bus)
      if not o or abs(o.lateral_raw) > max_lat or not 8 < o.distance < 40:
        continue
      h = hist[o.slot, o.object_id]
      h.append((e.logMonoTime / 1e9, o.distance, o.relative_speed(v)))
      while h and h[-1][0] - h[0][0] > 1.0:
        h.pop(0)
      if len(h) >= 20 and h[-1][0] - h[0][0] > 0.8 and v > 1:
        t = np.array(h)
        if np.max(np.abs(np.diff(t[:, 1]))) <= 1.0:
          out.append((v, np.mean(t[:, 2]) - np.polyfit(t[:, 0], t[:, 1], 1)[0]))
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--starpilot", help="checkout whose decoder to run (default: auto-detect)")
  p.add_argument("--max-lateral", type=float, default=1.2)
  args = p.parse_args()
  print(f"decoder from {add_starpilot_path(args.starpilot)}")
  files = [f for r in args.routes for f in segment_files(r, "rlog")]
  a = np.array([r for out in pmap(scan, [(f, args.max_lateral) for f in files]) for r in out])
  if not len(a):
    print("no tracks"); return
  print(f"{', '.join(short(r) for r in args.routes)}: n={len(a)}  decoder vRel - range rate median {np.median(a[:, 1]):+.3f} m/s")
  for lo, hi in ((1, 5), (5, 15), (15, 35)):
    s = (a[:, 0] >= lo) & (a[:, 0] < hi)
    if s.sum() > 50:
      print(f"   ego {lo:2d}-{hi:<2d} m/s n={s.sum():6d} median {np.median(a[s, 1]):+.3f}")


if __name__ == "__main__":
  main()
