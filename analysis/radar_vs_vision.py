#!/usr/bin/env python3
"""Radar-backed lead (radarState.leadOne with radar=True) vs the camera's lead (modelV2.leadsV3[0]) from rlogs.

Distance compares in the radar frame (model x minus radard's RADAR_TO_CAMERA of 1.52 m), so a constant offset
up close points at the radar-to-camera distance for this car rather than at the radar. Also reports how often the
lead came from radar, camera only or nothing at low speed with openpilot in control.

Usage: radar_vs_vision.py ROUTE [ROUTE ...] [--min-prob 0.9]
"""
import argparse
from collections import Counter

import numpy as np

from logs import pmap, segment_files, short, sorted_events

RADAR_TO_CAMERA = 1.52


def scan(job):
  path, min_prob = job
  out = []; src = Counter(); ml = None; v = 0.0; la = False
  for e in sorted_events(path):
    w = e.which()
    if w == "carState":
      v = e.carState.vEgo
    elif w == "carControl":
      la = e.carControl.longActive
    elif w == "modelV2":
      l = e.modelV2.leadsV3[0]
      ml = (l.prob, l.x[0], l.y[0], l.v[0], e.logMonoTime)
    elif w == "radarState":
      L = e.radarState.leadOne
      if la and v < 3:
        src["none" if not L.status else "radar" if L.radar else "vision"] += 1
      if ml and ml[0] > min_prob and abs(e.logMonoTime - ml[4]) < 60e6 and L.status and L.radar:
        out.append((L.dRel, L.dRel - (ml[1] - RADAR_TO_CAMERA), L.yRel + ml[2], L.vLead - ml[3], v, L.vRel))
  return out, src


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--min-prob", type=float, default=0.9, help="camera lead probability to count")
  args = p.parse_args()
  files = [f for r in args.routes for f in segment_files(r, "rlog")]
  res = pmap(scan, [(f, args.min_prob) for f in files])
  rows = np.array([r for out, _ in res for r in out]); src = sum((s for _, s in res), Counter())
  print(f"{', '.join(short(r) for r in args.routes)}: {len(files)} rlogs, {len(rows)} matched samples")
  if not len(rows):
    return
  iqr = lambda x: f"{np.median(x):+5.2f} [{np.percentile(x, 25):+5.2f},{np.percentile(x, 75):+5.2f}]"
  print("  range        n   distance (radar-vision) m   lateral m              lead speed m/s")
  for lo, hi in ((0, 15), (15, 30), (30, 50), (50, 80), (80, 200)):
    s = rows[(rows[:, 0] >= lo) & (rows[:, 0] < hi)]
    if len(s) >= 20:
      print(f"  {lo:3d}-{hi:<3d}m {len(s):6d}   {iqr(s[:, 1])}   {iqr(s[:, 2])}   {np.median(s[:, 3]):+5.2f}")
  near = rows[:, 0] < 15
  print("  under 15 m by motion:")
  for name, s in (("standstill", near & (rows[:, 4] < 0.1)), ("steady", near & (np.abs(rows[:, 5]) < 0.2) & (rows[:, 4] > 1)),
                  ("closing", near & (rows[:, 5] < -0.5)), ("opening", near & (rows[:, 5] > 0.5))):
    if s.sum():
      print(f"    {name:10s} n={s.sum():6d} distance (radar-vision) {np.median(rows[s, 1]):+.2f} m")
  n = sum(src.values())
  if n:
    print(f"  openpilot in control under 3 m/s: lead from radar {100 * src['radar'] / n:.0f}%, camera only "
          f"{100 * src['vision'] / n:.0f}%, none {100 * src['none'] / n:.0f}%")


if __name__ == "__main__":
  main()
