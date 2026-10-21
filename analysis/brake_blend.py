#!/usr/bin/env python3
"""How the car delivers openpilot's requested accel: throttle (EMS12 TPS) vs brakes (TCS13 BrakeLight).

Binned by requested accel, with openpilot longitudinal active and no driver pedal input, split highway / city.

Usage: brake_blend.py ROUTE [ROUTE ...] [--starpilot DIR]
"""
import argparse

import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short, sorted_events

BINS = ((0.2, 9), (0, 0.2), (-0.2, 0), (-0.4, -0.2), (-0.6, -0.4), (-0.8, -0.6), (-1.0, -0.8), (-1.5, -1.0), (-2.5, -1.5), (-9, -2.5))


def scan(path):
  from opendbc.can import CANParser  # imported before the first log read, so logs.py reuses its schema
  p = CANParser("hyundai_kia_generic", [("TCS13", 0), ("EMS12", 0)], 0)
  out = []; cmd = None; la = False; v = 0.0; pedal = False; a = 0.0
  for e in sorted_events(path):
    w = e.which()
    if w == "carControl":
      cmd = e.carControl.actuators.accel; la = e.carControl.longActive
    elif w == "carState":
      v = e.carState.vEgo; a = e.carState.aEgo; pedal = e.carState.brakePressed or e.carState.gasPressed
    elif w == "can":
      p.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
      if la and cmd is not None and not pedal and v > 3 and p.can_valid:
        out.append((v, cmd, a, p.vl["TCS13"]["BrakeLight"], p.vl["EMS12"]["TPS"]))
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--starpilot")
  args = p.parse_args()
  add_starpilot_path(args.starpilot)
  files = [f for r in args.routes for f in segment_files(r, "rlog")]
  rows = np.array([r for out in pmap(scan, files) for r in out])
  if not len(rows):
    print("no samples with openpilot longitudinal active"); return
  print(f"{', '.join(short(r) for r in args.routes)}: {len(files)} rlogs")
  for name, sel in (("highway > 45 mph", rows[:, 0] > 20), ("city 7-45 mph", rows[:, 0] <= 20)):
    r = rows[sel]
    print(f"== {name}: {len(r)} samples\n   requested accel      n   brake light on   throttle median   actual accel median")
    for lo, hi in BINS:
      s = r[(r[:, 1] >= lo) & (r[:, 1] < hi)]
      if len(s) >= 30:
        print(f"   {lo:+5.1f}..{hi:+4.1f} {len(s):7d}       {100 * s[:, 3].mean():4.0f}%          {np.median(s[:, 4]):5.1f}%            {np.median(s[:, 2]):+5.2f}")


if __name__ == "__main__":
  main()
