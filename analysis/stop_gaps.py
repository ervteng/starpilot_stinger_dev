#!/usr/bin/env python3
"""Stopping gap behind a parked lead for openpilot stops: gap 0.5 s after standstill, counting only stops where
the lead stays put (|vLead| < 0.3 m/s) for the next 3 s and the driver isn't on the pedals.

Usage: stop_gaps.py ROUTE [ROUTE ...] [--kind qlog|rlog]
"""
import argparse
import numpy as np

from load import find_stops, load_long
from logs import segment_files, seg_num


def clean_stop_gaps(s):
  out = []
  t0 = s["t0"]
  for ts, op, brk in find_stops(s):
    if not op or brk:
      continue
    t_stop = t0 + int(ts * 1e9)
    window = [(t, l) for t, l in zip(s["rs"].t, s["rs"].v) if t_stop <= t <= t_stop + 3e9]
    cs = [c for t, c in zip(s["cs"].t, s["cs"].v) if t_stop <= t <= t_stop + 3e9]
    if len(window) < 3 or any(c["gas"] or c["brake"] for c in cs):
      continue
    if not all(l["status"] and abs(l["v"]) < 0.3 and abs(l["y"]) < 1.5 for _, l in window):
      continue
    gap = next((l["d"] for t, l in window if t >= t_stop + 0.5e9), window[0][1]["d"])
    out.append((ts, gap, window[0][1]["radar"]))
  return out


def main():
  p = argparse.ArgumentParser()
  p.add_argument("routes", nargs="+")
  p.add_argument("--kind", default="qlog", choices=("qlog", "rlog"))
  args = p.parse_args()
  gaps = []
  for route in args.routes:
    for f in segment_files(route, args.kind):
      s = load_long(f)
      for ts, gap, radar in clean_stop_gaps(s):
        gaps.append(gap)
        print(f"  {route.split('|')[-1][:8]} seg {seg_num(f):2d} @{ts:5.1f}s  gap {gap:4.1f} m  ({'radar' if radar else 'vision'} lead)")
  if gaps:
    g = np.array(gaps)
    print(f"{len(g)} clean stops: median {np.median(g):.1f} m, p25 {np.percentile(g, 25):.1f}, p75 {np.percentile(g, 75):.1f}, range {g.min():.1f}-{g.max():.1f}")


if __name__ == "__main__":
  main()
