#!/usr/bin/env python3
"""Summarize a downloaded route: build, CarParams, and per-segment speed / openpilot long / stops.

Usage: route_scan.py "dongle|route" [--kind qlog|rlog]
"""
import argparse

from load import find_stops, load_long
from logs import segment_files, seg_num


def main():
  p = argparse.ArgumentParser()
  p.add_argument("route")
  p.add_argument("--kind", default="qlog", choices=("qlog", "rlog"))
  args = p.parse_args()

  files = segment_files(args.route, args.kind)
  if not files:
    raise SystemExit(f"no {args.kind} files for {args.route}; download with comma_dl.py")
  shown = False
  for f in files:
    s = load_long(f)
    if not shown and s["info"]:
      print(s["info"])
      shown = True
    if not s["cs"].t:
      print(f"seg {seg_num(f):2d}: empty")
      continue
    vmax = max(c["v"] for c in s["cs"].v)
    active = [s["cc"].at(t) for t in s["cs"].t]
    frac = sum(bool(a and a["long_active"]) for a in active) / len(active)
    stops = ", ".join(f"{t:.0f}s({'OP' if op else 'driver'}{' brk' if brk else ''})" for t, op, brk in find_stops(s))
    print(f"seg {seg_num(f):2d}: vmax {vmax:4.1f} m/s  longActive {frac:4.0%}  stops: {stops or '-'}")


if __name__ == "__main__":
  main()
