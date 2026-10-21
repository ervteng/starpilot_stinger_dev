#!/usr/bin/env python3
"""List alert changes matching a pattern, with the car state at that moment.

Times are seconds from route start (60 s per segment).

Usage: alerts.py ROUTE [ROUTE ...] [--match steerTemp|fcw|...] [--kind qlog|rlog]
"""
import argparse
import re

from logs import events, pmap, route_time, segment_files, short


def scan(job):
  path, pattern = job
  out = []; prev = ""; cs = None; t0 = None
  for e in events(path):
    w = e.which()
    if w == "carState":
      cs = e.carState
      t0 = t0 or e.logMonoTime
    elif w == "selfdriveState":
      a = e.selfdriveState.alertType
      if a != prev and t0 and re.search(pattern, a, re.I):
        state = (f"v={cs.vEgo * 2.237:4.1f} mph standstill={int(cs.standstill)} steerFaultTemp={int(cs.steerFaultTemporary)} "
                 f"brake={int(cs.brakePressed)} cruise={int(cs.cruiseState.enabled)}") if cs else ""
        out.append(f"{short(path)} {route_time(path, e.logMonoTime, t0):7.1f}s {a:45s} engaged={int(e.selfdriveState.enabled)} {state}")
      prev = a
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--match", default="steerTemp|steerUnavailable|fcw")
  p.add_argument("--kind", default="qlog")
  args = p.parse_args()
  for route in args.routes:
    files = segment_files(route, args.kind)
    lines = [l for out in pmap(scan, [(f, args.match) for f in files]) for l in out]
    print(f"== {short(route)}: {len(lines)} alert changes matching '{args.match}' in {len(files)} {args.kind}s")
    print("\n".join(lines))


if __name__ == "__main__":
  main()
