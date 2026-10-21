#!/usr/bin/env python3
"""Dashboard speed limit (starpilotCarState.dashboardSpeedLimit) and what the speed limit controller did with it.

While moving: how often the cluster limit was present, how often SLC used it, how often map data agreed, the SLC
source mix, and the SLC priority settings the drive started with.

Usage: speed_limit.py ROUTE [ROUTE ...]
"""
import argparse
from collections import Counter

from logs import events, pmap, segment_files, short

MPH = 2.23694
PARAMS = ("SpeedLimitController", "SLCPriority1", "SLCPriority2", "SLCPriority3", "SLCFallback", "SLCOverride")


def scan(path):
  c = Counter(); src = Counter(); params = {}; dash = 0.0; v = 0.0
  for e in events(path):
    w = e.which()
    if w == "initData":
      params = {p.key: p.value.decode(errors="replace") for p in e.initData.params.entries if p.key in PARAMS}
    elif w == "starpilotCarState":
      dash = e.starpilotCarState.dashboardSpeedLimit
    elif w == "carState":
      v = e.carState.vEgo
    elif w == "starpilotPlan" and v > 3:
      p = e.starpilotPlan
      d, slc, mp = round(dash * MPH), round(p.slcSpeedLimit * MPH), round(p.slcMapSpeedLimit * MPH)
      c["n"] += 1
      if d > 0:
        c["dash"] += 1; c["slc_uses"] += slc == d
        if mp > 0:
          c["map_avail"] += 1; c["map_agrees"] += mp == d
      src[p.slcSpeedLimitSource or "(none)"] += 1
  return c, src, params


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  args = p.parse_args()
  pct = lambda a, b: f"{100 * a / b:.0f}%" if b else "-"
  for route in args.routes:
    c = Counter(); src = Counter(); params = {}
    for cc, ss, pp in pmap(scan, segment_files(route, "qlog")):
      c.update(cc); src.update(ss); params = params or pp
    print(f"== {short(route)}: {c['n']} moving samples | dashboard limit present {pct(c['dash'], c['n'])} | "
          f"SLC used it {pct(c['slc_uses'], c['dash'])} | map agreed {pct(c['map_agrees'], c['map_avail'])} when available")
    print(f"   SLC source: {dict(src.most_common())}")
    print(f"   settings: {params or '-'}")


if __name__ == "__main__":
  main()
