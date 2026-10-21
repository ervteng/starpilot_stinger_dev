#!/usr/bin/env python3
"""Compare openpilot stops behind a lead between groups of routes (e.g. two stop tunes).

Per stop: gap 0.5 s after standstill, speed when longControlState entered stopping, peak decel and peak jerk over
the last 3 s, and what happened in the next 15 s while the lead stayed parked: an automatic nudge forward
(openpilot moves the car without gas), a driver gas press, or nothing.

Usage: compare_stops.py --group NAME ROUTE [ROUTE ...] [--group NAME ROUTE ...]
"""
import sys
import numpy as np

from load import find_stops, load_long
from logs import segment_files


def analyse(s):
  out = []
  t0 = s["t0"]
  T = np.array(s["cs"].t, float)
  V = np.array([c["v"] for c in s["cs"].v])
  A = np.convolve([c["a"] for c in s["cs"].v], np.ones(3) / 3, mode="same")
  for ts, op, brk in find_stops(s):
    if not op or brk:
      continue
    t_stop = t0 + ts * 1e9
    lead = s["rs"].at(int(t_stop + 0.5e9))
    if not (lead and lead["status"] and lead["d"] < 15 and abs(lead["y"]) < 1.5):
      continue
    pre = (T >= t_stop - 3e9) & (T <= t_stop)
    if pre.sum() < 5:
      continue
    jerk = np.abs(np.diff(A[pre])) / np.maximum(np.diff(T[pre]) / 1e9, 1e-3)
    entry = None
    for t, v in zip(T[pre], V[pre]):
      c = s["cc"].at(int(t))
      if c and c["state"].startswith("stopping"):
        entry = float(v)
        break
    # follow-up while the lead stays parked
    redo, gap2 = "none", None
    post = np.where((T > t_stop + 0.3e9) & (T < t_stop + 15e9))[0]
    for i in post:
      l = s["rs"].at(int(T[i])); c = s["cs"].v[i]; cc = s["cc"].at(int(T[i]))
      if not (l and l["status"] and abs(l["v"]) < 0.3):
        break                      # lead left: normal stop-and-go, not a redo
      if V[i] > 0.15:
        redo = "driver gas" if c["gas"] else ("auto nudge" if cc and cc["long_active"] else "other")
        later = [j for j in post if j > i and V[j] < 0.05]
        if later:
          l2 = s["rs"].at(int(T[later[0]] + 0.5e9)); gap2 = l2["d"] if l2 and l2["status"] else None
        break
    out.append(dict(gap=lead["d"], entry=entry, decel=float(-A[pre].min()), jerk=float(jerk.max()) if len(jerk) else np.nan,
                    redo=redo, gap2=gap2))
  return out


def main():
  groups, cur = {}, None
  args = sys.argv[1:]
  i = 0
  while i < len(args):
    if args[i] == "--group":
      cur = args[i + 1]; groups[cur] = []; i += 2
    else:
      groups[cur].append(args[i]); i += 1
  for name, routes in groups.items():
    stops = []
    for r in routes:
      for f in segment_files(r, "qlog"):
        stops += analyse(load_long(f))
    if not stops:
      print(f"{name}: no stops"); continue
    g = np.array([x["gap"] for x in stops]); e = np.array([x["entry"] for x in stops if x["entry"] is not None])
    d = np.array([x["decel"] for x in stops]); j = np.array([x["jerk"] for x in stops])
    redo = [x for x in stops if x["redo"] != "none"]
    final = np.array([x["gap2"] if x["gap2"] is not None else x["gap"] for x in stops])
    print(f"== {name}: {len(stops)} stops behind a lead")
    print(f"   first-stop gap  median {np.median(g):.1f} m, middle half {np.percentile(g,25):.1f}-{np.percentile(g,75):.1f}, range {g.min():.1f}-{g.max():.1f}")
    print(f"   final gap       median {np.median(final):.1f} m (after any re-approach)")
    print(f"   needed a second approach with lead parked: {len(redo)}/{len(stops)} "
          f"(auto nudge {sum(x['redo']=='auto nudge' for x in redo)}, driver gas {sum(x['redo']=='driver gas' for x in redo)})")
    print(f"   speed when stopping began: median {np.median(e):.2f} m/s | peak decel last 3 s: median {np.median(d):.2f} m/s^2 | peak jerk: median {np.nanmedian(j):.1f} m/s^3")


if __name__ == "__main__":
  main()
