#!/usr/bin/env python3
"""Count planner cycles where the logged MPC-derived accel drops >1.0 m/s^2 in one cycle while radarState.leadOne dRel/vRel/aLeadK
barely change (<1 m, <0.3 m/s, <0.3 m/s^2). Uses only logs (no replay): longitudinalPlan.aTarget and accels[0].
Usage: scan_mpc_drops.py ROUTE SEG [SEG ...]"""
import os, sys
DEV = os.environ.get("STINGER_DEV_HOME", os.path.expanduser("~/stinger/starpilot-dev"))
BUILD = os.environ.get("STARPILOT_BUILD", os.path.join(DEV, "x86"))
for p in (BUILD, os.path.join(BUILD, "opendbc_repo"), os.path.dirname(os.path.abspath(__file__))):
  sys.path.insert(0, p)
import planner_delay_replay as R  # noqa

route, segs = sys.argv[1], [int(x) for x in sys.argv[2:]]
tot = 0
for s in segs:
  frames, _ = R.load_frames(route, [s])
  n = 0
  for prev, cur in zip(frames, frames[1:]):
    if cur["t"] - prev["t"] > 0.2e9:
      continue
    lp, lc = prev["sm"]["radarState"].leadOne, cur["sm"]["radarState"].leadOne
    if not (lp.status and lc.status):
      continue
    da = cur["log"].aTarget - prev["log"].aTarget
    da0 = cur["log"].accels[0] - prev["log"].accels[0]
    dd, dv, dal = abs(lc.dRel - lp.dRel), abs(lc.vLead - lp.vLead), abs(lc.aLeadK - lp.aLeadK)
    if (da < -1.0 or da0 < -1.0) and dd < 1 and dv < 0.3 and dal < 0.3:
      n += 1
      print(f"seg{s} t={cur['route_t']:.2f} aTarget {prev['log'].aTarget:.2f}->{cur['log'].aTarget:.2f} accels0 {prev['log'].accels[0]:.2f}->{cur['log'].accels[0]:.2f} "
            f"src {cur['log'].longitudinalPlanSource} dRel {lc.dRel:.1f} d(d,v,a)=({dd:.2f},{dv:.2f},{dal:.2f}) vEgo {cur['sm']['carState'].vEgo:.1f} radar={lc.radar}")
  print(f"seg {s}: {len(frames)} cycles, {n} events")
  tot += n
print("TOTAL", tot)
