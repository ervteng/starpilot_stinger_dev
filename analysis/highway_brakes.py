#!/usr/bin/env python3
"""Openpilot braking episodes at highway speed, with the lead situation that caused each.

An episode starts when openpilot (longitudinal active, no pedals) commands below --threshold and lasts while the
command stays below -0.3 m/s^2. The cause is a rough label from the lead at the start: no lead, cut-in / new lead
(lead appeared or the gap dropped > 4 m in the 3 s before), lead braking (aLeadK < -1), closing on a slower lead,
gap shorter than tFollow, or other; "+vCruise below speed" and "+curve" flag speed-limit / curve slowdowns.

With --at SECONDS, print a 0.25 s timeline of ego, command and the two radard leads around that route time instead.

Usage: highway_brakes.py ROUTE [ROUTE ...] [--min-speed 20] [--threshold -0.8]
       highway_brakes.py ROUTE --at 708
"""
import argparse

from logs import events, pmap, route_time, segment_files, short

MPH = 2.23694


def load(path):
  rows = []; st = {}; t0 = None
  for e in events(path):
    w = e.which()
    if w == "carState":
      cs = e.carState
      st.update(v=cs.vEgo, a=cs.aEgo, pedal=cs.brakePressed or cs.gasPressed, cruise=cs.cruiseState.enabled)
      t0 = t0 or e.logMonoTime
    elif w == "carControl":
      st.update(cmd=e.carControl.actuators.accel, la=e.carControl.longActive)
    elif w == "starpilotPlan":
      p = e.starpilotPlan
      st.update(tf=p.tFollow, vc=p.vCruise, csc=p.cscBindingDistance, exp=p.experimentalMode)
    elif w == "radarState" and t0 and "cmd" in st:
      l1, l2 = e.radarState.leadOne, e.radarState.leadTwo
      rows.append((route_time(path, e.logMonoTime, t0), dict(st, ls=l1.status, d=l1.dRel, y=l1.yRel, vr=l1.vRel, vl=l1.vLead,
                   al=l1.aLeadK, rad=l1.radar, mp=l1.modelProb, d2=l2.dRel, v2=l2.vLead, s2=l2.status)))
  return rows


def episodes(rows, min_speed, threshold):
  out = []; i = 0
  while i < len(rows):
    t, s = rows[i]
    if s.get("la") and s["v"] > min_speed and s["cmd"] < threshold and not s["pedal"]:
      j = i
      while j < len(rows) and rows[j][1]["cmd"] < -0.3 and not rows[j][1]["pedal"]:
        j += 1
      ep = rows[i:j]
      if ep[-1][0] - t >= 0.3:
        pre = [r[1] for r in rows[max(0, i - 60):i] if t - r[0] <= 3]
        seen = [r["d"] for r in pre if r["ls"]]
        cut = any(not r["ls"] for r in pre) or (seen and min(seen) < max(seen) - 4 and s["d"] < pre[0]["d"] - 4)
        gap_t = s["d"] / max(s["v"], 1)
        if not s["ls"]: cause = "no lead"
        elif cut: cause = "cut-in / new lead"
        elif s["al"] < -1.0: cause = "lead braking"
        elif s["vr"] < -1.5: cause = "closing on slower lead"
        elif gap_t < s.get("tf", 1.45) - 0.3: cause = "gap too short"
        else: cause = "other"
        if s.get("vc", 99) < s["v"] - 1: cause += " +vCruise below speed"
        if s.get("csc", 0) > 0: cause += " +curve"
        out.append((t, s, min(e[1]["cmd"] for e in ep), min(e[1]["a"] for e in ep), ep[-1][0] - t, gap_t, cause))
      i = j + 1
    else:
      i += 1
  return out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--min-speed", type=float, default=20.0, help="m/s (default 20 = 45 mph)")
  p.add_argument("--threshold", type=float, default=-0.8, help="command that starts an episode, m/s^2")
  p.add_argument("--at", type=float, help="route time (s) for a lead timeline")
  args = p.parse_args()
  for route in args.routes:
    files = segment_files(route, "qlog")
    if args.at is not None:
      seg = int(args.at // 60)
      files = [f for f in files if seg - 1 <= int(f.rsplit("--", 2)[1]) <= seg + 1]
    rows = sorted((r for rs in pmap(load, files) for r in rs), key=lambda r: r[0])
    if args.at is not None:
      print(f"== {short(route)} around {args.at:.0f}s (dt from that time)")
      last = None
      for t, s in rows:
        if args.at - 4 <= t <= args.at + 4 and (last is None or t - last >= 0.25):
          last = t
          print(f"{t - args.at:5.1f} v={s['v'] * MPH:4.1f} aEgo={s['a']:5.2f} cmd={s['cmd']:5.2f} | lead1 {'Y' if s['ls'] else '-'} "
                f"d={s['d']:5.1f} y={s['y']:5.1f} vLead={s['vl'] * MPH:4.1f}mph vRel={s['vr']:5.2f} aLeadK={s['al']:5.2f} "
                f"radar={int(s['rad'])} prob={s['mp']:.2f} | lead2 {'Y' if s['s2'] else '-'} d={s['d2']:5.1f} vLead={s['v2'] * MPH:4.1f}")
      continue
    eps = episodes(rows, args.min_speed, args.threshold)
    print(f"== {short(route)}: {len(eps)} braking episodes above {args.min_speed * MPH:.0f} mph")
    for t, s, pk, pa, dur, gap_t, cause in eps:
      print(f"{t:6.0f}s v={s['v'] * MPH:3.0f}mph peak cmd={pk:5.2f} peak aEgo={pa:5.2f} {dur:4.1f}s | lead {'Y' if s['ls'] else 'N'} "
            f"d={s['d']:5.1f}m gap={gap_t:3.1f}s tFollow={s.get('tf', 0):.2f} vRel={s['vr']:5.2f} aLeadK={s['al']:5.2f} "
            f"radar={int(s['rad'])} prob={s['mp']:.2f} exp={int(bool(s.get('exp')))} -> {cause}")


if __name__ == "__main__":
  main()
