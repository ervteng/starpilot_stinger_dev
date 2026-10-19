#!/usr/bin/env python3
"""Print a time window of one segment: speed, long state, accel commanded vs sent to the car,
planner output, radar lead (dRel, vLead, aLeadK) and model lead.

Usage: stop_window.py LOGFILE START END [--step 0.25]
"""
import argparse

from load import load_long

RADAR_TO_CAMERA = 1.52


def main():
  p = argparse.ArgumentParser()
  p.add_argument("log")
  p.add_argument("start", type=float)
  p.add_argument("end", type=float)
  p.add_argument("--step", type=float, default=0.25)
  args = p.parse_args()

  s = load_long(args.log)
  print(s["info"])
  print("   t     v   ss | state   cmd   sent | plan aT stop v_end | lead d  vLead aLeadK src | model p  d")
  nxt = args.start
  for t, c in zip(s["cs"].t, s["cs"].v):
    rt = (t - s["t0"]) / 1e9
    if not args.start <= rt <= args.end or rt < nxt:
      continue
    nxt = rt + args.step
    cc, co, lp, l, md = (s[k].at(t) for k in ("cc", "co", "lp", "rs", "md"))
    cc = cc or dict(state="-", accel=float("nan"))
    lp = lp or dict(a_target=float("nan"), should_stop=False, v_end=float("nan"))
    lead = f"{l['d']:5.1f} {l['v']:+.2f} {l['a']:+.2f} {'R' if l['radar'] else 'V'}" if l and l["status"] else "    -              "
    model = f"{md[0]:.2f} {md[1] - RADAR_TO_CAMERA:5.1f}" if md else "-"
    print(f"{rt:5.1f} {c['v']:5.2f} {int(c['standstill'])}{'B' if c['brake'] else ' '}{'G' if c['gas'] else ' '}"
          f"| {cc['state'][:6]:6s} {cc['accel']:5.2f} {co if co is not None else float('nan'):5.2f} | "
          f"{lp['a_target']:5.2f} {int(lp['should_stop'])} {lp['v_end']:5.2f} | {lead} | {model}")


if __name__ == "__main__":
  main()
