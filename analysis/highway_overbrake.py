#!/usr/bin/env python3
"""Does openpilot over-brake at highway speed when a lead slows while the gap is still large?

Episodes come from highway_brakes.py (same loading and detection: command below --threshold with ego above
--min-speed). For stock ACC drives (openpilot not in control) the episode signal is carState aEgo instead of the
command, and "active" is carState.cruiseState.enabled (as in lead_braking.py / stopped_lead_approaches.py qlogs).
A group is treated as stock when its name contains "stock".

Desired gap D, from StarPilot's selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py (stinger-combined), copied
rather than imported to avoid importing the openpilot package: COMFORT_BRAKE=2.5 (l.155), STOP_DISTANCE=6.0 (l.156),
get_stopped_equivalence_factor (l.265), get_safe_obstacle_distance (l.268), desired_follow_distance (l.271). t_follow is
the logged starpilotPlan.tFollow.
  --gap-model headway (default): D = t_follow*v_ego + STOP_DISTANCE, the time-headway part of
      get_safe_obstacle_distance (l.269). The kinematic need below already accounts for closing speed, so the planner's
      braking-distance terms v^2/(2*COMFORT_BRAKE) are left out (they would double count it).
  --gap-model mpc: the full desired_follow_distance(v_ego, v_lead, t_follow) = v_ego^2/5 + t_follow*v_ego + 6 - v_lead^2/5,
      which at highway speed with a slower lead is far larger than the actual gap, so almost everything is "inside".

Needed decel (positive magnitude), gap d, ego v, lead vL, aL = min(aLeadK, 0), taken over a trailing
--al-smooth window (default 1.0 s, restarted at a lead switch). The raw 20 Hz aLeadK swings by +-1 m/s^2 between
samples and lags, which inflates the peak, the default (--al-source leadk) uses the mean aLeadK over that window;
--al-source vlead uses the slope of vLead instead (noisy right after a switch). The tables also give a "hold" need
and ratio with the lead's decel ignored (aL = 0, lead holds speed), a lower bound on the need:
  v > vL, d > D :  -aL + (v - vL)^2 / (2 (d - D))
  v > vL, d <= D:  -aL + (v - vL)^2 / (2 * 1.0)   (flagged "inside desired gap")
  v <= vL       :  -aL
Lead switch: in the episode or the 2 s before, leadOne dRel jumps > 5 m between consecutive samples, the radar flag
flips, the lead appears (status 0 -> 1), or highway_brakes' cut-in rule fires at onset (then dt = 0). Peaks of needed
and command (aEgo for stock) are taken from onset up to the first switch after onset (whole episode if none; a switch
before onset is only shown as "earlier": the new lead is already the lead at onset, so it is judged against that lead). Over-braking:
excess = peak command - peak needed > 0.5 and ratio > 1.5, measured before any lead switch.

Usage: highway_overbrake.py --group NAME ROUTE [ROUTE ...] [--group NAME ROUTE ...] [--min-speed 20] [--threshold -0.8]
"""
import argparse
import sys

import numpy as np

from highway_brakes import MPH, episodes, load
from logs import pmap, segment_files, short

COMFORT_BRAKE = 2.5  # long_mpc.py:155
STOP_DISTANCE = 6.0  # long_mpc.py:156
JUMP_M = 5.0
PRE_S = 2.0
MIN_EXCESS = 0.5
MIN_RATIO = 1.5


GAP_MODEL = "headway"
AL_SMOOTH = 1.0
AL_SOURCE = "leadk"


def desired_follow_distance(v_ego, v_lead, t_follow):
  if GAP_MODEL == "headway":  # long_mpc.py:269 without the braking-distance term
    return t_follow * v_ego + STOP_DISTANCE
  # long_mpc.py:265-272
  return v_ego ** 2 / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE - v_lead ** 2 / (2 * COMFORT_BRAKE)


def needed_decel(d, v, vl, al, tf):
  """(positive braking magnitude m/s^2, inside-desired-gap flag)."""
  al = min(al, 0.0)
  if v <= vl:
    return -al, False
  D = desired_follow_distance(v, vl, tf)
  if d > D:
    return -al + (v - vl) ** 2 / (2 * (d - D)), False
  return -al + (v - vl) ** 2 / 2.0, True


def lead_switch(rows, i0, i1, cutin):
  """(first switch after onset, last switch in the 2 s before onset) as route seconds or None; cutin -> onset time."""
  t_on = rows[i0][0]
  pre = None
  lo = i0
  while lo > 0 and t_on - rows[lo - 1][0] <= PRE_S:
    lo -= 1
  for k in range(max(lo, 1), i1):
    a, b = rows[k - 1][1], rows[k][1]
    if is_switch(a, b):
      if rows[k][0] > t_on:
        return rows[k][0], pre
      pre = rows[k][0]
  return (t_on if cutin else None), pre


def is_switch(a, b):
  return (not a["ls"] and b["ls"]) or (a["ls"] and b["ls"] and (abs(b["d"] - a["d"]) > JUMP_M or bool(a["rad"]) != bool(b["rad"])))


def smooth_al(rows, win, source):
  """Rows with s["al"] = the lead acceleration estimate (raw aLeadK kept as s["al_raw"]).

  "vlead": slope of vLead over the trailing `win` s (>= 4 samples spanning >= 0.4 s, else the mean aLeadK there).
  "leadk": trailing mean of aLeadK. Both restart at a lead switch so the old lead is not averaged in.
  """
  out = []; j = 0
  for i, (t, s) in enumerate(rows):
    if i and is_switch(rows[i - 1][1], s):
      j = i
    while win > 0 and rows[j][0] < t - win:
      j += 1
    w = rows[j:i + 1] if win > 0 else rows[i:i + 1]
    mean = sum(r[1]["al"] for r in w) / len(w)
    al = mean
    if source == "vlead" and len(w) >= 4 and w[-1][0] - w[0][0] >= 0.4:
      tt = np.array([r[0] for r in w]); vv = np.array([r[1]["vl"] for r in w])
      al = float(np.polyfit(tt - tt.mean(), vv, 1)[0])
    out.append((t, dict(s, al_raw=s["al"], al=al)))
  return out


def analyze(rows, min_speed, threshold, stock):
  rows = smooth_al(rows, AL_SMOOTH, AL_SOURCE)
  if stock:  # stock ACC: engaged = cruiseState.enabled, braking signal = aEgo
    rows = [(t, dict(s, la=s.get("cruise", False), cmd=s["a"])) for t, s in rows]
  out = []
  for t, s, pk_cmd, pk_a, dur, gap_t, cause in episodes(rows, min_speed, threshold):
    i0 = next(i for i, r in enumerate(rows) if r[0] == t)
    i1 = i0
    while i1 < len(rows) and rows[i1][0] <= t + dur + 1e-9:
      i1 += 1
    tsw, tpre = lead_switch(rows, i0, i1, cause.startswith("cut-in"))
    stop = i1
    if tsw is not None and tsw > t:
      stop = next(k for k in range(i0, i1) if rows[k][0] >= tsw)
    seg = rows[i0:max(stop, i0 + 1)]
    needs = [needed_decel(r["d"], r["v"], r["vl"], r["al"], r.get("tf") or 1.45)[0] if r["ls"] else 0.0 for _, r in seg]
    need0, inside = (needed_decel(s["d"], s["v"], s["vl"], s["al"], s.get("tf") or 1.45) if s["ls"] else (0.0, False))
    pk_need = max(needs)
    pk_hold = max(needed_decel(r["d"], r["v"], r["vl"], 0.0, r.get("tf") or 1.45)[0] if r["ls"] else 0.0 for _, r in seg)
    pk_c = max(-r["cmd"] for _, r in seg)
    ratio = pk_c / max(pk_need, 0.05)
    excess = pk_c - pk_need
    t_pk = seg[int(np.argmax([-r["cmd"] for _, r in seg]))][0]
    out.append(dict(t=t, mph=s["v"] * MPH, d=s["d"], lead=bool(s["ls"]), closing=s["v"] - s["vl"], al=s["al_raw"], tf=s.get("tf") or 0.0,
                    need0=need0, inside=inside, pk_need=pk_need, pk_hold=pk_hold, pk_cmd=pk_c, ratio=ratio, excess=excess, t_pk=t_pk,
                    sw=None if tsw is None else tsw - t, sw_pre=None if tpre is None else tpre - t, exp=bool(s.get("exp")), cause=cause, rule=bool(tsw is not None and tsw == t),
                    over=bool(s["ls"] and excess > MIN_EXCESS and ratio > MIN_RATIO)))
  return out


def load_route(args):
  route, stock, min_speed, threshold = args
  files = segment_files(route, "qlog")
  rows = sorted((r for rs in pmap(load, files) for r in rs), key=lambda r: r[0])
  return [dict(e, route=short(route)) for e in analyze(rows, min_speed, threshold, stock)]


def table(name, eps, stock):
  print(f"### {name}: per-episode ({'peak aEgo' if stock else 'peak cmd'} before any lead switch)\n")
  print("| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak "
        + ("aEgo" if stock else "cmd") + " | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |")
  print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
  for e in eps:
    sw = "-" if e["sw"] is None else f"{e['sw']:+.1f}s"
    if e["rule"]:
      sw = "cut-in rule"
    if e["sw_pre"] is not None:
      sw += f" (earlier {e['sw_pre']:+.1f}s)"
    print(f"| {e['route']} | {e['t']:.0f} | {e['mph']:.0f} | {e['d']:.1f} | {e['closing']:.1f} | {e['al']:.2f} | {e['tf']:.2f} | "
          f"{e['need0']:.2f} | {e['pk_need']:.2f} | {e['pk_cmd']:.2f} | {e['ratio']:.1f} | {e['pk_hold']:.2f} | {e['pk_cmd'] / max(e['pk_hold'], 0.05):.1f} | {sw} | {'Y' if e['inside'] else '-'} | "
          f"{'Y' if e['exp'] else '-'} | {'**OVER**' if e['over'] else '-'} | {e['cause']} |" + ("" if e["lead"] else " (no lead)"))
  print()


def summary_row(name, eps):
  lead = [e for e in eps if e["lead"]]
  r = np.array([e["ratio"] for e in lead]); x = np.array([e["excess"] for e in lead])
  med = lambda a, q=50: f"{np.percentile(a, q):.2f}" if len(a) else "-"
  return (f"| {name} | {len(eps)} | {len(lead)} | {sum(e['sw'] is not None or e['sw_pre'] is not None for e in lead)} | {sum(e['over'] for e in lead)} | "
          f"{med(r)} | {med(r, 90)} | {med(x)} | {med(np.array([e['pk_cmd'] / max(e['pk_hold'], 0.05) for e in lead]))} |")


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--group", nargs="+", action="append", metavar=("NAME", "ROUTE"), required=True)
  p.add_argument("--min-speed", type=float, default=20.0, help="m/s (default 20 = 45 mph)")
  p.add_argument("--threshold", type=float, default=-0.8, help="command (stock: aEgo) that starts an episode, m/s^2")
  p.add_argument("--gap-model", choices=("headway", "mpc"), default="headway", help="desired gap D (see docstring)")
  p.add_argument("--al-smooth", type=float, default=1.0, help="seconds of trailing mean on aLeadK (0 = raw)")
  p.add_argument("--al-source", choices=("vlead", "leadk"), default="leadk", help="lead accel for the needed decel (see docstring)")
  args = p.parse_args()
  global GAP_MODEL, AL_SMOOTH, AL_SOURCE
  GAP_MODEL, AL_SMOOTH, AL_SOURCE = args.gap_model, args.al_smooth, args.al_source
  allep = []; summ = []
  for name, *routes in args.group:
    if not routes:
      sys.exit(f"group {name} has no routes")
    stock = "stock" in name.lower()
    eps = []
    for r in routes:
      eps += load_route((r, stock, args.min_speed, args.threshold))
    for e in eps:
      e["group"] = name
    table(name, eps, stock)
    summ.append(summary_row(name, eps))
    allep += eps
  print("### Summary (episodes with a lead; ratio and excess = peak command (stock: aEgo) minus needed, before lead switch)\n")
  print("| group | n episodes | n with lead | n lead switch | n over-braking | ratio median | ratio p90 | excess median m/s^2 | ratio (aL=0) median |")
  print("|---|---|---|---|---|---|---|---|---|")
  print("\n".join(summ)); print()
  worst = sorted((e for e in allep if e["over"]), key=lambda e: -e["excess"])[:5]
  print("### Worst 5 over-braking episodes (by excess)\n")
  print("| group | route | t (s) | mph | gap m | peak need | peak cmd | excess | ratio | cause | replay |")
  print("|---|---|---|---|---|---|---|---|---|---|---|")
  for e in worst:
    print(f"| {e['group']} | {e['route']} | {e['t']:.0f} | {e['mph']:.0f} | {e['d']:.1f} | {e['pk_need']:.2f} | {e['pk_cmd']:.2f} | "
          f"{e['excess']:.2f} | {e['ratio']:.1f} | {e['cause']} | `highway_brakes.py {e['route']} --at {e['t']:.0f}` |")


if __name__ == "__main__":
  main()
