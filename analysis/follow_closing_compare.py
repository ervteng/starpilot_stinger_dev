#!/usr/bin/env python3
"""Following distance and closing on slower leads at highway speed: factory ACC ("stock") vs openpilot, same sensor.

Every group is measured from the qlogs with openpilot's radarState leadOne (stock drives ran openpilot passively, so
radarState exists there). Rows are one per radarState (4 Hz in qlogs) with the latest carState (10 Hz), carControl and
starpilotPlan values. "Engaged" is carState.cruiseState.enabled for a group whose name contains "stock" and
carControl.longActive otherwise; "no pedals" is neither gasPressed nor brakePressed. Time gap = dRel / vEgo.
Stock rlogs (when a route has them) add the factory view from raw CAN, joined to the qlog rows by timestamp (nearest SCC
message within 0.1 s): SCC11 ACC_ObjDist / ACC_ObjStatus / TauGapSet (the driver's distance setting) and SCC12 aReqValue.
Lead switch: highway_overbrake.is_switch (lead appears, dRel jumps > 5 m, radar flag flips). A gap > 0.6 s between rows
(missing segment) breaks every run. aEgo used for braking is the centered 0.5 s mean of carState.aEgo for all groups.

1. Steady following. Samples: engaged, no pedals, v >= --min-speed, lead tracked, |vRel| < 0.5 m/s, |smoothed aEgo| < 0.3,
   held for >= 5 s in one run (no lead switch, no sample failing). A run is a "stretch"; seconds are the run durations.
   Speed bins (mph) 45-55 (from --min-speed), 55-65, 65+ use the sample speed; a stretch is counted in every bin it touches.
   Stats are over samples (4 Hz, uniform in time): median / p10 / p90 of the time gap.
2. Closing events. Start: first row with engaged, no pedals, v >= --min-speed, lead tracked and closing v - vLead >=
   --close-start (1.5 m/s). The same lead must then stay tracked (engaged, no pedals, no switch) for >= --min-track (4 s) from
   the start, else the candidate is dropped. The event ends at the first of: closing < --close-end (0.3 m/s) = speed match,
   20 s, or the lead lost / switched / driver input. After a 20 s timeout a new event needs closing < 1.5 first.
   braking onset   first row of the event where the smoothed aEgo <= -0.3 (equal to "braking reaches 0.3 m/s^2"); if the event
                   starts already braking the onset is the start row (flagged "pre"); none -> "no braking" (coasted)
   cmd             op: actuators.accel at onset; stock: SCC12 aReqValue at onset (rlog drives only, else "-"); cmd gap is the
                   gap where the command first reached <= -0.3 (not shown when unavailable)
   peak decel      -min(smoothed aEgo) from start to event end; includes road grade
   min time gap    min dRel/vEgo from start to end;  tg@match  time gap at the speed-match row (matched events only)
   need            highway_overbrake.needed_decel with D = tFollow*v + 6 (lead accel = smoothed aLeadK, capped <= 0); stock
                   assumes tFollow = 1.3 (no logged value)
   MPC desired     v^2/(2*2.5) + tFollow*v + 6 - vLead^2/(2*2.5) (long_mpc.py l.155-156, 265-272), tFollow = logged
                   starpilotPlan.tFollow at onset (stock 1.3); ratio = gap / MPC desired
   Bins by closing speed at the start: 1.5-3, 3-5, 5+ m/s. The extra row "all, sustained" keeps the events whose mean
   closing over their first 3 s is still >= 1.5 m/s (a guard against single noisy vLead samples; vision leads are noisy).
3. Bottom line: per group steady time gap median, onset TTC median (braked events), peak decel median (all events),
   share of events with peak decel >= 1.5 m/s^2.

Usage: follow_closing_compare.py --group NAME ROUTE [ROUTE ...] [--group NAME ROUTE ...] [--min-speed 20] [--starpilot DIR]
Routes are short names XXXXXXXX--hash. Stock rlog parsing needs opendbc (--starpilot, read only).
"""
import argparse
import bisect
import sys

import numpy as np

import highway_overbrake as HO
from highway_brakes import MPH
from logs import add_starpilot_path, pmap, segment_files, short, sorted_events

STOCK_TF = 1.3
MAX_DT = 0.6
SMOOTH = 0.5
PERS = {0: "aggressive", 1: "standard", 2: "relaxed"}
SPEED_BINS = ((20.0, 55 / MPH, "45-55"), (55 / MPH, 65 / MPH, "55-65"), (65 / MPH, 999.0, "65+"))
CLOSE_BINS = ((1.5, 3.0, "1.5-3"), (3.0, 5.0, "3-5"), (5.0, 999.0, "5+"))


# ---------------------------------------------------------------- loading
def load_qlog(path):
  """(carState rows [t, aEgo], radar rows (dicts), initData LongitudinalPersonality or None)."""
  cs, rows, st, pers = [], [], {}, None
  for e in sorted_events(path):
    w = e.which()
    t = e.logMonoTime / 1e9
    if w == "initData":
      for p in e.initData.params.entries:
        if p.key == "LongitudinalPersonality":
          pers = int(p.value)
    elif w == "carState":
      c = e.carState
      st.update(v=c.vEgo, a=c.aEgo, ped=bool(c.brakePressed or c.gasPressed), cruise=bool(c.cruiseState.enabled))
      cs.append((t, c.aEgo))
    elif w == "carControl":
      st.update(cmd=e.carControl.actuators.accel, la=bool(e.carControl.longActive))
    elif w == "starpilotPlan":
      st["tf"] = e.starpilotPlan.tFollow
    elif w == "selfdriveState":
      st["pers"] = str(e.selfdriveState.personality)
    elif w == "radarState" and "v" in st and "cmd" in st:
      l = e.radarState.leadOne
      rows.append(dict(st, t=t, ls=bool(l.status), d=l.dRel, vr=l.vRel, vl=l.vLead, al=l.aLeadK, rad=bool(l.radar)))
  return cs, rows, pers


def load_scc(path):
  """Rows [t, ObjDist, ObjStatus, aReqValue, TauGapSet, ACCMode] at SCC12 rate from a stock rlog."""
  add_starpilot_path(ARGS_STARPILOT)
  from opendbc.can import CANParser
  p = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0)], 0)
  out = []
  for e in sorted_events(path):
    if e.which() != "can":
      continue
    p.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
    if p.can_valid and any(c.address == 0x421 and c.src == 0 for c in e.can):
      s11, s12 = p.vl["SCC11"], p.vl["SCC12"]
      out.append((e.logMonoTime / 1e9, s11["ACC_ObjDist"], s11["ACC_ObjStatus"], s12["aReqValue"], s11["TauGapSet"], s12["ACCMode"]))
  return np.array(out, dtype=float).reshape(-1, 6)


def smooth_centered(t, x, tq, w):
  c = np.concatenate([[0], np.cumsum(x)])
  lo, hi = np.searchsorted(t, tq - w / 2), np.searchsorted(t, tq + w / 2, side="right")
  with np.errstate(invalid="ignore", divide="ignore"):
    return np.where(hi > lo, (c[hi] - c[lo]) / np.maximum(hi - lo, 1), np.nan)


def load_group_routes(routes, stock):
  """{route: dict(rows, pers_param)} with rows as time-ordered dicts incl. as_ (smoothed aEgo), eng, scc_* for stock rlogs."""
  qfiles = [(r, f) for r in routes for f in segment_files(r, "qlog")]
  qres = pmap(load_qlog, [f for _, f in qfiles])
  sfiles = [(r, f) for r in routes for f in segment_files(r, "rlog")] if stock else []
  sres = pmap(load_scc, [f for _, f in sfiles]) if sfiles else []
  out = {}
  for r in routes:
    cs = sorted(x for (rr, _), res in zip(qfiles, qres) if rr == r for x in res[0])
    rows = sorted((x for (rr, _), res in zip(qfiles, qres) if rr == r for x in res[1]), key=lambda d: d["t"])
    pers = next((res[2] for (rr, _), res in zip(qfiles, qres) if rr == r and res[2] is not None), None)
    if not rows:
      print(f"  no qlog rows for {r}", file=sys.stderr)
      continue
    ct, ca = np.array([c[0] for c in cs]), np.array([c[1] for c in cs])
    sm = smooth_centered(ct, ca, np.array([x["t"] for x in rows]), SMOOTH)
    scc = [res for (rr, _), res in zip(sfiles, sres) if rr == r and len(res)]
    scc = np.vstack(scc)[np.argsort(np.concatenate([s[:, 0] for s in scc]), kind="stable")] if scc else None
    sccs = scc[:, 0] if scc is not None else None
    for x, a in zip(rows, sm):
      x["as_"] = a
      x["eng"] = x["cruise"] if stock else x["la"]
      x["scc_d"] = x["scc_st"] = x["areq"] = x["tau"] = x["scc_mode"] = np.nan
      if scc is not None:
        k = bisect.bisect_left(sccs, x["t"])
        k = min((j for j in (k - 1, k) if 0 <= j < len(sccs)), key=lambda j: abs(sccs[j] - x["t"]))
        if abs(sccs[k] - x["t"]) <= 0.1:
          x["scc_d"], x["scc_st"], x["areq"], x["tau"], x["scc_mode"] = scc[k, 1:6]
    out[r] = dict(rows=rows, pers=pers)
  return out


# ---------------------------------------------------------------- analysis
def runs(rows, ok):
  """Maximal index ranges [i0, i1] of consecutive rows with ok, dt <= MAX_DT and no lead switch between them."""
  out, i0 = [], None
  for i, r in enumerate(rows):
    if ok[i] and i0 is not None and (r["t"] - rows[i - 1]["t"] > MAX_DT or HO.is_switch(rows[i - 1], r)):
      out.append((i0, i - 1)); i0 = None
    if ok[i]:
      if i0 is None:
        i0 = i
    elif i0 is not None:
      out.append((i0, i - 1)); i0 = None
  if i0 is not None:
    out.append((i0, len(rows) - 1))
  return out


def steady(rows, min_speed, min_hold=5.0):
  """Samples of steady-following stretches: list of (stretch id, row)."""
  ok = [r["eng"] and not r["ped"] and r["v"] >= min_speed and r["ls"] and abs(r["vr"]) < 0.5
        and abs(r["as_"]) < 0.3 for r in rows]
  out = []
  for i0, i1 in runs(rows, ok):
    if rows[i1]["t"] - rows[i0]["t"] >= min_hold:
      out += [((i0, i1), r) for r in rows[i0:i1 + 1]]
  return out


def mpc_desired(v, vl, tf):
  return v ** 2 / (2 * HO.COMFORT_BRAKE) + tf * v + HO.STOP_DISTANCE - vl ** 2 / (2 * HO.COMFORT_BRAKE)


def closing_events(rows, stock, a):
  """Closing events of one route (see module docstring)."""
  rows = HO.smooth_al([(r["t"], r) for r in rows], 1.0, "leadk")
  rows = [s for _, s in rows]
  n = len(rows)
  elig = [r["eng"] and not r["ped"] and r["ls"] for r in rows]
  out, rejected, i, armed = [], 0, 0, True
  while i < n:
    r = rows[i]
    close = r["v"] - r["vl"]
    if not armed:
      if not r["ls"] or close < a.close_start:
        armed = True
      i += 1
      continue
    if not (elig[i] and r["v"] >= a.min_speed and close >= a.close_start):
      i += 1
      continue
    k = i
    while k + 1 < n and elig[k + 1] and rows[k + 1]["t"] - rows[k]["t"] <= MAX_DT and not HO.is_switch(rows[k], rows[k + 1]):
      k += 1
    if rows[k]["t"] - r["t"] < a.min_track:
      rejected += 1
      i = k + 1
      continue
    j, reason = i, None
    while True:
      if j > i and rows[j]["v"] - rows[j]["vl"] < a.close_end:
        reason = "matched"; break
      if rows[j]["t"] - r["t"] >= 20.0:
        reason = "timeout"; break
      if j == k:
        reason = "lost/driver"; break
      j += 1
    out.append(event(rows[i:j + 1], reason, stock, i))
    armed = reason != "timeout"
    i = j + 1
  return out, rejected


def event(seg, reason, stock, idx):
  s = seg[0]
  tf_of = lambda r: STOCK_TF if stock else (r.get("tf") or 1.45)
  close0 = s["v"] - s["vl"]
  ons = next((q for q, r in enumerate(seg) if r["as_"] <= -0.3), None)
  e = dict(t=s["t"], idx=idx, v0=s["v"], vl0=s["vl"], d0=s["d"], close0=close0, tg0=s["d"] / s["v"], ttc0=s["d"] / close0,
           reason=reason, dur=seg[-1]["t"] - s["t"], age_new=False, radar=s["rad"],
           peak=-min(r["as_"] for r in seg), mintg=min(r["d"] / r["v"] for r in seg),
           tg_match=seg[-1]["d"] / seg[-1]["v"] if reason == "matched" else np.nan,
           d_match=seg[-1]["d"] if reason == "matched" else np.nan,
           sust=float(np.mean([r["v"] - r["vl"] for r in seg if r["t"] - s["t"] <= 3.0])) >= 1.5, tf0=tf_of(s), onset=ons is not None, pre=ons == 0)
  cmd_key = "areq" if stock else "cmd"
  cm = next((r for r in seg if r[cmd_key] <= -0.3), None) if not np.isnan(seg[0][cmd_key]) else None
  e["cmd_gap"] = cm["d"] if cm else np.nan
  e["cmd_known"] = not np.isnan(seg[0][cmd_key])
  if ons is not None:
    r = seg[ons]
    c = r["v"] - r["vl"]
    tf = tf_of(r)
    need, inside = HO.needed_decel(r["d"], r["v"], r["vl"], r["al"], tf)
    mpc = mpc_desired(r["v"], r["vl"], tf)
    e.update(on_t=r["t"] - s["t"], on_v=r["v"], on_d=r["d"], on_close=c, on_tg=r["d"] / r["v"],
             on_ttc=r["d"] / c if c > 0.1 else np.nan, on_cmd=r[cmd_key], need=need, inside=inside,
             mpc=mpc, ratio=r["d"] / mpc, tf_on=tf)
  return e


# ---------------------------------------------------------------- reporting
def pc(x, q):
  x = np.asarray(x, dtype=float)
  x = x[~np.isnan(x)]
  return float(np.percentile(x, q)) if len(x) else np.nan


def f(x, p=2):
  return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{p}f}"


def steady_tables(name, per_route, stock):
  """Prints the steady-following table for one group; returns the all-speed median time gap."""
  samples = [(rt, sid, r) for rt, d in per_route.items() for sid, r in steady(d["rows"], ARGS.min_speed)]
  print(f"### {name}: steady following (>= 5 s, vRel < 0.5, |a| < 0.3, no lead switch)\n")
  print("| speed bin (mph) | stretches | seconds | samples | tg median | tg p10 | tg p90 | gap median m | radar-lead share |")
  print("|---|---|---|---|---|---|---|---|---|")
  allg = [r["d"] / r["v"] for _, _, r in samples]
  for lo, hi, lab in SPEED_BINS + ((0.0, 999.0, "all"),):
    sel = [(rt, sid, r) for rt, sid, r in samples if lo <= r["v"] < hi] if lab != "all" else samples
    g = [r["d"] / r["v"] for _, _, r in sel]
    secs = {}
    for rt, sid, r in sel:
      secs.setdefault((rt, sid), []).append(r["t"])
    sec = sum(max(v) - min(v) for v in secs.values())
    print(f"| {lab} | {len(secs)} | {sec:.0f} | {len(sel)} | {f(pc(g, 50))} | {f(pc(g, 10))} | {f(pc(g, 90))} | "
          f"{f(pc([r['d'] for _, _, r in sel], 50), 1)} | {f(np.mean([r['rad'] for _, _, r in sel]) if sel else np.nan, 2)} |")
  print()
  return pc(allg, 50), samples


def tfollow_summary(name, per_route, pers_info, samples, stock):
  """tFollow logged vs measured (op) and personality info."""
  print(f"### {name}: personality and logged tFollow\n")
  print("| route | LongitudinalPersonality param | selfdriveState.personality (rows) | logged tFollow median (p10-p90) engaged v>=min | n qlog rows |")
  print("|---|---|---|---|---|")
  for rt, d in per_route.items():
    rows = d["rows"]
    pc_ = {}
    for r in rows:
      pc_[r.get("pers")] = pc_.get(r.get("pers"), 0) + 1
    tf = [r["tf"] for r in rows if r["eng"] and r["v"] >= ARGS.min_speed and r.get("tf")]
    pname = "-" if d["pers"] is None else f"{d['pers']} ({PERS.get(d['pers'], '?')})"
    print(f"| {short(rt)} | {pname} | {', '.join(f'{k} {v / len(rows):.0%}' for k, v in sorted(pc_.items(), key=lambda kv: str(kv[0])))} | "
          f"{'stock: n/a (assumed %.2f)' % STOCK_TF if stock else f'{f(pc(tf, 50))} ({f(pc(tf, 10))}-{f(pc(tf, 90))})'} | {len(rows)} |")
  print()
  if not stock:
    g = [r["d"] / r["v"] for _, _, r in samples]
    tf = [r.get("tf") or np.nan for _, _, r in samples]
    print(f"Steady stretches: logged tFollow median {f(pc(tf, 50))} (p10 {f(pc(tf, 10))}, p90 {f(pc(tf, 90))}) vs measured time gap "
          f"median {f(pc(g, 50))} (p10 {f(pc(g, 10))}, p90 {f(pc(g, 90))}); measured minus logged median "
          f"{f(pc(g, 50) - pc(tf, 50))} s. (The MPC distance is tFollow*v + 6 m, i.e. +6/v s ~ +0.3 s at 20 m/s on top.)\n")


def stock_scc_tables(per_route, samples):
  """TauGapSet distribution and sensor cross-check for the stock rlog drives."""
  have = {rt: d for rt, d in per_route.items() if any(not np.isnan(r["scc_d"]) for r in d["rows"])}
  if not have:
    print("No stock rlog data joined; skipped.\n")
    return
  print(f"### stock: factory distance setting (SCC11 TauGapSet) on rlog drives {', '.join(short(r) for r in have)}\n")
  print("| subset | n | " + " | ".join(f"TauGapSet {k}" for k in range(1, 5)) + " | other |")
  print("|---|---|---|---|---|---|---|")
  eng = [r for rt in have for r in have[rt]["rows"] if r["eng"] and not r["ped"] and r["v"] >= ARGS.min_speed and not np.isnan(r["tau"])]
  st = [r for rt, _, r in samples if rt in have and not np.isnan(r["tau"])]
  for lab, sel in (("engaged, v >= min, no pedal (all rows)", eng), ("steady-following samples", st)):
    c = np.array([r["tau"] for r in sel])
    print(f"| {lab} | {len(sel)} | " + " | ".join(f"{np.mean(c == k):.0%}" for k in range(1, 5)) + f" | {np.mean(~np.isin(c, [1, 2, 3, 4])) if len(c) else 0:.0%} |")
  print()
  print("Steady-following time gap (radarState) by TauGapSet:\n")
  print("| TauGapSet | n samples | tg median | tg p10 | tg p90 |")
  print("|---|---|---|---|---|")
  for k in sorted({r["tau"] for r in st}):
    g = [r["d"] / r["v"] for r in st if r["tau"] == k]
    print(f"| {k:.0f} | {len(g)} | {f(pc(g, 50))} | {f(pc(g, 10))} | {f(pc(g, 90))} |")
  print()
  print("### Sensor cross-check: radarState dRel vs factory SCC11 ACC_ObjDist (same rows)\n")
  print("| subset | n | time gap radar median / mean | time gap SCC median / mean | SCC - radar tg median | SCC - radar tg mean | tg abs median | tg p10 / p90 | dist diff median m (p10 / p90) |")
  print("|---|---|---|---|---|---|---|---|---|")
  hw = [r for rt in have for r in have[rt]["rows"] if r["eng"] and not r["ped"] and r["v"] >= ARGS.min_speed and r["ls"]
        and r["scc_st"] != 0 and 25 <= r["d"] <= 120]
  stv = [r for r in st if r["scc_st"] != 0]
  for lab, sel in (("steady samples", stv), ("engaged, lead 25-120 m", hw)):
    if not sel:
      continue
    tr = np.array([r["d"] / r["v"] for r in sel]); ts = np.array([r["scc_d"] / r["v"] for r in sel])
    dd = np.array([r["scc_d"] - r["d"] for r in sel])
    print(f"| {lab} | {len(sel)} | {f(np.median(tr))} / {f(np.mean(tr))} | {f(np.median(ts))} / {f(np.mean(ts))} | {f(np.median(ts - tr), 3)} | {f(np.mean(ts - tr), 3)} | "
          f"{f(np.median(np.abs(ts - tr)), 3)} | {f(pc(ts - tr, 10), 3)} / {f(pc(ts - tr, 90), 3)} | {f(np.median(dd), 2)} ({f(pc(dd, 10), 1)} / {f(pc(dd, 90), 1)}) |")
  print()


def closing_tables(name, evs, rejected, stock):
  print(f"### {name}: closing events ({len(evs)}; {rejected} candidates dropped for < {ARGS.min_track:.0f} s of lead tracking)\n")
  ends = {}
  for e in evs:
    ends[e["reason"]] = ends.get(e["reason"], 0) + 1
  print(f"End reasons: {', '.join(f'{k} {v}' for k, v in ends.items()) or '-'}; started already braking: {sum(e['pre'] for e in evs)}; "
        f"vision-only lead at start: {sum(not e['radar'] for e in evs)}.\n")
  print("| closing at start (m/s) | n | braked share | onset gap m | onset time gap | onset TTC s | onset closing m/s | peak decel (m/s^2) | min time gap | tg @ speed match (n) | "
        "gap / MPC desired @ onset | cmd-onset gap m (n) | start v mph | start tg | start TTC |")
  print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
  for lo, hi, lab in CLOSE_BINS + ((0.0, 999.0, "all"), (0.0, 999.0, "all, sustained")):
    sel = [e for e in evs if lo <= e["close0"] < hi and (e["sust"] or lab != "all, sustained")]
    br = [e for e in sel if e["onset"]]
    gm = [e["tg_match"] for e in sel]
    cg = [e["cmd_gap"] for e in sel if e["cmd_known"]]
    print(f"| {lab} | {len(sel)} | {f(len(br) / len(sel) if sel else np.nan, 2)} ({len(br)}) | {f(pc([e['on_d'] for e in br], 50), 1)} | "
          f"{f(pc([e['on_tg'] for e in br], 50))} | {f(pc([e['on_ttc'] for e in br], 50), 1)} | {f(pc([e['on_close'] for e in br], 50), 1)} | "
          f"{f(pc([e['peak'] for e in sel], 50))} | {f(pc([e['mintg'] for e in sel], 50))} | {f(pc(gm, 50))} ({int(np.sum(~np.isnan(gm)))}) | "
          f"{f(pc([e['ratio'] for e in br], 50))} | {f(pc(cg, 50), 1) if cg else '-'} ({int(np.sum(~np.isnan(cg))) if cg else 0}) | "
          f"{f(pc([e['v0'] * MPH for e in sel], 50), 0)} | {f(pc([e['tg0'] for e in sel], 50))} | {f(pc([e['ttc0'] for e in sel], 50), 1)} |")
  print()
  br = [e for e in evs if e["onset"]]
  print(f"Braked events, medians: need@onset {f(pc([e['need'] for e in br], 50))} m/s^2 "
        f"(D = {'1.3' if stock else 'logged tFollow'}*v + 6), MPC desired {f(pc([e['mpc'] for e in br], 50), 1)} m, "
        f"gap/MPC p10 / p90 {f(pc([e['ratio'] for e in br], 10))} / {f(pc([e['ratio'] for e in br], 90))}, "
        f"tFollow at onset median {f(pc([e['tf_on'] for e in br], 50))}.\n")


def event_list(name, evs, stock):
  print(f"<details><summary>{name}: per-event table ({len(evs)})</summary>\n")
  print("| route | t (s) | v mph | vLead mph | gap m | close m/s | tg | TTC | end | dur | braked | onset +s | onset gap | onset tg | onset close | onset TTC | "
        "cmd@onset | cmd-onset gap | peak decel | min tg | tg@match | need | MPC des. | gap/MPC | tF |")
  print("|" + "---|" * 25)
  for e in evs:
    b = e["onset"]
    print(f"| {e['route']} | {e['t_rel']:.0f} | {e['v0'] * MPH:.0f} | {e['vl0'] * MPH:.0f} | {e['d0']:.0f} | {e['close0']:.1f} | {f(e['tg0'])} | "
          f"{f(e['ttc0'], 1)} | {e['reason']} | {e['dur']:.0f} | {'pre' if e['pre'] else 'Y' if b else 'no braking'} | "
          + (f"{e['on_t']:.1f} | {e['on_d']:.0f} | {e['on_tg']:.2f} | {e['on_close']:.1f} | {f(e['on_ttc'], 1)} | {f(e['on_cmd'])} | "
             f"{f(e['cmd_gap'], 0)} | " if b else "- | - | - | - | - | - | - | ")
          + f"{e['peak']:.2f} | {e['mintg']:.2f} | {f(e['tg_match'])} | " + (f"{e['need']:.2f}{'*' if e['inside'] else ''} | {e['mpc']:.0f} | {e['ratio']:.2f} | {e['tf_on']:.2f} |" if b else "- | - | - | " + f"{e['tf0']:.2f} |"))
  print("\n</details>\n")


def main():
  global ARGS, ARGS_STARPILOT
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--group", nargs="+", action="append", metavar=("NAME", "ROUTE"), required=True)
  p.add_argument("--min-speed", type=float, default=20.0, help="m/s (default 20 = 45 mph)")
  p.add_argument("--min-track", type=float, default=4.0, help="s the lead must stay tracked from the event start")
  p.add_argument("--close-start", type=float, default=1.5, help="closing speed that starts an event, m/s")
  p.add_argument("--close-end", type=float, default=0.3, help="closing speed below which the speeds are matched, m/s")
  p.add_argument("--starpilot", default="/home/openpilotstuff/wt/stinger-combined", help="checkout for opendbc (read only)")
  p.add_argument("--no-event-list", action="store_true")
  ARGS = p.parse_args()
  ARGS_STARPILOT = ARGS.starpilot
  HO.GAP_MODEL = "headway"
  res = {}
  for name, *routes in ARGS.group:
    if not routes:
      sys.exit(f"group {name} has no routes")
    stock = "stock" in name.lower()
    data = load_group_routes(routes, stock)
    evs, rej = [], 0
    for rt, d in data.items():
      es, rj = closing_events(d["rows"], stock, ARGS)
      t0 = d["rows"][0]["t"]
      for e in es:
        e.update(route=short(rt), t_rel=e["t"] - t0)
      evs += es
      rej += rj
    res[name] = dict(data=data, stock=stock, evs=evs, rej=rej, routes=routes)
    print(f"loaded {name}: {sum(len(d['rows']) for d in data.values())} rows, {len(evs)} closing events", file=sys.stderr)

  steady_out = {}
  import io, contextlib
  body = io.StringIO()
  with contextlib.redirect_stdout(body):
    for name, g in res.items():
      print(f"## Group {name} ({', '.join(short(r) for r in g['routes'])})\n")
      tg, samples = steady_tables(name, g["data"], g["stock"])
      steady_out[name] = (tg, samples)
      tfollow_summary(name, g["data"], None, samples, g["stock"])
      if g["stock"]:
        stock_scc_tables(g["data"], samples)
      closing_tables(name, g["evs"], g["rej"], g["stock"])
      if not ARGS.no_event_list:
        event_list(name, g["evs"], g["stock"])
  print("## Bottom line\n")
  print("| group | tFollow logged (median) | steady tg median (all speeds) | steady seconds | closing events | onset TTC median s (braked events) | "
        "braked share | onset gap median m | peak decel median m/s^2 | share with peak decel >= 1.5 |")
  print("|---|---|---|---|---|---|---|---|---|---|")
  for name, g in res.items():
    tg, samples = steady_out[name]
    evs = g["evs"]; br = [e for e in evs if e["onset"]]
    secs = {}
    for rt, sid, r in samples:
      secs.setdefault((rt, sid), []).append(r["t"])
    tfs = [r["tf"] for d in g["data"].values() for r in d["rows"] if r["eng"] and r["v"] >= ARGS.min_speed and r.get("tf")]
    print(f"| {name} | {'1.3 (assumed)' if g['stock'] else f(pc(tfs, 50))} | {f(tg)} | {sum(max(v) - min(v) for v in secs.values()):.0f} | {len(evs)} | "
          f"{f(pc([e['on_ttc'] for e in br], 50), 1)} | {len(br)}/{len(evs)} | {f(pc([e['on_d'] for e in br], 50), 1)} | {f(pc([e['peak'] for e in evs], 50))} | "
          f"{sum(e['peak'] >= 1.5 for e in evs)}/{len(evs)} ({f(np.mean([e['peak'] >= 1.5 for e in evs]) if evs else np.nan, 2)}) |")
  print()
  sys.stdout.write(body.getvalue())


if __name__ == "__main__":
  main()
