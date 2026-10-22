#!/usr/bin/env python3
"""Replay StarPilot's real LongitudinalPlanner on one drive's logged inputs, twice, changing only CP.longitudinalActuatorDelay.

Answers: does the 0.5 s delay that Stinger tune B sets (vs the 0.15 s default) make the longitudinal planner brake earlier or
harder? The planner (selfdrive/controls/lib/longitudinal_planner.py, acados MPC included) is imported unmodified from a
built StarPilot checkout (default $STINGER_DEV_HOME/x86, which has params_pyx and the acados solver compiled; it is only
read, never checked out or written). Per modelV2 cycle (one logged longitudinalPlan) it is fed the latest logged carState,
carControl, controlsState, liveParameters, radarState, modelV2, selfdriveState, starpilotCarState and starpilotPlan; the
starpilot toggles are the JSON the planner itself received (starpilotPlan.starpilotToggles) and CP is the logged carParams.
The StarPilot planner (starpilotPlan) is NOT re-run: it never reads longitudinalActuatorDelay, so the logged starpilotPlan
is what it would produce for either delay. time.monotonic() inside the planner is replaced by the log clock.

OPEN LOOP: the car's motion (vEgo, aEgo, radar, lead) is the logged one in both runs; a planner that brakes earlier here
would in reality have changed the gap and closing speed. Only the planner's *request* is compared.

Binding path: update() is line-traced and every change of the local output_a_target is recorded (source line + new value).
"final" = the last line that changed it (what produced the published value), "min" = the line that lowered it the most.
Run 1 (--delay 0.5) is checked against the logged longitudinalPlan.aTarget / shouldStop / source.

Usage: planner_delay_replay.py [ROUTE] [--segments 16-21] [--delays 0.5 0.15] [--events 1019 1158 ...] [--sp-lag 0]
"""
import argparse
import logging
import os
import pickle
import sys
import time as _time
import types
from collections import Counter, defaultdict

import numpy as np

DEV = os.environ.get("STINGER_DEV_HOME", "/home/user/starpilot-dev")
BUILD = os.environ.get("STARPILOT_BUILD", os.path.join(DEV, "x86"))
for p in (BUILD, os.path.join(BUILD, "opendbc_repo"), os.path.dirname(os.path.abspath(__file__))):
  if p not in sys.path:
    sys.path.insert(0, p)

from cereal import log as _cereal_log  # noqa: E402,F401  (load the capnp schema once, before logs.py would)
import logs  # noqa: E402

DEFAULT_ROUTE = "00000011--c7d29db317"
SERVICES = ("carState", "carControl", "controlsState", "liveParameters", "radarState", "modelV2", "selfdriveState",
            "starpilotCarState", "starpilotPlan")
EVENTS = (921, 1019, 1158, 1226, 1276)  # user-reported city hard-braking moments, route seconds
THRESHOLDS = (-0.3, -0.5, -1.0)


class FakeSM:
  """Just enough SubMaster for LongitudinalPlanner.update: indexing, logMonoTime, all_checks, frame/updated."""
  def __init__(self, d, mono, frame):
    self.d = d
    self.logMonoTime = defaultdict(lambda: mono)
    self.frame = frame
    self.updated = defaultdict(lambda: True)
    self.valid = defaultdict(lambda: True)
    self.alive = defaultdict(lambda: True)

  def __getitem__(self, k):
    return self.d[k]

  def all_checks(self, *a, **k):
    return True


def load_frames(route, segments, sp_lag=0):
  """One frame per logged longitudinalPlan (= per modelV2 cycle) with the latest logged inputs at that moment."""
  files = logs.segment_files(route, "rlog", segments)
  assert [logs.seg_num(f) for f in files] == list(segments), f"missing rlogs: {[logs.seg_num(f) for f in files]}"
  latest, frames, cp, toggles_text = {}, [], None, ""
  sp_hist = []
  for f in files:
    evs = logs.sorted_events(f)
    seg = logs.seg_num(f)
    seg0 = next(e.logMonoTime for e in evs if e.which() == "carState")
    for e in evs:
      w = e.which()
      if w in SERVICES:
        latest[w] = getattr(e, w)
        latest[w + "_t"] = e.logMonoTime
        if w == "starpilotPlan":
          sp_hist.append(latest[w])
          if latest[w].starpilotToggles:
            toggles_text = latest[w].starpilotToggles
      elif w == "carParams" and cp is None:
        cp = e.carParams
      elif w == "longitudinalPlan" and all(s in latest for s in SERVICES):
        d = {s: latest[s] for s in SERVICES}
        if sp_lag:
          d["starpilotPlan"] = sp_hist[max(0, len(sp_hist) - 1 - sp_lag)]
        frames.append(dict(t=e.logMonoTime, route_t=logs.route_time(f, e.logMonoTime, seg0), sm=d, toggles=toggles_text,
                           log=e.longitudinalPlan, seg=seg))
  return frames, cp


class CPProxy:
  """CarParams whose longitudinalActuatorDelay is `action` when read directly in LongitudinalPlanner.update (the action_t
  that picks the output accel off the plan) and `caps` when read anywhere else (reaction_t in the lead caps, dynamic t_follow)."""
  def __init__(self, cp, action, caps, update_code):
    self._cp, self._action, self._caps, self._code = cp, action, caps, update_code

  @property
  def longitudinalActuatorDelay(self):
    return self._action if sys._getframe(1).f_code is self._code else self._caps

  def __getattr__(self, k):
    return getattr(self._cp, k)


def parse_delay(spec):
  a, _, c = str(spec).partition(":")
  return (float(a), float(c)) if c else (float(a), float(a))


def replay(frames, cp, delay, tracer=True):
  from cereal import car
  from openpilot.starpilot.common.starpilot_variables import process_starpilot_toggles
  import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
  logging.disable(logging.CRITICAL)

  clock = [0.0]
  lp.time = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0], sleep=_time.sleep)
  action_d, caps_d = parse_delay(delay)
  CP = cp.as_builder()
  CP.longitudinalActuatorDelay = caps_d
  code = lp.LongitudinalPlanner.update.__code__
  if action_d != caps_d:
    CP = CPProxy(CP, action_d, caps_d, code)
  planner = lp.LongitudinalPlanner(CP)

  src = open(lp.__file__).read().splitlines()
  state = {}
  changes = []

  def local_trace(frame, event, arg):
    if event == "line":
      if state.get("prev") is not None:
        v = frame.f_locals.get("output_a_target")
        if v is not None and v != state.get("last"):
          changes.append((state["prev"], float(v)))
          state["last"] = v
      state["prev"] = frame.f_lineno
    elif event == "return":
      v = frame.f_locals.get("output_a_target")
      if v is not None and v != state.get("last"):
        changes.append((state["prev"], float(v)))
    return local_trace

  def global_trace(frame, event, arg):
    return local_trace if frame.f_code is code else None

  out = []
  for i, fr in enumerate(frames):
    clock[0] = fr["t"] / 1e9
    sm = FakeSM(fr["sm"], fr["t"], i)
    toggles = process_starpilot_toggles(fr["toggles"])
    state.clear(); changes.clear()
    if tracer:
      sys.settrace(global_trace)
    try:
      planner.update(sm, toggles)
    finally:
      sys.settrace(None)
    cs, lead = fr["sm"]["carState"], fr["sm"]["radarState"].leadOne
    final = changes[-1][0] if changes else 0
    drops = [(changes[k][1] - (changes[k - 1][1] if k else changes[k][1]), changes[k][0]) for k in range(len(changes))]
    minline = min(drops)[1] if drops and min(drops)[0] < -1e-3 else 0
    out.append(dict(
      t=fr["route_t"], a=float(planner.output_a_target), stop=bool(planner.output_should_stop), src=planner.mpc.source,
      a0=float(planner.a_desired_trajectory[0]), plan_a=changes[0][1] if changes else np.nan,
      final=final, minline=minline, v=float(cs.vEgo), vcruise=float(fr["sm"]["starpilotPlan"].vCruise),
      lead=bool(lead.status), d=float(lead.dRel), vl=float(lead.vLead), al=float(lead.aLeadK), radar=bool(lead.radar),
      log_a=float(fr["log"].aTarget), log_stop=bool(fr["log"].shouldStop), log_src=str(fr["log"].longitudinalPlanSource),
      log_a0=float(fr["log"].accels[0]), seg=fr["seg"], aego=float(cs.aEgo), chg=list(changes),
      e2e=float(fr["sm"]["modelV2"].action.desiredAcceleration), exp=bool(fr["sm"]["selfdriveState"].experimentalMode),
      tf=float(fr["sm"]["starpilotPlan"].tFollow), prob=float(lead.modelProb), minacc=float(fr["sm"]["starpilotPlan"].minAcceleration),
      model_stop=bool(fr["sm"]["modelV2"].action.shouldStop), forcing=bool(fr["sm"]["starpilotPlan"].forcingStop)))
  labels = {ln: src[ln - 1].strip() for r in out for ln in (r["final"], r["minline"]) if ln}
  return out, labels



def onset(t, a, t0, t1, thr):
  m = (t >= t0) & (t <= t1) & (a < thr)
  return float(t[m][0]) if m.any() else None


def describe(labels, ln):
  return f"L{ln}: {labels.get(ln, '-')[:70]}" if ln else "mpc"


def match_report(res):
  a, la = np.array([r["a"] for r in res]), np.array([r["log_a"] for r in res])
  t = np.array([r["t"] for r in res])
  for name, m in (("all frames", np.ones_like(t, bool)), ("after 30 s warmup", t > t[0] + 30)):
    d = np.abs(a - la)[m]
    print(f"  {name}: n={m.sum()} |aTarget diff| median {np.median(d):.4f} p95 {np.percentile(d, 95):.4f} p99 {np.percentile(d, 99):.3f} "
          f"max {d.max():.3f}; within 0.05: {(d < 0.05).mean() * 100:.1f}%  within 0.2: {(d < 0.2).mean() * 100:.1f}%")
  m = t > t[0] + 30
  stop, lstop = np.array([r["stop"] for r in res])[m], np.array([r["log_stop"] for r in res])[m]
  print(f"  shouldStop agreement {np.mean(stop == lstop) * 100:.2f}%  (log true {lstop.sum()}, replay true {stop.sum()})")
  s, ls = np.array([r["src"] for r in res])[m], np.array([r["log_src"] for r in res])[m]
  print(f"  MPC source agreement {np.mean(s == ls) * 100:.2f}%")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?", default=DEFAULT_ROUTE)
  ap.add_argument("--segments", default="16-21")
  ap.add_argument("--delays", nargs="+", default=["0.5", "0.15"],
                  help="logged delay first, then the alternative, then optional split variants ACTION:CAPS (e.g. 0.5:0.15 = "
                       "0.5 s for action_t only, 0.15 s for every reaction_t cap)")
  ap.add_argument("--approach", type=float, nargs="*", default=[], help="pairs T0 T1 (route s): print a stopped/slow-lead approach profile")
  ap.add_argument("--events", type=float, nargs="*", default=list(EVENTS), help="route seconds of hard-braking events")
  ap.add_argument("--window", type=float, default=8.0, help="seconds around each event (+/-)")
  ap.add_argument("--sp-lag", type=int, default=0, help="use starpilotPlan this many cycles old (race between processes)")
  ap.add_argument("--save", help="pickle the per-frame results here")
  ap.add_argument("--worker", help=argparse.SUPPRESS)
  ap.add_argument("--check-only", action="store_true", help="only replay the first delay and print the log match")
  args = ap.parse_args()
  lo, _, hi = args.segments.partition("-")
  segs = list(range(int(lo), int(hi or lo) + 1))

  delays = args.delays[:1] if args.check_only else args.delays
  if args.worker is not None:  # one replay in this process
    frames, cp = load_frames(args.route, segs, args.sp_lag)
    print(f"{len(frames)} planner cycles, route t {frames[0]['route_t']:.1f}..{frames[-1]['route_t']:.1f}; "
          f"logged CP delay {cp.longitudinalActuatorDelay}, fingerprint {cp.carFingerprint}", flush=True)
    out, labels = replay(frames, cp, args.worker)
    pickle.dump((args.worker, out, labels), open(args.save, "wb"))
    return
  # fork-after-import hangs (acados/capnp threads), so each delay runs in its own process
  import subprocess
  import tempfile
  tmp = tempfile.mkdtemp()
  procs = []
  for d in delays:
    f = os.path.join(tmp, f"r{d.replace(':', '_')}.pkl")
    cmd = [sys.executable, "-I", os.path.abspath(__file__), args.route, "--segments", args.segments, "--sp-lag", str(args.sp_lag),
           "--worker", d, "--save", f]
    procs.append((f, subprocess.Popen(cmd)))
  for f, p in procs:
    assert p.wait() == 0, "worker failed"
  results = [pickle.load(open(f, "rb")) for f, _ in procs]
  if args.save:
    pickle.dump(results, open(args.save, "wb"))
  report(results, args)


def report(results, args):
  (d0, r0, lab0) = results[0]
  print(f"\n== sanity: replay with delay {d0} vs logged longitudinalPlan ==")
  match_report(r0)
  if len(results) < 2:
    return
  (d1, r1, lab1) = results[1]
  labels = {**lab0, **lab1}
  t = np.array([r["t"] for r in r0])
  A = {d: np.array([r["a"] for r in r]) for d, r, _ in results}
  print(f"\n== per event: delay {d0} vs {d1} (open loop; same logged car motion) ==")
  for ev in args.events:
    t0, t1 = ev - args.window, ev + args.window
    m = (t >= t0) & (t <= t1)
    if not m.any():
      print(f"event {ev}: not in the replayed segments"); continue
    for d, r, _ in results:
      a = A[d]
      i = np.where(m)[0][np.argmin(a[m])]
      row = r[i]
      ons = "/".join(f"{onset(t, a, t0, t1, th) - ev:+.2f}" if onset(t, a, t0, t1, th) is not None else "  - " for th in THRESHOLDS)
      print(f"event {ev:5.0f} delay {d:>9}: onset(-0.3/-0.5/-1.0) rel event {ons} s; peak {a[i]:6.2f} at {t[i] - ev:+.2f}s "
            f"v={row['v']:.1f} lead={row['lead']} d={row['d']:.1f} vl={row['vl']:.1f} al={row['al']:.2f} src={row['src']} "
            f"final[{describe(labels, row['final'])}] min[{describe(labels, row['minline'])}]")
    i = np.where(m)[0][np.argmin(A[d0][m])]
    print(f"    logged aTarget peak in window: {min(r['log_a'] for r, mm in zip(r0, m) if mm):.2f}")
  print_overall(results, labels)
  for k in range(0, len(args.approach) - 1, 2):
    approach_report(results, labels, args.approach[k], args.approach[k + 1])


def binder(r, labels):
  """Which code path set the published accel: the line with the largest drop, else the plan read; e2e vs MPC told apart."""
  chg = r["chg"]
  if not chg:
    return "none"
  drops = [(chg[k][1] - chg[k - 1][1], chg[k][0]) for k in range(1, len(chg))]
  line = min(drops)[1] if drops and min(drops)[0] < -0.02 else chg[0][0]
  if line == 2459:
    return "e2e desiredAccel (experimental min)" if r["e2e"] <= chg[0][1] + 1e-6 else "MPC (min with e2e)"
  if line == 2456:
    return "MPC (src %s)" % r["src"]
  txt = labels.get(line) or ""
  name = txt.split("=")[1].strip()[:40] if "=" in txt else txt
  return {2998: "clip to accel limits"}.get(line, f"L{line} {name}")


def approach_report(results, labels, T0, T1):
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance
  print(f"\n== approach {T0:.0f}-{T1:.0f} s (logged car motion; lead gap = radarState.leadOne.dRel) ==")
  r0 = results[0][1]
  t = np.array([r["t"] for r in r0])
  m = (t >= T0) & (t <= T1)
  idx = np.where(m)[0]
  if not len(idx):
    print("  not in the replayed segments"); return
  v = np.array([r["v"] for r in r0])
  names = [d for d, _, _ in results]
  print("  t      v   gap  vl  desGap  exp tf  e2e   | " + " | ".join(f"a({d}) binder" for d in names))
  last = -9
  for i in idx:
    if t[i] - last < 1.0:
      continue
    last = t[i]
    r = r0[i]
    des = desired_follow_distance(r["v"], r["vl"], r["tf"]) if r["lead"] else float("nan")
    cols = " | ".join(f"{res[1][i]['a']:5.2f} {binder(res[1][i], labels)[:34]}" for res in results)
    print(f"  {t[i]:6.1f} {r['v']:4.1f} {r['d']:5.1f} {r['vl']:4.1f} {des:5.1f}  {int(r['exp'])}  {r['tf']:.2f} {r['e2e']:5.2f} | {cols}")
  below = idx[v[idx] < 2.5]
  stopped = idx[v[idx] < 0.1]
  if len(below):
    t_end = t[stopped[0]] if len(stopped) and stopped[0] > below[0] else t[idx[-1]]
    print(f"  logged motion: first v<2.5 m/s at {t[below[0]]:.1f} s (gap {r0[below[0]]['d']:.1f} m); "
          f"{'stopped' if len(stopped) and stopped[0] > below[0] else 'window end'} at {t_end:.1f} s -> time below 2.5 m/s {t_end - t[below[0]]:.1f} s; "
          f"final gap {r0[idx[-1]]['d']:.1f} m")
  for d, rr, _ in results:
    a = np.array([x["a"] for x in rr])
    on = "/".join(f"{onset(t, a, T0, T1, th) - T0:+.1f}" if onset(t, a, T0, T1, th) is not None else "-" for th in THRESHOLDS)
    hard = idx[a[idx] < -1.0]
    pre = hard[v[hard] >= 2.5] if len(hard) else hard
    line = f"  delay {d:>9}: onset(-0.3/-0.5/-1.0) {on} s after {T0:.0f}; peak {a[idx].min():.2f}"
    if len(hard):
      e = (pre if len(pre) else hard)[-1]
      line += (f"; hard phase (a<-1.0) {t[hard[0]]:.1f}-{t[e]:.1f} s ({len(hard) * 0.05:.1f} s total), last at v={v[e]:.1f} m/s gap {rr[e]['d']:.1f} m")
    print(line)
    for nm, sel in (("hard (a<-1.0)", hard), ("creep (v<2.5, a<0)", below[a[below] < 0] if len(below) else below)):
      if len(sel):
        c = Counter(binder(rr[i], labels) for i in sel)
        print(f"     binding during {nm}, {len(sel)} frames: " + "; ".join(f"{k} x{n}" for k, n in c.most_common(4)))


def print_overall(results, labels):
  (d0, r0, _), (d1, r1, _) = results[0], results[1]
  t = np.array([r["t"] for r in r0])
  a0, a1 = (np.array([r["a"] for r in r]) for r in (r0, r1))
  v = np.array([r["v"] for r in r0])
  lead = np.array([r["lead"] for r in r0])
  m = lead & (v < 20) & (v > 3) & (t > t[0] + 30) & ((a0 < -0.1) | (a1 < -0.1))
  diff = a1 - a0
  print(f"\n== overall, frames with a lead, 3<v<20 m/s and either run braking (< -0.1): n={m.sum()} of {lead.sum()} lead frames ==")
  for nm, d in ((f"a({d1}) - a({d0})", diff[m]),):
    print(f"  {nm}: mean {d.mean():+.3f} median {np.median(d):+.3f} p5 {np.percentile(d, 5):+.3f} p95 {np.percentile(d, 95):+.3f}")
  print(f"  fraction where delay {d0} brakes harder by >0.1: {(diff[m] > 0.1).mean() * 100:.1f}%  by >0.3: {(diff[m] > 0.3).mean() * 100:.1f}%; "
        f"the {d1} run harder by >0.1: {(diff[m] < -0.1).mean() * 100:.1f}%")
  allm = (t > t[0] + 30)
  print(f"  all frames (any lead): |diff| median {np.median(np.abs(diff[allm])):.4f}, frames differing >0.05: {(np.abs(diff[allm]) > 0.05).mean() * 100:.1f}%")
  for d, r, _ in results:
    a = np.array([x["a"] for x in r])
    br = m & (a < -0.1)
    c = Counter((x["final"]) for x, b in zip(r, br) if b)
    print(f"  final-binding line when braking, delay {d}: " + "; ".join(f"{describe(labels, k)} x{n}" for k, n in c.most_common(6)))
  # episodes: lead-approach braking = contiguous runs with a < -0.5 and lead
  print(f"\n  braking episodes (a < -0.5 for >= 0.5 s with lead, v < 20): onset/peak by delay")
  for d, r, _ in results:
    a = np.array([x["a"] for x in r])
    ep = episodes(t, a, lead & (v < 20), -0.5)
    print(f"   delay {d}: {len(ep)} episodes, median peak {np.median([e[2] for e in ep]) if ep else float('nan'):.2f}, "
          f"min peak {min([e[2] for e in ep]) if ep else float('nan'):.2f}")
  # matched: for each episode in run 0 find the first time run 1 crosses
  print("  matched onsets (episode in run %s): onset(-0.5) in run %s minus in run %s, peak diff" % (d0, d1, d0))
  ep0 = episodes(t, a0, lead & (v < 20), -0.5)
  rows = []
  for (s, e, pk) in ep0:
    lo = max(0, s - 100)
    i1 = next((i for i in range(lo, e + 1) if a1[i] < -0.5), None)
    rows.append(((t[i1] - t[s]) if i1 is not None else np.nan, a1[s:e + 1].min() - pk))
  if rows:
    on = np.array([r[0] for r in rows]); pd = np.array([r[1] for r in rows])
    print(f"   {len(rows)} episodes; onset shift median {np.nanmedian(on):+.2f}s (neg = {d1} earlier), p10/p90 {np.nanpercentile(on, 10):+.2f}/{np.nanpercentile(on, 90):+.2f}; "
          f"never crosses in {d1}: {np.isnan(on).sum()}; peak diff median {np.median(pd):+.2f}")


def episodes(t, a, ok, thr, min_len=0.5):
  out, i, n = [], 0, len(a)
  while i < n:
    if ok[i] and a[i] < thr:
      j = i
      while j < n and a[j] < thr and ok[j]:
        j += 1
      if t[j - 1] - t[i] >= min_len:
        out.append((i, j - 1, float(a[i:j].min())))
      i = j
    else:
      i += 1
  return out


if __name__ == "__main__":
  main()
