#!/usr/bin/env python3
"""Do the Stinger stock-like tunes delay urgent braking? Replay logged openpilot braking through A, B and C.

Replays carControl (actuators.accel, longControlState, longActive, cruiseControl.override) with carState (vEgo, aEgo)
and radarState leadOne (status, dRel) through three tunes from --starpilot's checkout (branch stinger-stock-tune):
"A" = today's openpilot (accel passed through), "B" (GV70 style jerk-limited shaper, 50 Hz) and "C" (the plain Ioniq 6 tune,
smoothing both sides, 20 Hz). The wiring around the real tune functions is the one in stock_mimic_check.py (TuneB / TuneC,
replicated from CarController). qlog carControl is ~10 Hz, so samples are held to rebuild the 100 Hz loop. The tunes
run over the whole timeline (seeded with the first command), so they are in their normal state when braking starts.

A braking episode is a run of longActive samples with accel < --threshold (default -1.0), merged across gaps up to
1 s, at least 0.3 s long. The window runs from 0.5 s before the episode to the moment the tuned accel has caught up
with the command at or after the command's peak (at most 2 s past it), so only the onset is scored, not the release
where the ramps keep braking after the command eases. Per tune:
  peak cmd / tuned   hardest command and hardest tuned accel in the window (m/s^2)
  delay -2.0 / -3.0  seconds the tuned accel reaches that level after the command did (n/a: the command never
                     got there; never: the tune never did)
  extra distance     double integral of (tuned - commanded) accel over that window, in metres closer to the lead
                     (positive = the tune braked later or less). The speed excess left at the window end is shown too.

Default mode: the hardest braking episode in the given segments (the window also gets loaded one segment either side).
--scan: every episode in all rlogs of the given routes, per tune the median / p90 / worst extra distance, the worst
delay to -2.0, and the worst episodes.
B shapes the braking side only: a rising command and urgent braking (accel <= -1.0) pass through. C is the Ioniq 6 tune:
rising accel is smoothed, and braking harder than the smoothed value skips smoothing (get_canfd_scc_decel_step fallback).

Usage: tune_safety_replay.py ROUTE SEG [SEG ...] KIND [--starpilot DIR]       (KIND: rlog or qlog)
       tune_safety_replay.py --scan ROUTE [ROUTE ...] [--starpilot DIR] [--threshold -1.0]
"""
import argparse
import sys

import numpy as np

from logs import TUNE_DIR, add_starpilot_path, pmap, seg_num, segment_files, short, sorted_events
from stock_mimic_check import TuneB, TuneC

TUNES = ("A", "B", "C")
DT = 0.01
MAX_HOLD = 0.5  # s: longer gaps in the log break the timeline instead of being held
MERGE_GAP = 1.0
MIN_EPISODE = 0.3
PRE, POST = 0.5, 2.0
CATCH_UP = 0.05  # m/s^2: tuned within this of the command counts as caught up
# row: t, seg, long_active, accel, long_control_state (int), override, v, a_ego, lead_status, lead_dist
T, SEG, ACTIVE, ACC, LCS, OVR, V, A, LS, LD = range(10)


def _lcs_enum():
  from opendbc.car import structs
  return structs.CarControl.Actuators.LongControlState


def load(path):
  from opendbc.car import structs  # imported before the first log read, so logs.py reuses its schema
  LCSEnum = structs.CarControl.Actuators.LongControlState
  seg = seg_num(path); v = a = ls = ld = 0.0; rows = []
  for e in sorted_events(path):
    w = e.which()
    if w == "carState":
      v, a = e.carState.vEgo, e.carState.aEgo
    elif w == "radarState":
      ls, ld = float(e.radarState.leadOne.status), e.radarState.leadOne.dRel
    elif w == "carControl":
      cc = e.carControl
      rows.append((e.logMonoTime / 1e9, seg, cc.longActive, cc.actuators.accel,
                   int(getattr(LCSEnum, str(cc.actuators.longControlState))), cc.cruiseControl.override, v, a, ls, ld))
  return rows


def densify(rows):
  """Hold each sample until the next so the timeline is 100 Hz; returns contiguous chunks."""
  chunks = []; cur = []
  for r, nxt in zip(rows, rows[1:] + [None]):
    gap = (nxt[T] - r[T]) if nxt else DT
    n = max(1, round(gap / DT)) if gap <= MAX_HOLD else 1
    cur += [(r[T] + j * DT, *r[1:]) for j in range(n)]
    if gap > MAX_HOLD:
      chunks.append(cur); cur = []
  if cur:
    chunks.append(cur)
  return chunks


def simulate(chunk):
  """Tuned accel per row for each tune, shape (3, len(chunk))."""
  first = chunk[0][ACC]
  tunes = [TuneB(), TuneC()]
  tunes[0].accel_last = first; tunes[0].out = (first, 3.0, 5.0)
  tunes[1].state.desired_accel = tunes[1].state.actual_accel = tunes[1].state.accel_last = tunes[1].accel_last = first
  tunes[1].out = (first, 3.0, 5.0)
  out = np.zeros((3, len(chunk))); out[0] = [r[ACC] for r in chunk]
  stopping_state = int(_lcs_enum().stopping)
  for k, r in enumerate(chunk):
    args = (k, r[ACC], r[V], r[A], r[LCS] == stopping_state, bool(r[ACTIVE]), bool(r[OVR]), bool(r[LS]), r[LD])
    for i, t in enumerate(tunes):
      out[i + 1, k] = t.step(*args)[0]
  return out


def find_episodes(chunk, threshold):
  idx = [k for k, r in enumerate(chunk) if r[ACTIVE] and r[ACC] < threshold]
  eps = []
  for k in idx:
    if eps and (chunk[k][T] - chunk[eps[-1][1]][T]) <= MERGE_GAP:
      eps[-1][1] = k
    else:
      eps.append([k, k])
  return [(a, b) for a, b in eps if chunk[b][T] - chunk[a][T] >= MIN_EPISODE]


def first_reach(acc, t, thr):
  hit = np.flatnonzero(acc <= thr)
  return t[hit[0]] if len(hit) else None


def evaluate(chunk, tuned, a, b):
  """Per-tune metrics for the episode chunk[a..b]."""
  t = np.array([r[T] for r in chunk]); cmd = np.array([r[ACC] for r in chunk])
  lo = int(np.searchsorted(t, t[a] - PRE)); hi = int(np.searchsorted(t, t[b] + POST, side="right"))
  pk = a + int(np.argmin(cmd[a:b + 1]))
  res = {}
  for i, name in enumerate(TUNES):
    # onset only: integrate until the tune has caught up with the command at/after its peak (the release phase,
    # where the ramps keep braking after the command eases, is not what this checks), at most POST s past the peak
    caught = np.flatnonzero(tuned[i, pk:hi] <= cmd[pk:hi] + CATCH_UP)
    end = min(hi, pk + (int(caught[0]) if len(caught) else int(POST / DT)) + 1)
    tw, cw, u = t[lo:end], cmd[lo:end], tuned[i, lo:end]
    dv = np.cumsum((u - cw) * DT); dd = float(np.sum(dv * DT))
    delay = {}
    for thr in (-2.0, -3.0):
      c, x = first_reach(cw, tw, thr), first_reach(u, tw, thr)
      delay[thr] = None if c is None else (np.inf if x is None else x - c)
    res[name] = dict(peak_cmd=cw.min(), peak_tuned=u.min(), delay=delay, extra=dd, dv=float(dv[-1]))
  return res, (lo, hi)


def fmt_delay(d):
  return "n/a" if d is None else ("never" if np.isinf(d) else f"{d:.2f} s")


def run_hardest(route, segs, kind, threshold):
  want = set(segs) | {s - 1 for s in segs} | {s + 1 for s in segs}
  files = segment_files(route, kind, want)
  if not files:
    print(f"{short(route)}: no {kind}s for segments {sorted(want)}"); return
  rows = sorted(r for rs in pmap(load, files) for r in rs)
  inseg = [r for r in rows if r[SEG] in segs and r[ACTIVE]]
  if not inseg:
    print(f"{short(route)} seg {segs}: no longActive carControl"); return
  peak_t = min(inseg, key=lambda r: r[ACC])[T]
  for chunk in densify(rows):
    if not chunk[0][T] <= peak_t <= chunk[-1][T]:
      continue
    tuned = simulate(chunk)
    t = np.array([r[T] for r in chunk])
    k = int(np.argmin(np.abs(t - peak_t)))
    eps = [e for e in find_episodes(chunk, threshold) if e[0] <= k <= e[1] + 1]
    a, b = eps[0] if eps else (k, k)
    res, (lo, hi) = evaluate(chunk, tuned, a, b)
    r0 = chunk[k]; seg0 = min(r[T] for r in rows if r[SEG] == r0[SEG])
    print(f"\n== {short(route)} seg {r0[SEG]} {kind}: hardest braking at {r0[T] - seg0:.1f} s into the segment, "
          f"v {r0[V]:.1f} m/s ({r0[V] * 2.237:.0f} mph), command {r0[ACC]:.2f} m/s^2, "
          f"lead {'%.1f m' % r0[LD] if r0[LS] else 'none'}")
    print(f"   episode {t[a] - seg0:.1f}-{t[b] - seg0:.1f} s, scoring from {t[lo] - seg0:.1f} s"
          f"{', samples held from ~10 Hz' if kind == 'qlog' else ''}")
    print(f"   {'tune':11s} {'peak cmd':>9s} {'peak tuned':>11s} {'delay -2.0':>11s} {'delay -3.0':>11s} {'extra dist':>11s} {'speed excess':>13s}")
    for name in TUNES:
      m = res[name]
      print(f"   {name:11s} {m['peak_cmd']:9.2f} {m['peak_tuned']:11.2f} {fmt_delay(m['delay'][-2.0]):>11s} "
            f"{fmt_delay(m['delay'][-3.0]):>11s} {m['extra']:9.2f} m {m['dv']:10.2f} m/s")
    return


def episodes_of(path_rows, threshold):
  out = []
  for chunk in densify(path_rows):
    eps = find_episodes(chunk, threshold)
    tuned = simulate(chunk)
    for a, b in eps:
      res, _ = evaluate(chunk, tuned, a, b)
      r = chunk[a]
      out.append((r[SEG], r[T], r[V], min(x[ACC] for x in chunk[a:b + 1]), res))
  return out


def summary(eps, threshold):
  print(f"   {'tune':5s} {'median':>8s} {'p90':>8s} {'worst':>8s} {'> 1 m':>6s} {'worst delay -2.0':>17s} {'episodes reaching -2.0':>23s}  (extra distance in m, positive = closer to lead)")
  for name in TUNES:
    ex = np.array([e[4][name]["extra"] for e in eps])
    d2 = [e[4][name]["delay"][-2.0] for e in eps if e[4][name]["delay"][-2.0] is not None]
    worst = max(d2) if d2 else None
    print(f"   {name:5s} {np.median(ex):8.2f} {np.percentile(ex, 90):8.2f} {ex.max():8.2f} {int((ex > 1).sum()):6d} "
          f"{fmt_delay(worst):>17s}{"" if worst is None or np.isinf(worst) else " (%.4f)" % worst} {len(d2):>23d}")


def run_scan(route, threshold):
  files = segment_files(route, "rlog")
  if not files:
    print(f"{short(route)}: no rlogs"); return []
  rows = sorted(r for rs in pmap(load, files) for r in rs)
  eps = episodes_of(rows, threshold)
  seg0 = {s: min(r[T] for r in rows if r[SEG] == s) for s in {r[SEG] for r in rows}}
  print(f"\n== {short(route)}: {len(files)} rlogs, {len(eps)} braking episodes below {threshold} m/s^2")
  if eps:
    summary(eps, threshold)
    print("   worst episodes (by the larger of B and C):")
    for e in sorted(eps, key=lambda e: -max(e[4]["B"]["extra"], e[4]["C"]["extra"]))[:3]:
      m = e[4]
      print(f"     seg {e[0]} +{e[1] - seg0[e[0]]:5.1f} s  v {e[2]:4.1f} m/s  peak cmd {e[3]:5.2f}  extra: " +
            "  ".join(f"{n} {m[n]['extra']:5.2f} m" for n in TUNES[1:]) +
            "  delay -2.0: " + "  ".join(f"{n} {fmt_delay(m[n]['delay'][-2.0])}" for n in TUNES[1:]))
  return eps


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("args", nargs="+", help="ROUTE SEG [SEG ...] KIND, or with --scan: ROUTE [ROUTE ...]")
  p.add_argument("--scan", action="store_true", help="all braking episodes of whole drives (rlogs)")
  p.add_argument("--threshold", type=float, default=-1.0, help="episode accel threshold (default %(default)s)")
  p.add_argument("--starpilot", default=TUNE_DIR, help="checkout with the tunes (default: %(default)s)")
  args = p.parse_args()
  print(f"tunes from {add_starpilot_path(args.starpilot)}")
  if args.scan:
    alleps = []
    for route in args.args:
      alleps += run_scan(route, args.threshold)
    if len(args.args) > 1 and alleps:
      print(f"\n== all routes: {len(alleps)} braking episodes below {args.threshold} m/s^2")
      summary(alleps, args.threshold)
    return
  route, kind = args.args[0], args.args[-1]
  if kind not in ("rlog", "qlog") or len(args.args) < 3:
    p.error("expected ROUTE SEG [SEG ...] KIND")
  run_hardest(route, {int(s) for s in args.args[1:-1]}, kind, args.threshold)


if __name__ == "__main__":
  sys.exit(main())
