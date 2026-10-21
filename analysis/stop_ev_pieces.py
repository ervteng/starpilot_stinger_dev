#!/usr/bin/env python3
"""How often do the Ioniq 6 stop brake cap and launch hold engage on logged openpilot stops, and what do they change?

Replays carControl/carState of rlogs through variant C (the plain Ioniq 6 tune, TuneC from stock_mimic_check.py) and
compares the sent accel with today's (accel = command). A stop is a run of longControlState == stopping (gaps up to 1 s
merged). Per stop: frames with the stop brake cap binding (stopping, v <= 2.0 m/s and the cap above the command, so the
sent desired accel is less braking than commanded), the largest braking given up, and the extra distance of the sent
accel vs the command over the stop (double integral of sent - command from 0.5 s before the stop to the first
sample with v < 0.05 m/s (+0.5 s), or 1 s after its end; positive = closer to the stopped lead). Launch hold: frames with launch_active and the largest accel it adds.
Usage: stop_ev_pieces.py ROUTE [ROUTE ...] [--starpilot DIR]
"""
import argparse
import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short
from tune_safety_replay import ACC, ACTIVE, A, DT, LCS, OVR, T, V, densify, load, _lcs_enum
from stock_mimic_check import TuneC


def analyse(chunk):
  c = TuneC()
  first = chunk[0][ACC]
  c.state.desired_accel = c.state.actual_accel = c.state.accel_last = c.accel_last = first
  c.out = (first, 3.0, 5.0)
  stopping_state = int(_lcs_enum().stopping)
  n = len(chunk)
  sent = np.zeros(n); cap = np.zeros(n, bool); launch = np.zeros(n, bool); stop = np.zeros(n, bool)
  from opendbc.car.hyundai.carcontroller import IONIQ_6_STOP_BRAKE_CAP_ACCEL_V, IONIQ_6_STOP_BRAKE_CAP_MAX_SPEED, IONIQ_6_STOP_BRAKE_CAP_SPEED_BP
  for k, r in enumerate(chunk):
    stop[k] = r[LCS] == stopping_state and bool(r[ACTIVE])
    sent[k] = c.step(k, r[ACC], r[V], r[A], r[LCS] == stopping_state, bool(r[ACTIVE]), bool(r[OVR]), False, 0.0)[0]
    launch[k] = c.state.launch_active and bool(r[ACTIVE])
    capv = float(np.interp(r[V], IONIQ_6_STOP_BRAKE_CAP_SPEED_BP, IONIQ_6_STOP_BRAKE_CAP_ACCEL_V))
    cap[k] = stop[k] and r[V] <= IONIQ_6_STOP_BRAKE_CAP_MAX_SPEED and r[ACC] < capv
  return sent, cap, launch, stop


def runs(mask, gap=100):
  idx = np.flatnonzero(mask); out = []
  for k in idx:
    if out and k - out[-1][1] <= gap:
      out[-1][1] = k
    else:
      out.append([k, k])
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="+"); ap.add_argument("--starpilot")
  args = ap.parse_args()
  print(f"tunes from {add_starpilot_path(args.starpilot)}")
  stops = []; launches = []; tot_frames = 0; starts = 0
  for route in args.routes:
    files = segment_files(route, "rlog")
    rows = sorted(r for rs in pmap(load, files) for r in rs)
    for chunk in densify(rows):
      sent, cap, launch, stop = analyse(chunk)
      cmd = np.array([r[ACC] for r in chunk]); t = np.array([r[T] for r in chunk])
      tot_frames += len(chunk)
      starts += sum(1 for r in chunk if r[LCS] == int(_lcs_enum().starting) and r[ACTIVE])
      for a, b in runs(stop):
        still = [k for k in range(a, min(len(chunk), b + 101)) if chunk[k][V] < 0.05]
        lo, hi = max(0, a - 50), (still[0] + 50 if still else min(len(chunk), b + 101))
        hi = min(hi, len(chunk))
        dv = np.cumsum((sent[lo:hi] - cmd[lo:hi]) * DT); dd = float(np.sum(dv * DT))
        v_end = chunk[b][V]
        stops.append((short(route), t[a], (b - a) * DT, int(cap[a:b + 1].sum()), float((sent[a:b + 1] - cmd[a:b + 1]).max()),
                      float((cmd[a:b + 1] - sent[a:b + 1]).max()), dd, chunk[a][V], v_end))
      for a, b in runs(launch):
        launches.append((short(route), t[a], (b - a + 1) * DT, float((sent[a:b + 1] - cmd[a:b + 1]).max())))
  print(f"{starts} frames in longControlState starting (long active)")
  print(f"{len(stops)} stopping runs, {sum(1 for s in stops if s[3] > 0)} with the stop brake cap binding at some point; {len(launches)} launch-hold runs")
  print(f"{'route':>12s} {'t':>9s} {'dur s':>6s} {'cap frames':>10s} {'max less brake':>14s} {'max more':>9s} {'extra dist m':>12s} {'v start':>8s} {'v end':>6s}")
  for s in stops:
    print(f"{s[0]:>12s} {s[1] % 1e5:9.1f} {s[2]:6.1f} {s[3]:10d} {s[4]:14.2f} {s[5]:9.2f} {s[6]:12.2f} {s[7]:8.1f} {s[8]:6.2f}")
  capped = [s for s in stops if s[3] > 0]
  if capped:
    ex = np.array([s[6] for s in capped])
    print(f"stops with cap: extra distance median {np.median(ex):.2f} m, worst {ex.max():.2f} m, total cap frames {sum(s[3] for s in capped)} ({sum(s[3] for s in capped) * DT:.1f} s)")
  print("launch hold runs: " + ", ".join(f"{l[2]:.1f}s (+{l[3]:.2f})" for l in launches[:40]))


if __name__ == "__main__":
  main()
