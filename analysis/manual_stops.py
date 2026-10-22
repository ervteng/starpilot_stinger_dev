#!/usr/bin/env python3
"""The driver's own stops behind a lead, with no longitudinal control engaged (qlogs of every local route).

A stop is the car reaching 0.3 m/s from >= 8 mph with a lead under 1 m/s within 30 m for the last 2 s. The approach
starts at the last gas press before the stop; it only counts if neither openpilot longitudinal (carControl.longActive)
nor stock ACC (carState.cruiseState.enabled) was on from 10 s before that to the stop. Braking onset is the first brake
press. Classes by lead speed at onset: stopped (< 1 m/s), slow (< 0.5 v), follow (both slowing together).
Decel is the 1 s smoothed carState.aEgo. Leads come from openpilot's radarState (camera-only on older builds).

Per class and per speed at onset: coast before braking, onset speed / gap / TTC, decel at 25/20/15/10/5/2 mph, peak
decel and where it falls (fraction of the braking distance), decel constancy (CV between 20 and 5 mph), creep below
3 mph, final gap; and a fit of decel = k v^2 / (2 (gap - g0)) over the stopped/slow samples.

Usage: manual_stops.py [--logs DIR]
"""
import argparse
import collections
import glob
import os

import numpy as np

from logs import pmap, sorted_events

SPEEDS_MPH = (25, 20, 15, 10, 5, 2)
MPH = 2.237


def segment(f):
  out = []; lead = (0, 0., 0., 0); long_active = 0
  try:
    for e in sorted_events(f):
      w = e.which()
      if w == "radarState":
        l = e.radarState.leadOne
        lead = (int(l.status), l.dRel, l.vLead, int(l.radar))
      elif w == "carControl":
        long_active = int(e.carControl.longActive)
      elif w == "carState":
        c = e.carState
        out.append((e.logMonoTime / 1e9, c.vEgo, c.aEgo, int(c.gasPressed), int(c.brakePressed), int(c.cruiseState.enabled),
                    long_active, *lead))
  except Exception:
    pass  # truncated qlog: keep what was read
  return f, out


def stops(rows):
  a = np.array(rows, dtype=float)
  if len(a) < 200:
    return []
  t, v, a_ego, gas, brake, cruise, long_active, status, d, v_lead, radar = a.T
  dt = np.median(np.diff(t))
  acc = np.convolve(a_ego, np.ones(max(int(round(1 / dt)), 1)) / max(int(round(1 / dt)), 1), mode='same')
  res = []
  for i in range(1, len(a)):
    if not (v[i] < 0.3 <= v[i - 1]):
      continue
    w = slice(max(i - int(2 / dt), 0), i + 1)
    if not (status[w].all() and (v_lead[w] < 1.0).all() and 0.5 < d[i] < 30):
      continue
    j = i
    while j > 0 and not gas[j] and t[i] - t[j] < 60:
      j -= 1
    k0 = j + 1
    if t[i] - t[j] >= 60 or j == 0 or k0 >= i or v[k0:i + 1].max() < 8 / MPH:
      continue
    manual = slice(max(k0 - int(10 / dt), 0), i + 1)
    if cruise[manual].any() or long_active[manual].any():
      continue
    pressed = np.where(brake[k0:i + 1] > 0)[0]
    if not len(pressed):
      continue
    kb = k0 + pressed[0]
    if not status[kb] or d[kb] <= 0:
      continue
    vb, vl = v[kb], v_lead[kb]
    seg_v, seg_a, seg_d = v[kb:i + 1], -acc[kb:i + 1], d[kb:i + 1]
    at = {}
    for mph in SPEEDS_MPH:
      idx = np.where(seg_v * MPH <= mph)[0]
      at[f"a{mph}"] = seg_a[idx[0]] if len(idx) and seg_v[0] * MPH > mph else np.nan
    band = (seg_v * MPH <= 20) & (seg_v * MPH >= 5)
    cv = np.std(seg_a[band]) / abs(np.mean(seg_a[band])) if band.sum() > 5 and abs(np.mean(seg_a[band])) > 0.1 else np.nan
    travelled = np.cumsum(seg_v * dt)
    res.append(dict(t=t[i], cls="stopped" if vl < 1.0 else "slow" if vl < 0.5 * vb else "follow",
                    v_release=v[k0] * MPH, coast_s=t[kb] - t[k0], v_brake=vb * MPH, gap_brake=d[kb],
                    ttc=d[kb] / max(vb - vl, 0.1), peak=seg_a.max(),
                    peak_frac=travelled[int(np.argmax(seg_a))] / max(travelled[-1], 0.1), cv=cv,
                    creep=(seg_v < 3 / MPH).sum() * dt, final=d[i], radar=radar[kb], samples=(seg_v, seg_a, seg_d), **at))
  return res


def stat(es, key, fmt="{:.1f}"):
  x = np.array([e[key] for e in es], float)
  x = x[~np.isnan(x)]
  if not len(x):
    return "-"
  return f"{fmt.format(np.median(x))} [{fmt.format(np.percentile(x, 25))}-{fmt.format(np.percentile(x, 75))}]"


def show(name, es):
  print(f"{name:24s} n={len(es):3d} | gas off at {stat(es, 'v_release')} mph, coast {stat(es, 'coast_s')} s | brake at "
        f"{stat(es, 'v_brake')} mph, gap {stat(es, 'gap_brake')} m, TTC {stat(es, 'ttc')} s | peak {stat(es, 'peak', '{:.2f}')} "
        f"at {stat(es, 'peak_frac', '{:.2f}')} | CV {stat(es, 'cv', '{:.2f}')} | creep {stat(es, 'creep')} s | final {stat(es, 'final')} m")
  print("   decel at speed: " + "  ".join(f"{m} mph {stat(es, f'a{m}', '{:.2f}').split(' ')[0]}" for m in SPEEDS_MPH))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("--logs", default=os.path.join(os.environ.get("STINGER_DEV_HOME", "/home/user/starpilot-dev"), "logs"))
  args = p.parse_args()
  by_route = collections.defaultdict(list)
  for f, out in pmap(segment, sorted(glob.glob(os.path.join(args.logs, "*--qlog.zst")))):
    route = os.path.basename(f).split('_', 1)[1].rsplit('--', 2)[0]
    by_route[route].append((int(f.split('--')[-2]), out))
  found = []
  for route, segs in by_route.items():
    for e in stops([x for _, o in sorted(segs) for x in o]):
      found.append(dict(e, route=route))
  print(f"{len(found)} manual stops behind a lead from >= 8 mph on {len(set(e['route'] for e in found))} routes: "
        f"{dict(collections.Counter(e['cls'] for e in found))}\n")
  for cls in ("stopped", "slow", "follow"):
    show(cls, [e for e in found if e['cls'] == cls])
  near = [e for e in found if e['cls'] in ("stopped", "slow")]
  print()
  for lo, hi in ((8, 15), (15, 25), (25, 45)):
    show(f"stopped+slow {lo}-{hi} mph", [e for e in near if lo <= e['v_brake'] < hi])
  v, a, d = (np.concatenate([e['samples'][n] for e in near]) for n in range(3))
  m = (v > 1.0) & (d > 0)
  v, a, d = v[m], a[m], d[m]
  err, k, g0 = min((np.mean((a - k * v**2 / (2 * np.maximum(d - g0, 0.5)))**2), k, g0)
                   for k in np.arange(0.5, 2.01, 0.05) for g0 in np.arange(0, 10.01, 0.25))
  pred = k * v**2 / (2 * np.maximum(d - g0, 0.5))
  print(f"\nfit decel = k v^2 / (2 (gap - g0)): k {k:.2f}, g0 {g0:.2f} m, RMSE {np.sqrt(err):.2f}, "
        f"R2 {1 - np.sum((a - pred)**2) / np.sum((a - a.mean())**2):.2f} ({len(a)} samples)")
  print(f"radar lead at brake onset: {np.mean([e['radar'] for e in near]):.0%}")


if __name__ == "__main__":
  main()
