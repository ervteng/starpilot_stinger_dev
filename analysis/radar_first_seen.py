#!/usr/bin/env python3
"""How far away is a stopped car first seen, by each source, when the Stinger stops behind it (rlogs only).

For every stop (ego below 0.3 m/s after >= 8 mph and >= 30 m travelled within the last 20 s) with a lead within 15 m at the stop, the stopped car's
position is fixed from the stop (ego distance travelled + lead gap) and walked back in time, so every source is checked against
the same object at its expected gap:
  radar    the Stinger radar decoder (opendbc RadarInterface from --starpilot) on the raw CAN: a track within 2.5 m of the expected
           gap (+ 5 %), |yRel| < 2 m, stationary (|vEgo + vRel| < 1.5 m/s), seen in >= 5 of the next 10 radar frames
  camera   modelV2.leadsV3[0] with prob >= 0.5 and x (minus the 1.52 m radar-to-camera offset) within 4 m + 15 % of the expected gap
  op lead  radarState.leadOne (status) within 2.5 m of the expected gap and vLead < 1.5 m/s; radar flag at that moment
  factory  SCC11 ACC_ObjStatus with ACC_ObjDist within 3 m of the expected gap (stock ACC drives; silent with openpilot long)
A source "first sees" the car at the start of the last unbroken run (gaps < 1 s) of matches (gaps < 2 s) that reaches the stop.
class:  stopped = no moving lead (vLead > 2 m/s) short of the stopped car's position (gap < expected + 3 m) in the 15 s before
        the op lead took it;
        follow = the car ahead was being followed while moving and then stopped (detection range is meaningless there).

Usage: radar_first_seen.py ROUTE [ROUTE ...] [--starpilot DIR]
"""
import argparse
import sys

import numpy as np

from logs import add_starpilot_path, pmap, seg_num, segment_files, short, sorted_events

RADAR_TO_CAMERA = 1.52
MPH = 2.23694


def scan(path):
  from opendbc.can import CANParser  # before the first log read, so logs.py reuses its schema
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.radar_interface import RadarInterface
  from opendbc.car.hyundai.values import CAR

  evs = sorted_events(path)
  fp = gen_empty_fingerprint()
  for e in evs[:3000]:
    if e.which() == "can":
      for c in e.can:
        if c.src < 3:
          fp[c.src][c.address] = len(c.dat)
  ri = RadarInterface(CarInterface.get_params(CAR.KIA_STINGER_2022, fp, [], True, False, False, None))
  p = CANParser("hyundai_kia_generic", [("SCC11", 0)], 0)
  t0 = evs[0].logMonoTime
  base = seg_num(path) * 60.0
  out = {"ego": [], "radar": [], "lead": [], "cam": [], "scc": [], "long": []}
  v = 0.0
  for e in evs:
    w = e.which()
    t = base + (e.logMonoTime - t0) / 1e9
    if w == "carState":
      v = e.carState.vEgo
      out["ego"].append((t, v))
    elif w == "carControl":
      out["long"].append((t, bool(e.carControl.longActive)))
    elif w == "radarState":
      l = e.radarState.leadOne
      out["lead"].append((t, bool(l.status), l.dRel, l.vLead, bool(l.radar)))
    elif w == "modelV2":
      ls = e.modelV2.leadsV3
      if len(ls):
        out["cam"].append((t, ls[0].prob, ls[0].x[0] - RADAR_TO_CAMERA))
    elif w == "can":
      frames = [(c.address, bytes(c.dat), c.src) for c in e.can]
      p.update([(e.logMonoTime, frames)])
      rr = ri.update([(e.logMonoTime, frames)])
      if rr is not None:
        out["radar"].append((t, v, [(pt.dRel, pt.yRel, pt.vRel) for pt in rr.points]))
        s = p.vl["SCC11"]
        if p.can_valid and s["ACC_ObjStatus"]:
          out["scc"].append((t, s["ACC_ObjDist"]))
  return out


def first_seen(times, hits, t_stop, max_gap=2.0):
  """Start of the last unbroken run of hit times that reaches t_stop (None if the source never had it near the stop)."""
  ts = [t for t, h in zip(times, hits) if h and t <= t_stop + 0.5]
  if not ts or t_stop - ts[-1] > 2.0:
    return None
  start = ts[-1]
  for a, b in zip(reversed(ts[:-1]), reversed(ts[1:])):
    if b - a > max_gap:
      break
    start = a
  return start


def analyse(route, segs):
  D = {k: sorted(x for s in segs for x in s[k]) for k in segs[0]}
  te = np.array([x[0] for x in D["ego"]]); ve = np.array([x[1] for x in D["ego"]])
  dt = np.clip(np.diff(te, prepend=te[0]), 0, 0.1)
  xe = np.cumsum(ve * dt)                     # distance travelled
  pos = lambda t: float(np.interp(t, te, xe))
  speed = lambda t: float(np.interp(t, te, ve))
  lt = np.array([x[0] for x in D["lead"]])
  rows = []
  i, n = 0, len(te)
  while i < n:
    if ve[i] < 0.3 and i > 0 and ve[i - 1] >= 0.3:
      t_stop = te[i]
      win = (te > t_stop - 20) & (te < t_stop)
      if win.any() and ve[win].max() >= 8 / MPH and pos(t_stop) - pos(t_stop - 20) >= 30:
        k = int(np.searchsorted(lt, t_stop))
        lead = D["lead"][min(k, len(lt) - 1)] if len(lt) else None
        scc_at = [d for t, d in D["scc"] if abs(t - t_stop) < 0.3]
        d_stop = lead[2] if lead and lead[1] and lead[2] < 15 else (scc_at[-1] if scc_at and scc_at[-1] < 15 else None)
        if d_stop is not None:
          P = pos(t_stop) + d_stop
          exp = lambda t: P - pos(t)
          lo = t_stop - 60
          rt = [x for x in D["radar"] if lo <= x[0] <= t_stop + 0.5]
          rhit = [any(abs(d - exp(t)) < 2.5 + 0.05 * exp(t) and abs(y) < 2 and abs(vv + vr) < 1.5 for d, y, vr in pts) for t, vv, pts in rt]
          # persistence: >= 5 of the next 10 frames
          rper = [h and sum(rhit[j:j + 10]) >= 5 for j, h in enumerate(rhit)]
          t_radar = first_seen([x[0] for x in rt], rper, t_stop)
          ct = [x for x in D["cam"] if lo <= x[0] <= t_stop + 0.5]
          t_cam = first_seen([x[0] for x in ct], [pr >= 0.5 and abs(x - exp(t)) < 4 + 0.15 * exp(t) for t, pr, x in ct], t_stop)
          ldt = [x for x in D["lead"] if lo <= x[0] <= t_stop + 0.5]
          lhit = [st and abs(d - exp(t)) < 2.5 + 0.05 * exp(t) and vl < 1.5 for t, st, d, vl, _ in ldt]
          t_op = first_seen([x[0] for x in ldt], lhit, t_stop)
          st = [x for x in D["scc"] if lo <= x[0] <= t_stop + 0.5]
          t_scc = first_seen([x[0] for x in st], [abs(d - exp(t)) < 3 + 0.05 * exp(t) for t, d in st], t_stop)
          op_radar = None
          if t_op is not None:
            op_radar = next((r for t, s_, d, vl, r in ldt if t >= t_op), None)
          ref = t_op if t_op is not None else t_stop
          moving = any(s_ and d < exp(t) + 3 and vl > 2 for t, s_, d, vl, _ in ldt if ref - 15 <= t < ref)
          la = [a for t, a in D["long"] if abs(t - t_stop) < 5]
          v_start = float(ve[win].max())
          rows.append(dict(route=route, t_stop=t_stop, v_start=v_start * MPH, d_stop=d_stop,
                           cls="follow" if moving else "stopped", mode="op" if (la and any(la)) else "stock",
                           **{f"{k}_d": (exp(tt) if tt is not None else None) for k, tt in
                              (("radar", t_radar), ("cam", t_cam), ("op", t_op), ("scc", t_scc))},
                           **{f"{k}_v": (speed(tt) * MPH if tt is not None else None) for k, tt in
                              (("radar", t_radar), ("cam", t_cam), ("op", t_op), ("scc", t_scc))},
                           op_radar=op_radar))
    i += 1
  return rows


def fmt(x, p=0):
  return "-" if x is None else f"{x:.{p}f}"


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--starpilot", default=None)
  a = ap.parse_args()
  add_starpilot_path(a.starpilot)
  rows = []
  for r in a.routes:
    files = segment_files(r, "rlog")
    if not files:
      print(f"no rlogs for {r}", file=sys.stderr)
      continue
    rows += analyse(short(r), pmap(scan, files))
  print("Gap (m) to the stopped car when each source first had it, and ego mph then. '-' = never (within 60 s).\n")
  print("| route | stop t (s) | mode | class | start mph | final gap | radar track | camera | op lead (src) | factory SCC |")
  print("|---|---|---|---|---|---|---|---|---|---|")
  for w in rows:
    src = "" if w["op_radar"] is None else (" radar" if w["op_radar"] else " vision")
    cell = lambda k: "-" if w[k + "_d"] is None else f"{w[k + '_d']:.0f} m @ {w[k + '_v']:.0f}"
    print(f"| {w['route']} | {w['t_stop']:.0f} | {w['mode']} | {w['cls']} | {w['v_start']:.0f} | {w['d_stop']:.1f} | {cell('radar')} | "
          f"{cell('cam')} | {cell('op')}{src} | {cell('scc')} |")
  st = [w for w in rows if w["cls"] == "stopped"]
  if st:
    print(f"\nStopped-car approaches (n={len(st)}), median first-seen gap:")
    for k, name in (("radar", "radar track"), ("cam", "camera"), ("op", "op lead"), ("scc", "factory SCC")):
      ds = [w[k + "_d"] for w in st if w[k + "_d"] is not None]
      print(f"  {name:12s} {np.median(ds):5.1f} m (n={len(ds)}, range {min(ds):.0f}-{max(ds):.0f})" if ds else f"  {name:12s} -")


if __name__ == "__main__":
  main()
