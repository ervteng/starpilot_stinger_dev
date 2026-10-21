#!/usr/bin/env python3
"""How closely the Stinger stock-like longitudinal tunes B and C reproduce the factory SCC, on stock-ACC rlogs.

Feeds the factory target (SCC12 aReqRaw) to each tune as openpilot's command, at the 100 Hz CarController frame rate,
and compares the output with what the factory SCC sent at the same time: SCC12 aReqValue and SCC14 JerkLowerLimit.
The tunes shape the braking side only (rising accel and urgent braking pass through, the upper jerk is today's), so
the fit and the ranking are on the braking side: the lower jerk and the "braking cmd" (aReqRaw < -0.1) and "falling
ramp" (factory aReqValue still above aReqRaw) groups; "rising ramp" is shown for information.
The tunes are the real functions from --starpilot's checkout (branch stinger-stock-tune); only the wiring around
them is replicated here from CarController (see TuneB / TuneC, which mirror create_can_msgs and the egmp block in
CarController.update):
  B  get_hyundai_canfd_scc_jerk_limits + shape_hyundai_canfd_scc_accel(kia_stinger_2022_tune=True, jerk_upper,
     lead_visible, lead_distance), every 2nd frame (50 Hz), accel_last = last accel sent.
  C  update_ioniq_6_longitudinal_tuning(kia_stinger_2022=True; dynamic lower jerk capped at 2.5, no lead) every 5th frame, smoothed only while
     urgent threshold < accel < the smoothed value, else the command passes through and the state follows it.
"today" is what openpilot sends without a tune (aReqValue = aReqRaw, jerk 3.0 / 5.0). Inputs per frame: accel =
SCC12 aReqRaw, enabled = SCC12 ACCMode == 1 with no gas/brake pedal, stopping = SCC12 StopReq (long control state
stopping, else pid), lead = SCC11 ACC_ObjStatus / ACC_ObjDist, v_ego = carState.vEgo, a_ego = carState.aEgo. Comfort
bands are not compared (the tunes send 0, the factory at most 0.1).

Samples with StopReq set are left out of the statistics by default (the tunes still run through them, but the stock
stop sequence says little about the ramps); --include-stopping keeps them. Per group (overall, speed bins, lead
bins, braking/ramp groups): aReqValue |error| median / p95 / mean in m/s^2, and the share of samples whose lower jerk
limit is within 0.15 of the factory's. The cache (--cache FILE.npz) stores the extracted per-frame samples so reruns
skip the rlog parsing.

Usage: stock_mimic_check.py [ROUTE ...] [--starpilot DIR] [--include-stopping] [--cache FILE.npz]
"""
import argparse
import os
import sys
from types import SimpleNamespace

import numpy as np

from logs import add_starpilot_path, pmap, segment_files, short, sorted_events

DEFAULT_ROUTES = ["0000001d--2ead59d2dd", "00000028--297af64c9c", "00000029--b3c2c13819", "00000034--5f8605e608"]
JERK_TOL = 0.15
TODAY = dict(jerk_upper=3.0, jerk_lower=5.0)
SPEED_BINS = ((0, 5), (5, 15), (15, 25), (25, 99))
LEAD_BINS = (("no lead", lambda r: r[:, OBJSTATUS] == 0), ("lead <60 m", lambda r: (r[:, OBJSTATUS] != 0) & (r[:, OBJDIST] < 60)),
             ("lead 60-90 m", lambda r: (r[:, OBJSTATUS] != 0) & (r[:, OBJDIST] >= 60) & (r[:, OBJDIST] < 90)),
             ("lead >=90 m", lambda r: (r[:, OBJSTATUS] != 0) & (r[:, OBJDIST] >= 90)))
TUNES = ("today", "B", "C")
# sample columns: one row per 100 Hz frame, `new` marks frames where a fresh SCC12 arrived (the only ones compared)
T, V, A, PEDAL, MODE, RAW, VALUE, STOP, JU, JL, CBU, CBL, OBJGAP, OBJSTATUS, OBJDIST, OBJRELSPD, NEW = range(17)
NCOL = 17
STINGER = SimpleNamespace(carFingerprint="KIA_STINGER_2022")


def today_upper(stopping):
  """SCC14 upper jerk openpilot sends today (3.0 in pid, 1.0 when stopping or starting)."""
  return 1.0 if stopping else TODAY["jerk_upper"]


class Today:
  """No tune: SCC12 aReqValue = command, SCC14 jerk 3.0 / 5.0."""
  def step(self, f, accel, v, a, stopping, enabled, override, lead, dist):
    return accel, today_upper(stopping), TODAY["jerk_lower"]


class TuneB:
  """Variant B wiring from CarController.create_can_msgs (50 Hz, accel_last = last accel sent, upper jerk as today)."""
  def __init__(self):
    from openpilot.selfdrive.controls.lib.longitudinal_vehicle_tunes import get_hyundai_canfd_scc_jerk_limits, shape_hyundai_canfd_scc_accel
    self.limits, self.shape = get_hyundai_canfd_scc_jerk_limits, shape_hyundai_canfd_scc_accel
    self.accel_last = 0.0
    self.out = (0.0, 3.0, 5.0)

  def step(self, f, accel, v, a, stopping, enabled, override, lead, dist):
    if f % 2 == 0:
      kw = {"kia_stinger_2022_tune": True, "jerk_upper": today_upper(stopping), "lead_visible": lead, "lead_distance": dist}
      jerk, lower = self.limits(STINGER, stopping, accel, **kw)
      accel = self.shape(STINGER, enabled, override, stopping, accel, self.accel_last, **kw)
      self.accel_last = accel
      self.out = (accel, jerk, lower)
    return self.out


class TuneC:
  """Variant C wiring from the egmp block of CarController.update (100 Hz frames, tune every 5th) and create_can_msgs."""
  def __init__(self):
    from opendbc.car import structs
    from opendbc.car.hyundai import carcontroller as cc
    from openpilot.selfdrive.controls.lib import longitudinal_vehicle_tunes as lvt
    self.cc, self.lvt, self.LCS = cc, lvt, structs.CarControl.Actuators.LongControlState
    self.state = cc.Ioniq6LongitudinalTuningState()

  def step(self, f, accel_cmd, v, a, stopping, enabled, override, lead, dist):
    lcs = self.LCS.stopping if stopping else self.LCS.pid
    st, lvt = self.state, self.lvt
    use = True
    if not enabled or override:
      self.state = st = self.cc.reset_egmp_longitudinal_tuning(st)
      use = False
    elif f % 5 == 0:
      self.state = st = self.cc.update_ioniq_6_longitudinal_tuning(st, accel_cmd, v, a, lcs, True, kia_stinger_2022=True)
    accel, jerk, lower = accel_cmd, today_upper(stopping), TODAY["jerk_lower"]
    if use:
      if lvt.KIA_STINGER_2022_SCC_URGENT_ACCEL < accel_cmd < st.actual_accel:
        accel = st.actual_accel
      else:  # rising or urgent: the command passes through and the state follows it
        st.desired_accel = st.actual_accel = st.accel_last = accel_cmd
        st.jerk_upper = lvt.KIA_STINGER_2022_SCC_JERK_UPPER if lcs == self.LCS.pid else 1.0
        if accel_cmd <= lvt.KIA_STINGER_2022_SCC_URGENT_ACCEL:
          st.jerk_lower = lvt.KIA_STINGER_2022_SCC_URGENT_JERK_LOWER
        elif st.jerk_lower <= 0.0:
          st.jerk_lower = lvt.KIA_STINGER_2022_SCC_JERK_LOWER
        st.launch_active = False
        st.stopping = stopping
        st.long_control_state_last = lcs
      jerk, lower = st.jerk_upper, st.jerk_lower
    return accel, jerk, lower


def make_tune(name):
  return {"today": Today, "B": TuneB, "C": TuneC}[name]()


def simulate(rows, names=TUNES):
  """Per-frame (accel, jerk_upper, jerk_lower) of each tune over the frames of one drive, shape (len(names), n, 3)."""
  tunes = [make_tune(n) for n in names]
  out = np.zeros((len(names), len(rows), 3))
  for k, r in enumerate(rows):
    enabled = r[MODE] == 1 and not r[PEDAL]
    args = (k, float(r[RAW]), float(r[V]), float(r[A]), bool(r[STOP] == 1), bool(enabled), bool(r[PEDAL]), bool(r[OBJSTATUS]),
            float(r[OBJDIST]))
    for i, t in enumerate(tunes):
      out[i, k] = t.step(*args)
  return out


def scan(path):
  """One row per carState frame with the latest SCC11 / SCC12 / SCC14 values; shape (n, NCOL)."""
  from opendbc.can import CANParser  # imported before the first log read, so logs.py reuses its schema
  p = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0), ("SCC14", 0)], 0)
  seen = set(); rows = []; new = False
  for e in sorted_events(path):
    w = e.which()
    if w == "can":
      frames = [(c.address, bytes(c.dat), c.src) for c in e.can]
      seen.update(a for a, _, s in frames if s == 0 and a in (905, 1056, 1057))
      new = new or any(a == 1057 and s == 0 for a, _, s in frames)
      p.update([(e.logMonoTime, frames)])
    elif w == "carState" and len(seen) == 3:
      cs = e.carState
      s11, s12, s14 = p.vl["SCC11"], p.vl["SCC12"], p.vl["SCC14"]
      rows.append((e.logMonoTime / 1e9, cs.vEgo, cs.aEgo, float(cs.gasPressed or cs.brakePressed), s12["ACCMode"], s12["aReqRaw"],
                   s12["aReqValue"], s12["StopReq"], s14["JerkUpperLimit"], s14["JerkLowerLimit"], s14["ComfortBandUpper"],
                   s14["ComfortBandLower"], s14["ObjGap"], s11["ACC_ObjStatus"], s11["ACC_ObjDist"], s11["ACC_ObjRelSpd"], float(new)))
      new = False
  return np.array(rows, dtype=float).reshape(-1, NCOL)


def load_route(route, cache):
  if cache and os.path.exists(cache):
    z = np.load(cache)
    if short(route) in z:
      return z[short(route)]
  files = segment_files(route, "rlog")
  if not files:
    return None
  rows = np.concatenate(pmap(scan, files))
  if cache:
    z = dict(np.load(cache)) if os.path.exists(cache) else {}
    z[short(route)] = rows
    np.savez(cache, **z)
  return rows


def evaluate(rows, include_stopping=False, names=TUNES):
  """Frames compared (fresh SCC12, enabled, not stopping unless asked) with each tune's output: (samples, outputs)."""
  out = simulate(rows, names)
  keep = (rows[:, NEW] == 1) & (rows[:, MODE] == 1) & (rows[:, PEDAL] == 0)
  if not include_stopping:
    keep &= rows[:, STOP] == 0
  return rows[keep], out[:, keep]


def metrics(r, o):
  """(a_med, a_p95, a_mean, jl_match %) of tune output o (n, 3) against the factory values in r."""
  err = np.abs(o[:, 0] - r[:, VALUE])
  return (np.median(err), np.percentile(err, 95), np.mean(err),
          100 * np.mean(np.abs(o[:, 2] - r[:, JL]) <= JERK_TOL))


def print_table(title, r, outs, names=TUNES):
  print(f"\n== {title}")
  print(f"{'group':14s} {'n':>8s} | " + " | ".join(f"{n:^29s}" for n in names))
  print(f"{'':14s} {'':>8s} | " + " | ".join(f"{'|err| med/p95/mean':>20s} {'jerkL %':>8s}" for _ in names))
  groups = [("overall", np.ones(len(r), bool))]
  groups += [(f"{lo}-{hi} m/s" if hi < 99 else f"{lo}+ m/s", (r[:, V] >= lo) & (r[:, V] < hi)) for lo, hi in SPEED_BINS]
  groups += [(name, f(r)) for name, f in LEAD_BINS]
  # factory aReqValue still moving toward aReqRaw: down (braking ramp, what the tunes shape) and up (information only)
  groups += [("braking cmd", r[:, RAW] < -0.1), ("falling ramp", r[:, VALUE] > r[:, RAW]), ("rising ramp", r[:, VALUE] < r[:, RAW])]
  for name, m in groups:
    if not m.any():
      print(f"{name:14s} {0:8d}"); continue
    cells = []
    for o in outs:
      a = metrics(r[m], o[m])
      cells.append(f"{a[0]:5.3f}/{a[1]:5.3f}/{a[2]:5.3f} {a[3]:8.1f}")
    print(f"{name:14s} {int(m.sum()):8d} | " + " | ".join(f"{c:29s}" for c in cells))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="*", default=DEFAULT_ROUTES, help="stock-ACC drives with rlogs (default: %(default)s)")
  p.add_argument("--starpilot", default="/home/user/wt/stock-tune", help="checkout with the tunes (default: %(default)s)")
  p.add_argument("--include-stopping", action="store_true", help="keep samples where SCC12 StopReq is set")
  p.add_argument("--cache", help="npz file caching the extracted samples per route")
  args = p.parse_args()
  print(f"tunes from {add_starpilot_path(args.starpilot)}")
  samples = []; outs = []
  for route in args.routes:
    rows = load_route(route, args.cache)
    if rows is None or not len(rows):
      print(f"{short(route)}: no rlogs"); continue
    r, o = evaluate(rows, args.include_stopping)
    samples.append((short(route), r)); outs.append(o)
    print(f"{short(route)}: {len(rows)} frames, {len(r)} stock-ACC active samples ({len(r) / 50:.0f} s)")
  if not samples:
    return 1
  allr = np.concatenate([r for _, r in samples]); allo = np.concatenate(outs, axis=1)
  print_table(f"all routes ({len(allr)} samples)", allr, allo)
  for (name, r), o in zip(samples, outs):
    print_table(f"{name}", r, o)


if __name__ == "__main__":
  sys.exit(main())
