#!/usr/bin/env python3
"""Approaches to a stopped (or nearly stopped) lead: how the factory ACC (stock SCC) and openpilot bring the Stinger to a stop.

One record per approach: the car ends below 0.3 m/s behind a lead that is stopped (<1 m/s) over the last 2 s, the longitudinal
controller is active (stock: SCC12 ACCMode == 1 for rlogs, carState.cruiseState.enabled for qlogs; openpilot:
carControl.longActive) with the driver on neither pedal from the start of the approach until at most the last <= 15 s: a driver
takeover (pedal / disengage) before the car has stopped is kept as a flagged partial approach (profile and model samples end
there, final gap / creep are left out), and the car started from >= 8 mph.
  start      last time the 1 s-smoothed acceleration was >= -0.1 (not braking) before the stop, within 60 s; v_start buckets
  onset      first command < -0.3 m/s^2 after start (stock rlog: SCC12 aReqValue, openpilot: actuators.accel, stock qlog: the 0.5 s
             smoothed carState.aEgo); gap / closing speed / TTC / gap / (v^2 / (2 * 1.5)) are taken there
  class      by lead speed at onset: stopped (<1 m/s), slow (1 m/s .. 0.5 v: lead creeping or still braking), follow (>= 0.5 v: both
             slowing together = stop-and-go following, listed separately and not used in the model), lead late (no lead tracked
             at onset)
  profile    measured deceleration (0.5 s smoothed aEgo) and the command at v = 20/15/10/5/2 mph; peak decel and where (fraction of
             the distance travelled from onset to stop); time and distance below 3 mph ("creep"); final gap to the lead
Sources: stock-rlog (factory SCC11 lead, 50 Hz, the 4 stock rlog drives), stock-qlog (openpilot's own radarState lead on the other
stock drives, ~10 Hz; no factory lead or command), and op (qlog carControl + radarState.leadOne; routes mapped to tunes below).
Fitted stock model: decel = k * v^2 / (2 * (gap - g0)) on the approach samples (onset to v > 1 m/s), pooled and per speed
bucket, plus how constant the decel is through the approach and a speed x gap table of the median decel.
Factory SCC lead distance and openpilot's radar lead differ by ~0.5-2.5 m (within +-0.6 m under 30 m, factory ~1.5-2.5 m farther
beyond ~32 m), so gaps are only comparable between sources to about that.

Usage: stopped_lead_approaches.py [--source all|stock-rlog|stock-qlog|op] [--list] [--rejects] [--cache DIR] [--starpilot DIR]
         [--stock-rlog ROUTE ...] [--stock-qlog ROUTE ...] [--op TUNE=ROUTE ...]
Needs the rlogs/qlogs in the log dir (comma_dl.py); stock-rlog parsing needs opendbc (--starpilot or PYTHONPATH, see logs.py).
"""
import argparse
import os
import sys

import numpy as np

from logs import add_starpilot_path, pmap, route_time, segment_files, short, sorted_events

STOCK_RLOG = ["0000001d--2ead59d2dd", "00000028--297af64c9c", "00000029--b3c2c13819", "00000034--5f8605e608"]
STOCK_QLOG = ["0000002b--66bdc92988", "00000035--fcfb8f4e99", "00000023--3f5d2c1b16", "00000013--c09c96b6b2", "00000024--c157d1aa98",
              "00000027--95ce23ee58", "00000032--9efb5c0a74", "00000033--f7e46742e3", "00000036--d0d092a03d", "00000037--6c52a9db6d"]
OP = ["old=0000000d--5e0d1d244b", "old=0000000e--5ebf4636a1", "A=0000000f--79cf51e7fa", "B=00000011--c7d29db317",
      "C=00000012--2aada6ce32"]
MPH = 2.23694
BUCKETS = ((8, 15), (15, 25), (25, 40), (40, 999))
PROFILE_MPH = (20, 15, 10, 5, 2)
T_, V_, A_, PED_, ACT_, CMD_, LEAD_, D_, VL_, EXP_, PERS_, FORCE_, RADAR_ = range(13)
NCOL = 13
GAP_BREAK = 0.6  # s; a bigger jump in time is a missing segment, not continuous control


# ---------------------------------------------------------------- extraction (one file -> array of rows)
def _stock_rlog_seg(path):
  add_starpilot_path()
  from opendbc.can import CANParser
  p = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0)], 0)
  rows, v, a, ped, t0 = [], 0.0, 0.0, 0, None
  for e in sorted_events(path):
    w = e.which()
    if w == "carState":
      c = e.carState
      v, a, ped = c.vEgo, c.aEgo, int(c.gasPressed or c.brakePressed)
      if t0 is None:
        t0 = e.logMonoTime
    elif w == "can" and t0 is not None:
      p.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
      if p.can_valid and any(c.address == 0x421 and c.src == 0 for c in e.can):
        s11, s12 = p.vl["SCC11"], p.vl["SCC12"]
        # ACC_ObjRelSpd is lead minus ego (negative while closing)
        rows.append((route_time(path, e.logMonoTime, t0), v, a, ped, int(s12["ACCMode"] == 1 and not ped), s12["aReqValue"],
                     int(s11["ACC_ObjStatus"] != 0), s11["ACC_ObjDist"], v + s11["ACC_ObjRelSpd"], 0, -1, 0, 0))
  return np.array(rows, dtype=float).reshape(-1, NCOL)


def _qlog_seg(args):
  path, stock = args
  rows, t0, cs = [], None, None
  lead = (0, 0.0, 0.0, 0)
  exp, pers, force, enabled = 0, -1, 0, 0
  for e in sorted_events(path):
    w = e.which()
    if w == "carState":
      c = e.carState
      cs = (c.vEgo, c.aEgo, int(c.gasPressed or c.brakePressed))
      enabled = int(c.cruiseState.enabled)
      if t0 is None:
        t0 = e.logMonoTime
    elif w == "radarState":
      l = e.radarState.leadOne
      lead = (int(l.status), l.dRel, l.vLead, int(l.radar))
    elif w == "selfdriveState":
      exp, pers = int(e.selfdriveState.experimentalMode), int(e.selfdriveState.personality.raw)
    elif w == "starpilotPlan":
      force = int(e.starpilotPlan.forcingStop or e.starpilotPlan.redLight)
    elif w == "carControl" and cs is not None:
      cc = e.carControl
      act = enabled if stock else int(cc.longActive)
      cmd = np.nan if stock else cc.actuators.accel
      rows.append((route_time(path, e.logMonoTime, t0), cs[0], cs[1], cs[2], int(act and not cs[2]), cmd, lead[0], lead[1], lead[2],
                   exp, pers, force, lead[3]))
  return np.array(rows, dtype=float).reshape(-1, NCOL)


def load_drive(route, source, kind, cache):
  """All segments of one drive as a single time-ordered array (columns: T_ .. RADAR_)."""
  cf = os.path.join(cache, f"sla_{source}_{short(route)}_{kind}.npy") if cache else None
  if cf and os.path.exists(cf):
    return np.load(cf)
  files = segment_files(route, kind)
  if not files:
    print(f"  no {kind} files for {route}", file=sys.stderr)
    return np.zeros((0, NCOL))
  parts = pmap(_stock_rlog_seg, files) if source == "stock-rlog" else pmap(_qlog_seg, [(f, source == "stock-qlog") for f in files])
  a = np.vstack([p for p in parts if len(p)])
  a = a[np.argsort(a[:, T_], kind="stable")]
  if cf:
    np.save(cf, a)
  return a


# ---------------------------------------------------------------- per-drive analysis
def smooth(x, t, w):
  """Centered time-window mean (NaN-aware) over +-w/2 s."""
  ok = ~np.isnan(x)
  c = np.concatenate([[0], np.cumsum(np.where(ok, x, 0.0))])
  n = np.concatenate([[0], np.cumsum(ok)])
  lo, hi = np.searchsorted(t, t - w / 2), np.searchsorted(t, t + w / 2, side="right")
  cnt = n[hi] - n[lo]
  with np.errstate(invalid="ignore", divide="ignore"):
    return np.where(cnt > 0, (c[hi] - c[lo]) / cnt, np.nan)


def seg_blocks(t):
  """Index ranges of continuous recording (no time jump > GAP_BREAK)."""
  cuts = np.where(np.diff(t) > GAP_BREAK)[0] + 1
  return list(zip(np.r_[0, cuts], np.r_[cuts, len(t)]))


def cross_down(v, k0, k1, thr):
  idx = np.where((v[k0:k1] >= thr) & (v[k0 + 1:k1 + 1] < thr))[0] if k1 > k0 else []
  return k0 + int(idx[0]) + 1 if len(idx) else None


def find_approaches(a, source, route, tune, stop_thr=0.3, min_start_mph=8.0, max_lookback=60.0, max_takeover_mph=99.0):
  """Returns (approach records, list of rejection reasons, stationary-lead run records, closing samples).

  A driver pedal press or controller disengage during the last part of the stop (the 15 s before it) is kept as a partial
  approach (max_takeover_mph can limit it): the record is flagged `takeover`, its profile and model samples end at that moment
  and its final gap / creep are the driver's, so the tables leave them out of those two columns.  Stops with the controller
  off for more than 15 s (manual driving) are not approaches."""
  if len(a) < 100:
    return [], [], [], np.zeros((0, 7))
  t, v, aego, ped, act, cmd = (a[:, c] for c in (T_, V_, A_, PED_, ACT_, CMD_))
  a_s, a_1, c_s = (np.full(len(t), np.nan) for _ in range(3))
  has_cmd = not np.all(np.isnan(cmd))
  for lo, hi in seg_blocks(t):
    a_s[lo:hi] = smooth(aego[lo:hi], t[lo:hi], 0.5)
    a_1[lo:hi] = smooth(aego[lo:hi], t[lo:hi], 1.0)
    if has_cmd:
      c_s[lo:hi] = smooth(cmd[lo:hi], t[lo:hi], 0.1 if source == "stock-rlog" else 0.3)
  dt = float(np.median(np.diff(t)))
  sig = c_s if has_cmd else a_s  # onset / "braking" signal
  ctl = (act > 0) & (ped == 0)
  out, rej = [], []
  for lo, hi in seg_blocks(t):
    last_end = lo
    for i in range(lo + 1, hi):
      if not (v[i] < stop_thr <= v[i - 1]):
        continue
      if np.max(v[max(lo, i - int(max_lookback / dt)):i]) * MPH < min_start_mph:
        continue  # not a stop from speed (creeping / stop-and-go shuffle)
      # last controlled sample at or before the stop; the driver may only take over below max_takeover_mph
      e = i
      while e > lo and not ctl[e] and t[i] - t[e] < 15.0:
        e -= 1
      if not ctl[e]:
        rej.append("controller off / pedal for > 15 s before the stop")
        last_end = i
        continue
      if e < i and v[e] * MPH > max_takeover_mph:
        rej.append("driver took over above %g mph" % max_takeover_mph)
        last_end = i
        continue
      # walk back from e to where the car was last not braking (and still above ~walking speed), controller on
      j = e
      while j > last_end and ctl[j] and t[i] - t[j] < max_lookback and not (a_1[j] >= -0.1 and v[j] > 1.0):
        j -= 1
      if not ctl[j:e + 1].all():
        rej.append("pedal / controller off during approach")
      elif t[i] - t[j] >= max_lookback:
        rej.append("braking phase longer than 60 s")
      elif j <= last_end and last_end > lo:
        rej.append("overlaps previous approach")
      elif v[j] * MPH < min_start_mph:
        rej.append("start < %g mph" % min_start_mph)
      else:
        r = analyse(a, a_s, c_s, sig, has_cmd, j, i, dt, source, route, tune, e)
        if "rejected" in r:
          rej.append(r["rejected"])
        else:
          out.append(r)
      last_end = i
  return out, rej, find_runs(a, a_s, c_s, sig, has_cmd, dt, source, route, tune), closing_samples(a, a_s, c_s, ctl, dt)


def closing_samples(a, a_s, c_s, ctl, dt, min_closing=1.5):
  """10 Hz samples with the controller on and a tracked lead that the car is closing on (any lead speed):
  columns closing speed, gap, lead speed, ego speed, measured decel, commanded decel (nan without), required decel to stop 3 m behind."""
  v, lead, d, vl = a[:, V_], a[:, LEAD_], a[:, D_], a[:, VL_]
  idx = np.arange(0, len(v), max(1, int(round(0.1 / dt))))
  idx = idx[ctl[idx] & (lead[idx] > 0) & (d[idx] < 100) & (v[idx] > 1.0) & ((v[idx] - vl[idx]) > min_closing) & ~np.isnan(a_s[idx])]
  clo = v[idx] - vl[idx]
  return np.column_stack([clo, d[idx], vl[idx], v[idx], -a_s[idx], -c_s[idx], clo ** 2 / (2 * np.maximum(d[idx] - 3.0, 1.0))])


def find_runs(a, a_s, c_s, sig, has_cmd, dt, source, route, tune, min_dur=2.0, min_v_mph=8.0):
  """Runs where the controller is active with a tracked, stationary lead (smoothed lead speed < 1 m/s, gap < 120 m) ahead of a
  moving car: the approach to a stopped lead whether or not it ends in a stop (the lead may move off, be lost, or the driver
  takes over). One record per run with the state when the lead became stationary, the braking onset inside the run, the decel
  profile, and why it ended (stop / lead moved / lead lost / takeover / other)."""
  t, v, ped, act, lead, d, vl = (a[:, c] for c in (T_, V_, PED_, ACT_, LEAD_, D_, VL_))
  vls = np.full(len(t), np.nan)
  for lo, hi in seg_blocks(t):
    vls[lo:hi] = smooth(np.where(lead[lo:hi] > 0, vl[lo:hi], np.nan), t[lo:hi], 0.6)
  ctl = (act > 0) & (ped == 0)
  m = ctl & (lead > 0) & (vls < 1.0) & (d < 120) & (v > 1.0)
  idx = np.where(m)[0]
  if not len(idx):
    return []
  cut = np.where((t[idx[1:]] - t[idx[:-1]]) > 0.5)[0] + 1
  out = []
  for run in np.split(idx, cut):
    s0, e = run[0], run[-1]
    if t[e] - t[s0] < min_dur or v[s0] * MPH < min_v_mph or d[s0] - d[e] < 2.0:
      continue
    r = dict(route=route, source=source, tune=tune, t_start=t[s0], t_end=t[e], v_ls=v[s0] * MPH, v_start=v[s0] * MPH, gap_ls=float(d[s0]),
             ratio_ls=float(d[s0] / (v[s0] ** 2 / 3.0)), v_end=v[e] * MPH, gap_end=float(d[e]), min_gap=float(np.min(d[run])), cls="run",
             exp=float(np.mean(a[s0:e + 1, EXP_])), pers=int(np.bincount(a[s0:e + 1, PERS_].astype(int) + 1).argmax()) - 1,
             force=bool(a[s0:e + 1, FORCE_].any()), radar=float(np.mean(a[s0:e + 1, RADAR_])) if source == "op" else np.nan)
    nxt = slice(e + 1, min(len(t), e + 1 + int(2.0 / dt)))
    if v[e] < 1.2 or np.any(v[nxt] < 0.3):
      r["end"] = "stop"
    elif not ctl[min(e + 1, len(t) - 1)]:
      r["end"] = "takeover"
    elif lead[min(e + 1, len(t) - 1)] == 0:
      r["end"] = "lead lost"
    else:
      r["end"] = "lead moved"
    brk = np.where(sig[s0:e + 1] < -0.3)[0]
    if not len(brk):
      r["end"] += " (no braking)"
      r["samples"] = np.zeros((0, 5))
      out.append(r)
      continue
    k0 = s0 + int(brk[0])
    pre = k0 == s0  # already braking when the lead became stationary / was acquired
    g0 = float(d[k0])
    r.update(pre_braking=pre, v_onset=v[k0] * MPH, gap_onset=g0, closing=v[k0] - float(vl[k0]), ratio=g0 / (v[k0] ** 2 / 3.0),
             ttc=g0 / max(v[k0] - float(vl[k0]), 0.1), t_onset=t[k0])
    sl = slice(k0, e + 1)
    kp = k0 + int(np.nanargmin(a_s[sl]))
    r["peak_meas"] = float(a_s[kp])
    trav = np.concatenate([[0], np.cumsum(0.5 * (v[k0:e] + v[k0 + 1:e + 1]) * np.diff(t[k0:e + 1]))])
    r["peak_frac"] = float(trav[kp - k0] / max(trav[-1], 0.1))
    r["need_ls"] = float(v[s0] ** 2 / (2 * max(d[s0] - (r["gap_end"] if r["end"] == "stop" else 0.0) - 3.0, 1.0)))  # to stop 3 m short
    for mph in PROFILE_MPH:
      k = cross_down(v, k0, e, mph / MPH)
      r[f"a{mph}"] = float(a_s[k]) if k is not None else np.nan
      r[f"c{mph}"] = float(c_s[k]) if (k is not None and has_cmd) else np.nan
      r[f"g{mph}"] = float(d[k]) if k is not None else np.nan
    step = max(1, int(round(0.1 / dt)))
    ii = np.arange(s0, e + 1, step)
    ii = ii[~np.isnan(a_s[ii])]
    r["samples"] = np.column_stack([v[ii], d[ii], -a_s[ii], -c_s[ii] if has_cmd else np.full(len(ii), np.nan), vl[ii]])
    out.append(r)
  return out


def analyse(a, a_s, c_s, sig, has_cmd, j, i, dt, source, route, tune, e):
  t, v, d, vl, lead = a[:, T_], a[:, V_], a[:, D_], a[:, VL_], a[:, LEAD_]
  r = dict(route=route, source=source, tune=tune, t_stop=t[i], t_start=t[j], v_start=v[j] * MPH,
           exp=float(np.mean(a[j:i + 1, EXP_])), pers=int(np.bincount(a[j:i + 1, PERS_].astype(int) + 1).argmax()) - 1,
           force=bool(a[j:i + 1, FORCE_].any()), radar=float(np.mean(a[j:i + 1, RADAR_])) if source == "op" else np.nan)
  r["takeover"] = e < i
  r["v_takeover"] = v[e] * MPH if e < i else np.nan
  r["gap_takeover"] = float(a[e, D_]) if (e < i and a[e, LEAD_] > 0) else np.nan
  seg = sig[j:e + 1] < -0.3
  if not seg.any():
    return dict(rejected="no braking below -0.3")
  k0 = j + int(np.argmax(seg))
  w = (t >= t[i] - 2.0) & (t <= t[i]) & (lead > 0)  # lead over the last 2 s before the stop
  if w.sum() < 3:
    return dict(rejected="no lead at the stop")
  r["vl_end"] = float(np.median(vl[w]))
  after = (t >= t[i]) & (t <= t[i] + 0.6) & (lead > 0)
  r["final_gap"] = float(np.median(d[after])) if after.sum() >= 2 else float(d[w][-1])
  if r["vl_end"] >= 1.0 or r["final_gap"] > 30:
    return dict(rejected="lead moving or too far at the stop")
  r.update(t_onset=t[k0], v_onset=v[k0] * MPH)
  w0 = (t >= t[k0] - 0.5) & (t <= t[k0] + 1.0) & (lead > 0)  # lead as tracked around the onset
  if w0.sum() >= 2:
    g0, vl0 = float(np.median(d[w0])), float(np.median(vl[w0]))
    closing = v[k0] - vl0
    r.update(gap_onset=g0, vl_onset=vl0, closing=closing, ttc=g0 / max(closing, 0.1), ratio=g0 / (v[k0] ** 2 / 3.0),
             total_closed=g0 - r["final_gap"], cls="stopped" if vl0 < 1.0 else ("slow" if vl0 < 0.5 * v[k0] else "follow"))
  else:
    r["cls"] = "lead late"
  s = np.concatenate([[0], np.cumsum(0.5 * (v[k0:e] + v[k0 + 1:e + 1]) * np.diff(t[k0:e + 1]))])  # distance travelled from onset
  r["travel"], r["dur"] = float(s[-1]), float(t[e] - t[k0])
  kpk = k0 + int(np.nanargmin(a_s[k0:e + 1]))
  r["peak_meas"], r["peak_frac"] = float(a_s[kpk]), float(s[kpk - k0] / max(s[-1], 0.1))
  if has_cmd:
    kc = k0 + int(np.nanargmin(c_s[k0:e + 1]))
    r["peak_cmd"], r["peak_cmd_frac"] = float(c_s[kc]), float(s[kc - k0] / max(s[-1], 0.1))
  for m in PROFILE_MPH:
    k = cross_down(v, k0, e, m / MPH)
    r[f"a{m}"] = float(a_s[k]) if k is not None else np.nan
    r[f"c{m}"] = float(c_s[k]) if (k is not None and has_cmd) else np.nan
    r[f"g{m}"] = float(d[k]) if (k is not None and lead[k] > 0) else np.nan
  kc = e  # creep: the last continuous stretch below 3 mph before the stop (or the driver taking over)
  while kc > k0 and v[kc - 1] < 3 / MPH:
    kc -= 1
  r["creep_t"] = float(t[e] - t[kc])
  r["creep_d"] = float(np.sum(0.5 * (v[kc:e] + v[kc + 1:e + 1]) * np.diff(t[kc:e + 1]))) if e > kc else 0.0
  # state when the lead became stationary (vl < 1 m/s for 0.5 s) with the car still moving: what the controller had to solve
  w_ls = np.where((lead[k0:e + 1] > 0) & (vl[k0:e + 1] < 1.0) & (v[k0:e + 1] > 1.5))[0]
  if len(w_ls):
    ls = k0 + int(w_ls[0])
    r.update(v_ls=v[ls] * MPH, gap_ls=float(d[ls]), ratio_ls=float(d[ls] / (v[ls] ** 2 / 3.0)),
             need_ls=float(v[ls] ** 2 / (2 * max(d[ls] - r["final_gap"], 1.0))), peak_after_ls=float(np.nanmin(a_s[ls:e + 1])),
             t_ls=float(t[ls]), a_ls=float(a_s[ls]))
  step = max(1, int(round(0.1 / dt)))  # model samples: v > 1 m/s with a lead, ~10 Hz
  idx = np.arange(k0, e + 1, step)
  idx = idx[(v[idx] > 1.0) & (lead[idx] > 0) & ~np.isnan(a_s[idx])]
  r["samples"] = np.column_stack([v[idx], d[idx], -a_s[idx], -c_s[idx] if has_cmd else np.full(len(idx), np.nan), vl[idx]])
  return r


def catalog_op_routes(skip):
  """'build=route' for the openpilot-long drives in routes/ROUTES.md (All routes table) with local qlogs."""
  import re
  path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "routes", "ROUTES.md")
  seen, out = set(skip), []
  for line in open(path):
    m = re.match(r"\| `([0-9a-f]{8}--[0-9a-f]{10})` \|[^|]*\|[^|]*\| ([^|]*) \| openpilot-long", line)
    if m and m.group(1) not in seen and segment_files(m.group(1), "qlog"):
      seen.add(m.group(1))
      build = re.sub(r"\s*`.*`", "", m.group(2)).strip().replace(" ", "-")
      out.append(f"{build}={m.group(1)}")
  return out


# ---------------------------------------------------------------- reporting
def med(x):
  x = np.asarray([y for y in x if y is not None and not np.isnan(y)], dtype=float)
  return (float(np.median(x)), len(x)) if len(x) else (np.nan, 0)


def fmt(x, f="%.1f"):
  return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f % x


def group_table(title, groups, keyname):
  print(f"\n## {title}\n")
  cols = ("n (driver takeover)", "start", "gap@onset", "closing", "TTC", "gap/(v^2/3)", "peak decel", "peak at (frac)", "peak/avg need", "a@20", "a@15",
          "a@10", "a@5", "a@2", "creep s", "creep m", "final gap")
  print(f"| {keyname} | " + " | ".join(cols) + " |")
  print("|" + "---|" * (len(cols) + 1))
  for name, rs in groups:
    if not rs:
      continue
    def m(key, f="%.1f", neg=False, clean=False):
      x, n = med([r.get(key) for r in rs if not (clean and r["takeover"])])  # creep / final gap: only if the controller finished the stop
      return fmt(-x if neg else x, f)
    both = [r for r in rs if "total_closed" in r]
    pk = [-r["peak_meas"] / (r["v_onset"] ** 2 / MPH ** 2 / (2 * max(r["total_closed"], 1.0))) for r in both]
    print(f"| {name} | {len(rs)} ({sum(r['takeover'] for r in rs)}) | {m('v_start', '%.0f')} mph | {m('gap_onset')} m | {m('closing')} m/s | {m('ttc')} s | "
          f"{m('ratio', '%.2f')} | {m('peak_meas', '%.2f', True)} | {m('peak_frac', '%.2f')} | {fmt(med(pk)[0], '%.1fx')} | "
          + " | ".join(m(f"a{x}", "%.2f", True) for x in PROFILE_MPH) + f" | {m('creep_t', clean=True)} | {m('creep_d', clean=True)} | {m('final_gap', clean=True)} m |")


def closing_report(closing, names):
  """Section C: every closing event (lead of any speed), decel achieved vs the decel needed to stop 3 m behind the lead."""
  print("\n# C. Closing events with any lead (controller on, tracked lead, closing > 1.5 m/s, gap < 100 m): decel achieved vs needed")
  print("\n'needed' = closing^2 / (2 * (gap - 3 m)); measured decel is the 0.5 s smoothed aEgo. Medians (p25-p75) by needed-decel bin; "
        "samples at 10 Hz, from several events each, so n is samples not events.\n")
  rb = [(0, 0.25), (0.25, 0.5), (0.5, 0.75), (0.75, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0), (3.0, 99)]
  for lead_kind, sel in (("stopped lead (lead speed < 1 m/s)", lambda x: x[:, 2] < 1.0), ("moving lead (>= 1 m/s)", lambda x: x[:, 2] >= 1.0)):
    print(f"\n{lead_kind}\n")
    print("| needed decel | " + " | ".join(f"{src}/{t} n, median (p25-p75)" for src, t in names) + " |")
    print("|---|" + "---|" * len(names))
    sets = {k: np.vstack([c[2] for c in closing if (c[0], c[1] if (c[0] != "op" or c[1] in ("A", "B", "C", "old")) else "other builds") == k]
                         or [np.zeros((0, 7))]) for k in names}
    for lo, hi in rb:
      row = []
      for k in names:
        x = sets[k]
        x = x[sel(x)]
        w = x[(x[:, 6] >= lo) & (x[:, 6] < hi)]
        row.append(f"{len(w)}, {np.median(w[:, 4]):.2f} ({np.percentile(w[:, 4], 25):.2f}-{np.percentile(w[:, 4], 75):.2f})" if len(w) >= 15 else "-")
      print(f"| {lo}-{hi if hi < 99 else '+'} | " + " | ".join(row) + " |")
  print("\n## Fit decel = k * closing^2 / (2 * (gap - g0 - th * lead_speed)), k free, grid g0 / th\n")
  for k in names:
    x = np.vstack([c[2] for c in closing if (c[0], c[1] if (c[0] != "op" or c[1] in ("A", "B", "C", "old")) else "other builds") == k]
                  or [np.zeros((0, 7))])
    for kind, sel in (("stopped lead", x[:, 2] < 1.0), ("moving lead", x[:, 2] >= 1.0)):
      y = x[sel]
      if len(y) < 60:
        continue
      best = None
      for g0 in np.arange(0, 10.1, 0.5):
        for th in (np.arange(0, 2.01, 0.2) if kind == "moving lead" else [0.0]):
          den = y[:, 1] - g0 - th * y[:, 2]
          ok = den > 1.0
          if ok.sum() < 0.9 * len(y):
            continue
          f = y[ok, 0] ** 2 / (2 * den[ok])
          kk = float(np.sum(f * y[ok, 4]) / np.sum(f * f))
          mse = float(np.mean((y[ok, 4] - kk * f) ** 2))
          if best is None or mse < best[0]:
            r2 = 1 - np.sum((y[ok, 4] - kk * f) ** 2) / np.sum((y[ok, 4] - y[ok, 4].mean()) ** 2)
            best = (mse, g0, th, kk, int(ok.sum()), r2)
      if best:
        print(f"- {k[0]}/{k[1]} {kind}: g0 {best[1]:.1f} m, th {best[2]:.1f} s, k {best[3]:.2f}, RMSE {best[0] ** 0.5:.2f} m/s^2, R^2 {best[5]:.2f}, n {best[4]} samples")


def run_table(title, groups, keyname):
  print(f"\n## {title}\n")
  cols = ("n runs", "ended: stop / lead moved / lead lost / takeover", "v_ls mph", "gap_ls m", "gap/(v^2/3)", "n onset", "onset gap m",
          "onset TTC s", "onset gap/(v^2/3)", "peak decel", "a@20", "a@15", "a@10", "a@5", "a@2", "min gap m")
  print(f"| {keyname} | " + " | ".join(cols) + " |")
  print("|" + "---|" * (len(cols) + 1))
  for name, rs in groups:
    if not rs:
      continue
    on = [r for r in rs if "gap_onset" in r and not r["pre_braking"]]
    def m(key, f="%.1f", neg=False, src=rs):
      x, n = med([r.get(key) for r in src])
      return fmt(-x if neg else x, f)
    ends = [sum(r["end"].startswith(k) for r in rs) for k in ("stop", "lead moved", "lead lost", "takeover")]
    print(f"| {name} | {len(rs)} | {' / '.join(map(str, ends))} | {m('v_ls', '%.0f')} | {m('gap_ls')} | {m('ratio_ls', '%.2f')} | {len(on)} | "
          f"{m('gap_onset', src=on)} | {m('ttc', src=on)} | {m('ratio', '%.2f', src=on)} | {m('peak_meas', '%.2f', True)} | "
          + " | ".join(m(f"a{x}", "%.2f", True) for x in PROFILE_MPH) + f" | {m('min_gap')} |")


def ls_table(title, groups, keyname):
  print(f"\n## {title}\n")
  cols = ("n (takeover)", "v_ls mph", "gap_ls m", "gap/(v^2/3)", "needed decel", "decel at that moment", "peak decel after", "peak/needed",
          "final gap (clean)", "creep s (clean)")
  print(f"| {keyname} | " + " | ".join(cols) + " |")
  print("|" + "---|" * (len(cols) + 1))
  for name, rs in groups:
    if not rs:
      continue
    def m(key, f="%.1f", neg=False, clean=False):
      x, n = med([r.get(key) for r in rs if not (clean and r["takeover"])])
      return fmt(-x if neg else x, f)
    pr = [-r["peak_after_ls"] / r["need_ls"] for r in rs]
    print(f"| {name} | {len(rs)} ({sum(r['takeover'] for r in rs)}) | {m('v_ls', '%.0f')} | {m('gap_ls')} | {m('ratio_ls', '%.2f')} | {m('need_ls', '%.2f')} | "
          f"{m('a_ls', '%.2f', True)} | {m('peak_after_ls', '%.2f', True)} | {fmt(med(pr)[0], '%.1fx')} | {m('final_gap', clean=True)} | {m('creep_t', clean=True)} |")


def bucket_of(mph):
  for lo, hi in BUCKETS:
    if lo <= mph < hi:
      return f"{lo}-{hi}" if hi < 999 else f"{lo}+"
  return None


def fit_model(samples, label, dmax=80.0):
  """decel = k * v^2 / (2 (gap - g0)): grid over g0, least-squares k (and the k = 1 variant)."""
  if not samples:
    return
  s = np.vstack(samples)
  s = s[(s[:, 1] < dmax) & (s[:, 4] < 1.0)]  # lead stopped samples only
  v, g, dec = s[:, 0], s[:, 1], s[:, 2]
  if len(s) < 50:
    print(f"- {label}: only {len(s)} samples")
    return
  best = {}
  for g0 in np.arange(0.0, 12.01, 0.25):
    ok = g - g0 > 1.0
    if ok.sum() < 0.9 * len(s):
      continue
    x = v[ok] ** 2 / (2 * (g[ok] - g0))
    k = float(np.sum(x * dec[ok]) / np.sum(x * x))
    for name, kk in (("k free", k), ("k=1", 1.0)):
      mse = float(np.mean((dec[ok] - kk * x) ** 2))
      if name not in best or mse < best[name][0]:
        best[name] = (mse, g0, kk, int(ok.sum()), dec[ok], kk * x)
  for name in ("k=1", "k free"):
    if name in best:
      mse, g0, k, n, y, yh = best[name]
      r2 = 1 - np.sum((y - yh) ** 2) / np.sum((y - y.mean()) ** 2)
      print(f"- {label} [{name}]: g0 {g0:.2f} m, k {k:.2f}, RMSE {np.sqrt(mse):.2f} m/s^2, R^2 {r2:.2f}, n {n} samples (10 Hz), "
            f"mean decel {y.mean():.2f}")


def binned_decel(samples, label):
  if not samples:
    return
  s = np.vstack(samples)
  s = s[s[:, 4] < 1.0]
  vb = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 30), (30, 99)]
  gb = [(0, 4), (4, 7), (7, 12), (12, 20), (20, 35), (35, 80)]
  print(f"\n{label}: median measured decel in m/s^2 by speed (rows, mph) and lead gap (columns, m); (n samples), cells need >= 20\n")
  print("| mph \\ gap | " + " | ".join(f"{lo}-{hi}" for lo, hi in gb) + " |")
  print("|" + "---|" * (len(gb) + 1))
  for lo, hi in vb:
    row = []
    for glo, ghi in gb:
      w = (s[:, 0] * MPH >= lo) & (s[:, 0] * MPH < hi) & (s[:, 1] >= glo) & (s[:, 1] < ghi)
      row.append(f"{np.median(s[w, 2]):.2f} ({w.sum()})" if w.sum() >= 20 else "-")
    print(f"| {lo}-{hi} | " + " | ".join(row) + " |")


def constancy(rs, label):
  """Per approach: variation of the measured decel while v is between 20 and 5 mph."""
  cv, ratio = [], []
  for r in rs:
    s = r["samples"]
    w = (s[:, 0] * MPH < 20) & (s[:, 0] * MPH > 5)
    if w.sum() < 8:
      continue
    x = s[w, 2]
    cv.append(np.std(x) / max(np.mean(x), 0.05))
    ratio.append(np.percentile(x, 90) / max(np.median(x), 0.05))
  if cv:
    print(f"- {label}: decel between 20 and 5 mph per approach, CV median {np.median(cv):.2f} (p25 {np.percentile(cv, 25):.2f}, "
          f"p75 {np.percentile(cv, 75):.2f}), p90/median {np.median(ratio):.2f}, n {len(cv)} approaches")


def list_runs(rs):
  cols = ("source", "route", "start t (s)", "end t", "tune/build", "v_ls mph", "gap_ls m", "onset gap m", "onset TTC s", "peak decel", "end",
          "end mph", "end gap m", "exp", "pers", "force-stop", "radar lead")
  print("| " + " | ".join(cols) + " |")
  print("|" + "---|" * len(cols))
  for r in rs:
    print(f"| {r['source']} | {short(r['route'])} | {r['t_start']:.0f} | {r['t_end']:.0f} | {r['build']} | {r['v_ls']:.0f} | {r['gap_ls']:.1f} | "
          f"{fmt(r.get('gap_onset'))}{'*' if r.get('pre_braking') else ''} | {fmt(r.get('ttc'))} | {fmt(-r['peak_meas'], '%.2f') if 'peak_meas' in r else '-'} | "
          f"{r['end']} | {r['v_end']:.1f} | {r['gap_end']:.1f} | {r['exp']:.1f} | {r['pers']} | {int(r['force'])} | {fmt(r['radar'], '%.2f')} |")
  print("\n* braking was already under way when the lead became stationary / was first tracked")


def list_approaches(rs):
  cols = ("source", "route", "stop t (s)", "start t", "tune", "class", "start mph", "onset gap m", "closing m/s", "TTC s", "v_ls mph", "gap_ls m", "peak decel",
          "final gap m", "creep s", "takeover mph", "exp", "pers", "force-stop", "radar lead")
  print("| " + " | ".join(cols) + " |")
  print("|" + "---|" * len(cols))
  for r in rs:
    print(f"| {r['source']} | {short(r['route'])} | {r['t_stop']:.0f} | {r['t_start']:.0f} | {r['build']} | {r['cls']} | {r['v_start']:.0f} | "
          f"{fmt(r.get('gap_onset'))} | {fmt(r.get('closing'))} | {fmt(r.get('ttc'))} | {fmt(r.get('v_ls'), '%.0f')} | {fmt(r.get('gap_ls'))} | {fmt(-r['peak_meas'], '%.2f')} | {fmt(r['final_gap'])} | "
          f"{fmt(r['creep_t'])} | {fmt(r['v_takeover'], '%.1f')} | {r['exp']:.1f} | {r['pers']} | {int(r['force'])} | {fmt(r['radar'], '%.2f')} |")


def main():
  p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  p.add_argument("--source", default="all", choices=("all", "stock-rlog", "stock-qlog", "op"))
  p.add_argument("--stock-rlog", nargs="*", default=STOCK_RLOG)
  p.add_argument("--stock-qlog", nargs="*", default=STOCK_QLOG)
  p.add_argument("--op", nargs="*", default=OP, metavar="TUNE=ROUTE")
  p.add_argument("--op-catalog", action="store_true", help="also every other openpilot-long drive of routes/ROUTES.md that has qlogs "
                 "locally, tune = its build")
  p.add_argument("--list", action="store_true", help="print every accepted approach")
  p.add_argument("--rejects", action="store_true", help="count rejected stops by reason")
  p.add_argument("--cache", help="directory for per-drive extracted arrays (skips log parsing on reruns)")
  p.add_argument("--starpilot", help="StarPilot checkout for opendbc (stock-rlog parsing)")
  args = p.parse_args()
  if args.starpilot:
    add_starpilot_path(args.starpilot)
  if args.cache:
    os.makedirs(args.cache, exist_ok=True)
  if args.op_catalog:
    args.op = args.op + catalog_op_routes([x.split("=", 1)[1] for x in args.op])
  jobs = []
  if args.source in ("all", "stock-rlog"):
    jobs += [("stock-rlog", "rlog", "stock", r) for r in args.stock_rlog]
  if args.source in ("all", "stock-qlog"):
    jobs += [("stock-qlog", "qlog", "stock", r) for r in args.stock_qlog]
  if args.source in ("all", "op"):
    jobs += [("op", "qlog", *x.split("=", 1)) for x in args.op]
  keep, allruns, reasons, closing = [], [], {}, []
  for source, kind, tune, route in jobs:
    a = load_drive(route, source, kind, args.cache)
    found, rej, runs, clo = find_approaches(a, source, route, tune)
    closing.append((source, tune, clo))
    print(f"{source} {short(route)} ({tune}): {len(a)} rows, {len(found)} stop approaches, {len(rej)} rejected stops, {len(runs)} stationary-lead runs",
          file=sys.stderr)
    keep += found
    allruns += runs
    for x in rej:
      reasons[(source, x)] = reasons.get((source, x), 0) + 1
  for r in keep + allruns:  # catalog builds are one group, the build stays in r["build"]
    r["build"] = r["tune"]
    if r["source"] == "op" and r["tune"] not in ("A", "B", "C", "old"):
      r["tune"] = "other builds"
  names = sorted(dict.fromkeys((r["source"], r["tune"]) for r in keep + allruns),
                 key=lambda x: (x[0], "~" if x[1] == "other builds" else x[1]))
  srcs = list(dict.fromkeys(r["source"] for r in keep + allruns))
  bk = [bucket_of((lo + hi) / 2) for lo, hi in BUCKETS]
  print("# Stopped-lead approaches\n")
  print(f"{len(allruns)} stationary-lead runs (controller active, tracked lead < 1 m/s, car starting from >= 8 mph) and {len(keep)} stops "
        "behind a stopped lead (car reached 0.3 m/s, controller on from the start of braking).")
  if args.rejects:
    print("\nRejected stops (car stopped from >= 8 mph but not usable): " + "; ".join(f"{s}: {r} = {n}" for (s, r), n in sorted(reasons.items())))

  print("\n# A. Stationary-lead runs (every approach to a stationary lead, whether or not the car got to a stop)")
  run_table("Per source / tune (medians; onset columns only for runs where braking began after the lead was stationary)",
            [(f"{s}/{t}", [r for r in allruns if (r["source"], r["tune"]) == (s, t)]) for s, t in names], "source/tune")
  run_table("Per source and speed when the lead became stationary (mph)",
            [(f"{s} {b}", [r for r in allruns if r["source"] == s and bucket_of(r["v_ls"]) == b]) for s in srcs for b in bk], "source / mph")
  run_table("openpilot tunes by speed",
            [(f"op/{t} {b}", [r for r in allruns if r["source"] == "op" and r["tune"] == t and bucket_of(r["v_ls"]) == b])
             for s, t in names if s == "op" for b in bk], "tune / mph")
  pool = [r for r in allruns if len(r["samples"])]
  print("\n## Fitted model: decel = k * v^2 / (2 * (gap - g0)) on run samples (10 Hz, lead stationary, gap < 80 m)\n")
  for src in srcs:
    rs = [r for r in pool if r["source"] == src]
    fit_model([r["samples"] for r in rs], f"{src} all (n {len(rs)} runs)")
    for lo, hi in BUCKETS:
      sub = [r for r in rs if lo <= r["v_ls"] < hi]
      if len(sub) >= 4:
        fit_model([r["samples"] for r in sub], f"{src} v_ls {lo}-{hi} mph (n {len(sub)} runs)")
  for src, tune in names:
    if src == "op":
      rs = [r for r in pool if r["tune"] == tune and r["source"] == "op"]
      if len(rs) >= 3:
        fit_model([r["samples"] for r in rs], f"op/{tune} (n {len(rs)} runs)")
  print("\n## Constancy of decel through the approach (runs, v between 20 and 5 mph)\n")
  for src, tune in names:
    constancy([r for r in pool if (r["source"], r["tune"]) == (src, tune)], f"{src}/{tune}")
  for src in srcs:
    binned_decel([r["samples"] for r in pool if r["source"] == src], src)

  closing_report(closing, names)

  print("\n# B. Completed stops behind a stopped lead (car reached 0.3 m/s)")
  print("\n## Class counts (lead speed at braking onset: stopped < 1 m/s, slow < 0.5 v, follow = both slowing together)\n")
  print("| source | tune | stopped | slow | follow | lead late |\n|---|---|---|---|---|---|")
  for src, tune in names:
    rs = [r for r in keep if (r["source"], r["tune"]) == (src, tune)]
    print(f"| {src} | {tune} | " + " | ".join(str(sum(r["cls"] == c for r in rs)) for c in ("stopped", "slow", "follow", "lead late")) + " |")
  main_cls = [r for r in keep if r["cls"] == "stopped"]
  group_table("Per source / tune, class 'stopped' (medians)",
              [(f"{s}/{t}", [r for r in main_cls if (r["source"], r["tune"]) == (s, t)]) for s, t in names], "source/tune")
  group_table("Per source and start speed, class 'stopped' (medians)",
              [(f"{s} {b}", [r for r in main_cls if r["source"] == s and bucket_of(r["v_start"]) == b]) for s in srcs for b in bk], "source / mph")
  other = [r for r in keep if r["cls"] in ("slow", "follow", "lead late") and "peak_meas" in r]
  group_table("Classes 'slow' and 'follow' (not 'approach to a stopped lead'; medians)",
              [(f"{s}/{t} {c}", [r for r in other if (r["source"], r["tune"]) == (s, t) and r["cls"] == c]) for s, t in names
               for c in ("slow", "follow", "lead late")], "source / tune / class")
  if args.list:
    print("\n## Stationary-lead runs\n")
    list_runs(sorted(allruns, key=lambda r: (r["source"], r["tune"], short(r["route"]), r["t_start"])))
    print("\n## Stops\n")
    list_approaches(sorted(keep, key=lambda r: (r["source"], r["tune"], short(r["route"]), r["t_stop"])))


if __name__ == "__main__":
  main()
