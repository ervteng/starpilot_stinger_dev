#!/usr/bin/env python3
"""Lateral (torque controller) tracking, centre chatter and driver interventions of a drive, or two drives side by side.

Reads controlsState.lateralControlState.torqueState (desired / actual lateral accel, error, output, p/i/d/f,
saturated), carState (vEgo, steering angle / pressed / torque, blinkers), carControl.latActive,
starpilotLateralState (effective friction threshold), liveTorqueParameters, liveParameters, liveDelay, carParams and
the initData params. qlog carries these at 10 Hz, rlog at 100 Hz; use the same kind for drives you compare.

Only lateral-active samples count: lane changes (modelV2 laneChangeState != off, via drivingModelData), driver
overrides (steeringPressed, +-1 s around), engagement settling (1 s) and vEgo < 2 m/s are excluded. Everything is
split by speed band (0-20, 20-40, 40-60, 60+ mph), by |desired| band (centre < 0.3, 0.3-1.0, > 1.0 m/s^2), by phase
(turn-in: desired*jerk > 0.2, unwind: < -0.2, else steady; centre has no phase) and by side (left = desired > 0).

Per cell:
  under   mean sign(desired)*(desired - actual): positive = the car is short of the plan (understeer / late),
          negative = beyond it. In the centre band the signed mean (desired - actual) is shown instead.
  rms     RMS of desired - actual (m/s^2).        sat%  share of samples with the controller's saturated flag (it needs a
  timer and is often false while the command sits at 1.0, so peg% = share with |output| >= 0.98 is shown too).   |out| p50/p90/p99.
Per speed band also: lag = shift of the actual that maximises its correlation with desired (positive = actual
follows desired by that long; resolution is interpolated but data is 0.1 s on qlog).
Straights (contiguous runs of >= 4 s with |desired| < 0.3 m/s^2 and radius > 285 m): output and error sign changes per second (hysteresis 0.01 /
0.03), and RMS of actual lat accel, steering angle and output in 0.2-2 Hz and 2-5 Hz (Hann periodogram per run).
Interventions: every steeringPressed episode (gaps <= 1 s merged) that starts while lateral is active outside lane
changes, with the situation 1.0 .. 0.2 s before it (speed, curve side, phase, understeer / oversteer by +-0.15 m/s^2,
whether the driver pushed into the curve or against it), counted per hour of lateral-active time.
Also: effective friction threshold by speed, learned torque params and liveValid per segment, steerRatio, stiffness,
lateral delay, the lateral-related initData params (and where two drives differ), and with two routes the difference
(second minus first) with a 95% bootstrap interval over segments ("n.s." when it spans 0 or there is < 60 s).

Usage: lateral_tracking.py ROUTE [ROUTE2] [--kind qlog|rlog] [--segments 0,3-7] [--events] [--json OUT] [--workers N]
  e.g. lateral_tracking.py 00000011--c7d29db317 00000012--2aada6ce32
"""
import argparse
import json
import sys

import numpy as np

from logs import events, pmap, segment_files, seg_num, short

SPEED_EDGES_MPH = [0, 20, 40, 60, 1000]
SPEED_NAMES = ["0-20", "20-40", "40-60", "60+"]
LAT_EDGES = [0.0, 0.3, 1.0, 1e9]
LAT_NAMES = ["centre <0.3", "0.3-1.0", ">1.0"]
MPH = 0.44704
JERK_PHASE = 0.2
UNDER_EPS = 0.15
MIN_SPEED = 2.0
OVERRIDE_PAD = 1.0
SETTLE = 1.0
STRAIGHT_LAT = 0.3
STRAIGHT_CURV = 0.0035                                            # 1/m: a "straight" is also > 285 m radius (matters below ~25 mph)
LAG_MIN_S = 120.0
STRAIGHT_MIN_S = 4.0
OUT_HYST = 0.01
ERR_HYST = 0.03
BOOT = 1000
PARAM_KEYS = ["AdvancedLateralTune", "ForceAutoTune", "ForceAutoTuneOff", "UseAutoSteerDelay", "SteerDelay", "SteerFriction",
              "SteerKP", "SteerLatAccel", "SteerRatio", "SteerOffset", "SteerDelayStock", "SteerFrictionStock", "SteerKPStock",
              "SteerLatAccelStock", "SteerRatioStock", "ForceTorqueController", "LaneCentering", "NNFF", "NNFFLite", "LateralTune",
              "QOLLateral", "AlwaysOnLateral", "AlwaysOnLateralLKAS", "LatSmoothSeconds", "VASMSmoothSeconds", "PauseLateralOnSignal",
              "PauseLateralSpeed", "HumanLaneChanges", "LaneChangeSmoothing", "StingerObjectShadow", "DrivingModelName",
              "FLMActiveProfileId", "FLMTrialApplied", "FLMActiveOverrides", "GitBranch"]


def scan(job):
  """One log file -> dict of per-controlsState arrays plus the slow signals."""
  path, = job
  latest = dict(v=0.0, ang=0.0, pressed=False, tq=0.0, lb=False, rb=False, rate=0.0, lat=False, en=False, lc=0, thr=0.0, fsc=0.0,
                unw=False, lsf=0.0, otq=0.0, roll=0.0)
  rows = []; lt = []; lp = []; ld = []; cp = None; init = None
  for e in events(path):
    w = e.which(); t = e.logMonoTime
    if w == "controlsState":
      ls = e.controlsState.lateralControlState
      if ls.which() != "torqueState":
        continue
      q = ls.torqueState
      rows.append((t * 1e-9, latest["v"], latest["ang"], latest["pressed"], latest["tq"], latest["lb"] or latest["rb"], latest["rate"],
                   latest["lat"], q.active, latest["lc"] != 0, latest["en"], q.desiredLateralAccel, q.actualLateralAccel,
                   q.desiredLateralJerk, q.error, q.errorRate, q.output, q.p, q.i, q.d, q.f, q.saturated, latest["thr"],
                   latest["fsc"], latest["unw"], latest["lsf"], latest["otq"], latest["roll"]))
    elif w == "carState":
      c = e.carState
      latest.update(v=c.vEgo, ang=c.steeringAngleDeg, pressed=c.steeringPressed, tq=c.steeringTorque, lb=c.leftBlinker,
                    rb=c.rightBlinker, rate=c.steeringRateDeg)
    elif w == "carControl":
      latest["lat"] = bool(e.carControl.latActive)
    elif w == "carOutput":
      latest["otq"] = e.carOutput.actuatorsOutput.torque
    elif w == "selfdriveState":
      latest["en"] = bool(e.selfdriveState.enabled)
    elif w == "drivingModelData":
      latest["lc"] = int(str(e.drivingModelData.meta.laneChangeState) != "off")
    elif w == "starpilotLateralState":
      s = e.starpilotLateralState
      latest.update(thr=s.frictionThreshold, fsc=s.frictionScale, unw=s.unwindDetected, lsf=s.lowSpeedFactor)
    elif w == "liveParameters":
      p = e.liveParameters
      latest["roll"] = p.roll
      lp.append((t * 1e-9, p.steerRatio, p.stiffnessFactor, p.angleOffsetDeg, p.angleOffsetAverageDeg, p.roll, p.steerRatioStd, float(p.valid)))
    elif w == "liveTorqueParameters":
      p = e.liveTorqueParameters
      lt.append((t * 1e-9, p.latAccelFactorFiltered, p.latAccelFactorRaw, p.frictionCoefficientFiltered, p.frictionCoefficientRaw,
                 p.latAccelOffsetFiltered, p.latAccelOffsetRaw, float(p.liveValid), p.totalBucketPoints, float(p.useParams), p.calPerc))
    elif w == "liveDelay":
      ld.append((t * 1e-9, e.liveDelay.lateralDelay, float(e.liveDelay.status == "estimated")))
    elif w == "carParams" and cp is None:
      tq = e.carParams.lateralTuning.torque
      cp = dict(latAccelFactor=tq.latAccelFactor, friction=tq.friction, latAccelOffset=tq.latAccelOffset,
                steerActuatorDelay=e.carParams.steerActuatorDelay, steerRatio=e.carParams.steerRatio,
                steeringAngleDeadzoneDeg=tq.steeringAngleDeadzoneDeg, which=e.carParams.lateralTuning.which())
    elif w == "initData" and init is None:
      init = {p.key: (bytes(p.value).decode(errors="replace") if p.value else "") for p in e.initData.params.entries}
      init = {k: v for k, v in init.items() if k in PARAM_KEYS}
      init["_commit"] = e.initData.gitCommit[:10]
  names = ["t", "v", "ang", "pressed", "tq", "blink", "rate", "lat", "tsact", "lc", "en", "d", "a", "j", "err", "errrate", "out", "p", "i",
           "dd", "f", "sat", "thr", "fsc", "unw", "lsf", "otq", "roll"]
  arr = {n: np.array([r[k] for r in rows], dtype=float) for k, n in enumerate(names)} if rows else {n: np.zeros(0) for n in names}
  arr["seg"] = np.full(len(rows), seg_num(path), dtype=int)
  return dict(arr=arr, lt=np.array(lt).reshape(-1, 11), lp=np.array(lp).reshape(-1, 8), ld=np.array(ld).reshape(-1, 3), cp=cp, init=init,
              seg=seg_num(path))


def set_speed_bands(edges):
  global SPEED_EDGES_MPH, SPEED_NAMES
  e = [0] + [int(x) if float(x).is_integer() else x for x in edges] + [1000]
  SPEED_EDGES_MPH = e
  SPEED_NAMES = [f"{e[i]}-{e[i + 1]}" for i in range(len(e) - 2)] + [f"{e[-2]}+"]


def dilate(mask, k):
  if k <= 0 or not mask.any():
    return mask.copy()
  c = np.cumsum(np.r_[0, mask.astype(int)])
  n = len(mask); i = np.arange(n)
  return (c[np.minimum(i + k + 1, n)] - c[np.maximum(i - k, 0)]) > 0


def load_drive(route, kind, segments, workers):
  files = segment_files(route, kind, segments)
  if not files:
    sys.exit(f"no {kind} files for {route}")
  res = [r for r in pmap(scan, [(f,) for f in files], workers) if len(r["arr"]["t"])]
  res.sort(key=lambda r: r["seg"])
  arr = {k: np.concatenate([r["arr"][k] for r in res]) for k in res[0]["arr"]}
  order = np.argsort(arr["t"], kind="stable")
  arr = {k: v[order] for k, v in arr.items()}
  dt = float(np.median(np.diff(arr["t"])))
  d = dict(name=short(route), kind=kind, arr=arr, dt=dt, segs=[r["seg"] for r in res], res=res)
  d["init"] = next((r["init"] for r in res if r["init"]), {})
  d["cp"] = next((r["cp"] for r in res if r["cp"]), {})
  # learned / live params per segment
  d["lt"] = {r["seg"]: r["lt"] for r in res}; d["lp"] = {r["seg"]: r["lp"] for r in res}; d["ld"] = {r["seg"]: r["ld"] for r in res}
  build_masks(d)
  return d


def build_masks(d):
  a = d["arr"]; dt = d["dt"]; n = len(a["t"])
  gap = np.r_[True, np.diff(a["t"]) > max(5 * dt, 0.3)]          # new contiguous run starts (also across missing segments)
  run_id = np.cumsum(gap) - 1
  k = int(round(OVERRIDE_PAD / dt))
  lat = (a["lat"] > 0) & (a["tsact"] > 0)
  pressed = a["pressed"] > 0
  lc = a["lc"] > 0                                                # model laneChangeState != off (junction turns with a blinker stay in)
  rising = lat & ~np.r_[False, lat[:-1]]
  settle = np.zeros(n, bool)
  idx = np.flatnonzero(rising)
  for i in idx:
    settle[i:i + int(round(SETTLE / dt))] = True
  excl = dilate(pressed, k) | dilate(lc, k) | settle
  d["run"] = run_id; d["lat"] = lat; d["pressed"] = pressed; d["lcm"] = lc
  d["valid"] = lat & ~excl & (a["v"] >= MIN_SPEED)
  d["spd"] = np.digitize(a["v"] / MPH, SPEED_EDGES_MPH[1:-1])
  d["latb"] = np.digitize(np.abs(a["d"]), LAT_EDGES[1:-1])
  ph = np.zeros(n, int)                                           # 0 centre/none, 1 turn-in, 2 unwind, 3 steady
  big = np.abs(a["d"]) >= LAT_EDGES[1]
  dj = a["d"] * a["j"]
  ph[big] = 3
  ph[big & (dj > JERK_PHASE)] = 1
  ph[big & (dj < -JERK_PHASE)] = 2
  d["phase"] = ph
  d["side"] = np.where(a["d"] > 0, 1, -1)                         # 1 = left
  d["u"] = np.sign(a["d"]) * (a["d"] - a["a"])                    # >0 short of the plan
  d["e"] = a["d"] - a["a"]
  segs = sorted(set(d["segs"])); d["seg_index"] = np.searchsorted(segs, a["seg"]); d["nseg"] = len(segs)
  d["rate_hz"] = 1.0 / dt


# ---- statistics with segment-bootstrap -----------------------------------------------------------------------------------------

class Boot:
  """Bootstrap over segments; the same resample indices are used for both drives of a comparison."""
  def __init__(self, seed=1):
    self.rng = np.random.default_rng(seed)
    self.cache = {}

  def idx(self, nseg):
    if nseg not in self.cache:
      self.cache[nseg] = self.rng.integers(0, nseg, size=(BOOT, nseg))
    return self.cache[nseg]


def seg_sums(d, mask, vals):
  """Per-segment sums [nseg, len(vals)+1] (first column = count)."""
  si = d["seg_index"][mask]
  cols = [np.bincount(si, minlength=d["nseg"]).astype(float)]
  for v in vals:
    cols.append(np.bincount(si, weights=v[mask], minlength=d["nseg"]))
  return np.stack(cols, 1)


def metric_from_sums(s, kind):
  n = s[..., 0]
  with np.errstate(invalid="ignore", divide="ignore"):
    if kind == "under":
      return s[..., 1] / n
    if kind == "rms":
      return np.sqrt(s[..., 2] / n)
    if kind == "sat":
      return 100 * s[..., 3] / n
  raise ValueError(kind)


def cell_summary(d, mask):
  """n seconds, under, rms, sat% for a mask plus the per-segment sums for bootstrap."""
  a = d["arr"]
  centre = d["latb"] == 0
  u = np.where(centre, d["e"], d["u"])
  sums = seg_sums(d, mask, [u, d["e"] ** 2, a["sat"], (np.abs(a["out"]) >= 0.98).astype(float)])
  tot = sums.sum(0)
  n = tot[0]
  out = dict(sec=n * d["dt"], sums=sums)
  if n:
    out.update(under=tot[1] / n, rms=float(np.sqrt(tot[2] / n)), sat=100 * tot[3] / n, peg=100 * tot[4] / n)
    o = np.abs(a["out"][mask])
    out.update(p50=float(np.percentile(o, 50)), p90=float(np.percentile(o, 90)), p99=float(np.percentile(o, 99)))
  return out


def ci_diff(s1, s2, kind, boot, ref_rows=None):
  """95% interval of metric(s2) - metric(s1) over segment resamples (None if either side is empty)."""
  if s1[:, 0].sum() == 0 or s2[:, 0].sum() == 0:
    return None
  i1 = boot.idx(len(s1)); i2 = boot.idx(len(s2))
  m1 = metric_from_sums(s1[i1].sum(1), kind); m2 = metric_from_sums(s2[i2].sum(1), kind)
  dd = m2 - m1
  dd = dd[np.isfinite(dd)]
  if len(dd) < BOOT // 2:
    return None
  return float(np.percentile(dd, 2.5)), float(np.percentile(dd, 97.5))


def fmt(x, w=6, p=3):
  return f"{x:{w}.{p}f}" if x is not None and np.isfinite(x) else " " * (w - 1) + "-"


# ---- lag ------------------------------------------------------------------------------------------------------------------------

LAG_MAX_S = 1.2


def lag_sums(d, spd_band):
  """Per-segment cross-correlation sums sum d(t)*a(t+k) for k = 0..K (samples) over contiguous valid runs of >= 3 s."""
  a = d["arr"]; dt = d["dt"]
  K = int(round(LAG_MAX_S / dt)); step = max(1, K // 24)
  kneg = (int(round(0.3 / dt)) // step) * step
  ks = np.arange(-kneg, K + 1, step)
  out = np.zeros((d["nseg"], len(ks)))
  m = d["valid"] & (d["spd"] == spd_band)
  # runs of valid samples
  edges = np.flatnonzero(np.diff(np.r_[False, m, False].astype(int)))
  for s, e in zip(edges[::2], edges[1::2]):
    if (e - s) * dt < 3.0 or d["run"][s] != d["run"][e - 1]:
      continue
    ds = a["d"][s:e] - a["d"][s:e].mean(); ac = a["a"][s:e] - a["a"][s:e].mean()
    seg = d["seg_index"][s]
    for j, k in enumerate(ks):
      if k >= 0:
        out[seg, j] += np.dot(ds[:len(ds) - k], ac[k:])
      else:
        out[seg, j] += np.dot(ds[-k:], ac[:len(ac) + k])
  return ks * dt, out


def lag_from(ks, sums):
  c = sums.sum(0)
  if not np.isfinite(c).all() or np.all(c == 0):
    return None
  j = int(np.argmax(c))
  if 0 < j < len(c) - 1:
    den = c[j - 1] - 2 * c[j] + c[j + 1]
    off = 0.5 * (c[j - 1] - c[j + 1]) / den if den != 0 else 0.0
    return float(ks[j] + off * (ks[1] - ks[0]))
  return float(ks[j])


def lag_ci(ks, sums, boot):
  i = boot.idx(len(sums))
  vals = [lag_from(ks, sums[ix]) for ix in i[:300]]
  vals = [v for v in vals if v is not None]
  return (float(np.percentile(vals, 2.5)), float(np.percentile(vals, 97.5))) if len(vals) > 100 else None


# ---- straights ------------------------------------------------------------------------------------------------------------------

BANDS = [(0.2, 0.75), (0.75, 2.0), (2.0, 5.0)]


def band_var(x, dt, bands):
  n = len(x)
  t = np.arange(n)
  x = x - np.polyval(np.polyfit(t, x, 1), t)
  w = np.hanning(n)
  X = np.fft.rfft(x * w)
  psd = (np.abs(X) ** 2) / (np.sum(w ** 2) * 1.0)               # variance per bin (sums to var of windowed signal)
  psd[1:-1] *= 2
  f = np.fft.rfftfreq(n, dt)
  return [float(psd[(f >= lo) & (f < hi)].sum()) / n for lo, hi in bands]  # variance share per band (Parseval, per-sample)


def straight_runs(d):
  a = d["arr"]; dt = d["dt"]
  m = d["valid"] & (np.abs(a["d"]) < STRAIGHT_LAT) & (np.abs(a["d"]) < STRAIGHT_CURV * np.maximum(a["v"], 1.0) ** 2)
  edges = np.flatnonzero(np.diff(np.r_[False, m, False].astype(int)))
  runs = []
  for s, e in zip(edges[::2], edges[1::2]):
    if (e - s) * dt < STRAIGHT_MIN_S or d["run"][s] != d["run"][e - 1]:
      continue
    runs.append((s, e))
  return runs


def sign_changes(x, hyst):
  state = 0; c = 0
  for v in x:
    if v > hyst:
      if state == -1:
        c += 1
      state = 1
    elif v < -hyst:
      if state == 1:
        c += 1
      state = -1
  return c


def straight_stats(d):
  """[band, nseg, cols]: sec, output / error sign changes, then n*variance in the 3 BANDS for actual lat accel, angle, output, then n."""
  a = d["arr"]; dt = d["dt"]
  nb = len(SPEED_NAMES)
  # in qlog data above 5 Hz is aliased; the 2-5 Hz band is informative only at 100 Hz or as a relative figure
  sig = [a["a"], a["ang"], a["out"]]
  S = np.zeros((nb, d["nseg"], 3 + 9 + 1))
  for s, e in straight_runs(d):
    v = float(a["v"][s:e].mean() / MPH); b = int(np.digitize(v, SPEED_EDGES_MPH[1:-1]))
    seg = d["seg_index"][s]; n = e - s
    row = [n * dt, sign_changes(a["out"][s:e] - a["out"][s:e].mean() * 0, OUT_HYST), sign_changes(a["err"][s:e], ERR_HYST)]
    for x in sig:
      row += [bv * n for bv in band_var(x[s:e], dt, BANDS)]
    row += [n]
    S[b, seg] += row
  return S


def straight_metrics(S):
  """S: [nseg, cols] -> dict of per second / RMS metrics from totals."""
  t = S.sum(0)
  sec = t[0]
  if sec == 0:
    return None
  n = t[-1]
  b = lambda k: t[3 + k]                                          # k: 0-2 actual lat accel, 3-5 angle, 6-8 output
  return dict(sec=sec, out_sc=t[1] / sec, err_sc=t[2] / sec,
              a_lo=np.sqrt((b(0) + b(1)) / n), a_hi=np.sqrt(b(2) / n), ang_lo=np.sqrt((b(3) + b(4)) / n), ang_hi=np.sqrt(b(5) / n),
              out_lo=np.sqrt((b(6) + b(7)) / n), out_hi=np.sqrt(b(8) / n),
              out_slow=b(6) / max(b(6) + b(7) + b(8), 1e-12))


def straight_boot(S1, S2, key, boot):
  i1 = boot.idx(len(S1)); i2 = boot.idx(len(S2))
  def m(S, ix):
    t = S[ix].sum(1)
    sec = t[:, 0]; n = t[:, -1]
    with np.errstate(invalid="ignore", divide="ignore"):
      return {"out_sc": t[:, 1] / sec, "err_sc": t[:, 2] / sec, "a_lo": np.sqrt((t[:, 3] + t[:, 4]) / n),
              "out_lo": np.sqrt((t[:, 9] + t[:, 10]) / n), "ang_lo": np.sqrt((t[:, 6] + t[:, 7]) / n)}[key]
  if S1[:, 0].sum() < 1 or S2[:, 0].sum() < 1:
    return None
  dd = m(S2, i2) - m(S1, i1)
  dd = dd[np.isfinite(dd)]
  return (float(np.percentile(dd, 2.5)), float(np.percentile(dd, 97.5))) if len(dd) > BOOT // 2 else None


# ---- interventions --------------------------------------------------------------------------------------------------------------

def interventions(d):
  a = d["arr"]; dt = d["dt"]
  p = d["pressed"]
  gap = int(round(1.0 / dt))
  idx = np.flatnonzero(p)
  if not len(idx):
    return []
  starts = [idx[0]]; ends = []
  for i0, i1 in zip(idx[:-1], idx[1:]):
    if i1 - i0 > gap:
      ends.append(i0); starts.append(i1)
  ends.append(idx[-1])
  pre0 = max(1, int(round(1.0 / dt))); pre1 = max(1, int(round(0.2 / dt)))
  out = []
  lcd = dilate(d["lcm"], int(round(1.0 / dt)))
  for s, e in zip(starts, ends):
    if s < pre0 or not d["lat"][max(s - pre1, 0)] or lcd[s]:
      continue
    if a["t"][s] - a["t"][s - pre0] > 2.0:
      continue
    w = slice(s - pre0, s - pre1 + 1)
    v = float(np.median(a["v"][w])); dsr = float(np.median(a["d"][w])); jj = float(np.median(a["j"][w])); ac = float(np.median(a["a"][w]))
    u = np.sign(dsr) * (dsr - ac)
    centre = abs(dsr) < LAT_EDGES[1]
    ph = "centre" if centre else ("turn-in" if dsr * jj > JERK_PHASE else "unwind" if dsr * jj < -JERK_PHASE else "steady")
    err = "n/a" if centre else ("under" if u > UNDER_EPS else "over" if u < -UNDER_EPS else "ok")
    seg_tq = a["tq"][s:e + 1]
    pk = seg_tq[np.argmax(np.abs(seg_tq))]
    into = "n/a" if centre else ("into" if np.sign(pk) == np.sign(dsr) else "against")
    es = dsr - ac
    vs_err = "n/a" if abs(es) < 0.05 else ("corrects" if np.sign(pk) == np.sign(es) else "opposes")
    out.append(dict(t=float(a["t"][s]), seg=int(a["seg"][s]), v_mph=v / MPH, band=SPEED_NAMES[int(np.digitize(v / MPH, SPEED_EDGES_MPH[1:-1]))],
                    desired=dsr, actual=ac, side="left" if dsr > 0 else "right", phase=ph, err=err, into=into, tq_peak=float(pk),
                    dur=float((e - s + 1) * dt), err_signed=float(es), vs_err=vs_err, sat=float(np.mean(a["sat"][w])),
                    out_sign=float(np.sign(np.median(a["out"][w])))))
  return out


def torque_sign_check(d):
  a = d["arr"]
  m = d["pressed"] & d["lat"]
  if m.sum() < 20:
    return None
  rate = a["rate"][m]; tq = a["tq"][m]
  return float(np.corrcoef(tq, rate)[0, 1])


# ---- reporting ------------------------------------------------------------------------------------------------------------------

def header(title):
  print("\n" + title + "\n" + "-" * len(title))


def drive_summary(d):
  a = d["arr"]; dt = d["dt"]
  lat_h = d["lat"].sum() * dt / 3600
  valid_h = d["valid"].sum() * dt / 3600
  tot_h = (a["t"][-1] - a["t"][0]) / 3600
  km_lat = float((a["v"] * d["lat"]).sum() * dt / 1000)
  print(f"{d['name']} [{d['kind']}, {d['rate_hz']:.0f} Hz] segments {len(d['segs'])}  span {tot_h * 60:.1f} min  lat-active {lat_h * 60:.1f} min "
        f"({100 * lat_h / max(tot_h, 1e-9):.0f}%)  {km_lat:.1f} km  valid (no override / lane change) {valid_h * 60:.1f} min  "
        f"mean speed on lat {float((a['v'] * d['lat']).sum() / max(d['lat'].sum(), 1)) / MPH:.1f} mph")
  rows = []
  for b, nm in enumerate(SPEED_NAMES):
    m = d["valid"] & (d["spd"] == b)
    cs = [(d["valid"] & (d["spd"] == b) & (d["latb"] == k)).sum() * dt / 60 for k in range(3)]
    rows.append(f"{nm} mph: {m.sum() * dt / 60:5.1f} min (centre {cs[0]:.1f} / mid {cs[1]:.1f} / high {cs[2]:.1f})")
  print("  valid minutes by speed band: " + "; ".join(rows))
  print(f"  commit {d['init'].get('_commit', '?')}  carParams torque: " + ", ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in d["cp"].items()))


def settings_report(drives):
  header("Lateral-related settings in effect (initData params, identical in every segment of each drive)")
  keys = [k for k in PARAM_KEYS if any(k in d["init"] for d in drives)]
  w = max(len(k) for k in keys)
  print(f"{'param':{w}s}  " + "  ".join(f"{d['name']:>24s}" for d in drives))
  for k in keys:
    vals = [str(d["init"].get(k, "-")) for d in drives]
    flag = "  <-- differs" if len(set(vals)) > 1 else ""
    if k == "FLMActiveOverrides":
      vals = [("{}" if v in ("{}", "-", "") else "set (see below)") for v in vals]
    print(f"{k:{w}s}  " + "  ".join(f"{v[:24]:>24s}" for v in vals) + flag)
  for d in drives:
    ov = d["init"].get("FLMActiveOverrides", "{}")
    if ov and ov != "{}":
      o = json.loads(ov)
      print(f"\nFLMActiveOverrides {d['name']}: base friction thresholds " + json.dumps(o.get("baseFrictionThresholds", {})) +
            "\n  vehicleKnobs " + json.dumps(o.get("vehicleKnobs", {})))


def live_report(d):
  header(f"Learned / live parameters, {d['name']} (per segment mean; liveTorqueParameters, liveParameters, liveDelay)")
  print("seg  latAccel(flt/raw)   friction(flt/raw)   offset(flt)  valid%  buckets  useParams | steerRatio  stiffness  angleOff(avg)  roll_deg | latDelay")
  for s in d["segs"]:
    lt = d["lt"][s]; lp = d["lp"][s]; ld = d["ld"][s]
    if not len(lt):
      continue
    m = lt.mean(0); l = lp.mean(0) if len(lp) else np.full(8, np.nan); g = ld.mean(0) if len(ld) else np.full(3, np.nan)
    print(f"{s:3d}  {m[1]:6.3f}/{m[2]:6.3f}      {m[3]:6.3f}/{m[4]:6.3f}      {m[5]:7.3f}    {100 * m[7]:5.0f}  {m[8]:7.0f}  {m[9]:8.0f}  | "
          f"{l[1]:9.3f}  {l[2]:9.3f}  {l[4]:11.3f}   {np.degrees(l[5]):7.2f}  | {g[1]:.3f}")
  alls = np.concatenate([d["lt"][s] for s in d["segs"] if len(d["lt"][s])])
  print(f"  overall: latAccelFactor filtered {alls[:, 1].min():.3f}..{alls[:, 1].max():.3f} (raw mean {alls[:, 2].mean():.3f}, sd {alls[:, 2].std():.3f}); "
        f"friction filtered {alls[:, 3].min():.3f}..{alls[:, 3].max():.3f} (raw mean {alls[:, 4].mean():.3f}); "
        f"liveValid {100 * alls[:, 7].mean():.0f}% of samples; first -> last filtered factor {alls[0, 1]:.3f} -> {alls[-1, 1]:.3f}, "
        f"friction {alls[0, 3]:.3f} -> {alls[-1, 3]:.3f}")


def threshold_report(drives):
  header("Effective friction threshold logged by the controller (starpilotLateralState.frictionThreshold), lateral-active, |desired| < 0.3")
  print("speed band   " + "   ".join(f"{d['name']:>22s}" for d in drives))
  for b, nm in enumerate(SPEED_NAMES):
    cells = []
    for d in drives:
      m = d["valid"] & (d["spd"] == b) & (d["latb"] == 0) & (np.abs(d["arr"]["j"]) < 0.3)
      x = d["arr"]["thr"][m]
      cells.append(f"{np.median(x):.3f} (p10 {np.percentile(x, 10):.3f} p90 {np.percentile(x, 90):.3f})" if m.sum() > 20 else "-")
    print(f"{nm:>6s} mph   " + "   ".join(f"{c:>22s}" for c in cells))
  print("  by speed (m/s) median frictionThreshold, calm centre samples:")
  edges = [0, 3, 6, 9, 12, 16, 20, 25, 40]
  for lo, hi in zip(edges[:-1], edges[1:]):
    cells = []
    for d in drives:
      m = d["valid"] & (d["arr"]["v"] >= lo) & (d["arr"]["v"] < hi) & (d["latb"] == 0) & (np.abs(d["arr"]["j"]) < 0.3)
      cells.append(f"{np.median(d['arr']['thr'][m]):.3f} n={m.sum()}" if m.sum() > 20 else "-")
    print(f"    {lo:2d}-{hi:2d} m/s  " + "   ".join(f"{c:>22s}" for c in cells))


def tracking_tables(drives, boot):
  ref = drives[0]
  two = len(drives) == 2
  # ---- table 1
  header("Tracking by speed band x |desired| band (all phases, both sides). under = sign(des)*(des-act) [centre: des-act], rms, sat% (controller flag), peg% (|out| >= 0.98 = torque command pegged), |out| p50/p90/p99")
  cells = {}
  for b, sn in enumerate(SPEED_NAMES):
    for k, ln in enumerate(LAT_NAMES):
      for d in drives:
        cells[(d["name"], b, k)] = cell_summary(d, d["valid"] & (d["spd"] == b) & (d["latb"] == k))
  print(f"{'speed mph':9s} {'|des| band':12s} " + " | ".join(f"{d['name']:^47s}" for d in drives) + ("  | delta (2nd - 1st): rms [95% CI]; under [95% CI]; sat%" if two else ""))
  for b, sn in enumerate(SPEED_NAMES):
    for k, ln in enumerate(LAT_NAMES):
      parts = []
      for d in drives:
        c = cells[(d["name"], b, k)]
        parts.append(f"{c['sec']:6.0f}s u{fmt(c.get('under'))} rms{fmt(c.get('rms'))} sat{fmt(c.get('sat'), 5, 1)}% peg{fmt(c.get('peg'), 5, 1)}% o{fmt(c.get('p50'), 5, 2)}/{fmt(c.get('p90'), 5, 2)}/{fmt(c.get('p99'), 5, 2)}")
      tail = ""
      if two:
        c1 = cells[(drives[0]["name"], b, k)]; c2 = cells[(drives[1]["name"], b, k)]
        tail = "  | " + delta_text(c1, c2, boot)
      print(f"{sn:9s} {ln:12s} " + " | ".join(parts) + tail)
  # ---- table 2: phases
  header("Turn-in / unwind / steady (|desired| >= 0.3, both sides), by speed band")
  print(f"{'speed mph':9s} {'phase':8s} " + " | ".join(f"{d['name']:^47s}" for d in drives) + ("  | delta (2nd - 1st)" if two else ""))
  for b, sn in enumerate(SPEED_NAMES):
    for pi, pn in ((1, "turn-in"), (2, "unwind"), (3, "steady")):
      parts = []; cs = []
      for d in drives:
        c = cell_summary(d, d["valid"] & (d["spd"] == b) & (d["phase"] == pi)); cs.append(c)
        parts.append(f"{c['sec']:6.0f}s u{fmt(c.get('under'))} rms{fmt(c.get('rms'))} sat{fmt(c.get('sat'), 5, 1)}% peg{fmt(c.get('peg'), 5, 1)}% o{fmt(c.get('p50'), 5, 2)}/{fmt(c.get('p90'), 5, 2)}/{fmt(c.get('p99'), 5, 2)}")
      tail = ("  | " + delta_text(cs[0], cs[1], boot)) if two else ""
      print(f"{sn:9s} {pn:8s} " + " | ".join(parts) + tail)
  # ---- table 3: side x phase (all speeds, plus fast/slow)
  header("Left vs right x phase (|desired| >= 0.3), speed groups: <40 mph and >=40 mph")
  print(f"{'group':8s} {'side':5s} {'phase':8s} " + " | ".join(f"{d['name']:^47s}" for d in drives) + ("  | delta (2nd - 1st)" if two else ""))
  for gname, sel in (("<40", lambda d: d["arr"]["v"] < 40 * MPH), (">=40", lambda d: d["arr"]["v"] >= 40 * MPH)):
    for side, sn in ((1, "left"), (-1, "right")):
      for pi, pn in ((1, "turn-in"), (2, "unwind"), (3, "steady")):
        parts = []; cs = []
        for d in drives:
          c = cell_summary(d, d["valid"] & sel(d) & (d["side"] == side) & (d["phase"] == pi)); cs.append(c)
          parts.append(f"{c['sec']:6.0f}s u{fmt(c.get('under'))} rms{fmt(c.get('rms'))} sat{fmt(c.get('sat'), 5, 1)}% peg{fmt(c.get('peg'), 5, 1)}% o{fmt(c.get('p50'), 5, 2)}/{fmt(c.get('p90'), 5, 2)}/{fmt(c.get('p99'), 5, 2)}")
        tail = ("  | " + delta_text(cs[0], cs[1], boot)) if two else ""
        print(f"{gname:8s} {sn:5s} {pn:8s} " + " | ".join(parts) + tail)


def delta_text(c1, c2, boot):
  if not c1.get("sec") or not c2.get("sec") or min(c1["sec"], c2["sec"]) < 60:
    return "n.s. (<60 s)"
  out = []
  for kind in ("rms", "under"):
    dv = c2[kind] - c1[kind]
    ci = ci_diff(c1["sums"], c2["sums"], kind, boot)
    if ci is None:
      out.append(f"{kind} {dv:+.3f} [no CI]")
    else:
      sig = "" if (ci[0] <= 0 <= ci[1]) else " *"
      out.append(f"{kind} {dv:+.3f} [{ci[0]:+.3f},{ci[1]:+.3f}]{sig}")
  ds = c2["sat"] - c1["sat"]
  ci = ci_diff(c1["sums"], c2["sums"], "sat", boot)
  sig = "" if (ci is None or ci[0] <= 0 <= ci[1]) else " *"
  out.append(f"sat {ds:+.1f}pp" + sig)
  return "; ".join(out)


def pool_drives(drives):
  """Both drives as one (for the cells where the profile makes no difference and a bigger sample is wanted)."""
  p = dict(name="pooled", dt=drives[0]["dt"], kind=drives[0]["kind"])
  keys = ["valid", "spd", "latb", "phase", "side", "u", "e"]
  for k in keys:
    p[k] = np.concatenate([d[k] for d in drives])
  p["arr"] = {k: np.concatenate([d["arr"][k] for d in drives]) for k in ("sat", "out", "v", "d", "a")}
  off = 0; si = []
  for d in drives:
    si.append(d["seg_index"] + off); off += d["nseg"]
  p["seg_index"] = np.concatenate(si); p["nseg"] = off
  return p


def pooled_tables(drives):
  p = pool_drives(drives)
  header("Pooled over both drives (curves only; use when the two drives show no difference): side x phase x speed group")
  print(f"{'group':6s} {'|des|':8s} {'side':5s} {'phase':8s}   seconds  under    rms   sat%   |out| p50/p90/p99")
  groups = (("<20", lambda: p["arr"]["v"] < 20 * MPH), ("20-40", lambda: (p["arr"]["v"] >= 20 * MPH) & (p["arr"]["v"] < 40 * MPH)),
            (">=40", lambda: p["arr"]["v"] >= 40 * MPH))
  for gname, sel in groups:
    for lname, lsel in (("0.3-1.0", p["latb"] == 1), (">1.0", p["latb"] == 2)):
      for side, sn in ((1, "left"), (-1, "right")):
        for pi, pn in ((1, "turn-in"), (2, "unwind"), (3, "steady")):
          c = cell_summary(p, p["valid"] & sel() & lsel & (p["side"] == side) & (p["phase"] == pi))
          if c["sec"] < 5:
            continue
          print(f"{gname:6s} {lname:8s} {sn:5s} {pn:8s} {c['sec']:7.0f}  {fmt(c['under'])} {fmt(c['rms'])} {fmt(c['sat'], 5, 1)}  peg{fmt(c['peg'], 5, 1)}%  "
                f"{fmt(c['p50'], 5, 2)}/{fmt(c['p90'], 5, 2)}/{fmt(c['p99'], 5, 2)}")
  print("  rows with < 5 s of data are omitted; a cell needs a few tens of seconds before its under / rms is more than an anecdote.")


def hard_curves(drives, thr=1.0):
  """Episodes with |desired| > thr while lateral is active, override NOT excluded (these are the ones the tables above mostly lose)."""
  header(f"Hard-curve episodes (|desired| > {thr} m/s^2 while lateral is active; overrides and their +-1 s are NOT excluded here)")
  for d in drives:
    a = d["arr"]; dt = d["dt"]
    m = d["lat"] & (np.abs(a["d"]) > thr)
    idx = np.flatnonzero(m)
    n_ep = 0
    print(f"{d['name']}:")
    if not len(idx):
      print("  none")
      continue
    gap = int(round(1.0 / dt)); st = [idx[0]]; en = []
    for i0, i1 in zip(idx[:-1], idx[1:]):
      if i1 - i0 > gap:
        en.append(i0); st.append(i1)
    en.append(idx[-1])
    t0 = a["t"][0]
    print("  seg    t(s)   mph  side  max|des| max|act| under(max) under@press  press+s  |out|>=.98  sat%  jerk0  len   (under = sign(des)*(des-act); @press = median over the 0.5 s before the first override)")
    for s_, e_ in zip(st, en):
      sl = slice(max(s_ - gap, 0), min(e_ + gap, len(a["t"])) + 1)
      sgn = np.sign(a["d"][s_])
      u_all = sgn * (a["d"][s_:e_ + 1] - a["a"][s_:e_ + 1])
      pr = np.flatnonzero(d["pressed"][sl])
      if len(pr):
        p0 = sl.start + pr[0]
        w = slice(max(p0 - int(round(0.5 / dt)), 0), max(p0, 1))
        upress = float(np.median(sgn * (a["d"][w] - a["a"][w]))) if w.stop > w.start else float("nan")
        pst = f"{(a['t'][p0] - a['t'][s_]):+7.1f}"
        upt = f"{upress:10.2f}"
      else:
        pst = "     no"; upt = "         -"
      maxed = 100 * (np.abs(a["out"][s_:e_ + 1]) >= 0.98).mean()
      print(f"  {int(a['seg'][s_]):3d} {a['t'][s_] - t0:7.1f} {a['v'][s_] / MPH:5.1f}  {'left ' if sgn > 0 else 'right'} {np.abs(a['d'][s_:e_ + 1]).max():7.2f} "
            f"{np.abs(a['a'][s_:e_ + 1]).max():7.2f} {u_all.max():9.2f} {upt} {pst} {maxed:9.0f}% {100 * a['sat'][s_:e_ + 1].mean():5.0f} {a['j'][s_]:+6.2f} "
            f"{(e_ - s_ + 1) * dt:4.1f}s")


def lag_table(drives, boot):
  header("Lag of actual lat accel behind desired (s), by speed band (valid contiguous runs >= 3 s, mean-removed)")
  print(f"{'speed mph':9s} " + "  ".join(f"{d['name']:>28s}" for d in drives))
  res = {}
  for b, sn in enumerate(SPEED_NAMES):
    cells = []
    for d in drives:
      ks, sums = lag_sums(d, b)
      res[(d["name"], b)] = (ks, sums)
      enough = (d["valid"] & (d["spd"] == b)).sum() * d["dt"] >= LAG_MIN_S
      lg = lag_from(ks, sums) if sums.sum() != 0 and enough else None
      ci = lag_ci(ks, sums, boot) if lg is not None else None
      cells.append("-" if lg is None else f"{lg:.2f}" + (f" [{ci[0]:.2f},{ci[1]:.2f}]" if ci else ""))
    print(f"{sn:9s} " + "  ".join(f"{c:>28s}" for c in cells))
  print(f"  (needs >= {LAG_MIN_S:.0f} s valid in the band; the correlation peak is flat, so treat differences under ~0.1 s as noise. The controller already "
        "leads the request by liveDelay lateralDelay, so this is the residual lag.)")


def straights_table(drives, boot):
  header("Straights (|desired| < 0.3 and radius > 285 m for >= 4 s): sign changes per second; RMS in 0.2-2 Hz / 2-5 Hz of actual lat accel, steering angle (deg), output; slow = share of output power below 0.75 Hz")
  S = {d["name"]: straight_stats(d) for d in drives}
  two = len(drives) == 2
  print(f"{'speed mph':9s} " + " | ".join(f"{d['name']:^82s}" for d in drives) + ("  | delta (2nd - 1st): out sc/s, err sc/s, a_lo, ang_lo [95% CI]" if two else ""))
  hz = drives[0]["rate_hz"]
  for b, sn in enumerate(SPEED_NAMES):
    parts = []; ms = []
    for d in drives:
      m = straight_metrics(S[d["name"]][b]); ms.append(m)
      if m is None:
        parts.append(f"{'-':^82s}")
      else:
        parts.append(f"{m['sec']:5.0f}s outSC{m['out_sc']:5.2f} errSC{m['err_sc']:5.2f} a {m['a_lo']:.3f}/{m['a_hi']:.3f} ang {m['ang_lo']:.2f}/{m['ang_hi']:.2f} out {m['out_lo']:.3f}/{m['out_hi']:.3f} slow{100 * m['out_slow']:3.0f}%")
    tail = ""
    if two and ms[0] and ms[1] and min(ms[0]["sec"], ms[1]["sec"]) >= 60:
      bits = []
      for key, lab in (("out_sc", "outSC"), ("err_sc", "errSC"), ("a_lo", "a"), ("ang_lo", "ang")):
        dv = ms[1][key] - ms[0][key]
        ci = straight_boot(S[drives[0]["name"]][b], S[drives[1]["name"]][b], key, boot)
        sig = "" if ci is None or ci[0] <= 0 <= ci[1] else "*"
        bits.append(f"{lab}{dv:+.3f}" + (f"[{ci[0]:+.3f},{ci[1]:+.3f}]{sig}" if ci else ""))
      tail = "  | " + " ".join(bits)
    elif two:
      tail = "  | n.s. (<60 s)"
    print(f"{sn:9s} " + " | ".join(parts) + tail)
  tot = [straight_metrics(S[d["name"]].sum(0)) for d in drives]
  print("all speeds " + " | ".join((f"{m['sec']:5.0f}s outSC{m['out_sc']:5.2f} errSC{m['err_sc']:5.2f} a {m['a_lo']:.3f}/{m['a_hi']:.3f} ang {m['ang_lo']:.2f}/{m['ang_hi']:.2f} out {m['out_lo']:.3f}/{m['out_hi']:.3f} slow{100 * m['out_slow']:3.0f}%" if m else '-') for m in tot))
  if hz < 50:
    print("  note: qlog is 10 Hz, so the 2-5 Hz band is aliased/limited and only comparable between qlog drives; sign changes use hysteresis "
          f"{OUT_HYST} (output) and {ERR_HYST} m/s^2 (error).")


def intervention_tables(drives, list_events, verbose_sign=True):
  header("Driver interventions (steeringPressed episodes that start while lateral is active, outside lane changes)")
  evs = {d["name"]: interventions(d) for d in drives}
  for d in drives:
    e = evs[d["name"]]
    lat_h = d["lat"].sum() * d["dt"] / 3600
    km = float((d["arr"]["v"] * d["lat"]).sum() * d["dt"] / 1000)
    sc = torque_sign_check(d)
    print(f"{d['name']}: {len(e)} interventions in {lat_h * 60:.1f} min lateral-active ({len(e) / max(lat_h, 1e-9):.1f}/h, {len(e) / max(km, 1e-9):.2f}/km)"
          + (f"; corr(steeringTorque, steeringRate) while pressed = {sc:+.2f} (positive = torque sign matches angle sign)" if sc is not None else ""))
  def tab(title, key, order):
    print(f"\n  by {title}:")
    print("    " + f"{'':10s}" + "".join(f"{d['name']:>26s}" for d in drives))
    for o in order:
      cells = []
      for d in drives:
        e = evs[d["name"]]; n = sum(1 for x in e if x[key] == o)
        cells.append(f"{n:3d} ({100 * n / max(len(e), 1):3.0f}%)")
      print("    " + f"{o:10s}" + "".join(f"{c:>26s}" for c in cells))
  tab("speed band (mph)", "band", SPEED_NAMES)
  tab("curve side", "side", ["left", "right"])
  tab("phase before it", "phase", ["centre", "turn-in", "unwind", "steady"])
  tab("tracking error before it (curves only)", "err", ["under", "ok", "over", "n/a"])
  tab("driver pushed ... the curve (steeringTorque > 0 assumed = left)", "into", ["into", "against", "n/a"])
  tab("driver pushed ... the tracking error (needs |des-act| >= 0.05)", "vs_err", ["corrects", "opposes", "n/a"])
  print("\n  rate per lateral-active hour and per km by speed band (Poisson 95% interval on the hourly rate), rate ratio second/first (per hour | per km):")
  for b, sn in enumerate(SPEED_NAMES):
    cells = []; rs = []
    for d in drives:
      e = [x for x in evs[d["name"]] if x["band"] == sn]
      m = d["lat"] & (d["spd"] == b)
      h = m.sum() * d["dt"] / 3600; km = float((d["arr"]["v"] * m).sum() * d["dt"] / 1000)
      n = len(e)
      if h < 0.02:
        cells.append("-"); rs.append(None); continue
      lo = max(n - 1.96 * np.sqrt(max(n, 1)), 0) / h; hi = (n + 1.96 * np.sqrt(max(n, 1))) / h
      cells.append(f"{n:3d} in {h * 60:5.1f} min/{km:4.1f} km = {n / h:5.1f}/h [{lo:4.1f},{hi:5.1f}] {n / max(km, 1e-9):4.2f}/km"); rs.append((n, h, km))
    tail = ""
    if len(drives) == 2 and rs[0] and rs[1] and rs[0][0] > 0 and rs[1][0] > 0:
      se = np.sqrt(1 / rs[0][0] + 1 / rs[1][0])
      parts = []
      for k, lab in ((1, "/h"), (2, "/km")):
        r = (rs[1][0] / rs[1][k]) / (rs[0][0] / rs[0][k])
        lo, hi = r * np.exp(-1.96 * se), r * np.exp(1.96 * se)
        parts.append(f"{r:.2f} [{lo:.2f},{hi:.2f}]" + ("" if lo <= 1 <= hi else " *"))
      tail = "   ratio " + " | ".join(parts)
    print(f"    {sn:>6s}: " + " | ".join(f"{c:>62s}" for c in cells) + tail)
  tot = [(len(evs[d["name"]]), d["lat"].sum() * d["dt"] / 3600, float((d["arr"]["v"] * d["lat"]).sum() * d["dt"] / 1000)) for d in drives]
  if len(drives) == 2 and tot[0][0] and tot[1][0]:
    se = np.sqrt(1 / tot[0][0] + 1 / tot[1][0])
    parts = []
    for k, lab in ((1, "/h"), (2, "/km")):
      r = (tot[1][0] / tot[1][k]) / (tot[0][0] / tot[0][k])
      parts.append(f"{r:.2f} [{r * np.exp(-1.96 * se):.2f},{r * np.exp(1.96 * se):.2f}]")
    print("    all speeds: rate ratio " + " | ".join(parts))
  cen = {d["name"]: [x for x in evs[d["name"]] if x["phase"] == "centre"] for d in drives}
  for d in drives:
    c = cen[d["name"]]
    if c:
      calm = sum(1 for x in c if abs(x["err_signed"]) < 0.1)
      print(f"  {d['name']}: {len(c)} of {len(evs[d['name']])} interventions happen with |desired| < 0.3 m/s^2, and {calm} of those with |desired - actual| < 0.1 "
            "(the car was following the plan; the driver wanted a different line or touched the wheel)")
  if list_events:
    for d in drives:
      print(f"\n  events {d['name']} (time = seconds in route; positive lat accel = left):")
      t0 = d["arr"]["t"][0]
      for x in evs[d["name"]]:
        print(f"    seg {x['seg']:2d} t={x['t'] - t0:7.1f}s {x['v_mph']:5.1f} mph des {x['desired']:+.2f} act {x['actual']:+.2f} {x['phase']:7s} "
              f"{x['err']:5s} driver {x['into']:7s} tq {x['tq_peak']:+.2f} dur {x['dur']:.1f}s sat {x['sat']:.1f}")
  return evs


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("routes", nargs="+", help="one or two routes; with two the second is compared against the first")
  ap.add_argument("--kind", default="qlog", choices=["qlog", "rlog"])
  ap.add_argument("--segments", default=None, help="restrict every route to these segments, e.g. 0,3-7")
  ap.add_argument("--events", action="store_true", help="list every intervention")
  ap.add_argument("--json", default=None, help="write the tables as JSON")
  ap.add_argument("--workers", type=int, default=4)
  ap.add_argument("--speed-edges", default="20,40,60", help="mph band edges, default 20,40,60 (bands 0-20, 20-40, 40-60, 60+)")
  args = ap.parse_args()
  set_speed_bands([float(x) for x in args.speed_edges.split(",")])
  segs = None
  if args.segments:
    segs = set()
    for tok in args.segments.split(","):
      lo, _, hi = tok.partition("-")
      segs.update(range(int(lo), int(hi or lo) + 1))
  drives = [load_drive(r, args.kind, segs, args.workers) for r in args.routes[:2]]
  boot = Boot()
  header("Drives")
  for d in drives:
    drive_summary(d)
  settings_report(drives)
  threshold_report(drives)
  tracking_tables(drives, boot)
  lag_table(drives, boot)
  if len(drives) == 2:
    pooled_tables(drives)
  hard_curves(drives)
  straights_table(drives, boot)
  evs = intervention_tables(drives, args.events)
  for d in drives:
    live_report(d)
  if args.json:
    json.dump({d["name"]: dict(interventions=evs[d["name"]], init=d["init"], carParams=d["cp"]) for d in drives}, open(args.json, "w"), indent=1, default=float)


if __name__ == "__main__":
  main()
