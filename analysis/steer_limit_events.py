#!/usr/bin/env python3
"""Where did openpilot's steering torque limits bind on the Stinger, and which one: rate-up, rate-down, cap, driver or controller saturation?

Torque chain (opendbc hyundai/carcontroller.py, KIA_STINGER_2022: STEER_MAX 384, STEER_DELTA_UP 3, STEER_DELTA_DOWN 7 per 10 ms frame,
STEER_DRIVER_ALLOWANCE 50, MULTIPLIER 2, FACTOR 1):
  new_torque   = round(carControl.actuators.torque * 384)          the controller's wish (the torque PID output is already clipped to +-1)
  apply_torque = apply_driver_steer_torque_limits(new_torque, last, carState.steeringTorque)   driver clip, then rate clip
  carOutput.actuatorsOutput.torqueOutputCan = apply_torque         == LKAS11 CR_Lkas_StrToqReq that is sent (verified against can src 128)
  carOutput.actuatorsOutput.torque          = apply_torque / 384
  apply_torque = 0 when not latActive; no other min-speed zeroing exists for the Stinger (the >90 deg fault avoidance only toggles ActToi).
carControl.actuators.torque is the unclipped wish (the only thing missing is anything >1.0, which the controller never produces, so
|requested| > 384 never happens and CAP is "sent at the +-384 ceiling").

Definitions (per 100 Hz frame, lateral active = carControl.latActive, v >= MIN_V; steeringPressed frames dropped except for DRIVER):
  requested  carControl.actuators.torque * 384 (rounded).      sent  carOutput torqueOutputCan.        last = sent of the previous frame.
  CAP        |requested| > 384 or |sent| >= 380.
  RATE_UP3   magnitude-increasing step limit binds: after the driver clip the wish is more than STEER_DELTA_UP (3) beyond the allowed step
             (sent rose by exactly 3 in magnitude and the wish is still more than 3 further out). Deficit = |wish - sent|. Most such frames
             are tiny: a deficit of 4-10 units is 0.03-0.07 m/s^2.
  RATE_UP    the same with deficit >= MAT = 40 units (~0.29 m/s^2 at 2.75 m/s^2 per unit output, the BEHIND threshold): the material version
             used for episodes and the alert attribution (the key question reports RATE_UP3 as primary and this as a sensitivity).  RATE_DOWN3 / RATE_DOWN: same for the release step (7).
  DRIVER3 / DRIVER  the driver-torque limiter clipped the wish (driver torque opposing beyond the allowance), any amount / >= MAT units.
             Includes steeringPressed frames.
  SAT        controller |output| >= 0.98 (controlsState torqueState) or its `saturated` flag.
  BEHIND     sign(desired) * (desired - actual lateral accel) > 0.3 m/s^2 (the car under-delivers); tracking error = |desired - actual|.
  The reconstruction of the limiter is checked against the logged sent value (match rate printed in section 0).
Episodes: frames of any class merged when closer than 0.5 s. Ranked by integral of tracking error (|desired - actual| * dt, steeringPressed frames excluded from the integral).
Alerts: selfdriveState / starpilotSelfdriveState alertType rising edges (audible = alertSound != none) and onroadEvents /
starpilotOnroadEvents name rising edges. Alert predicate of StarPilot's selfdrived (steerSaturated): latActive and no steeringPressed in
the last 2 s and |desired a| > 1.0 (modelV2 desiredCurvature * v^2) and undershooting (|desired|/(1e-3+|actual|) > 1.2, actual =
controlsState.curvature * v^2) and (saturated flag or |output| > 0.99). There is no speed gate and no timer. It is replicated as PRED.
qlogs (10 Hz) carry no modelV2, so PRED there uses torqueState desired/actual lateral accel, and the rate limit cannot be measured.
Sign note: torqueState lateral accels carry the sign opposite to the output; only magnitudes and sign(desired)*(desired-actual) are used.

Usage: steer_limit_events.py [--rlog ROUTES] [--qlog ROUTES] [--cache FILE] [--workers N]   (markdown on stdout)
"""
import argparse
import os
import pickle
from collections import Counter, defaultdict

import numpy as np

from logs import add_starpilot_path, events, pmap, segment_files, seg_num, short

RLOGS = ["0000000d--5e0d1d244b", "0000000e--5ebf4636a1", "00000015--2b757ccb00", "0000001d--2ead59d2dd", "00000028--297af64c9c",
         "00000029--b3c2c13819", "00000034--5f8605e608"]
QLOGS = ["00000016--1fe83bd745", "00000017--0c40138e36", "00000011--c7d29db317", "00000012--2aada6ce32", "00000013--6677db1144",
         "00000014--62bceb20db"]
STEER_MAX, UP, DOWN = 384, 3, 7
ALLOW, MULT, FACT = 50, 2, 1
CAP_T = 380
SAT_T = 0.98
MIN_V = 1.0
MPH = 0.44704
BANDS = [0, 10, 20, 40, 1000]
BN = ["0-10", "10-20", "20-40", "40+"]
MERGE_S = 0.5
BEHIND_T = 0.3
MAT = 40                      # material deficit: 40 units * (2.75 m/s^2 / 384) ~ 0.29 m/s^2, the BEHIND threshold
COLS = ["t", "v", "lat", "pressed", "drv", "ang", "rate", "req", "sent", "des", "act", "out", "sat", "csCurv", "mdlCurv", "ccCurv"]
C = {n: i for i, n in enumerate(COLS)}
STEER_WORDS = ("saturat", "steertemp", "steertimelimit", "steerunavail", "goat")


def scan(path):
  """One segment: frame rows at each carOutput (COLS), audible/other alert rising edges, first event time."""
  s = dict(v=0.0, lat=0.0, pressed=0.0, drv=0.0, ang=0.0, rate=0.0, req=np.nan, des=np.nan, act=np.nan, out=np.nan, sat=0.0,
           csCurv=np.nan, mdlCurv=np.nan, ccCurv=np.nan, lat_new=0.0, req_new=np.nan)
  rows, alerts = [], []
  last_type = {"sd": "", "sp": ""}
  last_ev = {"ev": set(), "spev": set()}
  t0 = None
  for e in sorted(events(path), key=lambda e: e.logMonoTime):
    w = e.which()
    t = e.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "carState":
      c = e.carState
      s["v"], s["pressed"], s["drv"], s["ang"], s["rate"] = c.vEgo, float(c.steeringPressed), c.steeringTorque, c.steeringAngleDeg, c.steeringRateDeg
    elif w == "carControl":
      # card applies the carControl of the previous controlsd step in the same loop that logs carOutput, so the wish behind a carOutput
      # row is the carControl logged *before* the latest one (verified: 99.98% exact reconstruction with this pairing, 37% without)
      s["lat"], s["req"] = s["lat_new"], s["req_new"]
      s["lat_new"], s["req_new"], s["ccCurv"] = float(e.carControl.latActive), e.carControl.actuators.torque * STEER_MAX, e.carControl.actuators.curvature
    elif w == "controlsState":
      s["csCurv"] = e.controlsState.curvature
      ls = e.controlsState.lateralControlState
      if ls.which() == "torqueState":
        q = ls.torqueState
        s["des"], s["act"], s["out"], s["sat"] = q.desiredLateralAccel, q.actualLateralAccel, q.output, float(q.saturated)
      else:
        s["des"] = s["act"] = s["out"] = np.nan
    elif w == "modelV2":
      s["mdlCurv"] = e.modelV2.action.desiredCurvature
    elif w == "carOutput":
      a = e.carOutput.actuatorsOutput
      rows.append((t, s["v"], s["lat"], s["pressed"], s["drv"], s["ang"], s["rate"], s["req"], float(a.torqueOutputCan), s["des"], s["act"],
                   s["out"], s["sat"], s["csCurv"], s["mdlCurv"], s["ccCurv"]))
    elif w in ("selfdriveState", "starpilotSelfdriveState"):
      d = getattr(e, w)
      k = "sd" if w == "selfdriveState" else "sp"
      ty = d.alertType
      if ty != last_type[k] and ty:
        alerts.append((t, k, ty, str(d.alertSound), str(d.alertStatus), d.alertText1, d.alertText2))
      last_type[k] = ty
    elif w in ("onroadEvents", "starpilotOnroadEvents"):
      k = "ev" if w == "onroadEvents" else "spev"
      lst = e.onroadEvents if w == "onroadEvents" else e.starpilotOnroadEvents.events
      names = {str(x.name) for x in lst}
      for n in names - last_ev[k]:
        alerts.append((t, k, n, "", "", "", ""))
      last_ev[k] = names
  return np.array(rows, dtype=np.float64).reshape(-1, len(COLS)), alerts, t0


def verify_can(path):
  """Compare carOutput torqueOutputCan with the LKAS11 CR_Lkas_StrToqReq echo (can src 128): match count at lag 0 / 1 / 2 can events."""
  add_starpilot_path('/home/openpilotstuff/wt/stinger-combined')
  from opendbc.can import CANParser
  snt = CANParser("hyundai_kia_generic", [("LKAS11", 128)], 128)
  hist, n, m, anyn = [], 0, [0] * 6, 0
  mx = 0
  for e in sorted(events(path), key=lambda e: e.logMonoTime):
    w = e.which()
    if w == "carOutput":
      hist.append(float(e.carOutput.actuatorsOutput.torqueOutputCan))
      mx = max(mx, abs(hist[-1]))
    elif w == "can":
      snt.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
      v = snt.vl["LKAS11"]["CR_Lkas_StrToqReq"]
      if len(hist) > 20 and snt.vl["LKAS11"]["CF_Lkas_ActToi"] == 1:
        n += 1
        for k in range(6):
          m[k] += int(hist[-1 - k] == v)
        anyn += int(v in hist[-20:])
  return n, m, mx, anyn


def driver_limits(drv, steer_max=STEER_MAX):
  hi = np.maximum(np.minimum(steer_max, steer_max + (ALLOW + drv * FACT) * MULT), 0)
  lo = np.minimum(np.maximum(-steer_max, -steer_max + (-ALLOW + drv * FACT) * MULT), 0)
  return lo, hi


def reconstruct(req, last, drv):
  """Vectorised apply_driver_steer_torque_limits. Returns (predicted, clipped by cap only, after driver clip, rate lo, rate hi)."""
  lo, hi = driver_limits(drv)
  capc = np.clip(req, -STEER_MAX, STEER_MAX)
  d1 = np.clip(capc, lo, hi)
  pos = last > 0
  rlo = np.where(pos, np.maximum(last - DOWN, -UP), last - UP)
  rhi = np.where(pos, last + UP, np.minimum(last + DOWN, UP))
  pred = np.clip(d1, rlo, rhi)
  return np.round(pred), capc, d1, rlo, rhi


def flags(F, dt):
  """Per-frame class flags for one segment array F (rows of COLS)."""
  n = len(F)
  g = lambda k: F[:, C[k]]
  v, lat, pressed, drv, req, sent, out, sat = g("v"), g("lat") == 1, g("pressed") == 1, g("drv"), g("req"), g("sent"), g("out"), g("sat")
  last = np.concatenate([[0], sent[:-1]])
  contig = np.concatenate([[False], np.diff(g("t")) < max(3 * dt, 0.03)])
  reqr = np.round(np.nan_to_num(req))
  pred, capc, d1, rlo, rhi = reconstruct(reqr, last, drv)
  act_ok = lat & (v >= MIN_V)
  base = act_ok & ~pressed
  lo, hi = driver_limits(drv)
  pos = last > 0
  up = np.where(pos, d1 - rhi, rlo - d1)             # amount the wish exceeds the allowed step, increasing magnitude
  down = np.where(pos, rlo - d1, d1 - rhi)           # same for decreasing magnitude / sign flip
  fl = {}
  fl["base"], fl["act"] = base, act_ok
  fl["CAP"] = base & ((np.abs(reqr) > STEER_MAX) | (np.abs(sent) >= CAP_T))
  fl["RATE_UP3"] = base & contig & (up > UP)
  fl["RATE_DOWN3"] = base & contig & (down > DOWN)
  fl["DRIVER3"] = act_ok & ((capc > hi) | (capc < lo))
  fl["RATE_UP"] = base & contig & (up >= MAT)
  fl["RATE_DOWN"] = base & contig & (down >= MAT)
  fl["DRIVER"] = act_ok & ((capc - hi >= MAT) | (lo - capc >= MAT))
  fl["SAT"] = base & ((np.abs(out) >= SAT_T) | (sat == 1))
  fl["match"], fl["match_n"] = act_ok & contig & (pred == sent), act_ok & contig
  fl["deficit"] = np.where(act_ok, np.abs(capc - sent), 0.0)
  fl["deficit_signed"] = np.where(act_ok, np.abs(capc) - np.abs(sent), 0.0)
  fl["rate_any_up"] = base & contig & (up > 0)
  des, act = g("des"), g("act")
  fl["err"] = np.abs(des - act)
  fl["err_np"] = np.where(pressed, 0.0, np.nan_to_num(fl["err"]))      # tracking error with the driver's own frames removed
  fl["behind"] = base & (np.sign(des) * (des - act) > BEHIND_T)
  # StarPilot steerSaturated predicate
  mdl = g("mdlCurv")
  vv = np.maximum(v, 0.3)
  if np.isfinite(mdl).any():
    des_a = mdl * vv ** 2
  else:
    des_a = des
  act_a = g("csCurv") * vv ** 2 if np.isfinite(mdl).any() else act
  und = np.abs(des_a) / np.abs(1e-3 + act_a) > 1.2
  recent = np.zeros(n, bool)
  pidx = np.where(pressed)[0]
  if len(pidx):
    last_p = np.maximum.accumulate(np.where(pressed, np.arange(n), -10**9))
    recent = (np.arange(n) - last_p) * dt < 2.0
  fl["PRED"] = (lat) & ~recent & (np.abs(des_a) > 1.0) & und & ((sat == 1) | (np.abs(out) > 0.99))
  return fl


def band(v):
  return np.digitize(v / MPH, BANDS[1:-1])


def table(head, rows):
  print("| " + " | ".join(head) + " |")
  print("|" + "|".join(["---"] * len(head)) + "|")
  for r in rows:
    print("| " + " | ".join(str(c) for c in r) + " |")
  print()


def pc(x, d=1):
  return f"{x * 100:.{d}f}%"


class Route:
  """All frames of a route concatenated with per-frame flags, route time and the alert list (times shifted to the same clock)."""
  def __init__(self, name, files, res, dt):
    self.name, self.dt = name, dt
    Fs, fls, rts, alerts = [], [], [], []
    self.nseg = len(files)
    for f, (F, al, t0) in zip(files, res):
      if len(F) < 20:
        continue
      Fs.append(F)
      fl = flags(F, dt)
      fls.append(fl)
      rts.append(seg_num(f) * 60 + (F[:, C["t"]] - t0))
      alerts += [(seg_num(f) * 60 + (a[0] - t0),) + a[1:] for a in al]
    self.F = np.concatenate(Fs) if Fs else np.zeros((0, len(COLS)))
    self.fl = {k: np.concatenate([x[k] for x in fls]) for k in fls[0]} if fls else {}
    self.rt = np.concatenate(rts) if rts else np.zeros(0)
    self.alerts = sorted(alerts)
    self.rt_alert_mono = None


def episodes(R, classes):
  """Merge flagged frames within MERGE_S. Returns list of dicts."""
  fl, F, rt, dt = R.fl, R.F, R.rt, R.dt
  anyf = np.zeros(len(F), bool)
  for k in classes:
    anyf |= fl[k]
  idx = np.where(anyf)[0]
  eps = []
  if not len(idx):
    return eps
  start = 0
  for j in range(1, len(idx) + 1):
    if j == len(idx) or rt[idx[j]] - rt[idx[j - 1]] > MERGE_S:
      i0, i1 = idx[start], idx[j - 1]
      sl = slice(i0, i1 + 1)
      eps.append(dict(route=R.name, i0=i0, i1=i1, t0=rt[i0], t1=rt[i1], dur=(rt[i1] - rt[i0]) + dt, sl=sl))
      start = j
  for ep in eps:
    sl = ep["sl"]
    v = F[sl, C["v"]]
    des, act = F[sl, C["des"]], F[sl, C["act"]]
    err = fl["err"][sl]
    ep["v"] = np.nanmean(v) / MPH
    ep["err_int"] = float(np.nansum(fl["err_np"][sl]) * dt)
    ep["press_s"] = float((F[sl, C["pressed"]] == 1).sum() * dt)
    ep["err_mean"], ep["err_peak"] = float(np.nanmean(err)), float(np.nanmax(err)) if np.isfinite(err).any() else np.nan
    k = int(np.nanargmax(np.abs(np.nan_to_num(des)))) if np.isfinite(des).any() else 0
    ep["des_pk"], ep["act_at_pk"] = abs(des[k]), abs(act[k])
    cc = np.abs(F[sl, C["ccCurv"]])
    ep["curv"] = float(np.nanmax(cc)) if np.isfinite(cc).any() else np.nan
    ep["radius"] = 1.0 / ep["curv"] if ep["curv"] and ep["curv"] > 1e-5 else np.inf
    ep["ang"], ep["rate"] = float(np.nanmax(np.abs(F[sl, C["ang"]]))), float(np.nanmax(np.abs(F[sl, C["rate"]])))
    ep["cls"] = {k2: float(fl[k2][sl].sum() * dt) for k2 in classes if fl[k2][sl].any()}
    first = {k2: int(np.argmax(fl[k2][sl])) for k2 in ep["cls"]}
    ep["order"] = sorted(ep["cls"], key=lambda k2: first[k2])
    ep["deficit"] = float(np.max(fl["deficit_signed"][sl] * (fl["RATE_UP"][sl] | fl["RATE_DOWN"][sl] | fl["CAP"][sl] | fl["SAT"][sl])))
    ep["pressed_near"] = bool(((F[:, C["pressed"]] == 1) & (rt >= ep["t0"]) & (rt <= ep["t1"] + 1.0)).any())
    ep["alerts"] = [a for a in R.alerts if ep["t0"] - 0.0 <= a[0] <= ep["t1"] + 1.0 and alert_audible(a)]
  return eps


def alert_audible(a):
  return a[1] in ("sd", "sp") and a[3] not in ("none", "")


def alert_steerish(a):
  return any(w in a[2].lower() for w in STEER_WORDS)


def band_summary(routes, kind, has_rate):
  eps_all = []
  for R in routes:
    eps_all += episodes(R, ["CAP", "RATE_UP", "RATE_DOWN", "DRIVER", "SAT"] if has_rate else ["CAP", "DRIVER", "SAT"])
  dt = routes[0].dt
  rows = []
  for b in range(4):
    tl, tb, td = 0.0, 0.0, 0.0
    cnt = Counter()
    for R in routes:
      bb = band(R.F[:, C["v"]])
      base, act = R.fl["base"] & (bb == b), R.fl["act"] & (bb == b)
      tl += base.sum() * dt
      td += act.sum() * dt
      for k in ["CAP", "RATE_UP", "RATE_DOWN", "SAT", "RATE_UP3", "RATE_DOWN3"]:
        cnt[k] += (R.fl[k] & (bb == b)).sum() * dt
      cnt["DRIVER"] += (R.fl["DRIVER"] & (bb == b)).sum() * dt
      cnt["DRIVER3"] += (R.fl["DRIVER3"] & (bb == b)).sum() * dt
    ne = sum(1 for e in eps_all if band(np.array([e["v"] * MPH]))[0] == b)
    rows.append([BN[b], f"{tl / 60:.1f}", ne] + [pc(cnt[k] / tl, 2) if tl else "-" for k in ["CAP", "RATE_UP", "RATE_UP3", "RATE_DOWN", "RATE_DOWN3", "SAT"]] +
                [pc(cnt["DRIVER"] / td, 2) if td else "-", pc(cnt["DRIVER3"] / td, 2) if td else "-"])
  return rows, eps_all


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--rlog", nargs="*", default=RLOGS)
  ap.add_argument("--qlog", nargs="*", default=QLOGS)
  ap.add_argument("--cache", default=None)
  ap.add_argument("--workers", type=int, default=10)
  a = ap.parse_args()
  add_starpilot_path('/home/openpilotstuff/wt/stinger-combined')

  cache = {}
  if a.cache and os.path.exists(a.cache):
    cache = pickle.load(open(a.cache, "rb"))
  def load(route, kind):
    key = (route, kind)
    if key not in cache:
      fs = segment_files(route, kind)
      cache[key] = (fs, pmap(scan, fs, a.workers)) if fs else ([], [])
      if a.cache:
        pickle.dump(cache, open(a.cache, "wb"))
    return cache[key]

  rl = []
  for r in a.rlog:
    fs, res = load(r, "rlog")
    if fs:
      rl.append(Route(short(r), fs, res, 0.01))
  ql = []
  for r in a.qlog:
    fs, res = load(r, "qlog")
    if fs:
      ql.append(Route(short(r), fs, res, 0.1))

  print("# Steering torque limit events (Kia Stinger 2022, STEER_MAX 384, +3 up / -7 down per 10 ms)\n")
  print("Definitions, the torque chain and usage are in the docstring of analysis/steer_limit_events.py.\n")

  # ---------------------------------------------------------------- 0 validation
  print("## 0. Signals and validation\n")
  allF = np.concatenate([R.F for R in rl])
  fl = {k: np.concatenate([R.fl[k] for R in rl]) for k in rl[0].fl}
  act = fl["act"]
  print(f"- rlog routes: {len(rl)}, {sum(R.nseg for R in rl)} segments, {len(allF)} frames at 100 Hz; lateral-active (v >= {MIN_V} m/s): "
        f"{act.sum() / 100 / 3600:.2f} h, of which steeringPressed {pc(((allF[:, C['pressed']] == 1) & act).sum() / act.sum())}.")
  print(f"- requested = carControl.actuators.torque * 384: max |requested| in lateral-active frames = "
        f"{np.nanmax(np.abs(allF[act, C['req']])):.1f} (controller output is clipped at +-1, so a request above 384 is never seen); "
        f"max |sent| = {np.nanmax(np.abs(allF[act, C['sent']])):.0f}.")
  print("- carOutput.actuatorsOutput.torqueOutputCan exists and holds the post-limit apply_torque; carControl.actuators.torque is the "
        "pre-limit wish (the carOutput row k answers the carControl logged one row earlier; the script pairs them that way). Both are logged at 100 Hz.")
  m, mn = fl["match"].sum(), fl["match_n"].sum()
  print(f"- Limiter reconstruction (driver clip then rate clip from last sent, using carState.steeringTorque): reproduces the sent value "
        f"exactly in {pc(m / mn, 3)} of {mn} lateral-active frames.")
  mism = fl["match_n"] & ~fl["match"]
  if mism.any():
    d = np.abs(np.round(np.clip(np.nan_to_num(allF[mism, C["req"]]), -384, 384)) - allF[mism, C["sent"]])
    print(f"  Mismatching frames: {int(mism.sum())}.")
  try:
    fsl = [segment_files("00000015--2b757ccb00", "rlog")[20], segment_files("0000001d--2ead59d2dd", "rlog")[3]]
    vr = [verify_can(f) for f in fsl]
    n = sum(x[0] for x in vr)
    print(f"- LKAS11 echo (can src 128, ActToi = 1) vs carOutput torqueOutputCan, 2 sample segments, {n} frames: equal to the carOutput value k rows back (k = 0..5; the can echo is sampled at an arbitrary instant): "
          + ", ".join(f"{k}: {pc(sum(x[1][k] for x in vr) / n)}" for k in range(6)) +
          f"; equal to any of the last 20 carOutput rows: {pc(sum(x[3] for x in vr) / n)} (max |sent| {max(x[2] for x in vr):.0f}). "
          "Weak check only: a can event batches several 10 ms frames and the parser keeps the last one, so exact timing alignment is not "
          "recoverable here; carOutput.torqueOutputCan is the value the controller passes to create_lkas11, so it is the sent torque.")
  except Exception as ex:
    print(f"- LKAS11 echo check failed: {ex}")
  print(f"- At any excess (> 0) the rate limiter is binding in {pc((fl['rate_any_up'] & fl['base']).sum() / fl['base'].sum(), 2)} of base frames; RATE_UP3 "
        f"class counts frames where the wish is more than one full step ({UP}) beyond what the step allows; RATE_UP needs a deficit >= {MAT} units.\n")

  # ---------------------------------------------------------------- 1 summary by band
  print("## 1. Summary by speed band (rlogs, 100 Hz)\n")
  print("Episodes use the material classes (CAP, RATE_UP, RATE_DOWN, DRIVER at deficit >= 40 units, SAT). Time shares are of lateral-active, unpressed time in the band (DRIVER: of lateral-active time including pressed). "
        "Classes overlap (SAT is nearly always inside RATE_UP/CAP).\n")
  rows, eps_all = band_summary(rl, "rlog", True)
  table(["band mph", "lat-active min", "episodes", "CAP", "RATE_UP (>=40)", "RATE_UP3 (>3)", "RATE_DOWN (>=40)", "RATE_DOWN3 (>7)", "SAT",
         "DRIVER (>=40)", "DRIVER3 (any)"], rows)
  print("Lateral-active minutes and episode counts per route:\n")
  rows = []
  for R in rl:
    n_ep = sum(1 for e in eps_all if e["route"] == R.name)
    rows.append([R.name, R.nseg, f"{R.fl['base'].sum() / 6000:.1f}", n_ep])
  table(["route", "segments", "lat-active min (unpressed)", "episodes"], rows)

  # episodes with alert / driver intervention: binding class before
  print("### Episodes with an audible alert or driver intervention: what bound just before\n")
  print("Binding class = the classes seen in the episode's first 1 s (or, for the alert, the 1 s before the alert).\n")
  # classify
  def label_window(R, i0, i1):
    f = R.fl
    sl = slice(max(0, i0), i1 + 1)
    ru, ca, sa, rd, dr = f["RATE_UP"][sl].any(), f["CAP"][sl].any(), f["SAT"][sl].any(), f["RATE_DOWN"][sl].any(), f["DRIVER"][sl].any()
    if ru and ca:
      return "rate+cap"
    if ru:
      return "rate-up"
    if ca:
      return "cap"
    if sa:
      return "saturation-only"
    if rd:
      return "rate-down"
    if dr:
      return "driver-limit"
    return "none"
  rbyname = {R.name: R for R in rl}
  rows_b = defaultdict(lambda: Counter())
  for e in eps_all:
    R = rbyname[e["route"]]
    if not (e["alerts"] or e["pressed_near"]):
      continue
    kind = ("alert+press" if e["alerts"] and e["pressed_near"] else "alert" if e["alerts"] else "press")
    j1 = min(len(R.F) - 1, e["i0"] + 100)
    rows_b[BN[band(np.array([e["v"] * MPH]))[0]]][(kind, label_window(R, e["i0"], j1))] += 1
  rows = []
  for b in BN:
    c = rows_b[b]
    for (kind, lab), n in sorted(c.items()):
      rows.append([b, kind, lab, n])
  table(["band", "episode had", "binding in first 1 s", "episodes"], rows)
  tot = Counter()
  for e in eps_all:
    tot["all"] += 1
    tot["alert"] += bool(e["alerts"])
    tot["press"] += e["pressed_near"]
  print(f"Total episodes {tot['all']}; with an audible alert during or within 1 s after: {tot['alert']}; with steeringPressed during/within 1 s after: "
        f"{tot['press']}.\n")

  # ---------------------------------------------------------------- 2 key question
  print("## 2. Tight low-speed turns: why does the car fall behind?\n")
  print("Frames: lateral active, unpressed, v < 20 mph, and (|desired lat accel| > 1.5 m/s^2 or desired radius < 50 m, i.e. |curvature| > 0.02). "
        "Behind = sign(desired)*(desired - actual) > 0.3 m/s^2. Categories are exclusive, in this priority: "
        "RATE_UP+SAT (wish at the +-1 ceiling and the ramp holding it back), RATE_UP only (wish below ceiling), CAP (sent >= 380 without "
        "a rate-up bind), SAT-only (|out| >= 0.98 but sent < 380 and no rate bind), RATE_DOWN, neither (controller not asking for more, "
        "and no limit binding).\n")
  def key_table(sub_lo, sub_hi, label, RU="RATE_UP"):
    cats = Counter()
    n_tight = n_behind = 0
    tight_t = 0.0
    outs = defaultdict(list)
    defs = defaultdict(list)
    recent_cats = Counter()
    for R in rl:
      F, f = R.F, R.fl
      v = F[:, C["v"]] / MPH
      curv = np.abs(F[:, C["ccCurv"]])
      tight = f["base"] & (v >= sub_lo) & (v < sub_hi) & ((np.abs(F[:, C["des"]]) > 1.5) | (curv > 0.02))
      beh = tight & f["behind"]
      n_tight += tight.sum()
      n_behind += beh.sum()
      # recent (previous 0.5 s) any limit flag
      lim = (f[RU] | f["CAP"] | f["SAT"]).astype(np.int32)
      cs = np.concatenate([[0], np.cumsum(lim)])
      idx = np.arange(len(lim))
      recent = (cs[idx + 1] - cs[np.maximum(0, idx - 50)]) > 0
      ru, sa, ca, rd = f[RU], f["SAT"], f["CAP"], f[RU.replace("UP", "DOWN")]
      cat = np.full(len(F), "neither", dtype=object)
      cat[rd] = "RATE_DOWN"
      cat[sa & ~ru & ~ca] = "SAT-only"
      cat[ca & ~ru] = "CAP"
      cat[ru & ~sa] = "RATE_UP only"
      cat[ru & sa] = "RATE_UP+SAT"
      for k in set(cat[beh]):
        cats[k] += int((cat[beh] == k).sum())
        outs[k].append(np.abs(F[beh & (cat == k), C["out"]]))
        defs[k].append(f["deficit_signed"][beh & (cat == k)])
      rc = np.where(beh & recent & (cat == "neither"))[0]
      recent_cats["neither now, limit flag within previous 0.5 s"] += len(rc)
    dt = 0.01
    print(f"**{label}**: tight-turn time {n_tight * dt / 60:.1f} min; behind (> {BEHIND_T}) {n_behind * dt / 60:.2f} min = "
          f"{pc(n_behind / max(n_tight, 1))} of tight-turn time.\n")
    order = ["RATE_UP+SAT", "RATE_UP only", "CAP", "SAT-only", "RATE_DOWN", "neither"]
    rows = []
    for k in order:
      o = np.concatenate(outs[k]) if outs[k] else np.zeros(0)
      rows.append([k, f"{cats[k] * dt:.1f}", pc(cats[k] / max(n_behind, 1)), f"{np.median(o):.2f}" if len(o) else "-",
                   f"{np.median(np.concatenate(defs[k])):.0f}" if defs[k] else "-"])
    table(["binding", "behind seconds", "share of behind time", "median |output|", "median |wish - sent| (units)"], rows)
    print(f"Of the 'neither' behind frames, {recent_cats['neither now, limit flag within previous 0.5 s']} frames "
          f"({pc(recent_cats['neither now, limit flag within previous 0.5 s'] / max(cats['neither'], 1))}) had a RATE_UP/CAP/SAT flag within the previous 0.5 s "
          f"(a tail of an episode that just ended).\n")
  print("Primary: RATE_UP = RATE_UP3 as specified (any wish more than 3 units beyond the step).\n")
  key_table(0, 20, "0-20 mph", "RATE_UP3")
  key_table(0, 10, "0-10 mph", "RATE_UP3")
  key_table(10, 20, "10-20 mph", "RATE_UP3")
  print("Sensitivity: same 0-20 mph table counting RATE_UP only when the deficit is >= 40 units (~0.29 m/s^2); the small-deficit tail of "
        "each ramp moves into SAT-only:\n")
  key_table(0, 20, "0-20 mph, material (>= 40 units) rate limit", "RATE_UP")

  # relation of output level when behind
  print("Behind frames in tight turns, controller |output| distribution (0-20 mph):\n")
  vals = []
  sentv = []
  for R in rl:
    F, f = R.F, R.fl
    v = F[:, C["v"]] / MPH
    curv = np.abs(F[:, C["ccCurv"]])
    tight = f["base"] & (v < 20) & ((np.abs(F[:, C["des"]]) > 1.5) | (curv > 0.02)) & f["behind"]
    vals.append(np.abs(F[tight, C["out"]]))
    sentv.append(np.abs(F[tight, C["sent"]]))
  vals, sentv = np.concatenate(vals), np.concatenate(sentv)
  table(["|output| bin", "share of behind frames"], [[f"{lo:.2f}-{hi:.2f}", pc(((vals >= lo) & (vals < hi)).mean())]
                                                    for lo, hi in ((0, .25), (.25, .5), (.5, .75), (.75, .9), (.9, .98), (.98, 1.01))])
  table(["|sent| bin", "share of behind frames"], [[f"{lo}-{hi}", pc(((sentv >= lo) & (sentv < hi)).mean())]
                                                    for lo, hi in ((0, 96), (96, 192), (192, 288), (288, 380), (380, 385))])

  # ---------------------------------------------------------------- 3 episodes
  print("## 3. Top 30 episodes by tracking error x duration (integral of |desired - actual| dt, m/s^2 * s)\n")
  print("Route time = 60 s * segment + time since the first event of the segment (approximate, a second or two of offset). v, desired/actual are "
        "episode mean speed and the |desired|/|actual| at the peak-desired frame. Radius = 1/max |desired curvature| (carControl.actuators.curvature). "
        "Class column lists classes in order of first occurrence with seconds active.\n")
  def row(e):
    cl = " > ".join(f"{k} {e['cls'][k]:.2f}s" for k in e["order"])
    al = ", ".join(sorted({f"{x[2]}@{x[0] - e['t0']:+.1f}" for x in e["alerts"]})) or "-"
    return [e["route"], f"{int(e['t0'] // 60)}:{e['t0'] % 60:04.1f}", f"{e['dur']:.1f}", f"{e['v']:.0f}", f"{e['des_pk']:.2f}/{e['act_at_pk']:.2f}",
            f"{e['err_peak']:.2f}", f"{e['err_int']:.2f}", f"{e['radius']:.0f}" if np.isfinite(e["radius"]) else "-",
            f"{e['ang']:.0f}/{e['rate']:.0f}", cl, f"{e['deficit']:.0f}", al, f"{e['press_s']:.1f}" if e["press_s"] else ("near" if e["pressed_near"] else "-")]
  head = ["route", "time m:s", "dur s", "mph", "des/act m/s^2", "peak err", "err*dur", "radius m", "max angle/rate deg", "classes",
          "peak deficit", "audible alert (offset s)", "pressed s"]
  top = sorted(eps_all, key=lambda e: -e["err_int"])[:30]
  table(head, [row(e) for e in top])
  print("`pressed s` = seconds of steeringPressed inside the episode; `near` = pressed only within 1 s after it.\n")
  print("### Top 30 episodes with no steeringPressed frame inside the episode (pure limit episodes, best for replay)\n")
  clean = [e for e in eps_all if e["press_s"] == 0]
  table(head, [row(e) for e in sorted(clean, key=lambda e: -e["err_int"])[:30]])
  print("### Every episode with a steering-related audible alert (steerSaturated, goat*Saturated, steerTempUnavailable*, steerTimeLimit)\n")
  se = [e for e in eps_all if any(alert_steerish(x) for x in e["alerts"])]
  print(f"{len(se)} episodes.\n")
  table(head, [row(e) for e in sorted(se, key=lambda e: (e["route"], e["t0"]))])

  # ---------------------------------------------------------------- 4 alerts
  print("## 4. Alerts\n")
  print("All selfdriveState / starpilotSelfdriveState alert rising edges and onroadEvents / starpilotOnroadEvents names across the rlog routes "
        "(a name listed under 'audible' has alertSound != none on the display message).\n")
  cnt = Counter()
  for R in rl:
    for x in R.alerts:
      cnt[(x[1], x[2], x[3])] += 1
  rows = [[k[0], k[1], k[2] or "-", n] for k, n in sorted(cnt.items(), key=lambda kv: -kv[1]) if k[0] in ("sd", "sp") and k[2] not in ("none", "")]
  print("Audible alerts (selfdriveState `sd`, starpilotSelfdriveState `sp`):\n")
  table(["source", "alertType", "alertSound", "count"], rows)
  rows = [[k[0], k[1], n] for k, n in sorted(cnt.items(), key=lambda kv: -kv[1]) if k[0] in ("ev", "spev") and any(w in k[1].lower() for w in STEER_WORDS)]
  print("Steering-related event names (onroadEvents `ev`, starpilotOnroadEvents `spev`):\n")
  table(["source", "event", "count"], rows)

  # steerSaturated alert: what preceded it
  print("### What precedes each steerSaturated / steering alert (rlogs)\n")
  rows, labs, labs_sat, ru3_s = [], Counter(), Counter(), []
  for R in rl:
    for x in R.alerts:
      if x[1] == "sd" and alert_steerish(x):
        i = int(np.searchsorted(R.rt, x[0]))
        if i >= len(R.F):
          continue
        lab = label_window(R, i - 100, i)
        labs[(BN[band(R.F[i:i + 1, C["v"]])[0] if False else int(band(np.array([R.F[i, C['v']]]))[0])], lab)] += 1
        sl = slice(max(0, i - 100), i + 1)
        if "saturat" in x[2].lower():
          labs_sat[lab] += 1
          ru3_s.append((R.fl["RATE_UP3"][sl].sum() * 0.01, R.fl["CAP"][sl].sum() * 0.01, R.fl["SAT"][sl].sum() * 0.01))
        rows.append([R.name, f"{int(x[0] // 60)}:{x[0] % 60:04.1f}", x[2], x[3], f"{R.F[i, C['v']] / MPH:.0f}", f"{np.nanmax(np.abs(R.F[sl, C['des']])):.2f}",
                     f"{np.nanmax(np.abs(R.F[sl, C['out']])):.2f}", f"{np.nanmax(np.abs(R.F[sl, C['sent']])):.0f}", lab,
                     f"{R.fl['RATE_UP'][sl].sum() * 0.01:.2f}", f"{R.fl['RATE_UP3'][sl].sum() * 0.01:.2f}", f"{R.fl['CAP'][sl].sum() * 0.01:.2f}", f"{R.fl['SAT'][sl].sum() * 0.01:.2f}"])
  print(f"{len(rows)} steering alerts. Binding class in the 1 s before, by speed band:\n")
  table(["band", "binding class", "alerts"], [[k[0], k[1], n] for k, n in sorted(labs.items())])
  if ru3_s:
    q = np.array(ru3_s)
    print(f"steerSaturated only ({int(sum(labs_sat.values()))} alerts), binding class in the 1 s before: "
          + ", ".join(f"{k} {n}" for k, n in labs_sat.most_common()) + f". Median seconds in that 1 s window: RATE_UP3 {np.median(q[:, 0]):.2f}, "
          f"CAP {np.median(q[:, 1]):.2f}, SAT {np.median(q[:, 2]):.2f}.\n")
  table(["route", "time", "alert", "sound", "mph", "max |des| 1 s before", "max |out|", "max |sent|", "binding", "RATE_UP s", "RATE_UP3 s", "CAP s", "SAT s"], rows)

  # predicate replication
  print("### The StarPilot alert predicate (PRED) vs actual alerts\n")
  pr = fl["PRED"]
  print(f"PRED frames (lat active, no press in 2 s, |desired a| > 1, undershooting x1.2, |out| > 0.99 or saturated flag): {int(pr.sum())} "
        f"({pr.sum() / 100:.0f} s). Alerts fired: {sum(1 for R in rl for x in R.alerts if x[1] == 'sd' and 'steerSaturated' in x[2])} steerSaturated.\n")
  rows = []
  v_all = allF[:, C["v"]] / MPH
  for b in range(4):
    mb = band(allF[:, C["v"]]) == b
    pp = pr & mb
    rows.append([BN[b], int(pp.sum()), f"{pp.sum() / 100:.1f}", pc((fl["RATE_UP"] & pp).sum() / max(pp.sum(), 1)),
                 pc((fl["CAP"] & pp).sum() / max(pp.sum(), 1)), pc((fl["SAT"] & pp).sum() / max(pp.sum(), 1))])
  table(["band mph", "PRED frames", "seconds", "also RATE_UP", "also CAP", "also SAT"], rows)

  # ---------------------------------------------------------------- 5 qlog
  print("## 5. qlog drives (10 Hz): cap, saturation, alerts\n")
  print("**Rate limiting cannot be measured at 10 Hz** (the +3/-7 per 10 ms step is invisible when only every 10th frame is logged); only the "
        "cap (sent >= 380), controller saturation and alerts are reported. The qlog drives have no modelV2, so PRED uses torqueState "
        "desired/actual lateral accel. qlog carOutput/carControl are decimated samples, not every frame.\n")
  rows = []
  for R in ql:
    f = R.fl
    dt = R.dt
    base = f["base"]
    nb = max(base.sum(), 1)
    alerts_s = [x for x in R.alerts if x[1] == "sd" and alert_audible(x)]
    st = [x for x in R.alerts if x[1] == "sd" and alert_steerish(x)]
    rows.append([R.name, R.nseg, f"{base.sum() * dt / 60:.1f}", pc((f["CAP"]).sum() / nb, 2), pc((f["SAT"]).sum() / nb, 2),
                 int(f["PRED"].sum()), len(alerts_s), len(st),
                 ", ".join(f"{k}x{n}" for k, n in Counter(x[2] for x in st).most_common(3)) or "-"])
  table(["route", "segs", "lat-active min", "% CAP (sent >= 380)", "% SAT", "PRED samples", "audible alerts", "steering alerts", "steering alert types"], rows)
  rows = []
  for R in ql:
    f = R.fl
    bb = band(R.F[:, C["v"]])
    for b in range(4):
      m = f["base"] & (bb == b)
      if m.sum() == 0:
        continue
      rows.append([R.name, BN[b], f"{m.sum() * R.dt / 60:.1f}", pc((f["CAP"] & m).sum() / m.sum(), 2), pc((f["SAT"] & m).sum() / m.sum(), 2),
                   f"{np.nanmax(np.abs(R.F[m, C['sent']])):.0f}"])
  print("By speed band:\n")
  table(["route", "band mph", "lat-active min", "% CAP", "% SAT", "max |sent|"], rows)
  print("Steering alerts in qlog drives (time, type, mph, desired accel, |out|, |sent| at the nearest sample, tracking error):\n")
  rows = []
  for R in ql:
    for x in R.alerts:
      if x[1] == "sd" and alert_steerish(x):
        i = min(int(np.searchsorted(R.rt, x[0])), len(R.F) - 1)
        sl = slice(max(0, i - 10), i + 1)
        rows.append([R.name, f"{int(x[0] // 60)}:{x[0] % 60:04.1f}", x[2], x[3], f"{R.F[i, C['v']] / MPH:.0f}",
                     f"{np.nanmax(np.abs(R.F[sl, C['des']])):.2f}", f"{np.nanmax(np.abs(R.F[sl, C['out']])):.2f}",
                     f"{np.nanmax(np.abs(R.F[sl, C['sent']])):.0f}", f"{np.nanmax(R.fl['err'][sl]):.2f}"])
  table(["route", "time", "alert", "sound", "mph", "max |des| 1 s before", "max |out|", "max |sent|", "peak err"], rows)
  print("Audible alerts in qlog drives:\n")
  cnt = Counter()
  for R in ql:
    for x in R.alerts:
      if x[1] in ("sd", "sp") and alert_audible(x):
        cnt[(R.name, x[1], x[2])] += 1
  table(["route", "source", "alertType", "count"], [[k[0], k[1], k[2], n] for k, n in sorted(cnt.items())])


if __name__ == "__main__":
  main()
