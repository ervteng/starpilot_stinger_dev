#!/usr/bin/env python3
"""How much EARLIER would the Stinger's steering torque request have to start in tight low-speed turns, so that the unchanged panda ramp limit
(+3 up / -7 down per 10 ms, cap 384) reaches the needed torque in time, and does the model plan know that early?

Analysis only: nothing here changes StarPilot or the panda safety limits. It extends steer_limit_events.py (same rlogs, same loading, same limiter
reconstruction, same `flags`), so read that docstring first.

Signals (100 Hz, rows at each carOutput; carOutput row k pairs with carControl row k-1, see steer_limit_events.scan):
  requested  x = round(carControl.actuators.torque * 384)       sent = carOutput.actuatorsOutput.torqueOutputCan
  des / act  controlsState torqueState desiredLateralAccel (the setpoint) / actualLateralAccel (steering-angle based), m/s^2
  a_ff       carControl.actuators.curvature * v^2 = the desired curvature handed to the torque controller (feed-forward input)
  plan       modelV2.orientationRate.z / velocity.x at T_IDXS (0..10 s): plan curvature kappa(t, tau), a_plan(t, tau) = kappa * v_ego^2 ('rate');
             'chord' = circle through the car (tangent to its heading) and modelV2.position at tau (the probe controlsd TURN_LEAD uses)
  lag        liveDelay.lateralDelay (modeld and controlsd use it, plus 0.1 s LAT_SMOOTH_SECONDS)
Everything is expressed in "turn space" (multiplied by the sign of the turn so the turn is positive); the sign relations between the signals are
measured from the data and printed in section 0.

Definitions:
  Tight turn frame   lateral active, 1 m/s <= v < 20 mph and (|des| > 1.5 m/s^2 or |a_ff curvature| > 0.02 1/m); as steer_limit_events section 2.
  Turn-in (lobe)     tight frames merged within 0.5 s form an episode; inside it des is cut into lobes of constant sign (a lobe ends when sign*des < 0.3 for
                     0.3 s or the sign flips). A lobe with a tight frame and peak |des| >= 0.5 is one turn-in. i_c = last frame before the lobe peak with
                     turn*des < 0.3 (des "starts rising" = the next frame, t_c); lobe end = des back below 0.3. Turn-ins with a peak request < 100 units are dropped.
  Timeline           t_a feed-forward input crosses 0.3 | t_r0 start of the final rise of the request (last frame below 10% of its peak) | t_rp request reaches
                     98% of its peak (<= 384) | t_sp sent reaches 98% of that peak | t_err first turn*(des - act) > 0.3. Lag = t_sp - t_rp. Ramp needed = peak / 3 per 10 ms.
  Counterfactual     the logged limiter is reconstructed as a recursion (driver clip with the logged carState.steeringTorque, then +3 / -7 from the previous output),
                     started from the logged sent 1.5 s before t_c; the unshifted run reproduces the logged sent (section 0). Two ways to advance the request by Delta:
                     pure shift x'(t) = x(t + Delta)      (as asked; advances releases and reversals too)
                     rise-only x'(t) = max of x over [t, t + Delta]   (the build-up comes early, the release does not)
  Required advance   smallest Delta in {0.1 .. 1.5 s} for which sent' stays within 19 units (5% of 384) of the torque the request definitely needs: the suffix-minimum
                     envelope of the driver-clipped request over [i_c, t_rp + 0.3 s] (immune to dips and one-frame spikes of the noisy request). 0 = the unshifted
                     limiter already meets it; inf = not met within 1.5 s. 'strict' = every frame of the driver-clipped request instead of the envelope.
                     Caveat: x contains the controller's own feedback to the lag, so it is an upper bound on what the turn-in needs.
  Behind removal     original BEHIND frames (turn*(des - act) > 0.3, tight, unpressed, < 20 mph) inside [t_c, lobe end] that no longer are behind, with
                     A  request matching: sent was > 19 units short of the driver-clipped request (the rate limit, not cap or driver) and sent' is within 19
                     B  static gain with delay: act' = act + g (sent' - sent)(t - d), d = median liveDelay, g fitted on the same turn-ins (section 0)
                     B2 the same with g = 2.26 m/s^2 per full torque (steer_torque_limit_results.md, measured at v > 8 m/s)
                     C  response shift (optimistic bound): act'(t) = act(t + s), s = min(Delta, how much earlier sent' reaches 98% of the peak request than sent did);
                        assumes the wheel angle keeps integrating the torque (wheel rate is 100-280 deg/s under a pegged request at 2-9 m/s)
                     No model can credit the 384 cap: frames where sent already equals the request are 'cap-bound' and never removed.
  Alerts             the 21 steerSaturated alerts (selfdrived.py L769-783: |desired| > 1, undershoot > 1.2x, output > 0.99, no press in 2 s, no timer, alert lasts 2 s).
                     PRED frames in [alert - 0.1, alert + 2.0 s] are re-evaluated: A as above; B / B2 / C recompute the undershoot with controlsState.curvature * v^2
                     moved by the model, the 'output at max' condition stays as logged (the controller is assumed to remain saturated). Alert removed = no PRED frame survives.
  Knowability        tau0 = median(lag) + 0.1 + 0.075 = modeld's lat_action_t, so a_plan(tau0) is today's feed-forward input. Plan crossing = start of the last run, going
                     back from the des peak, with turn*a_plan >= 0.3 m/s^2. Lead = t_r0 - plan crossing (positive = plan earlier than the request). 'Realized advance(e)' = crossing of
                     a_plan(tau0) minus crossing of a_plan(tau0 + e). 'any tau' = earliest crossing over tau in {tau0, 0.5, 1, 1.5, 2, 3, 4, 5 s} (rate, and rate or chord).
                     Predictor check: a_plan(t, tau0 + e) against a_ff(t + e) in the turn-in windows.
  Side effects       (a) early unwind / S-bend counter-torque: frames where sent' is > 40 units below the request after the peak, or pushes > 40 units against the turn after the lobe
                     (S-bend = an opposite lobe with peak > 1 within 3 s), newly caused by the advance; (b) gentle non-tight curves (0.3 < |des| < 1.5): share of frames where
                     des(t + Delta) differs from des(t) by more than X; (c) tracking error on non-tight frames: shift model act'(t) = act(t + e) (perfect plan, upper bound of earliness) and
                     plan model act'(t) = act(t) + [a_plan(t - d, tau0 + e) - a_plan(t - d, tau0)] (what the plan says).

Usage: steer_ramp_advance.py [--rlog ROUTES] [--cache FILE] [--workers N] [--max-seg N]   (markdown on stdout)
  Loads 222 rlog segments in under a minute with 8 workers; --cache stores the parsed arrays.
"""
import argparse
import os
import pickle
from collections import defaultdict

import numpy as np

import steer_limit_events as sle
from steer_limit_events import C, COLS, MPH, RLOGS, STEER_MAX, UP, DOWN, Route, flags, table
from logs import add_starpilot_path, events, pmap, segment_files, short

DT = 0.01
DELTAS = [0.0] + [round(0.1 * k, 1) for k in range(1, 16)]       # seconds; 0.0 is the limiter check
DK = np.array([int(round(d / DT)) for d in DELTAS])
TOL = 19.0                      # 5% of 384
MAT = 40.0
THR_A = 0.3                     # m/s^2: des start-rising / plan sighting threshold
EXTS = [0.2, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0]
SIGHT_TAUS = [0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0]
TIGHT_A, TIGHT_K = 1.5, 0.02
MIN_PEAK_REQ = 100.0
G2 = 2.26 / 384            # m/s^2 per torque unit measured at v > 8 m/s (steer_torque_limit_results.md section 3)
COLS2 = COLS + ["lag"]
C2 = {n: i for i, n in enumerate(COLS2)}
APPLY = [(0.3, "0.3"), (0.5, "0.5"), (0.8, "0.8"), (1.0, "1.0")]


# ------------------------------------------------------------------ loading
def scan2(path):
  """steer_limit_events.scan plus liveDelay.lateralDelay per row and the modelV2 plan (orientationRate.z, velocity.x) with the row index it applies from."""
  s = dict(v=0.0, lat=0.0, pressed=0.0, drv=0.0, ang=0.0, rate=0.0, req=np.nan, des=np.nan, act=np.nan, out=np.nan, sat=0.0,
           csCurv=np.nan, mdlCurv=np.nan, ccCurv=np.nan, lat_new=0.0, req_new=np.nan, lag=np.nan)
  rows, alerts = [], []
  mf, Z, V, PX, PY = [], [], [], [], []
  T = None
  last_type = ""
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
      m = e.modelV2
      s["mdlCurv"] = m.action.desiredCurvature
      z, vx = list(m.orientationRate.z), list(m.velocity.x)
      px, py = list(m.position.x), list(m.position.y)
      if len(z) == 33 and len(vx) == 33 and len(px) == 33 and len(py) == 33:
        if T is None:
          T = np.array(list(m.position.t), dtype=np.float64)
        mf.append(len(rows))
        Z.append(z)
        V.append(vx)
        PX.append(px)
        PY.append(py)
    elif w == "liveDelay":
      s["lag"] = e.liveDelay.lateralDelay
    elif w == "carOutput":
      a = e.carOutput.actuatorsOutput
      rows.append((t, s["v"], s["lat"], s["pressed"], s["drv"], s["ang"], s["rate"], s["req"], float(a.torqueOutputCan), s["des"], s["act"],
                   s["out"], s["sat"], s["csCurv"], s["mdlCurv"], s["ccCurv"], s["lag"]))
    elif w == "selfdriveState":
      ty = e.selfdriveState.alertType
      if ty != last_type and ty:
        alerts.append((t, "sd", ty, str(e.selfdriveState.alertSound), "", "", ""))
      last_type = ty
  F = np.array(rows, dtype=np.float64).reshape(-1, len(COLS2))
  return F, alerts, t0, np.array(mf, dtype=np.int64), np.array(Z, dtype=np.float32).reshape(-1, 33), np.array(V, dtype=np.float32).reshape(-1, 33), T, \
         np.array(PX, dtype=np.float32).reshape(-1, 33), np.array(PY, dtype=np.float32).reshape(-1, 33)


class R2(Route):
  """steer_limit_events.Route plus plan arrays, speed, continuity ids and turn-space helpers."""
  def __init__(self, name, files, res):
    super().__init__(name, files, [(r[0], r[1], r[2]) for r in res], DT)
    mfs, Zs, Vs, Xs, Ys, off = [], [], [], [], [], 0
    self.T = None
    for r in res:
      F = r[0]
      if len(F) < 20:
        continue
      mfs.append(r[3] + off)
      Zs.append(r[4])
      Vs.append(r[5])
      Xs.append(r[7])
      Ys.append(r[8])
      if self.T is None and r[6] is not None:
        self.T = r[6]
      off += len(F)
    self.mf = np.concatenate(mfs) if mfs else np.zeros(0, np.int64)
    self.Z = np.concatenate(Zs) if Zs else np.zeros((0, 33), np.float32)
    self.V = np.concatenate(Vs) if Vs else np.zeros((0, 33), np.float32)
    n = len(self.F)
    g = lambda k: self.F[:, C2[k]]
    self.n, self.v = n, g("v")
    self.des, self.act, self.req, self.sent = g("des"), g("act"), g("req"), g("sent")
    self.cc, self.mdl, self.cs, self.lag = g("ccCurv"), g("mdlCurv"), g("csCurv"), g("lag")
    self.pressed = g("pressed") == 1
    self.lat = g("lat") == 1
    brk = np.concatenate([[0], (np.diff(self.rt) > 0.03).astype(int)])
    self.seg = np.cumsum(brk)
    self.mi = np.searchsorted(self.mf, np.arange(n), side="right") - 1
    self.X = np.concatenate(Xs) if Xs else np.zeros((0, 33), np.float32)
    self.Y = np.concatenate(Ys) if Ys else np.zeros((0, 33), np.float32)
    self.kap = self.Z / np.maximum(self.V, 1.0)
    self.chord = 2 * self.Y / np.maximum(self.X ** 2 + self.Y ** 2, 1.0)    # circle through the car tangent to its heading and the plan point at tau
    self._ap = {}

  def a_plan(self, tau, kind="rate"):
    """Plan lateral accel at look-ahead tau seconds for every frame (curvature of the latest plan at tau times v_ego^2); NaN before the first plan.
    kind 'rate': orientationRate.z / velocity.x at tau; kind 'chord': circle through the car (tangent to its heading) and the plan position at tau."""
    key = (round(tau, 3), kind)
    if key not in self._ap:
      i1 = int(np.clip(np.searchsorted(self.T, tau), 1, len(self.T) - 1))
      i0 = i1 - 1
      w = float(np.clip((tau - self.T[i0]) / (self.T[i1] - self.T[i0]), 0, 1))
      src = self.kap if kind == "rate" else self.chord
      kt = (1 - w) * src[:, i0] + w * src[:, i1]
      a = np.where(self.mi >= 0, kt[np.maximum(self.mi, 0)], np.nan) * self.v ** 2
      self._ap[key] = a
    return self._ap[key]


# ------------------------------------------------------------------ helpers
def nanq(x, qs=(50, 75, 90)):
  x = np.asarray(x, dtype=float)
  x = x[~np.isnan(x)]
  if not len(x):
    return [np.nan] * len(qs)
  return [float(np.percentile(x, q, method="higher")) for q in qs]


def fmt(x, d=2, unit=""):
  if x is None or (isinstance(x, float) and np.isnan(x)):
    return "-"
  if np.isinf(x):
    return ">1.5"
  return f"{x:.{d}f}{unit}"


def limiter(rw, lo, hi, s_init, ks, n_out, rise_sr=None):
  """steer_limit_events.reconstruct as a recursion on the requested profile rw (raw torque) advanced by ks frames: driver clip [lo, hi] at the frame's own
  time (the driver's torque is as logged), then the +3 up / -7 down step limit from the previous output. Returns S[len(ks), n_out] (raw torque space).
  rise_sr None: pure shift, r'(t) = r(t + k).   rise_sr = turn sign: rise-only advance, r'(t) = max over s in [0, k] of the turn-signed request at t + s
  (the build-up comes k early, the release does not); the k = 0 row of both reproduces the logged sent torque."""
  n = len(rw)
  if rise_sr is None:
    idx = np.minimum(np.arange(n_out)[None, :] + ks[:, None], n - 1)
    Rm = rw[idx]
  else:
    xt = rise_sr * rw
    cur = xt[np.minimum(np.arange(n_out), n - 1)].astype(float)
    Rm = np.empty((len(ks), n_out))
    kset = {int(k): j for j, k in enumerate(ks)}
    if 0 in kset:
      Rm[kset[0]] = cur
    for sft in range(1, int(ks.max()) + 1):
      cur = np.maximum(cur, xt[np.minimum(np.arange(n_out) + sft, n - 1)])
      if sft in kset:
        Rm[kset[sft]] = cur
    Rm = rise_sr * Rm
  S = np.empty((len(ks), n_out))
  prev = np.full(len(ks), float(s_init))
  for i in range(n_out):
    pos = prev > 0
    rlo = np.where(pos, np.maximum(prev - DOWN, -UP), prev - UP)
    rhi = np.where(pos, prev + UP, np.minimum(prev + DOWN, UP))
    prev = np.clip(np.clip(Rm[:, i], lo[i], hi[i]), rlo, rhi)
    S[:, i] = prev
  return S


def first_at(mask, i0):
  """First index >= i0 where mask (array indexed from i0) is True, else -1."""
  k = np.flatnonzero(mask)
  return i0 + int(k[0]) if len(k) else -1


def find_lobes(R, e0, e1, tight):
  """Lobes of constant des sign inside [e0, e1]; returns list of (first idx, peak idx, sign)."""
  des = np.nan_to_num(R.des)
  out = []
  state, start, below = 0, None, 0
  i = e0
  def close(last):
    nonlocal state, start
    if state != 0:
      sl = slice(start, last + 1)
      ip = start + int(np.argmax(state * des[sl]))
      if state * des[ip] >= 0.5 and tight[sl].any():
        out.append((start, ip, state))
    state, start = 0, None
  while i <= e1:
    d = des[i]
    if abs(d) >= THR_A:
      sg = 1 if d > 0 else -1
      if state != 0 and sg != state:
        close(i - 1)
      if state == 0:
        state, start, below = sg, i, 0
      else:
        below = 0
    elif state != 0:
      below += 1
      if below * DT >= 0.3:
        close(i - below)
        below = 0
    i += 1
  close(min(e1, i - 1 - below))
  return out


def turn_in(R, ip, sg, i_anchor_lo=None):
  """Back/forward scan around a lobe peak: returns dict with i_c, i_end, seg bounds; None if the data is unusable."""
  des = np.nan_to_num(R.des)
  seg = R.seg
  lo = ip
  lim = max(0, ip - int(10 / DT))
  while lo > lim and seg[lo - 1] == seg[ip] and sg * des[lo - 1] >= THR_A:
    lo -= 1
  i_c = lo - 1 if (lo > 0 and seg[lo - 1] == seg[ip] and sg * des[lo - 1] < THR_A) else lo      # last frame below 0.3 (crossing is the next frame)
  censored = not (lo > 0 and seg[lo - 1] == seg[ip] and sg * des[lo - 1] < THR_A)
  hi = ip
  below = 0
  lim2 = min(R.n - 1, ip + int(12 / DT))
  while hi < lim2 and seg[hi + 1] == seg[ip]:
    d = sg * des[hi + 1]
    if d < -THR_A:
      break
    below = below + 1 if d < THR_A else 0
    hi += 1
    if below * DT >= 0.3:
      hi -= below
      break
  return dict(i_c=i_c, i_end=hi, ip=ip, sg=sg, cens_start=censored)


class Ctx:
  """Global fit results shared by the turn-in analysis."""
  s_rel = 1.0      # sign(des * req)
  s_cu = 1.0       # sign(des * mdl curvature)
  s_pl = 1.0       # sign(plan curvature * mdl curvature)
  g = np.nan       # m/s^2 per torque unit (torqueState actual)
  ga = np.nan      # m/s^2 per torque unit (controlsState.curvature based)
  d = 0.3          # s
  tau0 = 0.5


def analyze(R, ti, ctx):
  """Timeline, required advance and counterfactual for one turn-in. Returns dict, {"skip": ..} or None. Turn-space arrays are prefixed in the result:
  xo = requested (before driver clip), D1 = after driver clip (the torque the limiter is asked for), sent_w = logged sent, S[k] = sent' for DELTAS[k]."""
  i_c, i_end, ip, sg = ti["i_c"], ti["i_end"], ti["ip"], ti["sg"]
  n = R.n
  sr = sg * ctx.s_rel
  seg = R.seg[ip]
  rr = np.round(np.clip(np.nan_to_num(R.req), -STEER_MAX, STEER_MAX))
  x_all = sr * rr
  sent_s = sr * R.sent
  ref_end = min(i_end, i_c + int(8 / DT))
  hiw = min(ref_end, ip + int(1.0 / DT))
  if hiw <= i_c + 2:
    return None
  pk = float(np.max(x_all[i_c:hiw + 1]))
  if pk < MIN_PEAK_REQ:
    return dict(skip="low", pk=pk)
  i_rp = i_c + int(np.argmax(x_all[i_c:hiw + 1] >= 0.98 * pk))
  # start of the final rise of the request: last frame before i_rp below 10% of the peak (the window starts 1.5 s before i_c)
  lo_ = max(0, i_c - int(1.5 / DT))
  while lo_ < i_c and R.seg[lo_] != seg:
    lo_ += 1
  below = np.flatnonzero(x_all[lo_:i_rp + 1] < 0.1 * pk)
  i_r0 = lo_ + int(below[-1]) + 1 if len(below) else lo_
  se = min(n - 1, ref_end + int(2.0 / DT))
  while se > ref_end and R.seg[se] != seg:
    se -= 1
  i_sp = first_at(sent_s[i_c:se + 1] >= 0.98 * pk, i_c)
  des_s = sg * np.nan_to_num(R.des)
  err_s = sg * (R.des - R.act)
  i_err = first_at(np.nan_to_num(err_s[i_c:se + 1]) > 0.3, i_c)
  aff_s = ctx.s_cu * sg * np.nan_to_num(R.cc) * R.v ** 2
  j = i_c
  while j > lo_ and aff_s[j] >= THR_A:
    j -= 1
  i_a = j + 1 if aff_s[j] < THR_A else lo_
  t = R.rt
  out = dict(pk=pk, i_c=i_c, i_end=i_end, ip=ip, sg=sg, t_c=t[i_c + 1], t_r0=t[min(i_r0, n - 1)], t_rp=t[i_rp], t_a=t[min(i_a, n - 1)],
             t_sp=t[i_sp] if i_sp >= 0 else np.nan, t_err=t[i_err] if i_err >= 0 else np.nan,
             need_ramp=pk / 3 * DT, des_pk=float(des_s[ip]), v=float(np.mean(R.v[i_c:ip + 1]) / MPH),
             radius=1.0 / max(float(np.nanmax(np.abs(R.cc[i_c:ref_end + 1]))), 1e-5),
             pressed=bool(R.pressed[max(0, i_c - 50):i_rp + 50].any()), cens_start=ti["cens_start"], i_rp=i_rp)
  # counterfactual window
  w0 = lo_
  w1 = min(n - 1, ref_end + int(2.0 / DT))
  while w1 > ref_end and R.seg[w1] != seg:
    w1 -= 1
  nw = w1 - w0 + 1
  tail = min(n, w1 + 1 + int(DK[-1]) + 5)
  tail_ok = R.seg[w0:tail] == seg
  tail = w0 + (len(tail_ok) if tail_ok.all() else int(np.argmin(tail_ok)))
  tail = max(tail, w1 + 1)
  dlo, dhi = sle.driver_limits(R.F[w0:tail, C["drv"]])
  S = sr * limiter(rr[w0:tail], dlo, dhi, R.sent[w0], DK, nw)
  Sr = sr * limiter(rr[w0:tail], dlo, dhi, R.sent[w0], DK, nw, rise_sr=sr)
  capc = np.clip(rr, -STEER_MAX, STEER_MAX)
  D1 = sr * np.clip(capc[w0:w1 + 1], dlo[:nw], dhi[:nw])
  out.update(w0=w0, w1=w1, S=S, Sr=Sr, xo=x_all[w0:w1 + 1], D1=D1, sent_w=sent_s[w0:w1 + 1], pk98=0.98 * pk)
  a, b = i_c - w0, min(i_rp + int(0.3 / DT), w1) - w0
  Dw = D1[a:b + 1]
  L = np.minimum.accumulate(Dw[::-1])[::-1]            # torque the request definitely needs by t: suffix minimum, ignores dips and one-frame spikes
  short_env = L[None, :] - S[:, a:b + 1]
  short_strict = Dw[None, :] - S[:, a:b + 1]
  first_ok = lambda m: next((DELTAS[k] for k in range(len(DELTAS)) if m[k]), np.inf)
  out["min_delta"] = first_ok((short_env <= TOL).all(axis=1))
  out["min_delta_strict"] = first_ok((short_strict <= TOL).all(axis=1))
  out["min_delta_rise"] = first_ok(((L[None, :] - Sr[:, a:b + 1]) <= TOL).all(axis=1))
  out["limiter_err_rise"] = float(np.abs(Sr[0] - sent_s[w0:w1 + 1]).max())
  out["limiter_err"] = float(np.abs(S[0] - sent_s[w0:w1 + 1]).max())
  out["limiter_miss"] = int((np.abs(S[0] - sent_s[w0:w1 + 1]) > 0.5).sum())
  out["nwin"] = nw
  return out


def cf_model(R, r, ctx, key="S"):
  """Counterfactual arrays on the window of a turn-in r (w0..w1) for all DELTAS, turn space, for the shifted-sent variant r[key] ('S' pure shift, 'Sr' rise-only).
  orig_short[nw]: the rate limit (not the driver clip) held sent > 19 units below the driver-clipped request; new_short[nD, nw] the same with the shifted sent.
  act_B / act_B2 [nD, nw]: torqueState actual + g * (sent' - sent)(t - d) for the fitted and the speed-fit gain; cs_B / cs_B2 the same on controlsState.curvature * v^2.
  act_C / cs_C: response-shift bound: the car's response moves with the torque profile, act'(t) = act(t + s) with s = min(Delta, how much earlier sent' reaches 98% of the
  request peak than sent did) for the frames from the turn-in start on."""
  sg, w0, w1, S = r["sg"], r["w0"], r["w1"], r[key]
  sl = slice(w0, w1 + 1)
  D1, sent_w = r["D1"], r["sent_w"]
  nw = len(D1)
  d_k = int(round(ctx.d / DT))
  ds = S - sent_w[None, :]
  dsl = np.zeros_like(ds)
  if ds.shape[1] > d_k:
    dsl[:, d_k:] = ds[:, :ds.shape[1] - d_k]
  act_s = sg * np.nan_to_num(R.act[sl])
  cs_s = ctx.s_cu * sg * np.nan_to_num(R.cs[sl]) * R.v[sl] ** 2
  a0 = r["i_c"] - w0
  i_sent = next((j for j in range(a0, nw) if sent_w[j] >= r["pk98"]), nw - 1)
  act_C, cs_C = np.empty_like(S), np.empty_like(S)
  for k in range(len(DELTAS)):
    i_new = next((j for j in range(a0, nw) if S[k, j] >= r["pk98"]), None)
    lead = DK[k] if i_new is None else min(int(DK[k]), max(0, i_sent - i_new))
    jj = np.arange(nw)
    src = np.where(jj >= a0, np.minimum(jj + lead, nw - 1), jj)
    act_C[k], cs_C[k] = act_s[src], cs_s[src]
  return dict(orig_short=(D1 - sent_w) > TOL, new_short=(D1[None, :] - S) > TOL,
              act_B=act_s[None, :] + ctx.g * dsl, act_B2=act_s[None, :] + G2 * dsl, act_C=act_C,
              cs_B=cs_s[None, :] + ctx.ga * dsl, cs_B2=cs_s[None, :] + G2 * dsl, cs_C=cs_C,
              des_s=sg * np.nan_to_num(R.des[sl]), mdl_s=ctx.s_cu * sg * np.nan_to_num(R.mdl[sl]) * R.v[sl] ** 2)


def summary(results, clean, rem_by_var, acnt, nal, names, gent, te, ctx, behind_total):
  """Key numbers of this run in prose (computed from the tables above)."""
  print("## 8. Summary of this run\n")
  def pct(vals, x):
    return f"{np.mean(np.array(vals) <= x) * 100:.0f}%"
  for key, label in (("min_delta_rise", "rise-only advance"), ("min_delta", "pure shift")):
    d = np.array([r[key] for r in results])
    q = nanq(d)
    print(f"- Needed Delta, {label}, {len(results)} turn-ins 0-20 mph (peak request 384 in all but one): median {fmt(q[0], 1)} s, p75 {fmt(q[1], 1)}, p90 {fmt(q[2], 1)}; met within 0.3 / 0.5 / 0.8 / 1.0 s: "
          f"{pct(d, 0.3)} / {pct(d, 0.5)} / {pct(d, 0.8)} / {pct(d, 1.0)}; not met within 1.5 s: {np.mean(np.isinf(d)) * 100:.0f}%.")
  rs = np.array([r["t_rp"] - r["t_r0"] for r in results])
  print(f"- The request itself rises in a median {np.median(rs):.2f} s (t_rp - t_r0) while the ramp limit needs {np.median([r['need_ramp'] for r in results]):.2f} s from zero to 384: "
        f"that gap (~0.8 s) is the needed Delta; sent reaches the peak a median {np.nanmedian([r['t_sp'] - r['t_rp'] for r in results]):.2f} s after the request.")
  ld = np.array([r["lead_see"] for r in results], dtype=float)
  lr = np.array([r["lead_see_rate"] for r in results], dtype=float)
  print(f"- Plan anticipation (turn visible in the plan before the request starts its final rise; any tau <= 5 s): rate {np.nanmedian(lr):.2f} s median / {np.nanpercentile(lr, 25):.2f} p25 / {np.nanpercentile(lr, 10):.2f} p10, "
        f"rate or chord {np.nanmedian(ld):.2f} / {np.nanpercentile(ld, 25):.2f} / {np.nanpercentile(ld, 10):.2f}. Today's a_plan(tau0) crosses 0.3 {np.median([r['t_c'] - r['tx'][('rate', 0.0)] for r in results if np.isfinite(r['tx'][('rate', 0.0)])]):.2f} s "
        f"before des does (it already is the feed-forward input).")
  need = [r for r in results if r["min_delta_rise"] > 0]
  cov = np.mean([np.isfinite(r["lead_see"]) and r["lead_see"] >= r["min_delta_rise"] for r in need]) * 100
  cov2 = np.mean([np.isfinite(r["lead_see"]) and r["lead_see"] >= 0.5 * r["min_delta_rise"] for r in need]) * 100
  print(f"- Among the {len(need)} turn-ins that need an advance, the plan (rate or chord, any tau) showed the turn at least the needed Delta ahead in {cov:.0f}% and at least half of it in {cov2:.0f}%.")
  tot = sum(behind_total.values())
  for key, label in (("Sr", "rise-only"), ("S", "pure shift")):
    rem = rem_by_var[key]
    out = []
    for d, _ in APPLY:
      k = DELTAS.index(d)
      out.append(f"{d}: " + "/".join(f"{sum(rem[n_][x][k] for x in (0, 1)) / max(tot, 1) * 100:.0f}%" for n_ in names))
    print(f"- Behind time removed ({label}, share of all 0-20 mph behind seconds, A/B/B2/C): " + "; ".join(out) + ".")
  for key, label in (("Sr", "rise-only"), ("S", "pure shift")):
    print(f"- Alerts removed of {nal} ({label}, A/B/B2/C): " + "; ".join(f"{d}: " + "/".join(str(acnt[key][DELTAS.index(d)][n_]) for n_ in names) for d, _ in APPLY) + ".")
  print(f"- Gentle curves, share of non-tight frames where a pure shift of 0.5 s changes des by > 0.3 m/s^2: " + ", ".join(f"{b} {gent[(0.5, b)][0]:.0f}%" for b in ("0-10 mph", "10-20 mph", "20-40 mph", "40+ mph")) + ".")
  print("- Non-tight tracking error (mean |des - act| m/s^2 / share > 0.3), baseline vs plan model +0.8 s: " +
        ", ".join(f"{b} {te[(b, 'base')][0]:.3f}/{te[(b, 'base')][1]:.1f}% -> {te[(b, 'p0.8')][0]:.3f}/{te[(b, 'p0.8')][1]:.1f}%" for b in ("0-10 mph", "10-20 mph", "20-40 mph", "40+ mph")) + ".")
  print()


def cross_idx(R, a_s, i0):
  """first index of the run of a_s >= THR ending at i0 (going back, cap 10 s); None when a_s[i0] < THR or NaN."""
  if not (np.isfinite(a_s[i0]) and a_s[i0] >= THR_A):
    return None
  j = i0
  lim = max(0, i0 - 1000)
  while j > lim and R.seg[j - 1] == R.seg[i0] and np.isfinite(a_s[j - 1]) and a_s[j - 1] >= THR_A:
    j -= 1
  return j


def tcross(R, a_s, ip_):
  j = cross_idx(R, a_s, ip_)
  return R.rt[j] if j is not None else np.nan


def plan_leads(R, r, ctx):
  """When does the plan show the turn? Crossing times of turn*a_plan >= 0.3 for the look-aheads tau0 + e (rate), and tau in SIGHT_TAUS (rate and chord);
  lead_see = t_r0 - earliest crossing (rate or chord), lead_see_rate = same without chord."""
  sg, ip_ = r["sg"], r["ip"]
  sp = sg * ctx.s_cu * ctx.s_pl      # plan value -> model curvature convention (s_pl) -> des convention (s_cu) -> turn space (sg)
  r["tx"] = {("rate", 0.0): tcross(R, sp * R.a_plan(ctx.tau0), ip_)}
  for e in EXTS:
    r["tx"][("rate", e)] = tcross(R, sp * R.a_plan(ctx.tau0 + e), ip_)
  for tt in SIGHT_TAUS:
    r["tx"][("rate_tau", tt)] = tcross(R, sp * R.a_plan(tt), ip_)
    r["tx"][("chord_tau", tt)] = tcross(R, sp * R.a_plan(tt, "chord"), ip_)
  cand = [v for k, v in r["tx"].items() if k[0] in ("rate_tau", "chord_tau", "rate") and np.isfinite(v)]
  r["t_see"] = min(cand) if cand else np.nan
  r["t_see_rate"] = min([v for k, v in r["tx"].items() if k[0] in ("rate_tau", "rate") and np.isfinite(v)] or [np.nan])
  r["lead_see"] = r["t_r0"] - r["t_see"]
  r["lead_see_rate"] = r["t_r0"] - r["t_see_rate"]


def shift_idx(R, i, k):
  """i + k clipped to the same continuous segment."""
  j = np.minimum(i + k, R.n - 1)
  bad = R.seg[j] != R.seg[i]
  return np.where(bad, i, j)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--rlog", nargs="*", default=RLOGS)
  ap.add_argument("--cache", default=None)
  ap.add_argument("--workers", type=int, default=6)
  ap.add_argument("--max-seg", type=int, default=None)
  a = ap.parse_args()
  add_starpilot_path('/home/openpilotstuff/wt/stinger-combined')

  cache = {}
  if a.cache and os.path.exists(a.cache):
    cache = pickle.load(open(a.cache, "rb"))
  routes = []
  for r in a.rlog:
    key = (r, a.max_seg)
    if key not in cache:
      fs = segment_files(r, "rlog")
      if a.max_seg:
        fs = fs[:a.max_seg]
      cache[key] = (fs, pmap(scan2, fs, a.workers)) if fs else ([], [])
      if a.cache:
        pickle.dump(cache, open(a.cache, "wb"))
    fs, res = cache[key]
    if fs:
      routes.append(R2(short(r), fs, res))

  ctx = Ctx()
  print("# Steering ramp advance: how much earlier would the request have to start (Kia Stinger 2022, +3 / -7 per 10 ms, cap 384)\n")
  print("Definitions, models and usage are in the docstring of analysis/steer_ramp_advance.py. Analysis only: the panda safety limits are not touched.\n")

  # ---------------------------------------------------------------- 0 signals
  print("## 0. Signals, signs and fitted response\n")
  allf = sum(R.n for R in routes)
  print(f"- rlog routes: {len(routes)}, {sum(R.nseg for R in routes)} segments, {allf} frames (100 Hz).")
  num = lambda f: float(sum(np.nansum(f(R)) for R in routes))
  big = lambda R: (np.abs(np.nan_to_num(R.des)) > 1.0) & R.lat & (R.v > 1)
  ctx.s_rel = 1.0 if num(lambda R: np.where(big(R), np.nan_to_num(R.des) * np.nan_to_num(R.req), 0)) >= 0 else -1.0
  ctx.s_cu = 1.0 if num(lambda R: np.where(big(R), np.nan_to_num(R.des) * np.nan_to_num(R.mdl) * R.v ** 2, 0)) >= 0 else -1.0
  lags = np.concatenate([R.lag[R.lat & np.isfinite(R.lag)] for R in routes])
  ctx.tau0 = float(np.median(lags)) + 0.1 + 0.075
  for R in routes:
    R.a_plan(ctx.tau0)
  ctx.s_pl = 1.0 if num(lambda R: np.where(big(R) & np.isfinite(R.a_plan(ctx.tau0)), np.nan_to_num(R.mdl) * R.v ** 2 * np.nan_to_num(R.a_plan(ctx.tau0)), 0)) >= 0 else -1.0
  print(f"- Sign relations (from data): des vs request {'same' if ctx.s_rel > 0 else 'opposite'}; des vs modelV2 desiredCurvature*v^2 {'same' if ctx.s_cu > 0 else 'opposite'}; "
        f"plan curvature vs desiredCurvature {'same' if ctx.s_pl > 0 else 'opposite'}.")
  print(f"- liveDelay.lateralDelay while lateral active: median {np.median(lags):.3f} s (p10 {np.percentile(lags, 10):.3f}, p90 {np.percentile(lags, 90):.3f}). "
        f"modeld plans for lat_action_t = lag + 0.1 (LAT_SMOOTH_SECONDS) + 0.05 + 0.025 -> tau0 = {ctx.tau0:.3f} s; controlsd uses lat_delay = lag + 0.1.")
  # how well is the plan at tau0 the model action?
  both = []
  for R in routes:
    m = big(R) & np.isfinite(R.a_plan(ctx.tau0))
    both.append((np.nan_to_num(R.mdl[m]) * R.v[m] ** 2, R.a_plan(ctx.tau0)[m] * ctx.s_pl))
  am, ap_ = np.concatenate([b[0] for b in both]), np.concatenate([b[1] for b in both])
  sel = np.abs(am) > 0.3
  print(f"- Plan vs action check (frames with |des| > 1): s_pl * a_plan(tau0) vs modelV2 action * v^2: corr {np.corrcoef(am, ap_)[0, 1]:.2f}, median ratio "
        f"{np.median(ap_[sel] / am[sel]):.2f} (1.00 = the action is the plan evaluated at tau0). Sign of carControl.actuators.curvature vs modelV2 action: "
        f"{'same' if num(lambda R: np.where(big(R), np.nan_to_num(R.cc) * np.nan_to_num(R.mdl), 0)) >= 0 else 'opposite'}.")

  # ---------------------------------------------------------------- episodes and turn-ins
  tins = []
  n_ep = 0
  for R in routes:
    v = R.v
    des = np.nan_to_num(R.des)
    act_ok = R.lat & (v >= 1.0)
    tight = act_ok & (v < 20 * MPH) & ((np.abs(des) > TIGHT_A) | (np.abs(np.nan_to_num(R.cc)) > TIGHT_K))
    R.tight = tight
    R.base = R.fl["base"]
    idx = np.flatnonzero(tight)
    if not len(idx):
      continue
    st = 0
    for j in range(1, len(idx) + 1):
      if j == len(idx) or R.rt[idx[j]] - R.rt[idx[j - 1]] > 0.5 or R.seg[idx[j]] != R.seg[idx[j - 1]]:
        e0, e1 = idx[st], idx[j - 1]
        st = j
        if (e1 - e0 + 1) * DT < 0.3:
          continue
        n_ep += 1
        for (i_first, ip, sg) in find_lobes(R, e0, e1, tight):
          ti = turn_in(R, ip, sg)
          if ti is None:
            continue
          ti["route"], ti["ep"] = R.name, n_ep
          ti["R"] = R
          tins.append(ti)
  # drop duplicate lobes (same i_c, same route)
  seen, uniq = set(), []
  for ti in tins:
    k = (ti["route"], ti["i_c"])
    if k not in seen:
      seen.add(k)
      uniq.append(ti)
  tins = uniq

  # ---------------------------------------------------------------- fit g, d on tight turn-ins
  fits = []
  for ti in tins:
    R = ti["R"]
    sl = slice(ti["i_c"], min(ti["i_end"], ti["i_c"] + 800) + 1)
    msk = (~R.pressed[sl]) & R.tight[sl]
    if msk.sum() >= 20:
      fits.append((R, sl, msk, ti["sg"]))
  def fit_at(dk, key):
    sxy = sxx = syy = 0.0
    for (R, sl, msk, sg) in fits:
      i0, i1 = sl.start, sl.stop
      if i0 - dk < 0:
        continue
      u = sg * ctx.s_rel * R.sent[i0 - dk:i1 - dk]
      y = sg * np.nan_to_num(R.act[sl]) if key == "act" else sg * ctx.s_cu * np.nan_to_num(R.cs[sl]) * R.v[sl] ** 2
      u, y = u[msk], y[msk]
      sxy += float((u * y).sum())
      sxx += float((u * u).sum())
      syy += float((y * y).sum())
    return sxy / sxx, syy - sxy * sxy / sxx
  scan_fit = [(dk, *fit_at(dk, "act")) for dk in range(0, 61, 2)]
  d_opt = min(scan_fit, key=lambda x: x[2])[0] * DT
  ctx.d = round(float(np.median(lags)) / DT) * DT
  ctx.g = fit_at(int(round(ctx.d / DT)), "act")[0]
  ctx.ga = fit_at(int(round(ctx.d / DT)), "cs")[0]
  ctx.g_opt = min(scan_fit, key=lambda x: x[2])[1]
  print(f"- Static response fit on {len(fits)} tight turn-ins (turn-signed actual lat accel on turn-signed sent torque delayed by d, least squares through the origin, tight unpressed frames): "
        f"the best-fitting d is {d_opt:.2f} s (g = {ctx.g_opt * 384:.2f} m/s^2 per full torque); the model uses the measured liveDelay d = {ctx.d:.2f} s, g = {ctx.g * 384:.2f} m/s^2 per full torque "
        f"({ctx.g:.5f} per unit; controlsState.curvature based g_a = {ctx.ga * 384:.2f}). Reference: steer_torque_limit_results.md measured 2.26 m/s^2 per full torque at v > 8 m/s "
        f"(model B2 uses {G2 * 384:.2f}); the low-speed gain is lower because the steering-angle based lat accel at 2-9 m/s also depends on speed and the cap saturates it.")
  print(f"- Tight-turn episodes: {n_ep}; turn-ins (lobes with a tight frame, peak |des| >= 0.5): {len(tins)}.\n")

  # ---------------------------------------------------------------- analyze
  results = []
  nlow = 0
  for ti in tins:
    r = analyze(ti["R"], ti, ctx)
    if r is None:
      continue
    if "skip" in r:
      nlow += 1
      continue
    r.update(route=ti["route"], R=ti["R"], ep=ti["ep"])
    results.append(r)
  mph_band = lambda v: 0 if v < 10 else 1 if v < 20 else 2
  clean = [r for r in results if not r["pressed"] and not r["cens_start"]]
  nfr = sum(r["nwin"] for r in results)
  nmiss = sum(r["limiter_miss"] for r in results)
  print(f"- Limiter check (driver clip from carState.steeringTorque, then the +3/-7 step, started from the logged sent 1.5 s before each turn-in): the unshifted simulation reproduces the logged "
        f"sent torque exactly in {(1 - nmiss / nfr) * 100:.2f}% of {nfr} window frames (max deviation {max(r['limiter_err'] for r in results):.0f} units).")
  print(f"- Turn-ins analysed: {len(results)} (dropped {nlow} with peak request < {MIN_PEAK_REQ:.0f} units); with no steeringPressed from 0.5 s before the turn-in to 0.5 s after the request "
        f"peak and a des start inside the data: {len(clean)} ('clean'). {sum(r['pressed'] for r in results)} of {len(results)} turn-ins contain a driver press: in tight turns the driver intervenes "
        f"often, so the unpressed subset is small and the all-turn-ins numbers are shown next to it.\n")

  # ---------------------------------------------------------------- 1 controller path
  print("## 1. How the controller uses the lateral delay (read-only, /home/openpilotstuff/wt/stinger-combined)\n")
  print("""- `selfdrive/locationd/lagd.py:236-237`: `liveDelay.lateralDelay` = StarPilot `SteerDelay` param when `UseAutoSteerDelay` is off (`starpilot/common/starpilot_variables.py:808-810`, clamp 0.01-1.0 s; the UI value is `steerActuatorDelay + 0.2`, `starpilot/common/lateral_delay.py`), else the live estimate (0.32-0.35 s on this car).
- `selfdrive/modeld/modeld.py:1295-1296, 1362`: `lat_delay = liveDelay.lateralDelay + lat_smooth_seconds` (LAT_SMOOTH_SECONDS 0.1, or developer param `LatSmoothSeconds`, `modeld.py:87, 91-95`, `params_keys.h:535`); the action is taken `lat_action_t = lat_delay + DT_MDL + DT_MDL/2` ahead (about 0.5 s) -> `modelV2.action.desiredCurvature` (`modeld.py:399-455`, smoothed with `smooth_value`).
- `selfdrive/controls/controlsd.py:800` `lat_delay = liveDelay.lateralDelay + 0.1` goes to `LatControlTorque.update`. There (`latcontrol_torque.py:246, 278-286`): feed-forward input = `desired_curvature * v^2` (the model action, NOT delayed); the PID setpoint is the action delayed by lat_delay plus a jerk term (`expected_lateral_accel + desired_lateral_jerk * lat_delay`). So the feed-forward already leads the car's delay by about 0.4-0.5 s; the +3/10 ms ramp (0 to 384 takes 1.28 s) is what is not covered.
- Existing low-speed anticipation: `controlsd.py:303-368` TURN_LEAD_* (plan probed at a constant 1.3 s distance, blinker turns only, 3-7 m/s, capped at 0.12 1/m) and the curvature hold / pre-wind before it (`CURVATURE_HOLD_*`, `get_plan_turn_onset_dist`). These are blinker-gated, so they do not cover un-signalled tight curves.
- The panda limits (`opendbc hyundai carcontroller`: STEER_MAX 384, DELTA_UP 3, DELTA_DOWN 7) are not part of any of that and stay as they are.
""")

  # ---------------------------------------------------------------- 2 timeline
  print("## 2. Timeline of tight low-speed turn-ins\n")
  print("t_c = des crosses 0.3 m/s^2 heading to the peak (= 0 below). Median (p75) of each event in seconds relative to t_c. t_a: the controller's desired curvature * v^2 (feed-forward input) crosses 0.3; "
        "t_r0: start of the final rise of the request (last frame below 10% of its peak); t_rp: request reaches 98% of its peak (<= 384); t_sp: sent reaches 98% of that peak "
        "(missing when it does not before the lobe ended + 2 s); t_err: first turn*(des - act) > 0.3. Only turn-ins with peak request >= 100 units.\n")
  groups = [("0-10 mph", [r for r in results if mph_band(r["v"]) == 0]), ("10-20 mph", [r for r in results if mph_band(r["v"]) == 1]), ("0-20 mph", results), ("0-20 clean", clean)]
  groups = [(l, g_) for l, g_ in groups if g_]
  def mq(vals):
    q = nanq(vals, (50, 75))
    return f"{fmt(q[0])} ({fmt(q[1])})"
  rows = [["turn-ins"] + [str(len(g_)) for _, g_ in groups]]
  for nm, k in [("t_a feed-forward input >= 0.3", "t_a"), ("t_r0 request starts final rise", "t_r0"), ("t_rp request peak", "t_rp"), ("t_sp sent peak", "t_sp"), ("t_err tracking error > 0.3", "t_err")]:
    rows.append([nm] + [mq(np.array([r[k] - r["t_c"] for r in g_])) for _, g_ in groups])
  rows.append(["lag t_sp - t_rp"] + [mq(np.array([r["t_sp"] - r["t_rp"] for r in g_])) for _, g_ in groups])
  rows.append(["ramp needed = peak / 3 per 10 ms"] + [mq(np.array([r["need_ramp"] for r in g_])) for _, g_ in groups])
  rows.append(["request ramp duration t_rp - t_r0"] + [mq(np.array([r["t_rp"] - r["t_r0"] for r in g_])) for _, g_ in groups])
  rows.append(["peak request (units)"] + [mq(np.array([r["pk"] for r in g_])).replace("(", "(").replace(".00", "") for _, g_ in groups])
  rows.append(["sent never reaches 98% of peak"] + [f"{np.mean([np.isnan(r['t_sp']) for r in g_]) * 100:.0f}%" for _, g_ in groups])
  rows.append(["t_err missing (never behind by 0.3)"] + [f"{np.mean([np.isnan(r['t_err']) for r in g_]) * 100:.0f}%" for _, g_ in groups])
  table(["event"] + [l for l, _ in groups], rows)

  # ---------------------------------------------------------------- 3 required delta
  print("## 3. Required advance Delta\n")
  print("Smallest Delta in 0.1 s steps such that, after the same driver clip and +3/-7 limiter, sent' stays within 19 units (5%) of the torque the request definitely needs: the suffix-minimum "
        "envelope of the driver-clipped request over [i_c, t_rp + 0.3 s] (it ignores dips and one-frame spikes of the noisy request). '>1.5' = not met within 1.5 s, 'needs none' = the unshifted limiter already met it. "
        "Two ways to advance the request: **pure shift** x'(t) = x(t + Delta) (as specified; it also advances every release and every reversal) and **rise-only** x'(t) = max of x over [t, t + Delta] "
        "(the build-up comes Delta early, the release stays where it is: what an anticipatory feed-forward that only reads the plan ahead on the way in would do). "
        "Small n: the tight-turn set is a few dozen turn-ins, and the request often oscillates between the two torque limits while the driver fights it, which a pure shift cannot follow.\n")
  def dist_rows(groups, key="min_delta"):
    out = []
    for label, sub in groups:
      if not sub:
        continue
      d = np.array([r[key] for r in sub])
      q = nanq(d)
      out.append([label, len(sub)] + [fmt(x, 1) for x in q] + [f"{np.mean(d == 0) * 100:.0f}%", f"{np.mean(np.isinf(d)) * 100:.0f}%"] +
                 [f"{np.mean(d <= x) * 100:.0f}%" for x in (0.3, 0.5, 0.8, 1.0)])
    return out
  hd = ["group", "turn-ins", "min Delta median", "p75", "p90", "needs none", "> 1.5 s", "met at 0.3", "at 0.5", "at 0.8", "at 1.0"]
  rb = [("< 10 m", lambda r: r["radius"] < 10), ("10-20 m", lambda r: 10 <= r["radius"] < 20), ("20-50 m", lambda r: 20 <= r["radius"] < 50), (">= 50 m", lambda r: r["radius"] >= 50)]
  VARS = [("min_delta", "pure shift (as specified)"), ("min_delta_rise", "rise-only advance")]
  for key, label in VARS:
    print(f"### 3.{VARS.index((key, label)) + 1} {label}\n")
    g = [("0-10 mph", [r for r in results if mph_band(r["v"]) == 0]), ("10-20 mph", [r for r in results if mph_band(r["v"]) == 1]), ("0-20 mph", results),
         ("clean 0-10", [r for r in clean if mph_band(r["v"]) == 0]), ("clean 10-20", [r for r in clean if mph_band(r["v"]) == 1]), ("clean 0-20", clean)]
    table(hd, dist_rows(g, key))
    print("By peak radius (1 / max |curvature| over the turn-in), 0-20 mph:\n")
    table(hd, dist_rows([(l, [r for r in results if f(r)]) for l, f in rb], key))
  print("Strict criterion for the pure shift (every frame of the driver-clipped request must be matched within 19 units, no envelope):\n")
  table(hd, dist_rows([("0-20 mph", results), ("clean 0-20", clean)], "min_delta_strict"))
  osc = [r for r in results if r["min_delta_rise"] == np.inf]
  print(f"{len(osc)} of {len(results)} turn-ins are not met within 1.5 s even by the rise-only advance; in those the request reverses between the torque limits within the window or the driver clips it "
        f"({sum(r['pressed'] for r in osc)} of them contain a press), so no earlier start of the same request profile helps.\n")
  print("Peak request of the turn-ins: " + ", ".join(f"{lab} {sum(1 for r in results if f_(r['pk']))}" for lab, f_ in [("100-192", lambda x: x < 192), ("192-288", lambda x: 192 <= x < 288),
        ("288-380", lambda x: 288 <= x < 380), ("380-384 (cap)", lambda x: x >= 380)]) + " (every turn-in that matters wants the full 384, so the ramp needs 1.28 s from zero).\n")

  # ---------------------------------------------------------------- 4 removal of behind time and alerts
  print("## 4. Behind time and alerts removed by an advance of Delta\n")
  print(f"Behind = tight, unpressed, < 20 mph, turn*(des-act) > 0.3, inside a turn-in window [t_c, lobe end]. Four response models (turn-in only; none of them can credit the 384 cap):\n"
        f"- A request matching: removed if sent was > 19 units short of the driver-clipped request (the rate limit, not the cap or the driver) AND sent' is within 19 units.\n"
        f"- B static gain with delay: act' = act + g (sent' - sent)(t - d), d = {ctx.d:.2f} s (liveDelay), g = {ctx.g * 384:.2f} m/s^2 per full torque (fitted on these turn-ins).\n"
        f"- B2 the same with g = {G2 * 384:.2f} m/s^2 per full torque (steer_torque_limit_results.md, v > 8 m/s).\n"
        f"- C response shift (optimistic bound): the car's response moves with the torque profile, act'(t) = act(t + s), s = min(Delta, how much earlier sent' reaches 98% of the peak request than sent did). "
        "It assumes the wheel angle keeps integrating the torque (true under a pegged request at 2-9 m/s, where the wheel turns 100-280 deg/s), so the lead persists after the ramp.\n")
  names = ["A", "B", "B2", "C"]
  behind_total = {0: 0, 1: 0}
  for R in routes:
    bh = R.fl["behind"] & R.tight & R.base
    bb = np.where(R.v < 10 * MPH, 0, 1)
    for b in (0, 1):
      behind_total[b] += int((bh & (bb == b)).sum())
  tot = sum(behind_total.values())
  rem_by_var = {}
  for key, label in [("S", "pure shift"), ("Sr", "rise-only")]:
    inwin = {0: 0, 1: 0}
    rem = {n_: {b: np.zeros(len(DELTAS)) for b in (0, 1)} for n_ in names}
    cap_bound = {0: 0, 1: 0}
    claimed = {R.name: np.zeros(R.n, bool) for R in routes}
    for r in results:
      R = r["R"]
      w0, w1, i0 = r["w0"], r["w1"], r["i_c"]
      sl = slice(w0, w1 + 1)
      bh = (R.fl["behind"] & R.tight & R.base)[sl]
      pos = np.arange(w0, w1 + 1)
      use = bh & (pos >= i0) & (pos <= r["i_end"]) & ~claimed[R.name][sl]
      claimed[R.name][sl] |= use
      if not use.any():
        continue
      m = cf_model(R, r, ctx, key)
      bb = (R.v[sl] >= 10 * MPH).astype(int)
      remv = {"A": use[None, :] & m["orig_short"][None, :] & ~m["new_short"]}
      for n_, ak in (("B", "act_B"), ("B2", "act_B2"), ("C", "act_C")):
        remv[n_] = use[None, :] & ~((m["des_s"][None, :] - m[ak]) > 0.3)
      for b in (0, 1):
        u = use & (bb == b)
        inwin[b] += int(u.sum())
        cap_bound[b] += int((u & ~m["orig_short"]).sum())
        for n_ in names:
          rem[n_][b] += (remv[n_] & (bb == b)[None, :]).sum(axis=1)
    rem_by_var[key] = rem
    def cell(b_, d):
      k = DELTAS.index(d)
      bs = (0, 1) if b_ is None else (b_,)
      tot_ = sum(behind_total[x] for x in bs)
      return " / ".join(f"{sum(rem[n_][x][k] for x in bs) / max(tot_, 1) * 100:.0f}%" for n_ in names)
    rows = []
    for b, nm in [(0, "0-10"), (1, "10-20"), (None, "0-20")]:
      bs = (0, 1) if b is None else (b,)
      tt = sum(behind_total[x] for x in bs)
      iw = sum(inwin[x] for x in bs)
      rows.append([nm, f"{tt * DT:.1f}", f"{iw / max(tt, 1) * 100:.0f}%", f"{sum(cap_bound[x] for x in bs) / max(iw, 1) * 100:.0f}%"] + [cell(b, d) for d, _ in APPLY])
    print(f"### 4.{1 if key == 'S' else 2} Behind time removed, {label} (share of ALL behind seconds in the band; A / B / B2 / C)\n")
    print("'in turn-in windows' = share of behind time inside an analysed turn-in window (the rest is in dropped low-demand lobes or outside any lobe); 'cap-bound' = share of those frames where sent "
          "was not short of the driver-clipped request by more than 19 units (at the 384 cap, or the request itself is low), so advancing the ramp changes nothing there.\n")
    table(["band mph", "behind s", "in turn-in windows", "cap-bound share", "Delta 0.3", "0.5", "0.8", "1.0"], rows)
    print(f"Every Delta in the grid ({label}, 0-20 mph):\n")
    table(["Delta s"] + [f"{d:.1f}" for d in DELTAS[1:]],
          [[f"{n_} removed"] + [f"{sum(rem[n_][x][k] for x in (0, 1)) / max(tot, 1) * 100:.0f}%" for k in range(1, len(DELTAS))] for n_ in names])

  # alerts
  alert_rows = []
  for R in routes:
    for al in R.alerts:
      if al[1] != "sd" or "steerSaturated" not in al[2]:
        continue
      ia = int(np.searchsorted(R.rt, al[0]))
      if ia >= R.n:
        continue
      lo, hi = max(0, ia - 100), min(R.n - 1, ia + 20)
      d = np.nan_to_num(R.des)[lo:hi + 1]
      if not len(d) or np.abs(d).max() < THR_A:
        continue
      jm = lo + int(np.argmax(np.abs(d)))
      sg = 1 if R.des[jm] > 0 else -1
      lo2, hi2 = max(0, ia - 200), min(R.n - 1, ia + 50)
      ip = lo2 + int(np.argmax(sg * np.nan_to_num(R.des)[lo2:hi2 + 1]))
      ti = turn_in(R, ip, sg)
      r = analyze(R, ti, ctx)
      if r is None or "skip" in r:
        alert_rows.append((R, al, ia, None))
        continue
      r.update(route=R.name, R=R, t_alert=al[0], mph_alert=R.v[ia] / MPH)
      plan_leads(R, r, ctx)
      alert_rows.append((R, al, ia, r))
  print(f"### 4.3 The {len(alert_rows)} steerSaturated alerts\n")
  print("For each alert the turn-in containing it is analysed like the episodes (same limiter). The PRED predicate (selfdrived.py L769-783) is re-evaluated in [alert - 0.1, alert + 2.0 s]: "
        "A: a PRED frame survives unless the rate limit was what held sent short and sent' is within 19 units; B / B2 / C: undershoot recomputed with controlsState.curvature * v^2 moved by the model "
        "(the 'output at max' condition is kept as logged, i.e. the controller is assumed to stay saturated: conservative). The alert is removed when no PRED frame survives. "
        "Rise-only advance; the pure shift is summarised in the count table. 'peak req' = turn peak request, 'min Delta' = section 3 criterion (rise-only).\n")
  print("Timeline per alert (seconds relative to t_c, des crossing 0.3 of the turn-in that contains the alert):\n")
  rows = []
  for (R, al, ia, r) in alert_rows:
    if r is None:
      rows.append([R.name, f"{al[0]:.0f}", f"{R.v[ia] / MPH:.0f}", "dropped (no usable turn-in)"] + [""] * 7)
      continue
    rows.append([R.name, f"{al[0]:.0f}", f"{R.v[ia] / MPH:.0f}", f"{al[0] - r['t_c']:+.2f}", fmt(r["t_a"] - r["t_c"]), fmt(r["t_r0"] - r["t_c"]), fmt(r["t_rp"] - r["t_c"]),
                 fmt(r["t_sp"] - r["t_c"]) if np.isfinite(r["t_sp"]) else "never", fmt(r["t_err"] - r["t_c"]) if np.isfinite(r["t_err"]) else "never", fmt(r["lead_see"]) if np.isfinite(r["lead_see"]) else "n/a",
                 "press" if r["pressed"] else ""])
  table(["route", "t s", "mph", "alert at", "t_a", "t_r0", "t_rp request peak", "t_sp sent peak", "t_err", "plan lead any tau (rate or chord)", ""], rows)
  show = [DELTAS.index(d) for d, _ in APPLY]
  rows = []
  acnt = {v_: {k: {n_: 0 for n_ in names} for k in range(len(DELTAS))} for v_ in ("S", "Sr")}
  pred_tot = 0
  pred_surv = {k: {n_: 0 for n_ in names} for k in range(len(DELTAS))}
  delays = {k: {n_: [] for n_ in names} for k in range(len(DELTAS))}
  nal = 0
  for (R, al, ia, r) in alert_rows:
    mph = R.v[ia] / MPH
    if r is None:
      rows.append([R.name, f"{al[0]:.0f}", f"{mph:.0f}", "-", "-", "-", "-", "-", "-"])
      continue
    nal += 1
    w0, w1 = r["w0"], r["w1"]
    wsl = slice(max(ia - 10, 0), min(ia + 200, R.n - 1) + 1)
    pidx = np.flatnonzero(R.fl["PRED"][wsl]) + wsl.start
    pidx = pidx[(pidx >= w0) & (pidx <= w1)]
    c = pidx - w0
    pred_tot += len(c)
    gone_v = {}
    for key in ("S", "Sr"):
      m = cf_model(R, r, ctx, key)
      surv = {"A": ~(m["orig_short"][c][None, :] & ~m["new_short"][:, c])}
      for n_, ck in (("B", "cs_B"), ("B2", "cs_B2"), ("C", "cs_C")):
        surv[n_] = np.abs(m["mdl_s"][c])[None, :] / np.abs(1e-3 + m[ck][:, c]) > 1.2
      gone = {n_: ~surv[n_].any(axis=1) for n_ in names}
      gone_v[key] = gone
      if key == "Sr":
        for k in range(len(DELTAS)):
          for n_ in names:
            pred_surv[k][n_] += int(surv[n_][k].sum())
            if surv[n_][k].any() and len(c):
              delays[k][n_].append(float((np.argmax(surv[n_][k]) - 0) * 0.0 + (pidx[np.argmax(surv[n_][k])] - pidx[0]) * DT))
      for k in range(len(DELTAS)):
        for n_ in names:
          acnt[key][k][n_] += int(gone[n_][k])
    row = [R.name, f"{al[0]:.0f}", f"{mph:.0f}", f"{r['pk']:.0f}", fmt(r["min_delta_rise"], 1)]
    row += ["/".join("y" if gone_v["Sr"][n_][k] else "n" for n_ in names) for k in show]
    rows.append(row)
  table(["route", "t s", "mph", "peak req", "min Delta", "removed @0.3 (A/B/B2/C)", "@0.5", "@0.8", "@1.0"], rows)
  print(f"Alerts removed of the {nal} analysable ones, by Delta:\n")
  table(["Delta s"] + [f"{d:.1f}" for d in DELTAS[1:]],
        [[f"{lab} {n_}"] + [str(acnt[key][k][n_]) for k in range(1, len(DELTAS))] for key, lab in (("Sr", "rise-only"), ("S", "shift")) for n_ in names])
  print("Share of the PRED frames (alert predicate true, 100 Hz, in the alert windows) that survive with the rise-only advance: the alerts come 0.5-6 s after t_c, mostly while sent already sits at the "
        "cap and the car still undershoots, so the predicate stays true.\n")
  rows = []
  for n_ in names:
    rows.append([n_] + [f"{pred_surv[DELTAS.index(d)][n_] / max(pred_tot, 1) * 100:.0f}%" for d, _ in APPLY])
  table(["model", "Delta 0.3", "0.5", "0.8", "1.0"], rows)
  ar = [r for (R, al, ia, r) in alert_rows if r is not None]
  if ar:
    for key, lab in (("min_delta_rise", "rise-only"), ("min_delta", "pure shift")):
      d = np.array([x[key] for x in ar])
      print(f"Min Delta for the alert turn-ins ({lab}): median {fmt(nanq(d)[0], 1)}, p75 {fmt(nanq(d)[1], 1)}, p90 {fmt(nanq(d)[2], 1)} s; share needing > 1.5 s: {np.mean(np.isinf(d)) * 100:.0f}%; "
            f"needing none: {np.mean(d == 0) * 100:.0f}%.")
    print()

  # ---------------------------------------------------------------- 5 knowability
  print("## 5. Is the advance knowable? What the model plan shows ahead of time\n")
  print(f"a_plan(t, tau) = plan curvature at tau seconds ahead of the latest modelV2 * v_ego^2; 'rate' = orientationRate.z / velocity.x, 'chord' = circle through the car (tangent to its heading) and "
        f"the plan position at tau (the probe TURN_LEAD uses). tau0 = {ctx.tau0:.2f} s is the look-ahead modeld already uses for the action, so a_plan(tau0) is today's feed-forward input. "
        f"Plan crossing time = start of the last run, going back from the des peak, in which turn*a_plan >= {THR_A} m/s^2. Lead = t_r0 - plan crossing (positive = the plan showed the turn "
        f"before the request started its final rise; this is the anticipation available to start the ramp early); lead_c = t_c - plan crossing.\n")
  for r in results:
    plan_leads(r["R"], r, ctx)
  print("Plan crossing relative to the controller (seconds, median / p25 / p10; positive = plan earlier). 'today' = a_plan(tau0); 'plan(tau0 + e)' = the same plan read e seconds further ahead "
        "(what a larger look-ahead would feed); 'any tau' = the earliest crossing over tau in {tau0, 0.5, 1, 1.5, 2, 3, 4, 5 s}, rate or chord; 'any tau (rate)' without the chord probe. "
        "n/a = the plan never showed >= 0.3 at that tau by the des peak.\n")
  rows = []
  for nm, sset in [("0-10 mph", [r for r in results if r["v"] < 10]), ("10-20 mph", [r for r in results if r["v"] >= 10]), ("0-20 mph", results), ("0-20 clean", clean)]:
    if not sset:
      continue
    def qs(vals):
      v_ = np.array(vals, dtype=float)
      ok_ = v_[np.isfinite(v_)]
      return f"{np.median(ok_):.2f} / {np.percentile(ok_, 25):.2f} / {np.percentile(ok_, 10):.2f} (n/a {int((~np.isfinite(v_)).sum())})" if len(ok_) else "n/a"
    rows.append([nm, len(sset), qs([r["t_r0"] - r["tx"][("rate", 0.0)] for r in sset]), qs([r["t_c"] - r["tx"][("rate", 0.0)] for r in sset]),
                 qs([r["t_r0"] - r["tx"][("rate", 1.0)] for r in sset]), qs([r["lead_see_rate"] for r in sset]), qs([r["lead_see"] for r in sset])])
  table(["set", "n", "lead today", "lead_c today", "lead plan(tau0 + 1.0)", "lead any tau (rate)", "lead any tau (rate or chord)"], rows)
  print("Realized advance of a raised look-ahead = plan crossing today - plan crossing of a_plan(tau0 + e) (median / p25 / p10 seconds; n = turn-ins where both exist). The needed Delta (section 3) is "
        "the target. 'share >= 0.8 e' = how often the plan delivers at least 80% of the extension.\n")
  rows = []
  for nm, sset in [("0-10 mph", [r for r in results if r["v"] < 10]), ("10-20 mph", [r for r in results if r["v"] >= 10]), ("0-20 mph", results)]:
    for e in EXTS:
      adv = np.array([r["tx"][("rate", 0.0)] - r["tx"][("rate", e)] for r in sset], dtype=float)
      ok_ = adv[np.isfinite(adv)]
      if not len(ok_):
        continue
      rows.append([nm if e == EXTS[0] else "", f"+{e:.1f}", len(ok_), f"{np.median(ok_):.2f} / {np.percentile(ok_, 25):.2f} / {np.percentile(ok_, 10):.2f}", f"{np.mean(ok_ >= 0.8 * e) * 100:.0f}%"])
  table(["set", "e (s)", "n", "realized advance", "share >= 0.8 e"], rows)
  print("Available anticipation against the needed Delta, per turn-in (needed Delta from section 3.2, rise-only advance with the envelope criterion; turn-ins that need none are dropped; '> 1.5' counts as unmet):\n")
  rows = []
  need = [r for r in results if r["min_delta_rise"] > 0]
  for nm, sset in [("all 0-20 mph", need), ("0-10 mph", [r for r in need if r["v"] < 10]), ("10-20 mph", [r for r in need if r["v"] >= 10]),
                   ("clean", [r for r in need if not r["pressed"] and not r["cens_start"]])]:
    if not sset:
      continue
    nd = np.array([r["min_delta_rise"] for r in sset])
    la = np.array([r["lead_see_rate"] for r in sset], dtype=float)
    lc = np.array([r["lead_see"] for r in sset], dtype=float)
    fin = np.isfinite(nd)
    cov = lambda lead, frac: np.mean([np.isfinite(l_) and l_ >= frac * n_ for l_, n_ in zip(lead, nd)]) * 100
    ladd = lambda lead: f"{np.nanmedian(lead):.2f} / {np.nanpercentile(lead, 10):.2f}" if np.isfinite(lead).any() else "n/a"
    rows.append([nm, len(sset), fmt(np.median(nd[fin]), 1) + f" (> 1.5 s: {int((~fin).sum())})", ladd(la), f"{cov(la, 1.0):.0f}%", f"{cov(la, 0.5):.0f}%", ladd(lc), f"{cov(lc, 1.0):.0f}%"])
  table(["set", "n", "needed Delta median", "anticipation (rate, any tau) median / p10", "plan sees >= Delta", "plan sees >= Delta/2", "anticipation (rate+chord) median / p10", "plan sees >= Delta"], rows)

  # predictive accuracy of the raised look-ahead
  print("How good is the plan as a predictor? For every look-ahead extension e, compare a_plan(t, tau0 + e) with the controller's own feed-forward input a_ff at t + e "
        "(a_ff = actuators.curvature * v^2, i.e. what the command will be e seconds later). Baseline = today's command a_ff(t). Frames in the turn-in windows [i_c - 0.5 s, i_rp], turn space; "
        "error = mean |prediction - a_ff(t + e)| (m/s^2) and share of frames where the plan lands within 0.3 of it.\n")
  rows = []
  for e in EXTS:
    ke = int(round(e / DT))
    ep = eb = 0.0
    wp = wb = nn = 0
    for r in results:
      R = r["R"]
      i = np.arange(max(0, r["i_c"] - 50), r["i_rp"] + 1)
      j = shift_idx(R, i, ke)
      pl_all = ctx.s_pl * R.a_plan(ctx.tau0 + e)[i]
      okm = (j != i) & np.isfinite(pl_all)
      if not okm.any():
        continue
      fut = (np.nan_to_num(R.cc[j]) * R.v[j] ** 2)[okm]
      now = (np.nan_to_num(R.cc[i]) * R.v[i] ** 2)[okm]
      pl = pl_all[okm]
      ep += float(np.abs(pl - fut).sum())
      eb += float(np.abs(now - fut).sum())
      wp += int((np.abs(pl - fut) < 0.3).sum())
      wb += int((np.abs(now - fut) < 0.3).sum())
      nn += int(okm.sum())
    rows.append([f"+{e:.1f}", nn, f"{ep / max(nn, 1):.2f}", f"{eb / max(nn, 1):.2f}", f"{wp / max(nn, 1) * 100:.0f}%", f"{wb / max(nn, 1) * 100:.0f}%"])
  table(["extension e (s)", "frames", "plan(tau0+e) error vs a_ff(t+e)", "today's a_ff(t) error", "plan within 0.3", "today's within 0.3"], rows)

  # ---------------------------------------------------------------- 6 side effects
  print("## 6. Side effects\n")
  print("### 6a. Unwind and S-bends (shifted request, same limiter), turn-ins of 0-20 mph\n")
  rows = []
  sbend = 0
  for r in results:
    R, sg, ip, w0, w1 = r["R"], r["sg"], r["ip"], r["w0"], r["w1"]
    r["sbend"] = False
    hi = min(R.n - 1, r["i_end"] + 300)
    seg = R.seg[ip]
    d = (-sg) * np.nan_to_num(R.des[r["i_end"]:hi + 1])
    if len(d) and np.max(d) > 1.0 and (R.seg[r["i_end"]:hi + 1] == seg).all():
      r["sbend"] = True
      sbend += 1
  print(f"S-bends (opposite lobe with peak > 1 m/s^2 within 3 s of the lobe end): {sbend} of {len(results)} turn-ins ({sbend / len(results) * 100:.0f}%).\n")
  hdr = ["Delta s", "pure shift: early unwind s median (p90)", "turn-ins with > 0.3 s", "S-bend counter-torque s median (p90)", "S-bends with > 0.2 s", "extra torque vs original, median units (p90)",
         "rise-only: early unwind s (p90)", "rise-only: S-bend counter-torque s (p90)", "rise-only: extra torque, median units (p90)"]
  rows = []
  for dsel in (0.2, 0.3, 0.5, 0.8, 1.0, 1.5):
    k = DELTAS.index(dsel)
    res_ = {}
    for key in ("S", "Sr"):
      uw, sbv, lead = [], [], []
      for r in results:
        i0 = r["ip"] - r["w0"]
        D1, S = r["D1"], r[key]
        post = slice(i0, len(D1))
        uw.append(float((((D1[post] - S[k, post]) > MAT) & ~((D1[post] - S[0, post]) > MAT)).sum() * DT))
        lead.append(float(np.max(S[k] - S[0])))
        if r["sbend"]:
          e0 = r["i_end"] - r["w0"]
          sbv.append(float((((S[k, e0:] - D1[e0:]) < -MAT) & ~((S[0, e0:] - D1[e0:]) < -MAT)).sum() * DT))
      res_[key] = (np.array(uw), np.array(sbv), np.array(lead))
    uw, sbv, lead = res_["S"]
    uw2, sbv2, lead2 = res_["Sr"]
    rows.append([f"{dsel:.1f}", f"{np.median(uw):.2f} ({np.percentile(uw, 90):.2f})", f"{np.mean(uw > 0.3) * 100:.0f}%",
                 f"{np.median(sbv):.2f} ({np.percentile(sbv, 90):.2f})" if len(sbv) else "-", f"{np.mean(sbv > 0.2) * 100:.0f}%" if len(sbv) else "-",
                 f"{np.median(lead):.0f} ({np.percentile(lead, 90):.0f})",
                 f"{np.median(uw2):.2f} ({np.percentile(uw2, 90):.2f})", f"{np.median(sbv2):.2f} ({np.percentile(sbv2, 90):.2f})" if len(sbv2) else "-",
                 f"{np.median(lead2):.0f} ({np.percentile(lead2, 90):.0f})"])
  table(hdr, rows)
  print("(Early unwind = frames after the des peak where the shifted sent' is more than 40 units below the driver-clipped request while the unshifted one was not: a literal shift of the whole request "
        "releases the turn early. Counter-torque = in S-bends, after the first lobe ended, sent' pushes against the first turn by more than 40 units beyond the request. These are the costs of a "
        "perfect-foresight shift applied to everything; an advance applied only to the turn-in (look-ahead raised on the rise, not on the unwind) would not have the unwind cost.)\n")

  print("### 6b. Gentle non-tight curves: how big is a shift of Delta in lat accel on frames that were never in trouble?\n")
  print("Frames: lateral active, unpressed, not tight, 0.3 < |des| < 1.5. Share of frames where |des(t + Delta) - des(t)| > X (this is how much earlier a perfect-prediction advance would command than today), by speed band.\n")
  bands = [("0-10 mph", 0, 10), ("10-20 mph", 10, 20), ("20-40 mph", 20, 40), ("40+ mph", 40, 1000)]
  gent, te = {}, {}
  rows = []
  for dsel in (0.2, 0.3, 0.5, 0.8, 1.0):
    k = int(round(dsel / DT))
    row = [f"{dsel:.1f}"]
    for nm, lo, hi in bands:
      cnt = {0.3: 0, 0.5: 0, 1.0: 0}
      tot = 0
      for R in routes:
        m = R.base & ~R.tight & (np.abs(np.nan_to_num(R.des)) > 0.3) & (np.abs(np.nan_to_num(R.des)) < 1.5) & (R.v >= lo * MPH) & (R.v < hi * MPH)
        i = np.flatnonzero(m)
        j = shift_idx(R, i, k)
        ok = j != i
        diff = np.abs(np.nan_to_num(R.des[j]) - np.nan_to_num(R.des[i]))[ok]
        tot += len(diff)
        for x in cnt:
          cnt[x] += int((diff > x).sum())
      gent[(dsel, nm)] = [cnt[x] / max(tot, 1) * 100 for x in (0.3, 0.5, 1.0)]
      row.append(" / ".join(f"{cnt[x] / max(tot, 1) * 100:.0f}%" for x in (0.3, 0.5, 1.0)))
    rows.append(row)
  table(["Delta s"] + [b[0] + " (X = 0.3 / 0.5 / 1.0 m/s^2)" for b in bands], rows)

  print("### 6c. Tracking error on non-tight frames if the look-ahead were raised at low speed only\n")
  print("Frames: lateral active, unpressed, not tight (de-facto gentle and straight driving), by band. Baseline error = |des - act| as logged. "
        "Shift model: the car's response moves e earlier, act'(t) = act(t + e), measured against the unchanged reference des(t) (perfect plan; upper bound of the earliness cost). "
        f"Plan model: act'(t) = act(t) + [a_plan(t - d, tau0 + e) - a_plan(t - d, tau0)] with d = {ctx.d:.2f} s (what the plan actually says; plan in the des sign convention). "
        "Mean |error| m/s^2 / share > 0.3 m/s^2.\n")
  dk = int(round(ctx.d / DT))
  hdr = ["band", "baseline"] + [f"shift +{e:.1f}" for e in (0.2, 0.4, 0.8)] + [f"plan +{e:.1f}" for e in (0.2, 0.4, 0.8, 1.5)]
  rows = []
  for nm, lo, hi in bands:
    acc_ = defaultdict(lambda: [0.0, 0, 0])
    for R in routes:
      m = R.base & ~R.tight & np.isfinite(R.des) & np.isfinite(R.act) & (R.v >= lo * MPH) & (R.v < hi * MPH)
      i = np.flatnonzero(m)
      if not len(i):
        continue
      e0 = np.abs(R.des[i] - R.act[i])
      a = acc_["base"]
      a[0] += e0.sum(); a[1] += int((e0 > 0.3).sum()); a[2] += len(e0)
      for e in (0.2, 0.4, 0.8):
        j = shift_idx(R, i, int(round(e / DT)))
        ok = j != i
        er = np.abs(R.des[i][ok] - R.act[j][ok])
        a = acc_[f"s{e}"]
        a[0] += er.sum(); a[1] += int((er > 0.3).sum()); a[2] += len(er)
      for e in (0.2, 0.4, 0.8, 1.5):
        ii = np.maximum(i - dk, 0)
        ok = np.isfinite(R.a_plan(ctx.tau0 + e)[ii]) & np.isfinite(R.a_plan(ctx.tau0)[ii]) & (R.seg[ii] == R.seg[i])
        inc_des = np.where(ok, (R.a_plan(ctx.tau0 + e)[ii] - R.a_plan(ctx.tau0)[ii]) * ctx.s_pl * ctx.s_cu, 0.0)
        er = np.abs(R.des[i] - (R.act[i] + inc_des))
        a = acc_[f"p{e}"]
        a[0] += er.sum(); a[1] += int((er > 0.3).sum()); a[2] += len(er)
    def cell(key):
      s_, c_, n_ = acc_[key]
      te[(nm, key)] = (s_ / max(n_, 1), c_ / max(n_, 1) * 100)
      return f"{s_ / max(n_, 1):.3f} / {c_ / max(n_, 1) * 100:.1f}%"
    rows.append([nm, cell("base")] + [cell(f"s{e}") for e in (0.2, 0.4, 0.8)] + [cell(f"p{e}") for e in (0.2, 0.4, 0.8, 1.5)])
  table(hdr, rows)

  # ---------------------------------------------------------------- 7 per-turn-in top table
  print("## 7. Hardest turn-ins (0-20 mph, peak request >= 250, highest rise-only min Delta first)\n")
  rows = []
  for r in sorted([x for x in results if x["pk"] >= 250], key=lambda x: (-(x["min_delta_rise"] if np.isfinite(x["min_delta_rise"]) else 9), -x["pk"]))[:25]:
    rows.append([r["route"], f"{r['t_c']:.0f}", f"{r['v']:.0f}", f"{r['radius']:.0f}", f"{r['des_pk']:.2f}", f"{r['pk']:.0f}", fmt(r["t_rp"] - r["t_c"]), fmt(r["t_sp"] - r["t_c"]) if np.isfinite(r["t_sp"]) else "never",
                 fmt(r["need_ramp"]), fmt(r["min_delta"], 1) + " / " + fmt(r["min_delta_rise"], 1), fmt(r["t_rp"] - r["t_r0"]), fmt(r["lead_see"]), "press" if r["pressed"] else ""])
  table(["route", "route t s", "mph", "radius m", "peak des", "peak req", "t_rp - t_c", "t_sp - t_c", "ramp needed s", "min Delta shift / rise-only", "request ramp s", "plan lead any tau s", ""], rows)
  summary(results, clean, rem_by_var, acnt, nal, names, gent, te, ctx, behind_total)


if __name__ == "__main__":
  main()
