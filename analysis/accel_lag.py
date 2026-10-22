#!/usr/bin/env python3
"""Command -> aEgo lag of the Stinger on openpilot-longitudinal drives (the basis of longitudinalActuatorDelay).

Takes the accel openpilot actually sent (SCC12 aReqValue from sendcan bus 0, 50 Hz; --carcontrol uses carControl
actuators.accel instead) and carState.aEgo, resampled to 100 Hz. Only frames with longActive, no gas/brake pedal and
vEgo > 5 m/s count; both signals are high-passed (minus a 5 s moving average) so grade and drag offsets do not decide
the fit, and gaps longer than 0.5 s break the timeline. Two numbers per drive and overall:
  xcorr lag   the shift of the command that maximises its correlation with aEgo
  delay+tau   the best pure delay d and first-order time constant tau of aEgo(t) = LP_tau(cmd(t - d)) (grid search,
              least squares); d + tau is the total command -> aEgo lag, d the dead time alone
longitudinalActuatorDelay is a planner look-ahead, so it stands for the total lag of the car's own response. The
sent accel already contains the tune's braking smoothing (none in tune A, "today"); a drive on a smoothing tune
therefore shows the car's lag plus the tune's, so use drives on the installed tune for the car's own figure.

Usage: accel_lag.py [ROUTE ...] [--carcontrol] [--starpilot DIR]
"""
import argparse
import sys

import numpy as np

from logs import TUNE_DIR, add_starpilot_path, pmap, segment_files, short, sorted_events

DEFAULT_ROUTES = ["0000000d--5e0d1d244b", "0000000e--5ebf4636a1"]
DT = 0.01
MAX_GAP = 0.5
MIN_SPEED = 5.0
HP_WINDOW = 5.0
LAGS = np.arange(0.0, 1.01, 0.05)
TAUS = np.arange(0.05, 0.81, 0.05)


def scan(arg):
  """(t_cmd, cmd, t_state, aEgo, vEgo, ok) arrays for one segment, ok = longActive and no pedal."""
  path, use_cc = arg
  from opendbc.can import CANParser  # imported before the first log read, so logs.py reuses its schema
  p = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
  cmd = []; st = []; ok = False; pedal = False
  for e in sorted_events(path):
    w = e.which(); t = e.logMonoTime / 1e9
    if w == "sendcan" and not use_cc:
      frames = [(c.address, bytes(c.dat), c.src) for c in e.sendcan]
      if any(a == 1057 and s == 0 for a, _, s in frames):
        p.update([(e.logMonoTime, frames)])
        cmd.append((t, p.vl["SCC12"]["aReqValue"]))
    elif w == "carControl":
      ok = bool(e.carControl.longActive and not e.carControl.cruiseControl.override)
      if use_cc:
        cmd.append((t, e.carControl.actuators.accel))
    elif w == "carState":
      cs = e.carState
      pedal = bool(cs.gasPressed or cs.brakePressed)
      st.append((t, cs.aEgo, cs.vEgo, float(ok and not pedal)))
  return np.array(cmd).reshape(-1, 2), np.array(st).reshape(-1, 4)


def highpass(x):
  k = int(HP_WINDOW / DT)
  c = np.cumsum(np.r_[0, x]); n = len(x)
  lo = np.clip(np.arange(n) - k // 2, 0, n); hi = np.clip(np.arange(n) + k // 2 + 1, 0, n)
  return x - (c[hi] - c[lo]) / (hi - lo)


def lowpass(x, tau):
  a = DT / (tau + DT); y = np.empty_like(x); acc = x[0]
  for i, v in enumerate(x):
    acc += a * (v - acc); y[i] = acc
  return y


def chunks(cmd, st):
  """Contiguous 100 Hz chunks: (cmd, aEgo) where the state is ok, fast enough and the command is fresh."""
  out = []
  if len(cmd) < 10 or len(st) < 10:
    return out
  t = st[:, 0]
  good = (st[:, 3] == 1) & (st[:, 2] > MIN_SPEED)
  gaps = np.flatnonzero(np.diff(t) > MAX_GAP) + 1
  for a, b in zip(np.r_[0, gaps], np.r_[gaps, len(t)]):
    sel = good[a:b]
    if sel.sum() < 500:
      continue
    # runs of ok frames
    idx = np.flatnonzero(np.diff(np.r_[0, sel.astype(int), 0]))
    for s, e in zip(idx[::2], idx[1::2]):
      ts = t[a + s:a + e]
      if ts[-1] - ts[0] < 10:
        continue
      grid = np.arange(ts[0], ts[-1], DT)
      c = np.interp(grid, cmd[:, 0], cmd[:, 1]); ae = np.interp(grid, ts, st[a + s:a + e, 1])
      out.append((c, ae))
  return out


def fit(chs):
  """(xcorr lag, best d, best tau, rms, n) over chunks."""
  hp = [(highpass(c), highpass(a)) for c, a in chs]
  n = sum(len(c) for c, _ in hp)
  corr = []
  for lag in LAGS:
    k = int(round(lag / DT)); num = den1 = den2 = 0.0
    for c, a in hp:
      cc, aa = (c[:len(c) - k] if k else c), a[k:]
      num += float(cc @ aa); den1 += float(cc @ cc); den2 += float(aa @ aa)
    corr.append(num / np.sqrt(den1 * den2))
  best = (np.inf, 0, 0)
  for tau in TAUS:
    lp = [(lowpass(c, tau), a) for c, a in chs]
    for d in LAGS:
      k = int(round(d / DT)); err = 0.0
      for (c, a), (l, _) in zip(hp, lp):
        pred = highpass(l)
        pr = pred[:len(pred) - k] if k else pred
        err += float(np.sum((pr - a[k:]) ** 2))
      if err < best[0]:
        best = (err, d, tau)
  return LAGS[int(np.argmax(corr))], best[1], best[2], np.sqrt(best[0] / n), n, max(corr)


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="*", default=DEFAULT_ROUTES, help="openpilot-long drives with rlogs (default: %(default)s)")
  p.add_argument("--carcontrol", action="store_true", help="use carControl actuators.accel instead of the sent SCC12 aReqValue")
  p.add_argument("--starpilot", default=TUNE_DIR, help="checkout whose opendbc decodes SCC12 (default: %(default)s)")
  args = p.parse_args()
  add_starpilot_path(args.starpilot)
  allch = []
  print(f"{'drive':22s} {'seconds':>8s} {'xcorr lag':>10s} {'corr':>6s} {'delay d':>8s} {'tau':>6s} {'d+tau':>6s} {'rms':>6s}")
  for route in args.routes:
    files = segment_files(route, "rlog")
    if not files:
      print(f"{short(route)}: no rlogs"); continue
    chs = []
    for cmd, st in pmap(scan, [(f, args.carcontrol) for f in files]):
      chs += chunks(cmd, st)
    if not chs:
      print(f"{short(route)}: no usable frames"); continue
    allch += chs
    lag, d, tau, rms, n, c = fit(chs)
    print(f"{short(route):22s} {n * DT:8.0f} {lag:9.2f}s {c:6.2f} {d:7.2f}s {tau:5.2f}s {d + tau:5.2f}s {rms:6.3f}")
  if len(args.routes) > 1 and allch:
    lag, d, tau, rms, n, c = fit(allch)
    print(f"{'all':22s} {n * DT:8.0f} {lag:9.2f}s {c:6.2f} {d:7.2f}s {tau:5.2f}s {d + tau:5.2f}s {rms:6.3f}")


if __name__ == "__main__":
  sys.exit(main())
