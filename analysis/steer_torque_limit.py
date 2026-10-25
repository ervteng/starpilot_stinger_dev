#!/usr/bin/env python3
"""Real steering torque limit of the Stinger: factory LKAS request, openpilot request vs EPS response, achieved lateral accel.

Hyundai CAN: LKAS11 CR_Lkas_StrToqReq (request, +-STEER_MAX = 384 for KIA_STINGER_2022) with CF_Lkas_ActToi; the EPS answers in
MDPS12 CR_Mdps_OutTq (carState.steeringTorqueEps, DBC-scaled, 0.1/LSB), CR_Mdps_StrColTq (driver column torque),
CF_Mdps_ToiActive / ToiUnavail / ToiFlt. Buses as found in the rlogs: camera LKAS11 = can src 2, openpilot's own sent LKAS11 =
can src 128 (echo of sendcan bus 0), MDPS12 = src 0. Only openpilot's frame ever reaches bus 0 while it sends; the camera's
frame on bus 2 is what the factory system *would* request.

Sections:
  1. Factory LKAS (camera LKAS11, src 2) on the rlog routes given with --stock: samples where CF_Lkas_ActToi = 1 or the camera
     torque is non-zero, |request| max / p99.9 / p99 / p95, top values, max by speed band, EPS OutTq. Also reports whether the
     factory camera ever requested torque at all and whether openpilot lateral was active in the same drives (it matters).
  2. openpilot's sent request (src 128, lateral active = carControl.latActive and ActToi = 1, v > 2 m/s): distribution, share at /
     within 2% of 384, steady-state EPS response (|OutTq| vs request in 32-unit bins; steady = request range <= 24 over the last
     0.3 s, not steeringPressed, ToiActive), EPS faults (ToiUnavail / ToiFlt / steerFaultTemporary) overall and at high requests.
  3. Pegged torque controller (controlsState.lateralControlState.torqueState, |output| >= 0.98 same sign for >= 0.5 s): achieved
     actual lateral accel (signed with the output) by speed band; saturation share by speed and |desired| band; accel-vs-output
     curve (actual accel per unit output) vs LAT_ACCEL_FACTOR. Lateral-active samples only, steeringPressed excluded, v > 2 m/s.
     Run on rlog routes (100 Hz) and optionally qlog routes (10 Hz, --qlog).
  4. liveTorqueParameters per qlog route: first/last valid latAccelFactorFiltered, frictionCoefficientFiltered, totalBucketPoints,
     calPerc, useParams, and the trend (least-squares slope per hour and first vs last quarter means of the valid samples).

Usage: steer_torque_limit.py [--stock ROUTES] [--rlog ROUTES] [--qlog ROUTES] [--workers N]
  defaults are the Stinger drives listed in DEFAULT_*; output is markdown on stdout.
"""
import argparse
import re

import numpy as np

from logs import add_starpilot_path, events, pmap, segment_files, seg_num, short

STOCK = ["0000001d--2ead59d2dd", "00000028--297af64c9c", "00000029--b3c2c13819", "00000034--5f8605e608"]
OPLAT = ["0000000d--5e0d1d244b", "0000000e--5ebf4636a1", "00000015--2b757ccb00"]
QLOGS = ["0000000f--79cf51e7fa", "00000011--c7d29db317", "00000012--2aada6ce32", "00000013--6677db1144",
         "00000014--62bceb20db", "00000015--2b757ccb00", "00000016--1fe83bd745", "00000017--0c40138e36"]
STEER_MAX = 384
CFG_FACTOR, CFG_MAX, CFG_FRIC = 2.7499, 1.8497, 0.1205          # params.toml KIA_STINGER
USER_FACTOR, USER_FRIC = 2.789, 0.113                           # live settings SteerLatAccel / SteerFriction
MPH = 0.44704
BANDS = [0, 20, 40, 60, 1000]
BNAMES = ["0-20", "20-40", "40-60", "60+"]
MIN_V = 2.0
PEG = 0.98
PEG_S = 0.5
CAN_COLS = ["t", "v", "lat", "pressed", "camTq", "camAct", "sentTq", "sentAct", "epsOut", "epsCol", "toiAct", "toiUna", "toiFlt",
            "fault", "des", "act", "out"]
CTL_COLS = ["t", "v", "lat", "pressed", "fault", "des", "act", "out", "sat"]
CI = {n: i for i, n in enumerate(CAN_COLS)}
TI = {n: i for i, n in enumerate(CTL_COLS)}


def scan_rlog(path):
  """Per segment: rows at each can event (CAN_COLS, with latest carState/carControl/torqueState held) and at each controlsState."""
  add_starpilot_path('/home/openpilotstuff/wt/stinger-combined')
  from opendbc.can import CANParser
  cam = CANParser("hyundai_kia_generic", [("LKAS11", 2)], 2)
  snt = CANParser("hyundai_kia_generic", [("LKAS11", 128)], 128)
  eps = CANParser("hyundai_kia_generic", [("MDPS12", 0)], 0)
  s = dict(v=0.0, lat=0.0, pressed=0.0, fault=0.0, des=np.nan, act=np.nan, out=np.nan, sat=0.0)
  can, ctl = [], []
  bus0_lkas = 0
  for e in sorted(events(path), key=lambda e: e.logMonoTime):
    w = e.which()
    t = e.logMonoTime * 1e-9
    if w == "carState":
      c = e.carState
      s["v"], s["pressed"], s["fault"] = c.vEgo, float(c.steeringPressed), float(c.steerFaultTemporary)
    elif w == "carControl":
      s["lat"] = float(e.carControl.latActive)
    elif w == "controlsState":
      ls = e.controlsState.lateralControlState
      if ls.which() == "torqueState":
        q = ls.torqueState
        s["des"], s["act"], s["out"], s["sat"] = q.desiredLateralAccel, q.actualLateralAccel, q.output, float(q.saturated)
        ctl.append((t, s["v"], s["lat"], s["pressed"], s["fault"], s["des"], s["act"], s["out"], s["sat"]))
    elif w == "can":
      fr = [(c.address, bytes(c.dat), c.src) for c in e.can]
      bus0_lkas += sum(1 for a, _, b in fr if a == 832 and b == 0)
      for p in (cam, snt, eps):
        p.update([(e.logMonoTime, fr)])
      a, b, m = cam.vl["LKAS11"], snt.vl["LKAS11"], eps.vl["MDPS12"]
      can.append((t, s["v"], s["lat"], s["pressed"], a["CR_Lkas_StrToqReq"], a["CF_Lkas_ActToi"], b["CR_Lkas_StrToqReq"],
                  b["CF_Lkas_ActToi"], m["CR_Mdps_OutTq"], m["CR_Mdps_StrColTq"], m["CF_Mdps_ToiActive"], m["CF_Mdps_ToiUnavail"],
                  m["CF_Mdps_ToiFlt"], s["fault"], s["des"], s["act"], s["out"]))
  return np.array(can, dtype=np.float64).reshape(-1, len(CAN_COLS)), np.array(ctl, dtype=np.float64).reshape(-1, len(CTL_COLS)), bus0_lkas


def scan_qlog(path):
  """controlsState torqueState rows (qlog, 10 Hz) and liveTorqueParameters rows."""
  s = dict(v=0.0, lat=0.0, pressed=0.0, fault=0.0)
  ctl, ltp = [], []
  for e in sorted(events(path), key=lambda e: e.logMonoTime):
    w = e.which()
    t = e.logMonoTime * 1e-9
    if w == "carState":
      c = e.carState
      s["v"], s["pressed"], s["fault"] = c.vEgo, float(c.steeringPressed), float(c.steerFaultTemporary)
    elif w == "carControl":
      s["lat"] = float(e.carControl.latActive)
    elif w == "controlsState":
      ls = e.controlsState.lateralControlState
      if ls.which() == "torqueState":
        q = ls.torqueState
        ctl.append((t, s["v"], s["lat"], s["pressed"], s["fault"], q.desiredLateralAccel, q.actualLateralAccel, q.output,
                    float(q.saturated)))
    elif w == "liveTorqueParameters":
      p = e.liveTorqueParameters
      ltp.append((t, float(p.liveValid), p.latAccelFactorFiltered, p.frictionCoefficientFiltered, p.latAccelFactorRaw,
                  p.frictionCoefficientRaw, p.totalBucketPoints, float(p.calPerc), float(p.useParams)))
  return (np.array(ctl, dtype=np.float64).reshape(-1, len(CTL_COLS)),
          np.array(ltp, dtype=np.float64).reshape(-1, 9))


def load(route, kind):
  files = segment_files(route, kind)
  fn = scan_rlog if kind == "rlog" else scan_qlog
  return files, pmap(fn, files, 10)


def band(v):
  return np.digitize(v / MPH, BANDS[1:-1])


def pct(x, qs=(50, 90, 95, 99, 99.9, 100)):
  return [float(np.percentile(x, q)) for q in qs] if len(x) else [np.nan] * len(qs)


def table(head, rows):
  print("| " + " | ".join(head) + " |")
  print("|" + "|".join(["---"] * len(head)) + "|")
  for r in rows:
    print("| " + " | ".join(str(c) for c in r) + " |")
  print()


def f(x, n=1):
  return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{n}f}"


def concat(res, idx):
  return np.concatenate([r[idx] for r in res]) if res else np.zeros((0, 1))


# ---------------------------------------------------------------------------------------------------------------- section 1
def section1(stock, loaded):
  print("## 1. Factory LKAS request (camera LKAS11, can src 2)\n")
  rows = []
  allcam = []
  for r in stock:
    files, res = loaded[r]
    can = concat(res, 0)
    bus0 = sum(x[2] for x in res)
    n = len(can)
    camact = can[:, CI["camAct"]] == 1
    camnz = can[:, CI["camTq"]] != 0
    lat = can[:, CI["lat"]] == 1
    sent_act = can[:, CI["sentAct"]] == 1
    rows.append([short(r), len(files), n, f"{lat.mean() * 100:.0f}%", f"{sent_act.mean() * 100:.0f}%", int(camact.sum()), int(camnz.sum()),
                 f(np.abs(can[:, CI["camTq"]]).max(), 0), bus0])
    allcam.append(can)
  table(["route", "rlog segs", "can samples", "openpilot latActive", "openpilot LKAS11 ActToi=1", "camera ActToi=1 samples",
         "camera torque != 0 samples", "max |camera StrToqReq|", "camera LKAS11 frames seen on bus 0"], rows)
  can = np.concatenate(allcam)
  m = (can[:, CI["camAct"]] == 1) | (can[:, CI["camTq"]] != 0)
  print(f"Factory-steering samples (camera ActToi = 1 or camera torque != 0): {int(m.sum())} of {len(can)}.")
  mm = can[:, CI["camAct"]] == 1
  if mm.any():
    print(f"Camera ActToi = 1 samples all have torque {sorted(set(np.abs(can[mm, CI['camTq']]).tolist()))} and EPS ToiActive in them: "
          f"{can[mm, CI['toiAct']].mean() * 100:.0f}% (openpilot latActive in them: {can[mm, CI['lat']].mean() * 100:.0f}%).")
  if not m.any() or np.abs(can[m, CI["camTq"]]).max() == 0:
    print("\n**No factory LKAS torque request exists in these logs**: the camera's LKAS11 frame is alive (counter increments) but its "
          "CR_Lkas_StrToqReq is 0 in every sample. In all four 'stock' drives openpilot lateral was active for most of the time and "
          "was the sender of the torque that the EPS acted on (EPS ToiActive = 1 follows openpilot's ActToi, not the camera's). "
          "The factory maximum (<384 or not) therefore cannot be measured from these logs; sections 2 and 3 are the proxy.\n")
  else:
    a = np.abs(can[m, CI["camTq"]])
    print("\n" + " ".join(f"{k}={v:.0f}" for k, v in zip(["p50", "p90", "p95", "p99", "p99.9", "max"], pct(a))))


# ---------------------------------------------------------------------------------------------------------------- section 2
def steady_mask(sent, t, win=0.3, tol=24):
  """True where the request moved <= tol over the preceding win seconds (vectorised on a ~100 Hz series)."""
  from numpy.lib.stride_tricks import sliding_window_view
  k = max(2, int(round(win / np.median(np.diff(t)))))
  mx = np.full(len(sent), np.nan); mn = np.full(len(sent), np.nan)
  if len(sent) > k:
    w = sliding_window_view(sent, k)
    mx[k - 1:] = w.max(1); mn[k - 1:] = w.min(1)
  return (mx - mn) <= tol


def section2(routes, loaded):
  print("## 2. openpilot's request and the EPS response (rlog routes)\n")
  per, pool, esteady = [], [], []
  fault_rows = []
  for r in routes:
    files, res = loaded[r]
    allr = []
    for c, _, _ in res:
      if len(c) < 100:
        continue
      act = (c[:, CI["lat"]] == 1) & (c[:, CI["sentAct"]] == 1) & (c[:, CI["v"]] > MIN_V)
      st = steady_mask(c[:, CI["sentTq"]], c[:, CI["t"]])
      sg = np.sign(c[:, CI["sentTq"]])
      ok = act & st & (c[:, CI["pressed"]] == 0) & (c[:, CI["toiAct"]] == 1)
      esteady.append(np.column_stack([c[ok, CI["sentTq"]], c[ok, CI["epsOut"]] * sg[ok], c[ok, CI["v"]]]))
      allr.append(c)
      pool.append(c[act])
    if not allr:
      continue
    c = np.concatenate(allr)
    act = (c[:, CI["lat"]] == 1) & (c[:, CI["sentAct"]] == 1) & (c[:, CI["v"]] > MIN_V)
    a = np.abs(c[act, CI["sentTq"]])
    per.append([short(r), len(files), int(act.sum()), f"{act.sum() / 100 / 60:.0f}"] + [f(x, 0) for x in pct(a)] +
               [f((a >= 0.98 * STEER_MAX).mean() * 100, 2), f((a >= STEER_MAX).mean() * 100, 2)])
  table(["route", "segs", "lat-active samples", "lat-active min", "p50", "p90", "p95", "p99", "p99.9", "max", "% >= 376 (98%)", "% = 384"], per)
  P = np.concatenate(pool)
  a = np.abs(P[:, CI["sentTq"]])
  print(f"Pooled: {len(a)} lateral-active samples ({len(a) / 100 / 3600:.2f} h). Share at/within 2% of 384 (>= 376.3): "
        f"{(a >= 0.98 * STEER_MAX).mean() * 100:.2f}% ; exactly 384: {(a >= STEER_MAX).mean() * 100:.2f}%. "
        f"Sample counts: |req| >= 300: {(a >= 300).sum()}, >= 350: {(a >= 350).sum()}, >= 380: {(a >= 380).sum()}.\n")
  print("Top sent values (|StrToqReq|) in lateral-active samples:\n")
  vals, cnt = np.unique(a, return_counts=True)
  top = np.argsort(vals)[::-1][:8]
  table(["|request|", "samples"], [[f"{vals[i]:.0f}", int(cnt[i])] for i in top])
  print("Max |request| by speed band:\n")
  b = band(P[:, CI["v"]])
  table(["band mph", "samples", "max", "p99.9", "p99", "% >= 376"],
        [[BNAMES[i], int((b == i).sum())] + [f(x, 0) for x in (a[b == i].max() if (b == i).any() else np.nan,
                                                                  *pct(a[b == i], (99.9, 99)))] +
         [f((a[b == i] >= 0.98 * STEER_MAX).mean() * 100, 2) if (b == i).any() else "-"] for i in range(4)])

  E = np.concatenate(esteady)
  print(f"EPS response, steady-state samples (request range <= 24 over 0.3 s, ToiActive, no driver press, v > {MIN_V} m/s): "
        f"{len(E)} samples. OutTq is signed with the request (so positive = EPS output follows request); units are DBC-scaled MDPS12 "
        f"CR_Mdps_OutTq. Gain = median OutTq / bin-mean request.\n")
  req, out = np.abs(E[:, 0]), E[:, 1]
  edges = list(range(0, 385, 32)) + [385]
  rows = []
  for lo, hi in zip(edges[:-1], edges[1:]):
    mk = (req >= lo) & (req < hi) if hi <= STEER_MAX else (req >= lo)
    if mk.sum() < 5:
      rows.append([f"{lo}-{min(hi, 384)}", int(mk.sum())] + ["-"] * 5)
      continue
    p10, p50, p90 = np.percentile(out[mk], [10, 50, 90])
    rows.append([f"{lo}-{min(hi, 384)}", int(mk.sum()), f(req[mk].mean(), 0), f(p50, 2), f"{p10:.2f} .. {p90:.2f}",
                 f"{p50 / req[mk].mean():.4f}" if req[mk].mean() > 0 else "-", f(np.median(np.abs(E[mk, 2])) / MPH, 0)])
  table(["request bin", "n", "mean request", "median OutTq", "p10 .. p90", "gain (OutTq/request)", "median mph"], rows)
  for lo, hi, name in ((20, 200, "20-200"), (200, 320, "200-320"), (320, 385, "320-384")):
    mk = (req >= lo) & (req < hi)
    if mk.sum() > 20:
      slope = float(np.sum(req[mk] * out[mk]) / np.sum(req[mk] ** 2))
      lin = np.polyfit(req[mk], out[mk], 1)
      print(f"- request {name}: through-origin gain {slope:.4f}, fit OutTq = {lin[0]:.4f} * request {lin[1]:+.2f} (n={mk.sum()})")
  print()

  print("EPS faults at lateral-active samples (any route above):\n")
  Pall = P
  hi = np.abs(Pall[:, CI["sentTq"]]) >= 300
  def rate(mask, col):
    return f"{int(Pall[mask, CI[col]].astype(bool).sum())} / {int(mask.sum())}"
  everyone = np.ones(len(Pall), bool)
  rows = []
  for name, mask in (("all lateral-active", everyone), ("|request| >= 300", hi), ("|request| >= 376", np.abs(Pall[:, CI["sentTq"]]) >= 376)):
    rows.append([name, rate(mask, "toiUna"), rate(mask, "toiFlt"), rate(mask, "fault")])
  table(["condition", "CF_Mdps_ToiUnavail samples", "CF_Mdps_ToiFlt samples", "carState.steerFaultTemporary samples"], rows)
  # rising edges of faults and request just before them, per segment on the unfiltered series
  edges_n, pre = 0, []
  for r in routes:
    for c, _, _ in loaded[r][1]:
      if len(c) < 100:
        continue
      fl = (c[:, CI["toiUna"]] == 1) | (c[:, CI["toiFlt"]] == 1) | (c[:, CI["fault"]] == 1)
      rise = np.where(fl[1:] & ~fl[:-1])[0] + 1
      for i in rise:
        if c[i, CI["lat"]] == 1:
          edges_n += 1
          w = c[max(0, i - 50):i, CI["sentTq"]]
          pre.append((np.abs(w).max() if len(w) else 0, c[i, CI["v"]] / MPH))
  print(f"Fault episodes starting while lateral active: {edges_n}.")
  if pre:
    pa = np.array(pre)
    print("Max |request| in the 0.5 s before each: " + ", ".join(f"{x:.0f}" for x in pa[:20, 0]) +
          f" ... (median {np.median(pa[:, 0]):.0f}); speeds mph: " + ", ".join(f"{x:.0f}" for x in pa[:20, 1]))
  print()


# ---------------------------------------------------------------------------------------------------------------- section 3
def peg_runs(ctl, kind_hz):
  """Runs where |output| >= PEG with a constant sign for >= PEG_S, within lateral-active, unpressed, v > MIN_V samples."""
  t = ctl[:, TI["t"]]
  out = ctl[:, TI["out"]]
  ok = (ctl[:, TI["lat"]] == 1) & (ctl[:, TI["pressed"]] == 0) & (ctl[:, TI["v"]] > MIN_V) & (np.abs(out) >= PEG)
  sgn = np.sign(out)
  # torqueState actual/desired lateral accel carry the opposite sign of the output (positive output = steer left = negative accel),
  # so "achieved" accel in the direction of the request is -actual * sign(output)
  runs = []
  i, n = 0, len(ctl)
  while i < n:
    if not ok[i]:
      i += 1
      continue
    j = i
    while j + 1 < n and ok[j + 1] and sgn[j + 1] == sgn[i] and t[j + 1] - t[j] < 0.5:
      j += 1
    if t[j] - t[i] >= PEG_S:
      sl = slice(i, j + 1)
      s = sgn[i]
      runs.append((t[i], t[j] - t[i], np.mean(ctl[sl, TI["v"]]), -np.median(ctl[sl, TI["act"]]) * s, np.max(-ctl[sl, TI["act"]] * s),
                   -np.median(ctl[sl, TI["des"]]) * s, s))
    i = j + 1
  return runs


def section3(routes, loaded, label):
  print(f"## 3. Torque controller saturation and achieved lateral accel ({label})\n")
  allc, runs = [], []
  per = []
  for r in routes:
    files, res = loaded[r]
    ctl = np.concatenate([x[0] for x in res]) if label.startswith("qlog") else np.concatenate([x[1] for x in res])
    keep = []
    rr = []
    for x in res:
      c = x[0] if label.startswith("qlog") else x[1]
      if len(c) > 10:
        rr += peg_runs(c, 0)
    runs += [(r,) + q for q in rr]
    m = (ctl[:, TI["lat"]] == 1) & (ctl[:, TI["pressed"]] == 0) & (ctl[:, TI["v"]] > MIN_V)
    allc.append(ctl[m])
    a = np.abs(ctl[m, TI["out"]])
    dt = np.median(np.diff(ctl[:, TI["t"]])) if len(ctl) > 1 else 0.01
    per.append([short(r), len(files), int(m.sum()), f"{m.sum() * dt / 60:.0f}", f((a >= PEG).mean() * 100, 2),
                f(ctl[m, TI["sat"]].mean() * 100, 2), len(rr), f(sum(q[1] for q in rr), 0)])
  table(["route", "segs", "lat-active samples", "lat-active min", "% |out| >= 0.98", "% saturated flag", "peg runs >= 0.5 s",
         "peg run seconds"], per)
  C = np.concatenate(allc)
  b = band(C[:, TI["v"]])
  a = np.abs(C[:, TI["out"]])
  des = np.abs(C[:, TI["des"]])
  print("Saturation by speed band (pooled, lateral-active, unpressed):\n")
  rows = []
  for i in range(4):
    m = b == i
    rows.append([BNAMES[i], int(m.sum()), f((a[m] >= PEG).mean() * 100, 2) if m.any() else "-",
                 f(C[m, TI["sat"]].mean() * 100, 2) if m.any() else "-", f(np.percentile(des[m], 99), 2) if m.any() else "-",
                 f(des[m].max(), 2) if m.any() else "-"])
  table(["band mph", "samples", "% pegged", "% sat flag", "|desired| p99", "|desired| max"], rows)
  print("Pegged samples by |desired| lateral accel (m/s^2):\n")
  rows = []
  for lo, hi in ((0, 0.5), (0.5, 1), (1, 1.5), (1.5, 2), (2, 2.5), (2.5, 3), (3, 99)):
    m = (des >= lo) & (des < hi)
    rows.append([f"{lo}-{hi if hi < 99 else '+'}", int(m.sum()), f((a[m] >= PEG).mean() * 100, 1) if m.any() else "-",
                 f(np.median(np.abs(C[m, TI['act']])), 2) if m.any() else "-"])
  table(["|desired|", "samples", "% pegged", "median |actual|"], rows)

  print(f"Peg runs (|output| >= {PEG} same sign >= {PEG_S} s): actual lateral accel signed with the output, by mean speed of the run.\n")
  R = np.array([q[1:] for q in runs]) if runs else np.zeros((0, 7))
  # columns: t0, dur, v, med_act, max_act, med_des, sign
  rows = []
  for i in range(4):
    m = band(R[:, 2]) == i if len(R) else np.zeros(0, bool)
    if m.sum() == 0:
      rows.append([BNAMES[i], 0] + ["-"] * 6)
      continue
    rows.append([BNAMES[i], int(m.sum()), f(R[m, 1].sum(), 0), f(np.median(R[m, 3]), 2), f(np.percentile(R[m, 3], 90), 2),
                 f(R[m, 3].max(), 2), f(R[m, 4].max(), 2), f(np.median(R[m, 5]), 2)])
  table(["band mph", "runs", "total s", "median of run-median actual", "p90", "max run-median", "max instantaneous", "median desired"], rows)
  if len(R):
    for sd, nm in ((1, "+"), (-1, "-")):
      m = R[:, 6] == sd
      if m.sum():
        print(f"- output sign {nm}: {int(m.sum())} runs, median actual {np.median(R[m, 3]):.2f}, p90 {np.percentile(R[m, 3], 90):.2f}, "
              f"max {R[m, 3].max():.2f} m/s^2")
    print()

  # accel vs output
  print("Achieved lateral accel in the direction of the request (-actual * sign(output)) per unit normalized output; binned medians, "
        "lateral-active, unpressed, v > 8 m/s, |desired - actual| < 0.5:\n")
  C = C.copy()
  C[:, TI["act"]] *= -1; C[:, TI["des"]] *= -1                   # positive = accel in the direction of positive output
  sgo = np.sign(C[:, TI["out"]])
  m0 = (C[:, TI["v"]] > 8) & (np.abs(C[:, TI["des"]] - C[:, TI["act"]]) < 0.5)
  rows, xs, ys = [], [], []
  for lo, hi in ((0.1, 0.2), (0.2, 0.3), (0.3, 0.4), (0.4, 0.5), (0.5, 0.6), (0.6, 0.7), (0.7, 0.8), (0.8, 0.9), (0.9, 0.98), (0.98, 1.01)):
    m = m0 & (a >= lo) & (a < hi)
    if m.sum() < 20:
      rows.append([f"{lo}-{hi}", int(m.sum()), "-", "-", "-"])
      continue
    act = C[m, TI["act"]] * sgo[m]
    xs.append(a[m].mean()); ys.append(np.median(act))
    rows.append([f"{lo}-{hi}", int(m.sum()), f(a[m].mean(), 3), f(np.median(act), 2), f(np.median(act) / a[m].mean(), 2)])
  table(["|output| bin", "n", "mean |output|", "median achieved accel m/s^2", "accel per unit output"], rows)
  mm = m0 & (a >= 0.2) & (a <= 0.9)
  if mm.sum() > 100:
    out = C[mm, TI["out"]]; act = C[mm, TI["act"]]
    sl, ic = np.polyfit(out, act, 1)
    fr = -ic / sl if sl else np.nan
    print(f"Linear fit over 0.2 <= |output| <= 0.9 (n={int(mm.sum())}, signed): actual = {sl:.3f} * output {ic:+.3f}  "
          f"(slope = implied LAT_ACCEL_FACTOR incl. friction offset {sl:.2f} vs {CFG_FACTOR} configured, {USER_FACTOR} user; zero-crossing at output "
          f"{fr:+.3f}). Extrapolated lateral accel at |output| = 1.0: {sl:.2f}; at 0.98: {0.98 * sl:.2f} m/s^2. "
          f"Torque factor in params.toml MAX_LAT_ACCEL_MEASURED = {CFG_MAX}.\n")
  p = a >= PEG
  pm = m0 & p
  if pm.sum():
    pa = (C[:, TI["act"]] * sgo)[p & (C[:, TI["v"]] > 8)]
    print(f"Pegged samples with tracking error < 0.5 (n={int(pm.sum())}) median achieved accel {np.median((C[:, TI['act']] * sgo)[pm]):.2f} m/s^2; "
          f"all pegged samples (any error, v > 8 m/s, n={len(pa)}) median {np.median(pa):.2f}, p90 {np.percentile(pa, 90):.2f}, "
          f"max {pa.max():.2f}.\n")


# ---------------------------------------------------------------------------------------------------------------- section 4
def section4(routes, loaded):
  print("## 4. liveTorqueParameters per qlog route\n")
  print(f"Configured: params.toml KIA_STINGER LAT_ACCEL_FACTOR {CFG_FACTOR}, friction {CFG_FRIC} (MAX_LAT_ACCEL_MEASURED {CFG_MAX}); "
        f"user settings {USER_FACTOR} / {USER_FRIC}.\n")
  rows = []
  raw_rows = []
  for r in routes:
    files, res = loaded[r]
    L = []
    for fp, x in zip(files, res):
      if len(x[1]):
        L.append(np.column_stack([x[1], np.full(len(x[1]), seg_num(fp))]))
    if not L:
      rows.append([short(r), "-", "-", "-", "-", "-", "-", "-", "-", "-"])
      continue
    L = np.concatenate(L)
    L = L[np.argsort(L[:, 0])]
    v = L[L[:, 1] == 1]
    if not len(v):
      rows.append([short(r), len(L), 0, "-", "-", "-", "-", "-", "-", "-"])
      continue
    th = (v[:, 0] - v[0, 0]) / 3600
    n = len(v)
    q = max(1, n // 4)
    sl_f = np.polyfit(th, v[:, 2], 1)[0] if th[-1] > 0.05 else np.nan
    sl_r = np.polyfit(th, v[:, 3], 1)[0] if th[-1] > 0.05 else np.nan
    rows.append([short(r), len(L), f"{n} ({n / len(L) * 100:.0f}%)", f"{v[0, 2]:.3f} -> {v[-1, 2]:.3f}",
                 f"{v[:q, 2].mean():.3f} -> {v[-q:, 2].mean():.3f}", f(sl_f, 3), f"{v[0, 3]:.3f} -> {v[-1, 3]:.3f}",
                 f"{v[:q, 3].mean():.3f} -> {v[-q:, 3].mean():.3f}", f(sl_r, 3),
                 f"{v[-1, 6]:.0f} / cal {v[-1, 7]:.0f}% / use {v[-1, 8]:.0f}"])
    raw_rows.append([short(r), f"{v[:, 2].min():.4f} .. {v[:, 2].max():.4f}", f"{v[:, 4].min():.3f} .. {np.median(v[:, 4]):.3f} .. {v[:, 4].max():.3f}",
                     f"{v[:, 3].min():.4f} .. {v[:, 3].max():.4f}", f"{v[:, 5].min():.3f} .. {np.median(v[:, 5]):.3f} .. {v[:, 5].max():.3f}",
                     f"{v[0, 6]:.0f} -> {v[-1, 6]:.0f}"])
  table(["route", "ltp samples", "liveValid samples", "factor first -> last valid", "factor first-quarter -> last-quarter mean",
         "factor slope /h", "friction first -> last", "friction q1 -> q4 mean", "friction slope /h", "last buckets / calPerc / useParams"], rows)
  print("Range of the filtered values and the raw per-sample estimates (min .. median .. max) per route:\n")
  table(["route", "filtered factor min .. max", "raw factor", "filtered friction min .. max", "raw friction", "bucket points first -> last"], raw_rows)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--stock", nargs="*", default=STOCK)
  ap.add_argument("--rlog", nargs="*", default=OPLAT)
  ap.add_argument("--qlog", nargs="*", default=QLOGS)
  a = ap.parse_args()
  rl = list(dict.fromkeys(a.stock + a.rlog))
  loaded = {r: load(r, "rlog") for r in rl}
  qloaded = {r: load(r, "qlog") for r in a.qlog}
  print("# Stinger steering torque limit\n")
  print(f"STEER_MAX {STEER_MAX} (KIA_STINGER_2022 default CarControllerParams); rlog routes: stock {', '.join(map(short, a.stock))}; "
        f"openpilot-lateral {', '.join(map(short, a.rlog))}; qlog routes {', '.join(map(short, a.qlog))}.\n")
  section1(a.stock, loaded)
  section2(rl, loaded)
  section3(rl, loaded, "rlog 100 Hz, all rlog routes")
  section3(a.qlog, qloaded, "qlog 10 Hz, qlog routes")
  section4(a.qlog, qloaded)


if __name__ == "__main__":
  main()
