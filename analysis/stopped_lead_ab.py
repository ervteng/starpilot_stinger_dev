#!/usr/bin/env python3
"""A/B tables for the Stinger stopped-lead tune from `sim_stopped_lead.py scenarios --series DIR` output (numpy only).

  sim_stopped_lead.py scenarios --stopped-lead-tune A,B --series DIR --csv out.csv ...
  stopped_lead_ab.py DIR [--late]

Per run (applied accel 'a' of the car model): onset TTC = gap / v at the first applied accel < -0.3; mid decel = mean of the applied decel while
the speed is between 70% and 30% of the onset speed, cv = std/mean there; peak; creep = time below 3 mph until the stop; final gap.
--late: the late-detection cases (detect 60/40/30 m): min gap and impact speed, B vs A, and the hard requirement (B min gap >= A, B impact <= A).
"""
import argparse, glob, os, re
import numpy as np

MPH = 2.23694


def load(path):
  with open(path) as f:
    head = f.readline().strip().split(",")
    rows = [l.strip().split(",") for l in f]
  cols = {h: np.array([float(r[i]) for r in rows]) for i, h in enumerate(head) if h in ("t", "gap", "v", "a")}
  return cols


def metrics(s):
  t, gap, v, a = s["t"], s["gap"], s["v"], s["a"]
  m = {}
  col = gap[-1] < 0.3
  m["contact"], m["impact"] = bool(col), float(v[-1] * MPH) if col else 0.0
  m["min_gap"] = float(gap.min())
  stop = np.where(v < 0.05)[0]
  end = int(stop[0]) if len(stop) else len(t) - 1
  idx = np.where(a[:end + 1] < -0.3)[0]
  i = int(idx[0]) if len(idx) else 0
  m["ttc"] = float(gap[i] / max(v[i], 0.1)) if len(idx) else np.nan
  m["onset_gap"] = float(gap[i])
  w = (v[:end + 1] <= 0.7 * v[i]) & (v[:end + 1] >= 0.3 * v[i]) & (np.arange(end + 1) >= i)
  if w.sum() > 3:
    d = -a[:end + 1][w]
    m["mid"], m["cv"] = float(d.mean()), float(d.std() / max(d.mean(), 1e-6))
  else:
    m["mid"] = m["cv"] = np.nan
  m["peak"] = float(-a[:end + 1].min())
  b3 = np.where(v[:end + 1] <= 3 / MPH)[0]
  m["creep"] = float(t[end] - t[b3[0]]) if len(b3) and len(stop) else np.nan
  m["final"] = float(gap[end]) if len(stop) and not col else np.nan
  return m


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--late", action="store_true")
  a = ap.parse_args()
  runs = {}
  for p in sorted(glob.glob(os.path.join(a.dir, "*.csv"))):
    mt = re.match(r"(\w+?)_d([\d.]+)_(\w)_slt(\w)_v(\d+)_det(\d+)\.csv", os.path.basename(p))
    if not mt:
      continue
    pers, delay, car, slt, v, det = mt.groups()
    runs[(pers, delay, car, int(v), int(det), slt)] = metrics(load(p))
  f = lambda x, p=2: "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{p}f}"
  groups = sorted({k[:3] for k in runs})
  for g in groups:
    print(f"\n#### {g[0]}, planner delay {g[1]}, car tune {g[2]}\n")
    if not a.late:
      print("| mph | TTC A | TTC B | mid A | mid B | cv A | cv B | peak A | peak B | creep A | creep B | final A | final B |\n|" + "---|" * 13)
      for v in sorted({k[3] for k in runs if k[:3] == g}):
        A, B = runs.get(g + (v, 0, "A")), runs.get(g + (v, 0, "B"))
        if A and B:
          print(f"| {v} | " + " | ".join(f"{f(A[k], 1 if k == 'ttc' else 2)} | {f(B[k], 1 if k == 'ttc' else 2)}" for k in ("ttc", "mid", "cv", "peak", "creep", "final")) + " |")
    else:
      print("| mph | det m | min gap A | min gap B | impact A | impact B | peak A | peak B | OK |\n|" + "---|" * 9)
      for v in sorted({k[3] for k in runs if k[:3] == g}):
        for det in (60, 40, 30):
          A, B = runs.get(g + (v, det, "A")), runs.get(g + (v, det, "B"))
          if A and B:
            ok = B["min_gap"] >= A["min_gap"] - 0.05 and B["impact"] <= A["impact"] + 0.05
            print(f"| {v} | {det} | {f(A['min_gap'], 1)} | {f(B['min_gap'], 1)} | {f(A['impact'], 0)} | {f(B['impact'], 0)} | {f(A['peak'])} | {f(B['peak'])} | {'ok' if ok else '**FAIL**'} |")


if __name__ == "__main__":
  main()
