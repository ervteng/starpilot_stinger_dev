#!/usr/bin/env python3
"""Check the shipped Stinger brake-delivery table against stinger_brake_delivery_criteria.md on the logged drives.

Imports opendbc from the x86-sg checkout (run run_tests.sh on the branch first). Needs /tmp/bd_t3_*, /tmp/bd_t4_* (dump script) and,
for A5, /tmp/lw_*.pkl (10-28 takeovers) and /tmp/Q_am.pkl, /tmp/Q_pm.pkl (10-29 qlogs).
"""
import os
import pickle
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.environ.get('STINGER_DEV_HOME', '/home/openpilotstuff/starpilot-dev'), 'x86-sg', 'opendbc_repo'))
from opendbc.car.hyundai import stinger_brake_delivery as bd  # noqa: E402
from stinger_brake_delivery_fit import load, windows  # noqa: E402

comp = np.vectorize(bd.compensate)
short = np.vectorize(bd.expected_shortfall)


def a1(name, W):
  v, r, y = W[:, 0], W[:, 1], W[:, 2] - W[:, 1]
  res = y - short(v, r)
  m = (v >= 1.0) & (v < 4.5) & (r >= -1.0) & (r < -0.45)
  print(f'A1 {name}: band n={int(m.sum())} shortfall {np.median(y[m]):+.2f} -> residual {np.median(res[m]):+.2f} (pass <= 0.10)')
  worst = 0.0
  for vlo, vhi in ((1.0, 2.0), (2.0, 3.0), (3.0, 4.5)):
    for rlo, rhi in ((-0.7, -0.45), (-1.0, -0.7)):
      k = (v >= vlo) & (v < vhi) & (r >= rlo) & (r < rhi)
      if k.sum() >= 15:
        worst = max(worst, abs(np.median(res[k])))
        print(f'   v{vlo}-{vhi} req{rlo}..{rhi}: n={int(k.sum()):4d} shortfall {np.median(y[k]):+.2f} residual {np.median(res[k]):+.2f}')
  return worst


def a2_a3(name, D):
  act = (D['la'] == 1) & (D['ovr'] == 0) & np.isin(D['lcs'], (1, 2))
  r, v = D['req'], D['v']
  out = np.where(act, comp(r, v), r)
  extra = r - out
  untouched = act & ((v >= 6) | (r > -0.2) | ((r <= -1.5) & (v >= 4.5)))
  print(f'A2 {name}: max added in untouched regimes {extra[untouched].max():.3f} (pass < 0.05)')
  cont = act[1:] & act[:-1] & (np.diff(D['t']) < 0.07)
  excess = np.abs(np.diff(out)) - np.abs(np.diff(r))
  print(f'A3 {name}: added min {extra[act].min():+.3f} max {extra[act].max():.3f} (pass 0..0.35), out min {out.min():.2f}, '
        f'step excess max {excess[cont].max():.3f} p99 {np.percentile(excess[cont], 99):.3f} (pass <= 0.05)')
  return extra[untouched].max(), extra[act].min(), extra[act].max(), excess[cont].max()


def a5():
  print('A5 (descriptive): extra decel the table adds over the 2 s before each driver-braked takeover, and the distance it would '
        'save if delivered as modelled')
  srcs = []
  R = []
  for s in range(60):
    R += pickle.load(open(f'/tmp/lw_{s}.pkl', 'rb'))
  A = np.array(R, float)
  A = A[np.argsort(A[:, 0])]
  srcs.append(('10-28', A[:, 0], A[:, 1], A[:, 3], A[:, 4], A[:, 5], A[:, 7], A[:, 10], A[:, 2]))
  for tag in ('am', 'pm'):
    A = np.array(pickle.load(open(f'/tmp/Q_{tag}.pkl', 'rb')), float)
    srcs.append((f'10-29{tag}', A[:, 0], A[:, 1], A[:, 3], A[:, 4], A[:, 5], A[:, 7], A[:, 10], A[:, 2]))
  for name, t, v, cmd, la, brk, d, pres, ae in srcs:
    idx = np.where((la[:-1] == 1) & (la[1:] == 0) & (brk[1:] == 1) & (pres[:-1] == 1) & (d[:-1] < 8) & (v[:-1] < 6.5))[0]
    last = -99
    for i in idx:
      if t[i] - last < 5:
        continue
      last = t[i]
      w = np.where((t > t[i] - 2) & (t <= t[i]) & (la == 1))[0]
      if len(w) < 2:
        continue
      extra = cmd[w] - comp(cmd[w], v[w])
      dt = np.diff(np.concatenate([[t[w][0] - 0.05], t[w]]))
      dv = np.cumsum(extra * dt)
      saved = float(np.sum(dv * dt))
      print(f'  {name} {t[i] / 60:6.1f} min: v={v[i]:.1f} d={d[i]:.1f} cmd={-cmd[i]:.2f} aEgo={-ae[i]:.2f} | extra mean {extra.mean():.2f} '
            f'max {extra.max():.2f} -> about {saved:.2f} m more gap at takeover')


if __name__ == '__main__':
  W3, W4 = windows(load('t3', 32)), windows(load('t4', 60))
  w3, w4 = a1('t3', W3), a1('t4', W4)
  print(f'A1 worst in-band cell: t3 {w3:.2f} (n>=15) t4 {w4:.2f} (pass <= 0.15)')
  for name, n in (('t3', 32), ('t4', 60)):
    a2_a3(name, load(name, n))
  a5()
