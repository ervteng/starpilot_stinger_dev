#!/usr/bin/env python3
"""Fit and cross-validate the Stinger brake-delivery shortfall sf(req, v) = s(v) * g(req). Criteria: stinger_brake_delivery_criteria.md.

Needs /tmp/bd_t3_*.pkl (drive 00000003--0f3f0eb0f2) and /tmp/bd_t4_*.pkl (00000004--a5fd1905e8) from stinger_brake_delivery_dump.py.
Usage: stinger_brake_delivery_fit.py
"""
import pickle
import numpy as np

COLS = 't v a gas brk stand req out la en lcs d vl pres ovr'.split()
DT = 0.05
SV = np.array([0.5, 1.0, 3.5, 4.5, 6.0])           # s(v) knots; s = 1 on the plateau 1.0-3.5 (normalisation)
GR = np.array([-2.6, -2.0, -1.5, -1.25, -1.0, -0.7, -0.45, -0.2, 0.0])  # g(req) knots (ascending for np.interp), g(0) = 0


def load(tag, n):
  rows = []
  for s in range(n):
    try:
      rows += pickle.load(open(f'/tmp/bd_{tag}_{s}.pkl', 'rb'))
    except FileNotFoundError:
      pass
  A = np.array(rows, float)
  A = A[np.argsort(A[:, 0])]
  return {c: A[:, i] for i, c in enumerate(COLS)}


def windows(D, W=10, rng=0.3, lag=4):
  """(v, req, aEgo) per 0.5 s window with a steady request ending `lag` samples (0.2 s) before the aEgo window."""
  t = D['t']
  ok = (D['la'] == 1) & (D['gas'] == 0) & (D['brk'] == 0) & (D['ovr'] == 0) & (D['stand'] == 0) & (D['v'] > 0.3) & (D['lcs'] != 3)
  out = []
  brk = np.where(np.diff(t) > 0.2)[0] + 1
  for idx in np.split(np.arange(len(t)), brk):
    for k in range(W + lag, len(idx)):
      i = idx[k]
      sl = slice(i - W, i)
      rq = D['req'][i - W - lag:i - lag]
      if ok[sl].all() and rq.max() - rq.min() <= rng:
        out.append((D['v'][sl].mean(), rq.mean(), D['a'][sl].mean()))
  return np.array(out)


def hat(x, knots, j):
  e = np.zeros(len(knots))
  e[j] = 1
  return np.interp(x, knots, e)


def eval_sg(p, v, r):
  s = np.interp(v, SV, p['s'])
  g = np.interp(r, GR, p['g'])
  return s * g


def fit(W, iters=30):
  v, r, y = W[:, 0], W[:, 1], W[:, 2] - W[:, 1]
  m = (v < 8) & (r < -0.1)
  v, r, y = v[m], r[m], y[m]
  p = {'s': np.array([0.5, 1, 1, 0.5, 0.0]), 'g': np.array([0, 0, 0, .12, .25, .25, .2, .1, 0.0])}
  free_s = [0, 3, 4]
  for _ in range(iters):
    s = np.interp(v, SV, p['s'])
    X = np.stack([s * hat(r, GR, j) for j in range(len(GR) - 1)], 1)       # g(0) = 0 fixed (last knot)
    g, *_ = np.linalg.lstsq(X, y, rcond=None)
    p['g'] = np.concatenate([g, [0.0]])
    gv = np.interp(r, GR, p['g'])
    Xs = np.stack([gv * hat(v, SV, j) for j in free_s], 1)
    yr = y - gv * (hat(v, SV, 1) + hat(v, SV, 2))
    sv, *_ = np.linalg.lstsq(Xs, yr, rcond=None)
    for j, val in zip(free_s, sv):
      p['s'][j] = np.clip(val, 0.0, 1.5)
  return p


def cell_report(name, p, W):
  v, r, y = W[:, 0], W[:, 1], W[:, 2] - W[:, 1]
  res = y - eval_sg(p, v, r)
  m = (v >= 1.0) & (v < 4.5) & (r >= -1.0) & (r < -0.45)
  print(f'  {name}: A1 band v1-4.5 req -1.0..-0.45 n={int(m.sum())}  median shortfall before {np.median(y[m]):+.2f} -> residual {np.median(res[m]):+.2f}')
  worst = 0.0
  for vlo, vhi in ((0.3, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.5), (4.5, 7.0)):
    for rlo, rhi in ((-0.45, -0.25), (-0.7, -0.45), (-1.0, -0.7), (-1.5, -1.0), (-2.6, -1.5)):
      k = (v >= vlo) & (v < vhi) & (r >= rlo) & (r < rhi)
      if k.sum() >= 15:
        mr = np.median(res[k])
        print(f'     v{vlo}-{vhi} req{rlo}..{rhi}: n={int(k.sum()):4d} before {np.median(y[k]):+.2f} residual {mr:+.2f}')
        if 1.0 <= vlo < 4.5 and rlo >= -1.0:
          worst = max(worst, abs(mr))
  return abs(np.median(res[m])), worst


if __name__ == '__main__':
  W3 = windows(load('t3', 32))
  W4 = windows(load('t4', 60))
  print('windows: t3', len(W3), 't4', len(W4))
  p3, p4 = fit(W3), fit(W4)
  pall = fit(np.vstack([W3, W4]))
  for name, p in (('fit t3', p3), ('fit t4', p4), ('pooled', pall)):
    print(name, 's(v) at', SV, np.round(p['s'], 2), ' g(req) at', GR, np.round(p['g'], 2))
  print('held-out checks:')
  a = cell_report('t3->t4', p3, W4)
  b = cell_report('t4->t3', p4, W3)
  print('A1 band medians:', [round(a[0], 3), round(b[0], 3)], 'worst cell (n>=15, in-band):', [round(a[1], 3), round(b[1], 3)])
  pickle.dump(pall, open('/tmp/stinger_bd_pooled.pkl', 'wb'))
