#!/usr/bin/env python3
"""Where time is lost in brake takeovers: lead brake onset -> planner req onset -> car decel onset, vs normal stops.
Needs /tmp/bd_t3_*.pkl and /tmp/bd_t4_*.pkl. Usage: stinger_takeover_delays.py"""
import sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stinger_brake_delivery_fit import load

DT = 0.05
EVENTS = {'t3': [263.4, 291.9], 't4': [196.9, 525.9, 568.6, 2355.8, 2386.7, 2394.6, 2431.8, 2545.8, 2578.3, 2989.2, 3333.2]}
FLAG = {('t4', 568.6)}


def prep(D):
  t = D['t'] - D['t'][0]
  n = len(t)
  vls = np.full(n, np.nan)
  seg = np.split(np.arange(n), np.where(np.diff(t) > 0.2)[0] + 1)
  for idx in seg:
    ok = D['pres'][idx] != 0
    v = np.where(ok, D['vl'][idx], np.nan)
    m = len(idx)
    for k in range(m):
      if not ok[k]: continue
      w = v[max(0, k - 5):k + 6]  # centered 0.5 s (+-0.25 s, 11 samples)
      if len(w) < 11 and (k < 5 or k > m - 6): continue  # incomplete window at segment edge
      vls[idx[k]] = np.nanmean(w)
  la = np.full(n, np.nan)
  for idx in seg:
    if len(idx) < 3: continue
    la[idx] = np.gradient(vls[idx], DT)
  return t, vls, la


def lead_onset(t, la, i_ev, back=8.0):
  """First sample of last continuous stretch la<=-0.5 lasting >=0.3 s (>=6 samples) in [ev-8, ev]."""
  i0 = np.searchsorted(t, t[i_ev] - back)
  best = None
  i = i0
  while i <= i_ev:
    if la[i] <= -0.5:
      j = i
      while j + 1 <= i_ev and la[j + 1] <= -0.5 and t[j + 1] - t[j] < 0.2: j += 1
      if j - i + 1 >= 6: best = i
      i = j + 1
    else:
      i += 1
  return best


def delays(D, t, la, i_ev):
  iL = lead_onset(t, la, i_ev)
  if iL is None: return None, 'no lead brake stretch'
  iS = np.searchsorted(t, t[iL] - 1.0)
  thr = min(D['req'][iL] - 0.3, -0.3)
  c = np.where(D['req'][iS:i_ev + 1] <= thr)[0]
  r = dict(iL=iL, tL=t[iL], v=D['v'][iL], d=D['d'][iL], vl=D['vl'][iL], req_ev=D['req'][i_ev], a_ev=D['a'][i_ev], d_ev=D['d'][i_ev])
  if len(c) == 0: return r, 'no planner onset before event'
  iP = iS + c[0]
  r['iP'] = iP; r['dP'] = t[iP] - t[iL]; r['aP'] = D['a'][iP]
  thc = min(D['a'][iP] - 0.3, -0.3)
  c = np.where(D['a'][iP:i_ev + 1] <= thc)[0]
  if len(c) == 0: return r, 'no car onset before event'
  iC = iP + c[0]
  r['dC'] = t[iC] - t[iP]
  return r, None


def controls(D, t, la):
  out = []
  v, n = D['v'], len(t)
  i = 1
  while i < n:
    if v[i] < 0.3 and v[i - 1] >= 0.3:
      i_ev = i
      # approach: speed > 2 within the previous 15 s, la==1/no gas/no brk from that point to the stop
      i0 = np.searchsorted(t, t[i] - 15)
      w = np.where(v[i0:i] > 2)[0]
      if len(w) and t[i] - t[i0 + w[-1]] < 15:
        ia = i0 + w[-1]
        s = slice(ia, i + 1)
        ok = (D['la'][s] == 1).all() and (D['gas'][s] == 0).all() and (D['brk'][s] == 0).all() and (np.diff(t[s]) < 0.2).all()
        j1 = np.searchsorted(t, t[i] + 3)
        nobrk = (D['brk'][i:j1] == 0).all() and t[min(j1, n) - 1] - t[i] >= 2.9
        if ok and nobrk and D['pres'][i] != 0 and D['d'][i] < 10:
          iL = lead_onset(t, la, i_ev)
          if iL is not None:
            out.append(i_ev)
    i += 1
  return out


def f(x, fmt='%.2f'):
  return 'n/a' if x is None or (isinstance(x, float) and np.isnan(x)) else fmt % x


def row(tag, te, r, err, flag=''):
  if r is None: return f'{tag:3} {te:7.1f} n/a ({err}){flag}'
  return (f"{tag:3} {te:7.1f} v={f(r['v'])} d={f(r['d'])} vl={f(r['vl'])} pDel={f(r.get('dP'))} cDel={f(r.get('dC'))} "
          f"aP={f(r.get('aP'))} req_ev={f(r['req_ev'])} a_ev={f(r['a_ev'])} d_ev={f(r['d_ev'])}" + (f' [{err}]' if err else '') + flag)


def stats(x):
  x = np.array([a for a in x if a is not None and not np.isnan(a)])
  if len(x) == 0: return 'n=0'
  q = np.percentile(x, [25, 50, 75])
  return f'n={len(x)} median={q[1]:.2f} IQR=[{q[0]:.2f},{q[2]:.2f}]'


def main():
  data = {'t3': load('t3', 32), 't4': load('t4', 60)}
  prepped = {k: prep(D) for k, D in data.items()}
  TK, CT, fails = [], [], []
  print('== Takeovers ==')
  for tag, evs in EVENTS.items():
    D = data[tag]; t, _, la = prepped[tag]
    for te in evs:
      i_ev = min(np.searchsorted(t, te), len(t) - 1)
      r, err = delays(D, t, la, i_ev)
      fl = ' (FLAG: possible driver-preference brake)' if (tag, te) in FLAG else ''
      print(row(tag, te, r, err, fl))
      if err: fails.append((tag, te, err))
      if r: TK.append(r)
  print('\n== Controls ==')
  for tag in data:
    D = data[tag]; t, _, la = prepped[tag]
    for i_ev in controls(D, t, la):
      r, err = delays(D, t, la, i_ev)
      if r: CT.append(r)
      print(row(tag, t[i_ev], r, err))
      if err: fails.append((tag + ' control', t[i_ev], err))
  print(f'\n#controls={len(CT)}')
  for name, S in (('takeover', TK), ('control', CT)):
    print(f'{name:9} planner delay {stats([r.get("dP") for r in S])} | car delay {stats([r.get("dC") for r in S])} | aP {stats([r.get("aP") for r in S])}')
  print('\nfailures:'); [print(' ', x) for x in fails]


main()
