# Gap behind lead by driving mode (Chill/Exp) at low speed. Uses /tmp/gap_{t3,t4}_*.pkl (20 Hz) and /tmp/Q_{am,pm}.pkl (3 Hz).
import glob, pickle, re, collections
import numpy as np

def load_r(tag):
  fs = sorted(glob.glob(f'/tmp/gap_{tag}_*.pkl'), key=lambda f: int(re.findall(r'_(\d+)\.pkl', f)[0]))
  rows = [r for f in fs for r in pickle.load(open(f, 'rb'))]
  c = lambda i: np.array([r[i] for r in rows], float)
  return dict(t=c(0), v=c(1), a=c(2), gas=c(3), brk=c(4), act=c(6), ovr=c(7), d=c(8), vl=c(9), pres=c(11), exp=c(13),
              src=np.array([r[14] for r in rows]), hz=20, minrows=40, rows=len(rows))

def load_q(name):
  q = pickle.load(open(f'/tmp/Q_{name}.pkl', 'rb'))
  c = lambda i: np.array([r[i] for r in q], float)
  return dict(t=c(0), v=c(1), a=c(2), gas=c(6), brk=c(5), act=c(4), ovr=np.zeros(len(q)), d=c(7), vl=c(8), pres=c(10), exp=c(13),
              src=None, hz=3, minrows=6, rows=len(q))

drives = {'t3': load_r('t3'), 't4': load_r('t4'), 'Q_am': load_q('am'), 'Q_pm': load_q('pm')}
BINS = [(1, 2), (2, 3), (3, 4.5), (4.5, 6), (6, 9), (9, 13)]

def runs(mask, t, maxgap):
  # contiguous runs of True with no time gap > maxgap; yields (i0, i1) inclusive
  out = []; i = 0; n = len(mask)
  while i < n:
    if mask[i]:
      j = i
      while j + 1 < n and mask[j + 1] and t[j + 1] - t[j] <= maxgap: j += 1
      out.append((i, j)); i = j + 1
    else: i += 1
  return out

def stretches(D):
  m = ((D['act'] > 0) & (D['gas'] == 0) & (D['brk'] == 0) & (D['ovr'] == 0) & (D['pres'] > 0) &
       (np.abs(D['v'] - D['vl']) < 0.3) & (np.abs(D['a']) < 0.3) & (D['v'] > 1) & (D['v'] < 13))
  res = []
  for i, j in runs(m, D['t'], 2.5 / D['hz']):
    if j - i + 1 < D['minrows']: continue
    sl = slice(i, j + 1)
    mode = 'Exp' if np.mean(D['exp'][sl]) > 0.5 else 'Chill'
    v = np.median(D['v'][sl]); d = np.median(D['d'][sl])
    res.append((mode, v, d, d / max(v, 0.1), D['t'][j] - D['t'][i]))
  return res

def q(x, p): return np.percentile(x, p)
def fmt(xs, f='%.1f'):
  if not xs: return '-'
  return (f + ' [' + f + '-' + f + ']') % (np.median(xs), q(xs, 25), q(xs, 75))

ST = {k: stretches(D) for k, D in drives.items()}
ST['pooled'] = [s for k in drives for s in ST[k]]
print('== Steady following: per stretch median. cell = n | gap m med [IQR] | time gap s med [IQR]')
for name in ['t3', 't4', 'Q_am', 'Q_pm', 'pooled']:
  print('--', name)
  for lo, hi in BINS:
    cells = []
    for mode in ('Chill', 'Exp'):
      s = [x for x in ST[name] if x[0] == mode and lo <= x[1] < hi]
      cells.append(f'{mode}: n={len(s)} gap {fmt([x[2] for x in s])} tg {fmt([x[3] for x in s], "%.2f")}')
    print(f'  {lo}-{hi} m/s  ' + ' | '.join(cells))

print('\n== Stopped gap (gap 1 s after standstill start; lead<15 m, engaged, no brake 3 s before)')
def stops(D):
  t, v = D['t'], D['v']; res = []; skipped = collections.Counter()
  for i, j in runs(v < 0.1, t, 2.5 / D['hz']):
    if i == 0 or t[j] - t[i] < 1.0: continue
    k = np.searchsorted(t, t[i] + 1.0)
    if k >= len(t) or t[k] - t[i] > 1.5: continue
    pre = (t >= t[i] - 4) & (t < t[i]); pre3 = (t >= t[i] - 3) & (t < t[i])
    if pre.sum() < 0.5 * 4 * D['hz']: skipped['no prior data'] += 1; continue
    if not (D['act'][i] > 0 and D['act'][k] > 0): skipped['not engaged'] += 1; continue
    if D['brk'][pre3].any() or D['brk'][i] > 0: skipped['driver brake'] += 1; continue
    if not (D['pres'][k] > 0 and D['d'][k] < 15): skipped['no lead<15m'] += 1; continue
    mode = 'Exp' if np.mean(D['exp'][pre]) < 0.5 and False else ('Chill' if np.mean(D['exp'][pre] == 0) > 0.5 else 'Exp')
    res.append((mode, D['d'][k], t[i]))
  return res, skipped
SS = {}
for k, D in drives.items():
  SS[k], sk = stops(D)
  print(f'{k}: {len(SS[k])} stops; skipped {dict(sk)}')
SS['pooled'] = [s for k in drives for s in SS[k]]
for name in ['t3', 't4', 'Q_am', 'Q_pm', 'pooled']:
  for mode in ('Chill', 'Exp'):
    g = [round(s[1], 2) for s in SS[name] if s[0] == mode]
    if g: print(f'  {name} {mode}: n={len(g)} med {np.median(g):.2f} IQR {q(g,25):.2f}-{q(g,75):.2f} min {min(g):.2f} vals {sorted(g)}')
    else: print(f'  {name} {mode}: n=0')

print('\n== Engaged low-speed (longActive, v<13) time share by mode')
for k, D in drives.items():
  m = (D['act'] > 0) & (D['v'] < 13)
  n = m.sum()
  print(f"{k}: engaged-lowspeed {n/D['hz']/60:.1f} min; Chill {np.mean(D['exp'][m]==0)*100:.0f}%, Exp {np.mean(D['exp'][m]==1)*100:.0f}%, unknown {np.mean(D['exp'][m]<0)*100:.0f}%")

print('\n== Plan source per mode (rlogs, engaged v<13, % of samples)')
for k in ('t3', 't4'):
  D = drives[k]
  for mode, val in (('Chill', 0), ('Exp', 1)):
    m = (D['act'] > 0) & (D['v'] < 13) & (D['exp'] == val)
    c = collections.Counter(D['src'][m]); n = max(m.sum(), 1)
    print(f'{k} {mode}: ' + ', '.join(f'{s} {100*x/n:.0f}%' for s, x in c.most_common()) + f' (n={m.sum()})')

print('\n== Data checks')
for k, D in drives.items():
  m = (D['pres'][1:] > 0) & (D['pres'][:-1] > 0) & (D['v'][1:] < 13) & (np.diff(D['t']) < 2.5 / D['hz'])
  dd = np.abs(np.diff(D['d']))
  thr = 1.5 if D['hz'] == 20 else 4
  print(f"{k}: rows {D['rows']}, dRel jumps>{thr} m between consecutive rows with lead present: {int((m & (dd > thr)).sum())}; exp values {sorted(set(D['exp']))}; time gaps>1s: {int((np.diff(D['t'])>1).sum())}")
