#!/usr/bin/env python3
"""Generality scan on pickles written by `planner_delay_replay.py ... --dump 0 99999 --save X.pkl`.
Counts (a) cycles where MPC-derived accel (output_a_target_mpc, before clipping) drops >1.0 m/s^2 in one cycle while radar leadOne
dRel/vRel/aLeadK change <1 m, <0.3 m/s, <0.3 m/s^2 (a lead-model path flip is flagged); (b) model->raw lead-path flips and how stale
the lead_v_filter was at the flip. Cycles in the first 30 s of each pickle are skipped (filters/warm-up)."""
import pickle, sys
import numpy as np
for f in sys.argv[1:]:
  _, rows, _ = pickle.load(open(f, "rb"))[0]
  rows = [r for r in rows if "dump" in r]
  t0 = rows[0]["t"]
  nev = nflip = 0
  for p, c in zip(rows, rows[1:]):
    if c["t"] - t0 < 30 or c["t"] - p["t"] > 0.2:
      continue
    dc, dp = c["dump"], p["dump"]
    pl, cl = dp.get("lead0", {}), dc.get("lead0", {})
    flip = bool(pl.get("from_model_path")) and not cl.get("from_model_path") and cl.get("active")
    ap = p["chg"][0][1] if p["chg"] else np.nan
    ac = c["chg"][0][1] if c["chg"] else np.nan
    # raw MPC-derived accel before clipping: first recorded change of output_a_target in update() is the plan read (L2460)
    drop = ac - ap
    if (drop < -1.0 and p["lead"] and c["lead"] and abs(c["d"] - p["d"]) < 1 and abs((c["vl"] - c["v"]) - (p["vl"] - p["v"])) < 0.3
        and abs(c["al"] - p["al"]) < 0.3):
      nev += 1
      fv = cl.get("filt", {}).get("v")
      print(f"  {f.split('/')[-1]} t={c['t']:.2f} mpc-derived {ap:.2f}->{ac:.2f} (out {p['a']:.2f}->{c['a']:.2f}) pathflip={flip} "
            f"lead path={'model' if cl.get('from_model_path') else 'raw'} filt_v={fv} vLead={c['vl']:.1f} aLeadK {p['al']:.2f}->{c['al']:.2f}")
    if flip:
      nflip += 1
      fv = cl["filt"]["v"]
      print(f"    flip model->raw t={c['t']:.2f} filt_v={fv:.1f} vLead={c['vl']:.1f} stale={c['vl'] - fv:.1f} m/s  aLeadK={c['al']:.3f}  out {p['a']:.2f}->{c['a']:.2f}")
  print(f"{f.split('/')[-1]}: cycles {len(rows)}, drop-events {nev}, model->raw flips {nflip}")
