#!/usr/bin/env python3
"""Closed-loop A/B of the Stinger slot 9 B planner change on a lead that brakes steadily to a stop from highway speed.

  sustained_lead_brake.py run --label before --out before.json [--jobs 4]     (run once per checkout of the x86 tree)
  sustained_lead_brake.py report before.json after.json --out sustained_lead_brake_results.md

Each scenario uses sim_stopped_lead.run_sim (real LongitudinalPlanner + MPC + LongControl, car tune B shaping, measured 0.2 s delay /
0.3 s lag car response, planner actuator delay 0.5, --lead-filter fast, Testing Ground 9:B forced active) with a scripted lead (lead_track):
  - ego and lead start at v0 with gap = headway * v0, cruise set speed v0 + 10 mph (so ego follows the lead rather than being capped),
  - 5 s steady, then the lead's accel ramps linearly to a over 0.5 s (jerk limited) and holds until the lead stops, then it stays stopped,
  - control: a = -1.0 for 2 s after onset (same 0.5 s ramp in), then ramps back to 0 over 0.5 s and holds the reduced speed.
radarState lead fields come from radard's own Track (Kalman filter KF1D + the aLeadTau rule) fed the lead's true speed at 20 Hz without radar
noise (sim_stopped_lead.py radard_lead=True): aLeadK is the filter's accel estimate, aLeadTau decays to 0 while |aLeadK| >= 0.5 and
recovers x1.1/step to 0.6 otherwise, vLeadK the filter speed, vLead the true speed.
"""
import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import toolchain  # noqa: E402
import sim_stopped_lead as S  # noqa: E402

MPH = S.MPH
T_BRAKE = 5.0
RAMP = 0.5
HEADWAYS = (1.5, 1.9, 2.3)
SPEEDS = (45, 55, 65)
DECELS = (-1.5, -2.0, -2.5, -3.0)
PERSONALITIES = ("standard", "aggressive")
# (personality, min_accel override or None = the sim's logged value: standard -0.5, aggressive -2.0). The two supplementary cases are
# standard with a deeper cruise/brake floor (A_CRUISE_MIN -1.0 = STANDARD braking profile, and -2.0): with -0.5 the floor binds on a hard braking lead.
CASES = (("standard", None), ("aggressive", None), ("standard", -1.0), ("standard", -2.0))
def case_name(pers, mn):
  return pers if mn is None else f"{pers} (min accel {mn})"


def build_lead(v0, gap0, a_brake, control, t_end, dt=0.001):
  n = int(t_end / dt) + 1
  t = np.arange(n) * dt
  v = np.empty(n)
  a = np.zeros(n)
  x = np.empty(n)
  vv, xx = v0, gap0
  for i in range(n):
    tt = t[i] - T_BRAKE
    if tt < 0:
      ai = 0.0
    elif control:
      ai = a_brake * min(tt / RAMP, 1.0) if tt < 2.0 else a_brake * max(0.0, 1.0 - (tt - 2.0) / RAMP)
    else:
      ai = a_brake * min(tt / RAMP, 1.0)
    if not control and vv <= 0.0 and ai < 0:
      ai, vv = 0.0, 0.0
    if not control and vv + ai * dt < 0:
      ai, vv = 0.0, 0.0
    a[i], v[i], x[i] = ai, vv, xx
    vv += ai * dt
    xx += vv * dt
  return t, x, v, a


def one(job):
  if "Plant" not in vars(S):
    S._import_stack()
  v0 = job["mph"] / MPH
  gap0 = job["headway"] * v0
  control = job["kind"] == "control"
  a_brake = job["a"]
  t_end = T_BRAKE + (2.5 + 15.0 if control else v0 / -a_brake + RAMP + 40.0)
  t, x, v, a = build_lead(v0, gap0, a_brake, control, t_end)
  tab, i0, m = S.run_sim(v0_mph=job["mph"], gap0=gap0, personality=job["pers"], delay=0.5, car_tune="B", set_speed_mph=job["mph"] + 10.0,
                         lead_track=(t, x, v, a), max_t=t_end - 1.0, min_accel=job.get("minacc"), testing_ground="9:B", lead_filter="fast", radard_lead=True,
                         model_lead="const", model_lead_range=80.0)
  tt, gap, ve, ae = tab["t"], tab["gap"], tab["v"], tab["a"]
  ib = int(np.argmax(tt >= T_BRAKE - 1e-9))
  post = tt >= T_BRAKE - 1e-9
  res = dict(job)
  res["headway_onset"] = float(gap[ib] / max(ve[ib], 0.1))
  res["contact"] = bool(m["collision"])
  res["impact_mph"] = None if not m["collision"] else float(m["impact_mph"])
  res["min_gap"] = float(gap.min())
  mv = post & (ve > 2.0)
  res["min_tgap"] = float((gap[mv] / ve[mv]).min()) if mv.any() else None
  res["peak_decel"] = float(-ae[post].min())
  for thr in (0.5, 1.0, 2.0):
    j = np.where(post & (ae <= -thr))[0]
    res[f"t_{thr}"] = float(tt[j[0]] - T_BRAKE) if len(j) else None
  res["final_gap"] = float(gap[-1])
  res["final_v"] = float(ve[-1])
  res["a_lead_k_min"] = None
  res["t_end"] = float(tt[-1])
  return res


def verify():
  """Evidence that Testing Ground 9:B is active in the sim's planner (printed and stored)."""
  S._import_stack()
  plant = S.make_plant_class()(v0=25.0, gap0=50.0, set_speed=30.0, personality="standard", delay=0.5, car_tune="B", testing_ground="9:B",
                               lead_filter="fast", radard_lead=True)
  tunes = S.tunes
  mpc = plant.planner.mpc
  out = dict(
    extrapolate_lead=getattr(mpc.extrapolate_lead, "__name__", str(mpc.extrapolate_lead)),
    stock_tune=(tunes.get_kia_stinger_2022_stock_tune(plant.CP) if hasattr(tunes, "get_kia_stinger_2022_stock_tune") else "n/a (function absent)"),
    latch=dict(getattr(tunes, "_kia_stinger_2022_tune_latch", {})),
    has_close_cap_hook=hasattr(tunes, "get_kia_stinger_2022_close_lead_brake_cap_max_ttc"),
    far_follow_slew=list(plant.far_follow_slew),
  )
  import subprocess
  out["x86_head"] = subprocess.run(["git", "-C", S.BUILD, "rev-parse", "--short=9", "HEAD"], capture_output=True, text=True).stdout.strip()
  return out


def cmd_run(a):
  jobs = []
  for pers, mn in CASES:
    for mph in SPEEDS:
      for hw in HEADWAYS:
        for ad in DECELS:
          jobs.append(dict(kind="brake", pers=pers, minacc=mn, mph=mph, headway=hw, a=ad))
      jobs.append(dict(kind="control", pers=pers, minacc=mn, mph=mph, headway=1.9, a=-1.0))
  info = verify()
  print("verify:", info, flush=True)
  with ProcessPoolExecutor(max_workers=a.jobs) as ex:
    res = list(ex.map(one, jobs, chunksize=1))
  json.dump(dict(label=a.label, verify=info, results=res), open(a.out, "w"), indent=1)
  print(f"wrote {a.out}: {len(res)} runs", flush=True)


def f(x, p=1):
  return "-" if x is None else f"{x:.{p}f}"


def criteria(b, af):
  c1 = not (af["contact"] and not b["contact"])
  c2 = af["min_gap"] >= b["min_gap"] - 0.25
  c3 = af["peak_decel"] <= b["peak_decel"] + 0.3
  return c1, c2, c3


def cmd_report(a):
  B, A = (json.load(open(p)) for p in (a.before, a.after))
  key = lambda r: (r["kind"], r["pers"], r.get("minacc"), r["mph"], r["headway"], r["a"])
  bm = {key(r): r for r in B["results"]}
  am = {key(r): r for r in A["results"]}
  ct = lambda r: f"CONTACT {r['impact_mph']:.0f}mph" if r["contact"] else "no"
  L = ["# Sustained lead braking: slot 9 B planner change, closed-loop A/B", ""]
  L += [f"Before = x86 at `{B['verify']['x86_head']}` (623729c81), After = `{A['verify']['x86_head']}` (8e9cbdf84). Both: Testing Ground 9:B forced, car tune B, "
        "actuator delay 0.5, lead filter fast, vision-model lead = constant speed (sim default), radar lead from radard's Track (see below).", "",
        "Slot 9 B evidence (from the sim process, printed after planner construction):", "",
        f"- before: `{json.dumps(B['verify'])}`", f"- after: `{json.dumps(A['verify'])}`", "",
        "(The before commit has no Stinger hooks in the planner at all, so its extrapolate_lead is upstream and there is no stock_tune function; the 9:B car-side "
        "tune is --car-tune B in both.)", "",
        "Lead model: radard `Track` (KF1D + aLeadTau rule) fed the true lead speed at 20 Hz, no radar noise or latency. aLeadK is therefore the Kalman "
        "estimate (lags the true accel by a few hundred ms), aLeadTau starts at 0.6, decays x0.9/step (rc 0.45 s) once |aLeadK| >= 0.5 and recovers x1.1/step otherwise. "
        "Not modelled: radar noise/dropouts, a real vision-model lead prediction (constant-speed lead used), experimental mode, AEB/driver.", "",
        "Scenario: ego = lead = v0, gap = headway x v0, cruise set speed v0 + 10 mph, 5 s steady, lead accel ramps in over 0.5 s to a and holds until stopped. "
        "Peak decel = hardest applied accel (car response) after onset. t-a = seconds from lead brake onset to applied accel first <= -0.5/-1/-2 ('-' = never). "
        "Min time gap = min(gap / v_ego) after onset while v_ego > 2 m/s. Run ends at contact (gap < 0.3 m): then min gap ~0, final gap 0 and peak decel is only up to the impact. "
        "Criteria: (1) no new contact, (2) min gap >= before - 0.25 m, (3) peak decel <= before + 0.3. "
        "Primary cases are `standard` (logged minAcceleration -0.5) and `aggressive` (-2.0). The standard -0.5 floor binds on a hard-braking lead so BEFORE already crashes "
        "in most standard scenarios; two supplementary standard cases with a deeper floor (-1.0, -2.0) are included and reported separately.", ""]
  summaries = []
  for pers, mn in CASES:
    cn = case_name(pers, mn)
    rows = []
    for mph in SPEEDS:
      L += [f"## {cn}, {mph} mph", "",
            "| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |",
            "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
      for hw in HEADWAYS:
        for ad in DECELS:
          k = ("brake", pers, mn, mph, hw, ad)
          b, af = bm[k], am[k]
          c = criteria(b, af)
          rows.append((k, b, af, c))
          L.append(f"| {hw} | {ad} | {f(b['headway_onset'],2)} / {f(af['headway_onset'],2)} | {ct(b)} / {ct(af)} | {f(b['min_gap'])} / {f(af['min_gap'])} | "
                   f"{f(b['min_tgap'],2)} / {f(af['min_tgap'],2)} | {f(b['peak_decel'],2)} / {f(af['peak_decel'],2)} | "
                   f"{f(b['t_0.5'])} / {f(af['t_0.5'])} | {f(b['t_1.0'])} / {f(af['t_1.0'])} | {f(b['t_2.0'])} / {f(af['t_2.0'])} | "
                   f"{f(b['final_gap'])} / {f(af['final_gap'])} | {'P' if c[0] else 'F'} | {'P' if c[1] else 'F'} | {'P' if c[2] else 'F'} | "
                   f"{'PASS' if all(c) else 'FAIL'} |")
      L.append("")
    summaries.append((cn, rows))
  desc = lambda k: f"{case_name(k[1], k[2])} {k[3]} mph hw {k[4]} a {k[5]}"
  L += ["## Overall summary", ""]
  groups = [("PRIMARY (standard -0.5 + aggressive -2.0)", [r for cn, rr in summaries[:2] for r in rr])]
  groups += [(f"SUPPLEMENTARY {cn}", rr) for cn, rr in summaries[2:]]
  groups += [("ALL CASES", [r for cn, rr in summaries for r in rr])]
  for gname, allrows in groups:
    fails = [sum(not r[3][i] for r in allrows) for i in range(3)]
    nfail = sum(1 for r in allrows if not all(r[3]))
    both_contact = sum(1 for r in allrows if r[1]["contact"] and r[2]["contact"])
    d_gap = np.array([r[2]["min_gap"] - r[1]["min_gap"] for r in allrows])
    d_pk = np.array([r[2]["peak_decel"] - r[1]["peak_decel"] for r in allrows])
    L += [f"### {gname}", "", f"Scenarios: {len(allrows)}; failing at least one criterion: {nfail}. Overall: {'PASS' if nfail == 0 else 'FAIL'}.", "",
          f"- C1 (new contact) failures: {fails[0]}", f"- C2 (min gap worse by > 0.25 m) failures: {fails[1]}", f"- C3 (peak decel harder by > 0.3) failures: {fails[2]}",
          f"- Contacts: before {sum(r[1]['contact'] for r in allrows)}, after {sum(r[2]['contact'] for r in allrows)} (both: {both_contact})",
          f"- delta min gap (after-before): mean {d_gap.mean():+.2f} m, min {d_gap.min():+.2f}, max {d_gap.max():+.2f}; "
          f"delta peak decel: mean {d_pk.mean():+.2f}, min {d_pk.min():+.2f}, max {d_pk.max():+.2f}", ""]
    if gname.startswith("ALL"):
      continue
    L += ["Worst 5 by (after - before) min gap:", "", "| scenario | min gap before | after | delta | peak decel b/a | contact b/a |", "|---|---|---|---|---|---|"]
    for k, b, af, c in sorted(allrows, key=lambda r: r[2]["min_gap"] - r[1]["min_gap"])[:5]:
      L.append(f"| {desc(k)} | {f(b['min_gap'],2)} | {f(af['min_gap'],2)} | {af['min_gap']-b['min_gap']:+.2f} | {f(b['peak_decel'],2)} / {f(af['peak_decel'],2)} | {ct(b)} / {ct(af)} |")
    L += ["", "Worst 5 by (after - before) peak decel:", "", "| scenario | peak before | after | delta | min gap b/a |", "|---|---|---|---|---|"]
    for k, b, af, c in sorted(allrows, key=lambda r: -(r[2]["peak_decel"] - r[1]["peak_decel"]))[:5]:
      L.append(f"| {desc(k)} | {f(b['peak_decel'],2)} | {f(af['peak_decel'],2)} | {af['peak_decel']-b['peak_decel']:+.2f} | {f(b['min_gap'],2)} / {f(af['min_gap'],2)} |")
    L.append("")
    fl = [r for r in allrows if not all(r[3])]
    L += ["Failing scenarios:", ""]
    if fl:
      L += ["| scenario | failed | min gap b/a | peak decel b/a | contact b/a |", "|---|---|---|---|---|"]
      for k, b, af, c in fl:
        L.append(f"| {desc(k)} | {','.join(f'C{i+1}' for i in range(3) if not c[i])} | {f(b['min_gap'],2)} / {f(af['min_gap'],2)} | {f(b['peak_decel'],2)} / {f(af['peak_decel'],2)} | {ct(b)} / {ct(af)} |")
    else:
      L.append("none")
    L.append("")
  L += ["## Control scenarios (lead -1.0 m/s2 for 2 s, then holds speed; headway 1.9 s)", "",
        "Final gap/speed are at the end of the run (15 s after the lead stops slowing). Criteria applied for reference.", "",
        "| case | mph | min gap b/a | min tgap b/a | peak decel b/a | t-0.5 b/a | t-1.0 b/a | final gap b/a | final v ego mph b/a | C1 C2 C3 |", "|---|---|---|---|---|---|---|---|---|---|"]
  for pers, mn in CASES:
    for mph in SPEEDS:
      k = ("control", pers, mn, mph, 1.9, -1.0)
      b, af = bm[k], am[k]
      c = criteria(b, af)
      L.append(f"| {case_name(pers, mn)} | {mph} | {f(b['min_gap'])} / {f(af['min_gap'])} | {f(b['min_tgap'],2)} / {f(af['min_tgap'],2)} | {f(b['peak_decel'],2)} / {f(af['peak_decel'],2)} | "
               f"{f(b['t_0.5'])} / {f(af['t_0.5'])} | {f(b['t_1.0'])} / {f(af['t_1.0'])} | {f(b['final_gap'])} / {f(af['final_gap'])} | "
               f"{b['final_v']*MPH:.1f} / {af['final_v']*MPH:.1f} | {''.join('P' if x else 'F' for x in c)} |")
  open(a.out, "w").write("\n".join(L) + "\n")
  print("\n".join(L[L.index("## Overall summary"):]))


def main():
  p = argparse.ArgumentParser()
  sp = p.add_subparsers(dest="cmd", required=True)
  r = sp.add_parser("run")
  r.add_argument("--label", required=True)
  r.add_argument("--out", required=True)
  r.add_argument("--jobs", type=int, default=4)
  q = sp.add_parser("report")
  q.add_argument("before")
  q.add_argument("after")
  q.add_argument("--out", required=True)
  a = p.parse_args()
  if a.cmd == "run":
    toolchain.require_old("sustained_lead_brake.py run (forces Testing Ground 9:B and car tune B: Testing Grounds / longitudinal_vehicle_tunes)")
  {"run": cmd_run, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
  main()
