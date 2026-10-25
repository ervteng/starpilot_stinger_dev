#!/usr/bin/env python3
"""Classic-CAN Hyundai/Kia dashboard speed limit: data check and old-fork vs SG-port equivalence, fed the same rlog CAN.

  hist     STINGER_TOOLCHAIN=old ./run_py.sh analysis/dashboard_limit_equiv.py hist ROUTE [ROUTE ...]
           raw Navi_HU (0x544, bus 0) SpeedLim_Nav_Clu histogram (byte 0), message rate, cluster unit (CLU11 CF_Clu_SPEED_UNIT)
           and the 0x53E bus-2 lengths. Log-only; identical under both toolchains.
  dump     STINGER_TOOLCHAIN=old|sg ./run_py.sh analysis/dashboard_limit_equiv.py dump ROUTE --out FILE
           one CarInterface/CarState per segment, warmed with the last --warmup seconds of the previous segment (so the SG
           parser has learned its timeouts, like on a continuous drive), then one update per `can` event (one carState frame).
           old: starpilotCarState.dashboardSpeedLimit (m/s). sg: CS.dashboard_limit.observation, status reduced to
           "valid" only while observation.valid_until_ns > the frame time. Both: ret.canValid. Run-length encoded.
  compare  ./run_py.sh analysis/dashboard_limit_equiv.py compare OLD.json SG.json      (B: old dashboardSpeedLimit vs SG observation)
  canvalid ./run_py.sh analysis/dashboard_limit_equiv.py canvalid SG_BASE.json SG_BRANCH.json   (C: canValid sequences identical)
"""
import argparse
import json
import math
import sys
from collections import Counter
from types import SimpleNamespace

import toolchain
from logs import pmap, segment_files, seg_num, short, sorted_events, events

FP_EVENTS = 3000
NAV_ADDR, LKAS12_ADDR = 0x544, 0x53E


def _setup():
  toolchain.setup_paths(toolchain.BUILD)
  from opendbc.can import CANParser  # noqa: F401  (before the first log read, so logs.py reuses the schema)


# --- A: data check ---------------------------------------------------------------------------------------------------------
def hist_segment(path):
  _setup()
  import math as _m
  from opendbc.can import CANParser
  from opendbc.car import Bus
  from opendbc.car.hyundai.values import CAR, DBC
  clu = CANParser(DBC[CAR.KIA_STINGER_2022][Bus.pt], [("CLU11", _m.nan)], 0)
  clu11_addr = clu.dbc.name_to_msg["CLU11"].address
  vals, lens53e, units, times = Counter(), Counter(), Counter(), []
  nav_bus = Counter()
  for e in events(path):
    if e.which() != "can":
      continue
    frames = []
    for c in e.can:
      if c.address == NAV_ADDR:
        nav_bus[c.src] += 1
        if c.src == 0:
          vals[bytes(c.dat)[0]] += 1
          times.append(e.logMonoTime)
      elif c.address == LKAS12_ADDR and c.src == 2:
        lens53e[len(c.dat)] += 1
      elif c.address == clu11_addr and c.src == 0:
        frames.append((c.address, bytes(c.dat), 0))
    if frames:
      clu.update([(e.logMonoTime, frames)])
      units[int(clu.vl["CLU11"]["CF_Clu_SPEED_UNIT"])] += 1
  dur = (times[-1] - times[0]) / 1e9 if len(times) > 1 else 0.0
  return {"vals": dict(vals), "lens53e": dict(lens53e), "units": dict(units), "nav_bus": dict(nav_bus), "n": len(times), "dur": dur}


def cmd_hist(args):
  tot = {"vals": Counter(), "lens53e": Counter(), "units": Counter(), "nav_bus": Counter(), "n": 0, "dur": 0.0}
  for route in args.routes:
    files = segment_files(route, "rlog")
    r = Counter(); n = 0; dur = 0.0; l53 = Counter(); un = Counter(); nb = Counter()
    for s in pmap(hist_segment, files, args.workers):
      r.update({int(k): v for k, v in s["vals"].items()}); n += s["n"]; dur += s["dur"]
      l53.update({int(k): v for k, v in s["lens53e"].items()}); un.update({int(k): v for k, v in s["units"].items()})
      nb.update({int(k): v for k, v in s["nav_bus"].items()})
    print(f"== {short(route)}: {len(files)} rlog segments, 0x544 bus0 msgs {n} over {dur:.0f} s ({n / dur if dur else 0:.2f} Hz); 0x544 by bus {dict(nb)}")
    print(f"   SpeedLim_Nav_Clu raw values: {dict(sorted(r.items()))}")
    print(f"   CLU11 CF_Clu_SPEED_UNIT (0=kph, 1=mph) msgs: {dict(un)}; 0x53E bus2 lengths: {dict(l53)}")
    tot["vals"].update(r); tot["n"] += n; tot["dur"] += dur; tot["lens53e"].update(l53); tot["units"].update(un); tot["nav_bus"].update(nb)
  print(f"== TOTAL: 0x544 bus0 msgs {tot['n']} over {tot['dur']:.0f} s ({tot['n'] / tot['dur'] if tot['dur'] else 0:.2f} Hz)")
  print(f"   raw values: {dict(sorted(tot['vals'].items()))}")
  print(f"   units: {dict(tot['units'])}; 0x53E lengths: {dict(tot['lens53e'])}")
  zero, ff = tot["vals"].get(0, 0), tot["vals"].get(255, 0)
  print(f"   0 occurs {zero}x, 255 occurs {ff}x, in 1..254: {sum(v for k, v in tot['vals'].items() if 1 <= k <= 254)}")


# --- B/C: dumps --------------------------------------------------------------------------------------------------------------
def _frames(ev):
  return [(c.address, bytes(c.dat), c.src) for c in ev.can]


def dump_segment(job):
  path, prev, warmup_s, candidate, alpha_long = job
  _setup()
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.values import CAR

  evs = sorted_events(path)
  fp = gen_empty_fingerprint()
  for e in evs[:FP_EVENTS]:
    if e.which() == "can":
      for c in e.can:
        if c.src < 3:
          fp[c.src][c.address] = len(c.dat)
  toggles = SimpleNamespace(always_on_lateral_lkas=False, force_torque_controller=False, nnff=False, nnff_lite=False)
  car = getattr(CAR, candidate)
  old = not toolchain.SG
  if old:
    CP = CarInterface.get_params(car, fp, [], alpha_long, False, False, toggles)
    FPCP = CarInterface.get_starpilot_params(car, fp, [], CP, toggles)
    CI = CarInterface(CP, FPCP)
  else:
    try:
      CP = CarInterface.get_params(car, fp, [], alpha_long, False, False, None)
    except TypeError:
      CP = CarInterface.get_params(car, fp, [], alpha_long, False, False)
    CI = CarInterface(CP)

  def step(ev):
    packets = [(ev.logMonoTime, _frames(ev))]
    if old:
      for cp in CI.can_parsers.values():
        cp.update(packets)
      ret, fp_ret = CI.CS.update(CI.can_parsers, toggles)
      valid = all(cp.can_valid for cp in CI.can_parsers.values())
      return ("old", round(float(fp_ret.dashboardSpeedLimit), 9)), valid
    ret = CI.update(packets)
    obs = CI.CS.dashboard_limit.observation
    status = obs.status.value if (obs.status.value == "valid" and obs.valid_until_ns > ev.logMonoTime) else \
      ("unknown" if obs.status.value == "valid" else obs.status.value)
    return (status, round(obs.speed_mps, 9) if status == "valid" else 0.0), ret.canValid

  if prev is not None:
    pevs = [e for e in sorted_events(prev) if e.which() == "can"]
    if pevs:
      t_end = pevs[-1].logMonoTime
      for e in pevs:
        if e.logMonoTime >= t_end - int(warmup_s * 1e9):
          step(e)
  vals, valids = [], []   # run-length: [start_frame, value]
  n = 0
  for e in evs:
    if e.which() != "can":
      continue
    v, ok = step(e)
    if not vals or vals[-1][1] != list(v):
      vals.append([n, list(v)])
    if not valids or valids[-1][1] != ok:
      valids.append([n, ok])
    n += 1
  return {"segment": seg_num(path), "frames": n, "values": vals, "canValid": valids, "warmed": prev is not None,
          "is_metric": bool(getattr(CI.CS, "is_metric", False)), "flags": int(CP.flags)}


def cmd_dump(args):
  _setup()
  lo, hi = map(int, args.segments.split("-")) if args.segments else (0, 10**6)
  files = segment_files(args.route, "rlog", None if not args.segments else set(range(lo, hi + 1)))
  if not files:
    sys.exit(f"no rlogs for {args.route} in {toolchain.DEV}/logs")
  nums = {seg_num(f): f for f in segment_files(args.route, "rlog")}
  alpha_long = args.alpha_long
  if alpha_long is None:   # follow the drive: CarParams.openpilotLongitudinalControl from the first rlog that has one
    alpha_long = False
    for f in segment_files(args.route, "rlog"):
      cps = [e.carParams.openpilotLongitudinalControl for e in events(f) if e.which() == "carParams"]
      if cps:
        alpha_long = bool(cps[0]); break
  print(f"alpha_long={alpha_long}")
  jobs = [(f, nums.get(seg_num(f) - 1), args.warmup, args.candidate, alpha_long) for f in files]
  import subprocess
  head = subprocess.run(["git", "-C", toolchain.BUILD, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
  segs = pmap(dump_segment, jobs, args.workers)
  out = {"route": short(args.route), "alpha_long": alpha_long, "toolchain": toolchain.TOOLCHAIN, "build": toolchain.BUILD, "head": head, "segments": segs}
  with open(args.out, "w") as f:
    json.dump(out, f)
  print(f"[{toolchain.TOOLCHAIN}] {out['route']} @ {head[:10]}: {len(segs)} segments, {sum(s['frames'] for s in segs)} frames -> {args.out}")


def expand(rle, n):
  out = []
  for i, (start, v) in enumerate(rle):
    end = rle[i + 1][0] if i + 1 < len(rle) else n
    out.extend([v] * (end - start))
  return out


def cmd_compare(args):
  A, B = (json.load(open(p)) for p in (args.old, args.sg))
  print(f"old: {A['toolchain']} {A['head'][:10]}   sg: {B['toolchain']} {B['head'][:10]}   route {A['route']} / {B['route']}")
  sa = {s["segment"]: s for s in A["segments"]}
  sb = {s["segment"]: s for s in B["segments"]}
  if A["route"] != B["route"] or sa.keys() != sb.keys():
    print("FAIL: dumps are for different routes / segment sets")
    return 1
  tol = args.tol
  tot = pos = zero = 0
  mism, runs, check = Counter(), [], Counter()
  cold_n = 0
  for seg in sorted(sa):
    x, y = sa[seg], sb[seg]
    if x["frames"] != y["frames"]:
      print(f"FAIL seg {seg}: frame count {x['frames']} != {y['frames']}")
      return 1
    o, s = expand(x["values"], x["frames"]), expand(y["values"], y["frames"])
    run = None
    for i, (ov, sv) in enumerate(zip(o, s)):
      tot += 1
      old_v = ov[1]
      status, speed = sv
      if old_v > 0:
        pos += 1
        ok = status == "valid" and abs(speed - old_v) <= tol
        kind = "old>0 but sg " + (status if status != "valid" else "valid with different speed")
      else:
        zero += 1
        ok = status != "valid"
        kind = "old==0 but sg valid"
      if ok:
        check["ok"] += 1
        if run:
          runs.append(run); run = None
      else:
        cold = not y["warmed"] and i < args.warm_s * 100
        mism[(kind, "[cold start: no previous segment, parser still learning its timeout]" if cold else "")] += 1
        if cold:
          cold_n += 1
        if run and run[3] == kind:
          run[2] = i
        else:
          if run:
            runs.append(run)
          run = [seg, i, i, kind]
    if run:
      runs.append(run)
  print(f"frames compared {tot} (old>0: {pos}, old==0: {zero}); matches {check['ok']}; mismatches {sum(mism.values())}")
  for k, v in sorted(mism.items()):
    print(f"  {v:7d} {k[0]} {k[1]}")
  longest = max(runs, key=lambda r: r[2] - r[1], default=None)
  print(f"mismatch runs: {len(runs)}; longest {longest[2] - longest[1] + 1 if longest else 0} frames" +
        (f" (seg {longest[0]} frames {longest[1]}-{longest[2]}: {longest[3]})" if longest else ""))
  for r in runs[: args.show]:
    print(f"   seg {r[0]} frames {r[1]}-{r[2]} ({r[2] - r[1] + 1}): {r[3]}")
  warm_bad = sum(v for k, v in mism.items() if not k[1])
  print(f"mismatches outside cold starts: {warm_bad}; inside cold starts (first {args.warm_s} s of a segment with no predecessor): {cold_n}")
  ok = warm_bad == 0
  print("PASS (warm frames; cold-start lag listed above)" if ok else "FAIL")
  return 0 if ok else 1


def cmd_canvalid(args):
  A, B = (json.load(open(p)) for p in (args.base, args.branch))
  print(f"base {A['head'][:10]} vs branch {B['head'][:10]}  route {A['route']} / {B['route']}")
  sa = {s["segment"]: s for s in A["segments"]}
  sb = {s["segment"]: s for s in B["segments"]}
  if A["route"] != B["route"] or sa.keys() != sb.keys():
    print("FAIL: different routes / segment sets")
    return 1
  bad = frames = nvalid = 0
  for seg in sorted(sa):
    x, y = sa[seg], sb[seg]
    if x["frames"] != y["frames"] or x["canValid"] != y["canValid"]:
      bad += 1
      print(f"  seg {seg}: canValid sequence differs: base {x['canValid'][:6]} vs branch {y['canValid'][:6]}")
    frames += x["frames"]
    nvalid += sum(1 for f in expand(x["canValid"], x["frames"]) if f)
  print(f"{frames} frames, canValid true on {nvalid}; segments with differing canValid sequence: {bad}")
  print("PASS" if not bad else "FAIL")
  return 0 if not bad else 1


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd", required=True)
  h = sub.add_parser("hist"); h.add_argument("routes", nargs="+"); h.add_argument("--workers", type=int, default=6)
  d = sub.add_parser("dump")
  d.add_argument("route"); d.add_argument("--out", required=True)
  d.add_argument("--segments", help="A-B inclusive (default: every downloaded rlog segment)")
  d.add_argument("--warmup", type=float, default=5.0, help="seconds of the previous segment fed first (unrecorded)")
  d.add_argument("--candidate", default="KIA_STINGER_2022")
  d.add_argument("--alpha-long", type=lambda x: x.lower() in ("1", "true", "yes"), default=None,
                 help="CarParams openpilotLongitudinalControl for the fingerprint (default: from the log's carParams)")
  d.add_argument("--workers", type=int, default=6)
  c = sub.add_parser("compare"); c.add_argument("old"); c.add_argument("sg")
  c.add_argument("--tol", type=float, default=1e-6); c.add_argument("--show", type=int, default=15)
  c.add_argument("--warm-s", type=int, default=5, help="seconds at the start of a segment with no predecessor reported separately as cold start")
  v = sub.add_parser("canvalid"); v.add_argument("base"); v.add_argument("branch")
  args = p.parse_args()
  if args.cmd == "hist":
    cmd_hist(args)
    return
  sys.exit({"dump": cmd_dump, "compare": cmd_compare, "canvalid": cmd_canvalid}[args.cmd](args))


if __name__ == "__main__":
  main()
