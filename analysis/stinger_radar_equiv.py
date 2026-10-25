#!/usr/bin/env python3
"""Frame-by-frame equivalence of the Stinger 2022 RadarInterface between two builds, fed the same rlog CAN.

Each build runs in its own process/venv and dumps the RadarData it emits; `compare` then diffs the dumps:

  STINGER_TOOLCHAIN=old ./run_py.sh analysis/stinger_radar_equiv.py dump ROUTE --out /tmp/old.json   # old fork (x86 @ the decoder commit)
  STINGER_TOOLCHAIN=sg  ./run_py.sh analysis/stinger_radar_equiv.py dump ROUTE --out /tmp/sg.json    # SG tree (x86-sg at the port)
  ./run_py.sh analysis/stinger_radar_equiv.py compare /tmp/old.json /tmp/sg.json                     # any python with numpy-free stdlib

`dump` mirrors radar_vs_factory.py: one RadarInterface per segment (fresh state), fingerprint (bus -> address -> length) from the
first 3000 events of that segment, then one `ri.update([(logMonoTime, frames)])` per `can` event in logMonoTime order. Every
non-None return is recorded: (logMonoTime, canError, points). The build dir comes from the toolchain (STARPILOT_BUILD overrides);
its git HEAD is recorded in the dump.

compare checks: same segments / fingerprints / radarUnavailable, same number of emissions at the same logMonoTime, same canError,
same set of points per emission (matched in order) with dRel, yRel, vRel, aRel, yvRel equal within --tol (NaN == NaN), same
`measured`. trackId is compared separately: it is a running counter assigned on first sight, so identical logic gives identical
ids, but a mismatch is reported as a note (ids carry no meaning) and only point order / values decide pass/fail unless --strict-ids.
"""
import argparse
import json
import math
import os
import subprocess
import sys

import toolchain
from logs import pmap, segment_files, seg_num, short, sorted_events

FP_EVENTS = 3000


def pt_row(pt):
  dep = getattr(pt, "deprecated", None)   # sg: aRel / yvRel / measured live in the deprecated group
  src = dep if dep is not None else pt
  return [int(pt.trackId), float(pt.dRel), float(pt.yRel), float(pt.vRel), float(src.aRel), float(src.yvRel), bool(src.measured)]


def dump_segment(job):
  path, alpha_long = job
  toolchain.setup_paths(toolchain.BUILD)
  from opendbc.can import CANParser  # noqa: F401  (before the first log read, so logs.py reuses the schema)
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.radar_interface import RadarInterface
  from opendbc.car.hyundai.values import CAR

  evs = sorted_events(path)
  fp = gen_empty_fingerprint()
  for e in evs[:FP_EVENTS]:
    if e.which() == "can":
      for c in e.can:
        if c.src < 3:
          fp[c.src][c.address] = len(c.dat)
  try:
    CP = CarInterface.get_params(CAR.KIA_STINGER_2022, fp, [], alpha_long, False, False, None)
  except TypeError:
    CP = CarInterface.get_params(CAR.KIA_STINGER_2022, fp, [], alpha_long, False, False)
  radar_bus1 = sorted(a for a in fp[1] if 0x238 <= a < 0x256)
  ri = RadarInterface(CP)
  frames_out = []
  n_can = 0
  for e in evs:
    if e.which() != "can":
      continue
    n_can += 1
    rr = ri.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
    if rr is not None:
      frames_out.append([e.logMonoTime, bool(rr.errors.canError), [pt_row(p) for p in rr.points]])
  return {"segment": seg_num(path), "radarUnavailable": bool(CP.radarUnavailable), "radar_fp_addrs": len(radar_bus1),
          "can_events": n_can, "frames": frames_out}


def cmd_dump(args):
  lo, hi = map(int, args.segments.split("-")) if args.segments else (0, 10**6)
  files = segment_files(args.route, "rlog", None if not args.segments else set(range(lo, hi + 1)))
  if not files:
    sys.exit(f"no rlogs for {args.route} in {toolchain.DEV}/logs")
  head = subprocess.run(["git", "-C", toolchain.BUILD, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
  segs = pmap(dump_segment, [(f, args.alpha_long) for f in files], args.workers)
  out = {"route": short(args.route), "toolchain": toolchain.TOOLCHAIN, "build": toolchain.BUILD, "head": head,
         "alpha_long": args.alpha_long, "segments": segs}
  with open(args.out, "w") as f:
    json.dump(out, f)
  n = sum(len(s["frames"]) for s in segs)
  print(f"[{toolchain.TOOLCHAIN}] {out['route']} @ {head[:10]}: {len(segs)} segments, {n} emissions -> {args.out}")


def same(a, b, tol):
  if math.isnan(a) or math.isnan(b):
    return math.isnan(a) and math.isnan(b)
  return abs(a - b) <= tol


def cmd_compare(args):
  A, B = (json.load(open(p)) for p in (args.a, args.b))
  print(f"A: {A['toolchain']} {A['head'][:10]}   B: {B['toolchain']} {B['head'][:10]}   route {A['route']} / {B['route']}")
  bad = 0
  notes = []
  if A["route"] != B["route"] or A["alpha_long"] != B["alpha_long"]:
    print("FAIL: dumps are for different routes / alpha_long")
    return 1
  sa = {s["segment"]: s for s in A["segments"]}
  sb = {s["segment"]: s for s in B["segments"]}
  if sa.keys() != sb.keys():
    print(f"FAIL: segment sets differ: {sorted(sa.keys() ^ sb.keys())}")
    return 1
  tot_frames = tot_points = id_diff = 0
  max_err = 0.0
  for seg in sorted(sa):
    x, y = sa[seg], sb[seg]
    msgs = []
    for k in ("radarUnavailable", "radar_fp_addrs", "can_events"):
      if x[k] != y[k]:
        msgs.append(f"{k} {x[k]} != {y[k]}")
    fa, fb = x["frames"], y["frames"]
    if len(fa) != len(fb):
      msgs.append(f"emission count {len(fa)} != {len(fb)}")
    seg_id_diff = 0
    seg_points = 0
    for i, (ra, rb) in enumerate(zip(fa, fb)):
      if ra[0] != rb[0]:
        msgs.append(f"frame {i}: logMonoTime {ra[0]} != {rb[0]}")
        break
      if ra[1] != rb[1]:
        msgs.append(f"frame {i}: canError {ra[1]} != {rb[1]}")
        continue
      if len(ra[2]) != len(rb[2]):
        msgs.append(f"frame {i}: {len(ra[2])} points != {len(rb[2])}")
        continue
      for pa, pb in zip(ra[2], rb[2]):
        seg_points += 1
        if pa[0] != pb[0]:
          seg_id_diff += 1
        errs = [abs(u - v) if not (math.isnan(u) or math.isnan(v)) else 0.0 for u, v in zip(pa[1:6], pb[1:6])]
        max_err = max(max_err, *errs)
        if not all(same(u, v, args.tol) for u, v in zip(pa[1:6], pb[1:6])) or pa[6] != pb[6]:
          msgs.append(f"frame {i}: point {pa} != {pb}")
    tot_frames += min(len(fa), len(fb))
    tot_points += seg_points
    id_diff += seg_id_diff
    if msgs:
      bad += len(msgs)
      print(f"  seg {seg}: {len(msgs)} mismatches; first: {msgs[0]}")
    else:
      print(f"  seg {seg}: {len(fa)} emissions, {seg_points} points identical (radarUnavailable={x['radarUnavailable']}, "
            f"{x['radar_fp_addrs']}/30 radar addrs in fingerprint)")
  print(f"total: {tot_frames} emissions compared, {tot_points} points, max |diff| {max_err:.3g}, mismatches {bad}, trackId differences {id_diff}")
  if id_diff:
    print("note: trackId values differ (running counters); order and values were compared")
  ok = bad == 0 and (id_diff == 0 or not args.strict_ids)
  print("PASS" if ok else "FAIL")
  return 0 if ok else 1


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = p.add_subparsers(dest="cmd", required=True)
  d = sub.add_parser("dump")
  d.add_argument("route")
  d.add_argument("--out", required=True)
  d.add_argument("--segments", help="A-B inclusive (default: every downloaded rlog segment)")
  d.add_argument("--alpha-long", action="store_true", help="build the CarParams with openpilot longitudinal requested")
  d.add_argument("--workers", type=int, default=6)
  c = sub.add_parser("compare")
  c.add_argument("a")
  c.add_argument("b")
  c.add_argument("--tol", type=float, default=1e-6)
  c.add_argument("--strict-ids", action="store_true")
  args = p.parse_args()
  sys.exit(cmd_compare(args) if args.cmd == "compare" else cmd_dump(args))


if __name__ == "__main__":
  main()
