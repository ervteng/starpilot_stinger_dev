#!/usr/bin/env python3
"""Run the Stinger 2022 radar decoder from a StarPilot checkout over raw CAN in an rlog and check it:
rates, triplet outcomes, objects per cycle, track flicker, and agreement with the logged liveTracks.

Usage: radar_check.py RLOG [--radar-interface PATH]
  --radar-interface defaults to opendbc_repo/opendbc/car/hyundai/radar_interface.py in the StarPilot checkout,
  so you can point it at another branch's file to compare decoders.
"""
import argparse
import collections
import math
import os
from dataclasses import dataclass

import numpy as np

from logs import _starpilot_dir, events

KPH_TO_MS = 1 / 3.6


def load_decoder(path):
  src = open(path).read()
  block = src[src.index("STINGER_2022_RADAR_BUS ="):src.index("@dataclass(frozen=True)\nclass RadarTrackConfig")]
  ns = {"math": math, "dataclass": dataclass}
  exec(block, ns)
  return ns


def wheel_speed(dat: bytes) -> float:
  v = int.from_bytes(dat, "little")
  return 0.25 * sum(((v >> s) & 0x3fff) * 0.03125 for s in (0, 16, 32, 48)) * KPH_TO_MS


def main():
  p = argparse.ArgumentParser()
  p.add_argument("rlog")
  p.add_argument("--radar-interface")
  args = p.parse_args()
  ri = args.radar_interface or os.path.join(_starpilot_dir(), "opendbc_repo/opendbc/car/hyundai/radar_interface.py")
  ns = load_decoder(ri)
  field, start, count = ns["_field"], ns["STINGER_2022_RADAR_START_ADDR"], ns["STINGER_2022_RADAR_MSG_COUNT"]
  decoder = ns["Stinger2022ObjectDecoder"]()

  frames, outcomes, rate = {}, collections.Counter(), collections.Counter()
  objs, cycles, live = {}, [], []
  v_ego, t_first, t_last = None, None, None

  def classify(a, b, c):
    cy, oid = field(a, 44, 2), field(a, 56, 7)
    if cy != field(b, 0, 2) or cy != field(c, 0, 2):
      return "cycle mismatch"
    if oid != field(b, 57, 7) or oid != field(c, 43, 7):
      return "id mismatch"
    if oid == 0 and field(a, 0, 13) == 0:
      return "empty slot"
    if oid == 0 or field(a, 0, 13) == 0:
      return "id or distance 0"
    return "ok"

  for e in events(args.rlog):
    w = e.which()
    if w == "liveTracks":
      live.append((e.logMonoTime, sorted((round(q.dRel, 2), round(q.yRel, 2)) for q in e.liveTracks.points)))
      continue
    if w != "can":
      continue
    t = e.logMonoTime
    trigger = False
    for m in e.can:
      a, src, dat = m.address, m.src, bytes(m.dat)
      if a == 0x386 and src == 0 and len(dat) == 8:
        v_ego = wheel_speed(dat)
      if src != 1 or not start <= a < start + count or len(dat) != 8:
        continue
      t_first = t_first or t
      t_last = t
      rate[a] += 1
      slot, part = divmod(a - start, 3)
      frames[slot, part] = (t, dat)
      if part == 2:
        if all((slot, i) in frames for i in range(3)):
          trip = [frames.pop((slot, i)) for i in range(3)]
          r = classify(*(d for _, d in trip))
          if max(x for x, _ in trip) - min(x for x, _ in trip) > 20e6:
            r = "span > 20 ms"
          outcomes[r] += 1
        else:
          outcomes["incomplete"] += 1
      if (o := decoder.update(t, a, dat, src)) is not None:
        objs[o.object_id] = o
      if a == start + count - 1:
        trigger = True
    if trigger and v_ego is not None:
      cycles.append((t, v_ego, dict(objs)))
      objs.clear()

  if not rate:
    raise SystemExit("no Stinger radar frames on bus 1 (is this an rlog of a KIA_STINGER_2022?)")
  dur = (t_last - t_first) / 1e9
  hz = np.array(list(rate.values())) / dur
  print(f"radar addrs seen {len(rate)}/{count}, rate {hz.min():.1f}-{hz.max():.1f} Hz over {dur:.1f} s")
  total = sum(outcomes.values())
  print("triplets:", ", ".join(f"{k} {v} ({v / total:.1%})" for k, v in outcomes.most_common()))
  n = [len(c[2]) for c in cycles]
  print(f"cycles {len(cycles)}, objects/cycle mean {np.mean(n):.1f} (min {min(n)}, max {max(n)})")
  ids = [set(c[2]) for c in cycles]
  flicker = sum(1 for i in range(1, len(ids) - 1) for oid in ids[i - 1] - ids[i] if oid in ids[i + 1])
  print(f"single-cycle dropouts {flicker} of {sum(n)} object-cycles")

  if live:
    lt_t = np.array([x[0] for x in live])
    match = compared = 0
    for t, _, o in cycles:
      i = np.searchsorted(lt_t, t)
      if i < len(live) and lt_t[i] - t < 15e6:
        compared += 1
        mine = sorted((round(x.distance, 2), round(-x.lateral_raw, 2)) for x in o.values())
        match += mine == live[i][1]
    print(f"liveTracks: (dRel, yRel) sets match in {match}/{compared} cycles")
  else:
    print("no liveTracks in this log")


if __name__ == "__main__":
  main()
