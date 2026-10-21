#!/usr/bin/env python3
"""Auto Stop (Idle Stop and Go) events from rlogs and what openpilot made of them.

The MDPS sets MDPS12 CF_Mdps_ToiUnavail while LVR12 CF_Lvr_IsgState says the engine is auto-stopped. Builds
without the ISG fix report that as steerFaultTemporary and beep steerTempUnavailableSilent. Per event: duration,
engaged and brake state at the start, MDPS unavailable frames, frames carState reported a steering fault, alerts.

Usage: auto_stop.py ROUTE [ROUTE ...] [--starpilot DIR]
"""
import argparse
import sys

from logs import add_starpilot_path, pmap, route_time, segment_files, short, sorted_events

ISG_STOPPED = 1


def scan(path):
  from opendbc.can import CANParser  # imported before the first log read, so logs.py reuses its schema
  p = CANParser("hyundai_kia_generic", [("MDPS12", 0), ("LVR12", 0)], 0)
  out = []; prev = 0; ev = None; enabled = brake = fault = False; alert = ""; t0 = None
  for e in sorted_events(path):
    w = e.which()
    if w == "selfdriveState":
      enabled = e.selfdriveState.enabled; alert = e.selfdriveState.alertType
    elif w == "carState":
      fault = e.carState.steerFaultTemporary; brake = e.carState.brakePressed
      t0 = t0 or e.logMonoTime
    elif w == "can" and t0:
      p.update([(e.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in e.can])])
      isg = int(p.vl["LVR12"]["CF_Lvr_IsgState"])
      if isg == ISG_STOPPED and prev != ISG_STOPPED:
        ev = {"start": route_time(path, e.logMonoTime, t0), "engaged": enabled, "brake": brake,
              "unavail": 0, "fault": 0, "alerts": set()}
      if isg == ISG_STOPPED and ev:
        ev["unavail"] += p.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0
        ev["fault"] += fault
        if "steerTemp" in alert:
          ev["alerts"].add(alert.split("/")[0])
      if prev == ISG_STOPPED and isg != ISG_STOPPED and ev:
        ev["end"] = route_time(path, e.logMonoTime, t0); out.append(ev); ev = None
      prev = isg
  if ev:
    ev["end"] = None; out.append(ev)
  return path, out


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--starpilot", help="StarPilot checkout providing opendbc (default: auto-detect)")
  args = p.parse_args()
  add_starpilot_path(args.starpilot)
  for route in args.routes:
    files = segment_files(route, "rlog")
    if not files:
      print(f"{short(route)}: no rlogs"); continue
    n = faults = 0
    print(f"== {short(route)} ({len(files)} rlogs)")
    for path, evs in pmap(scan, files):
      for ev in evs:
        n += 1; faults += ev["fault"]
        end = f"{ev['end']:.0f}s" if ev["end"] is not None else "seg end"
        print(f"  {ev['start']:6.0f}s-{end:>7} engaged={int(ev['engaged'])} brake={int(ev['brake'])} MDPS unavailable "
              f"frames={ev['unavail']:5d} reported fault frames={ev['fault']:5d} alerts={sorted(ev['alerts']) or '-'}")
    print(f"  {n} auto-stop events (one stop spanning two segments counts twice); reported fault frames {faults}")


if __name__ == "__main__":
  sys.exit(main())
