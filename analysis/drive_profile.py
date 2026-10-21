#!/usr/bin/env python3
"""One line per drive: distance, top speed, stops, longitudinal mode and how much it was engaged, notable alerts.

Mode is openpilot-long, stock-acc (factory SCC with cruise engaged) or stock-capable-unused, from CarParams plus
how often cruise was engaged while moving. Only the segments downloaded locally are counted.

Usage: drive_profile.py ROUTE [ROUTE ...] [--kind qlog|rlog]
"""
import argparse
from collections import Counter

from logs import events, pmap, segment_files, short

ALERTS = ("steerTempUnavailableSilent", "steerTempUnavailable", "steerUnavailable", "fcw", "steerSaturated")


def scan(path):
  r = Counter(); op_long = None; prev_t = None; ss_prev = False; enabled = False; prev_alert = ""
  for e in events(path):
    w = e.which()
    if w == "carParams" and op_long is None:
      op_long = e.carParams.openpilotLongitudinalControl
    elif w == "selfdriveState":
      enabled = e.selfdriveState.enabled
      a = e.selfdriveState.alertType.split("/")[0]
      if a in ALERTS and a != prev_alert:
        r[f"alert:{a}"] += 1
      prev_alert = a
    elif w == "carControl":
      r["cc"] += 1; r["long"] += e.carControl.longActive; r["lat"] += e.carControl.latActive
    elif w == "carState":
      cs = e.carState
      if prev_t is not None and 0 < e.logMonoTime - prev_t < 1e9:
        dt = (e.logMonoTime - prev_t) / 1e9
        r["dist"] += cs.vEgo * dt
        r["stopped_s"] += dt if cs.standstill else 0
      prev_t = e.logMonoTime
      r["vmax"] = max(r["vmax"], cs.vEgo)
      if cs.vEgo > 1:
        r["moving"] += 1; r["cruise"] += cs.cruiseState.enabled
      if cs.standstill and not ss_prev:
        r["stops"] += 1; r["stops_engaged"] += enabled
      ss_prev = cs.standstill
  return op_long, r


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="+")
  p.add_argument("--kind", default="qlog")
  args = p.parse_args()
  for route in args.routes:
    files = segment_files(route, args.kind)
    if not files:
      print(f"{short(route)}: no {args.kind} files"); continue
    tot = Counter(); op_long = None
    for ol, r in pmap(scan, files):
      op_long = ol if op_long is None else op_long
      vmax = max(tot["vmax"], r["vmax"]); tot.update(r); tot["vmax"] = vmax
    cruise = 100 * tot["cruise"] / max(tot["moving"], 1)
    mode = "openpilot-long" if op_long else ("stock-acc" if op_long is False and cruise > 0 else
                                             "stock-capable-unused" if op_long is False else "unknown")
    alerts = {k[6:]: v for k, v in tot.items() if k.startswith("alert:")}
    print(f"{short(route)}: {len(files)} {args.kind}s, {tot['dist'] / 1609:.1f} mi, max {tot['vmax'] * 2.237:.0f} mph, "
          f"{mode} (cruise {cruise:.0f}% of moving), long active {100 * tot['long'] / max(tot['cc'], 1):.0f}%, "
          f"lat active {100 * tot['lat'] / max(tot['cc'], 1):.0f}%, standstills {tot['stops']} "
          f"({tot['stops_engaged']} engaged), {tot['stopped_s'] / 60:.1f} min stopped, alerts {alerts or '-'}")


if __name__ == "__main__":
  main()
