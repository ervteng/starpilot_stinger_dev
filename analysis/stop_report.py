#!/usr/bin/env python3
"""One line per openpilot stop: stopping entry speed, final radar gap vs model, double stop,
standstill hold (creep, brake sent), and the stopped-lead artifacts (negative vLead, positive aLeadK).

Usage: stop_report.py "dongle|route" [--kind rlog|qlog] [--segments 1,3,5]
"""
import argparse

from load import find_stops, load_long
from logs import segment_files, seg_num

RADAR_TO_CAMERA = 1.52


def report(s, ts):
  t0 = s["t0"]
  rel = lambda t: (t - t0) / 1e9
  cs = [(t, c) for t, c in zip(s["cs"].t, s["cs"].v) if ts - 10 <= rel(t) <= ts + 15]
  pre = [(t, c) for t, c in cs if rel(t) <= ts]
  post = [(t, c) for t, c in cs if rel(t) > ts]

  entry_v = None
  for t, c in pre:
    cc = s["cc"].at(t)
    if cc and cc["state"].startswith("stopping"):
      entry_v = c["v"] if entry_v is None else entry_v
    else:
      entry_v = None

  # double stop: speed dips below 0.8 m/s, then climbs again before the final stop
  reaccel, vmin = 0.0, 99.0
  for t, c in pre:
    if c["v"] < 0.8:
      vmin = min(vmin, c["v"])
    if vmin < 0.8:
      reaccel = max(reaccel, c["v"] - vmin)

  hold, sent = [], []
  for t, c in post:
    cc = s["cc"].at(t)
    if c["gas"] or c["v"] > 0.5 or (cc and cc["state"] == "pid"):
      break
    hold.append((t, c["v"]))
    if (o := s["co"].at(t)) is not None:
      sent.append(o)

  leads = [l for t, l in zip(s["rs"].t, s["rs"].v) if ts - 10 <= rel(t) <= ts + 0.5 and l["status"]]
  min_vlead = min((l["v"] for l in leads), default=float("nan"))
  fake_a = max((l["a"] for l in leads if abs(l["v"]) < 0.4), default=float("nan"))
  t_end = cs[-1][0] if not post else post[min(len(post) - 1, 15)][0]
  lead = s["rs"].at(t_end)
  md = s["md"].at(t_end)
  gap = lead["d"] if lead and lead["status"] else float("nan")
  model_gap = md[1] - RADAR_TO_CAMERA if md and md[0] > 0.5 else float("nan")
  brake = any(c["brake"] for t, c in pre if rel(t) >= ts - 3)
  return (f"entry v {entry_v if entry_v is not None else float('nan'):.2f} | gap radar {gap:4.1f} m model {model_gap:4.1f} m | "
          f"re-accel {reaccel:.2f} m/s{' DOUBLE' if reaccel > 0.4 else ''} | hold {rel(hold[-1][0]) - ts if hold else 0:4.1f}s "
          f"creep {max((v for _, v in hold), default=0):.2f} m/s hold accel sent {sent[-1] if sent else float('nan'):+.2f} | "
          f"lead vLead min {min_vlead:+.2f} aLeadK@stopped max {fake_a:+.2f}{' | driver brake' if brake else ''}")


def main():
  p = argparse.ArgumentParser()
  p.add_argument("route")
  p.add_argument("--kind", default="rlog", choices=("qlog", "rlog"))
  p.add_argument("--segments", type=lambda x: {int(i) for i in x.split(",")})
  args = p.parse_args()
  files = segment_files(args.route, args.kind, args.segments)
  if not files:
    raise SystemExit(f"no {args.kind} files for {args.route}; download with comma_dl.py")
  for f in files:
    s = load_long(f)
    for ts, op, _ in find_stops(s):
      if op:
        print(f"seg {seg_num(f):2d} @{ts:5.1f}s: {report(s, ts)}")


if __name__ == "__main__":
  main()
