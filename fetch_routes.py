#!/usr/bin/env python3
"""Download logs for drives listed in routes/routes.json, selected by name, tag, branch or longitudinal mode.

Usage:
  fetch_routes.py [ROUTE ...] [--tag TAG] [--branch BRANCH] [--mode MODE] [--rlog-only] [--type rlog|qlog|both] [--list]

  ROUTE      short "name--hash" (e.g. 0000000e--5ebf4636a1) or full "dongle|name--hash"
  --tag      e.g. stock-acc, auto-stop, speed-limit, highway, radar, stops, g90-tune (see routes/ROUTES.md)
  --branch   build branch the drive ran, e.g. stinger-combined
  --mode     openpilot-long, stock-acc or stock-capable-unused
  --rlog-only  only drives that have rlogs uploaded
  --type     both (default) fetches rlogs where uploaded plus all qlogs

Selections combine (all must match). Downloads go through comma_dl.py, so the same auth and output folder apply.
"""
import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("routes", nargs="*")
  p.add_argument("--tag")
  p.add_argument("--branch")
  p.add_argument("--mode")
  p.add_argument("--rlog-only", action="store_true")
  p.add_argument("--type", choices=("rlog", "qlog", "both"), default="both")
  p.add_argument("--list", action="store_true", help="only print the matching drives")
  args = p.parse_args()

  catalog = json.load(open(os.path.join(HERE, "routes", "routes.json")))
  selected = []
  for r in catalog["routes"]:
    short = r["route"].split("|")[1]
    if args.routes and short not in args.routes and r["route"] not in args.routes:
      continue
    if args.tag and args.tag not in r["tags"]:
      continue
    if args.branch and r["branch"] != args.branch:
      continue
    if args.mode and r["longitudinal"] != args.mode:
      continue
    if args.rlog_only and not r["rlog_segments"]:
      continue
    selected.append(r)

  if not selected:
    sys.exit("no drives match")

  for r in selected:
    print(f"{r['route']}  {r['start_pdt']}  {r['segments']} segs  {r['branch']} {r['commit']}  {r['longitudinal']}  "
          f"rlogs: {r['rlog_segments'] or '-'}")
  if args.list:
    return

  for r in selected:
    kinds = []
    if args.type in ("rlog", "both") and r["rlog_segments"]:
      kinds.append(("rlog", r["rlog_segments"]))
    if args.type in ("qlog", "both"):
      kinds.append(("qlog", "all"))
    for kind, segs in kinds:
      subprocess.run([sys.executable, os.path.join(HERE, "comma_dl.py"), r["route"], segs, "--type", kind], check=False)


if __name__ == "__main__":
  main()
