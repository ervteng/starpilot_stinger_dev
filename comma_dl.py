#!/usr/bin/env python3
"""Download openpilot logs for a comma connect route.

Usage:
  comma_dl.py ROUTE [SEGMENTS] [--type rlog|qlog] [--out DIR] [--list]

  ROUTE     "dongle|route" or "dongle/route", e.g. 59fa1d41318b87d3|00000004--51331d19d5
  SEGMENTS  "all" (default), "5", "4-7", or "0,3,6"

Public routes need no login. For private routes, export COMMA_JWT (from https://jwt.comma.ai).
Files land in OUT/<dongle>_<route>--<seg>--<type>.<ext>; existing files are skipped.
"""
import argparse
import json
import os
import re
import sys
import urllib.parse
import urllib.request

API = os.environ.get("COMMA_API", "https://api.commadotai.com")
DEFAULT_OUT = os.path.join(os.environ.get("STINGER_DEV_HOME", "/home/user/starpilot-dev"), "logs")
FILE_KEYS = {"rlog": "logs", "qlog": "qlogs"}


def api_get(path: str):
  req = urllib.request.Request(API + path, headers={"User-Agent": "comma_dl"})
  if token := os.environ.get("COMMA_JWT"):
    req.add_header("Authorization", f"JWT {token}")
  with urllib.request.urlopen(req, timeout=60) as r:
    return json.load(r)


def parse_segments(spec: str, available: list[int]) -> list[int]:
  if spec == "all":
    return available
  out = set()
  for part in spec.split(","):
    if "-" in part:
      a, b = map(int, part.split("-"))
      out.update(range(a, b + 1))
    else:
      out.add(int(part))
  return sorted(out)


def seg_num(url: str) -> int:
  # URLs look like .../<dongle>/<route>/<seg>/rlog.zst?<signature>
  m = re.search(r"/(\d+)/[^/]+\?", url) or re.search(r"/(\d+)/[^/]+$", url)
  return int(m.group(1))


def main():
  p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  p.add_argument("route")
  p.add_argument("segments", nargs="?", default="all")
  p.add_argument("--type", choices=FILE_KEYS, default="rlog")
  p.add_argument("--out", default=DEFAULT_OUT)
  p.add_argument("--list", action="store_true", help="only list available segments")
  args = p.parse_args()

  route = args.route.replace("/", "|", 1)
  files = api_get(f"/v1/route/{urllib.parse.quote(route)}/files")
  urls = {seg_num(u): u for u in files.get(FILE_KEYS[args.type], [])}
  if not urls:
    sys.exit(f"no {args.type} files for {route} (keys returned: {sorted(files)})")

  hosts = sorted({urllib.parse.urlparse(u).hostname for u in urls.values()})
  print(f"{route}: {args.type} segments {sorted(urls)} (served from {', '.join(hosts)})")
  if args.list:
    return

  os.makedirs(args.out, exist_ok=True)
  for seg in parse_segments(args.segments, sorted(urls)):
    if seg not in urls:
      print(f"  seg {seg}: no {args.type} uploaded, skipping")
      continue
    ext = os.path.basename(urllib.parse.urlparse(urls[seg]).path).split(".", 1)[-1]
    dest = os.path.join(args.out, f"{route.replace('|', '_')}--{seg}--{args.type}.{ext}")
    if os.path.exists(dest):
      print(f"  seg {seg}: already have {dest}")
      continue
    print(f"  seg {seg}: downloading -> {dest}")
    tmp = dest + ".part"
    with urllib.request.urlopen(urls[seg], timeout=300) as r, open(tmp, "wb") as f:
      while chunk := r.read(1 << 20):
        f.write(chunk)
    os.replace(tmp, dest)


if __name__ == "__main__":
  main()
