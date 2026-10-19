"""Shared helpers: read openpilot rlog/qlog files (zstd, bz2 or raw) with StarPilot's capnp schema."""
import bisect
import bz2
import glob
import os
import re

import capnp
import zstandard

DEV = os.environ.get("STINGER_DEV_HOME", "/home/user/starpilot-dev")
LOG_DIR = os.path.join(DEV, "logs")


def _starpilot_dir() -> str:
  for d in (os.environ.get("STARPILOT_DIR"), "/home/user/starpilot", "/home/user/ervteng/starpilot",
            os.path.join(DEV, "x86"), os.path.join(DEV, "StarPilot")):
    if d and os.path.exists(os.path.join(d, "cereal", "log.capnp")):
      return d
  raise RuntimeError("StarPilot checkout not found; set STARPILOT_DIR")


_log = None


def schema():
  global _log
  if _log is None:
    cereal = os.path.join(_starpilot_dir(), "cereal")
    _log = capnp.load(os.path.join(cereal, "log.capnp"),
                      imports=[cereal, os.path.join(cereal, "include"), os.path.dirname(capnp.__file__)])
  return _log


def read_bytes(path: str) -> bytes:
  data = open(path, "rb").read()
  if path.endswith(".zst"):
    return zstandard.ZstdDecompressor().stream_reader(data, read_across_frames=True).read()
  if path.endswith(".bz2"):
    return bz2.decompress(data)
  return data


def events(path: str):
  """Yield capnp Events; stops quietly at a truncated tail (e.g. the last segment of a route)."""
  it = schema().Event.read_multiple_bytes(read_bytes(path), traversal_limit_in_words=2**63)
  while True:
    try:
      yield next(it)
    except StopIteration:
      return
    except capnp.KjException:
      return


def segment_files(route: str, kind: str = "rlog", segments=None) -> list[str]:
  """Local files written by comma_dl.py for 'dongle|route', sorted by segment number."""
  pat = os.path.join(LOG_DIR, f"{route.replace('|', '_').replace('/', '_')}--*--{kind}.*")
  files = [f for f in glob.glob(pat) if not f.endswith(".part")]
  out = sorted(files, key=seg_num)
  return [f for f in out if segments is None or seg_num(f) in segments]


def seg_num(path: str) -> int:
  return int(re.search(r"--(\d+)--[qr]log", path).group(1))


class Series:
  """Time-ordered samples with 'latest at or before t' lookup."""
  def __init__(self):
    self.t: list[int] = []
    self.v: list = []

  def add(self, t, v):
    self.t.append(t)
    self.v.append(v)

  def at(self, t):
    i = bisect.bisect_right(self.t, t) - 1
    return self.v[i] if i >= 0 else None
