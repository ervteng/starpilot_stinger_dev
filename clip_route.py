#!/usr/bin/env python3
"""Record a route clip with StarPilot's tools/clip/run.py, with radar tracks forced on.

Usage:
  WT/.venv/bin/python clip_route.py [--worktree WT] ROUTE -s START -e END -q low -d DATA -f MB -o OUT.mp4

  --worktree  clip worktree (default /home/openpilotstuff/wt/clip)
  Everything else is passed straight through to tools/clip/run.py.

run.py restores the params saved in the drive log. Radar tracks are drawn only when
RadarTracksUI is on (selfdrive/ui/onroad/model_renderer.py _draw_radar_tracks), and
drives usually log it off, so it is switched on after the restore. No other param gates it.

Local-data workarounds (run.py assumes a full route): Route() asks the comma API for metadata
even with -d, which needs a login, so that is stubbed; and the params come from the first
qlog that exists rather than segment 0, since a partial data dir has no segment 0.
Replay is also run --headless: its console UI crashes on this route (ctime() of a bogus
initData wall time in consoleui.cc), and it needs a terminal anyway. Its -s is counted from
the first segment present, not from segment 0, so it is shifted by that segment's start.
"""
import importlib.util
import os
import sys

DEFAULT_WT = "/home/openpilotstuff/wt/clip"
FORCE_ON = ("RadarTracksUI",)
DEFAULT_PLAYBACK = "0.5"


def main():
  argv = sys.argv[1:]
  wt = DEFAULT_WT
  if "--worktree" in argv:
    i = argv.index("--worktree")
    wt = argv[i + 1]
    del argv[i:i + 2]
  wt = os.path.abspath(wt)
  # This machine (llvmpipe software GL) renders ~15 fps at 2160x1080, so real-time replay gives a
  # clip at ~15 fps. Half-speed replay lets every 1/20 s of route get its own frame.
  if "--playback" not in argv:
    argv += ["--playback", DEFAULT_PLAYBACK]
  os.chdir(wt)
  sys.path.insert(0, wt)
  os.environ["PYTHONPATH"] = wt + os.pathsep + os.environ.get("PYTHONPATH", "")

  spec = importlib.util.spec_from_file_location("clip_run", os.path.join(wt, "tools/clip/run.py"))
  run = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(run)

  run.Route._get_route_metadata = lambda self: {"url": ""}
  first_seg = []

  def get_logreader(route):
    paths = route.qlog_paths()
    first_seg.append(next(i for i, p in enumerate(paths) if p))
    return run.LogReader(paths[first_seg[0]])

  # run.py only accepts --playback 1..10, but replay itself takes 0.2..8. Below 1x the replay runs
  # slower than real time, so a UI that can't render 20 fps still gets a frame per 1/20 s of route.
  playback = [1.0]

  def validate_playback(value):
    playback[0] = float(value)
    if not 0.2 <= playback[0] <= run.MAX_PLAYBACK:
      raise run.ArgumentTypeError(f"playback must be between 0.2 and {run.MAX_PLAYBACK}")
    return playback[0]

  def managed_proc(cmd, env, _managed_proc=run.managed_proc):
    if cmd[0] == run.REPLAY:
      cmd = cmd + ["--headless"]
      if playback[0] < 1:
        cmd += ["-x", str(playback[0])]
      cmd[cmd.index("-s") + 1] = str(int(cmd[cmd.index("-s") + 1]) - 60 * first_seg[0])
    return _managed_proc(cmd, env)

  run.get_logreader = get_logreader
  run.validate_playback = validate_playback
  run.managed_proc = managed_proc

  orig = run.populate_car_params

  def populate_car_params(lr):
    from openpilot.common.params import Params
    orig(lr)
    params = Params()
    for key in FORCE_ON:
      params.put_bool(key, True)

  run.populate_car_params = populate_car_params
  sys.argv = [run.__file__] + argv
  run.main()


if __name__ == "__main__":
  main()
