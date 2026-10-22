# Recreating the Stinger StarPilot workspace locally (instructions for an agent)

You are setting up, on the user's own computer, the workspace that was previously used in a cloud session for the
user's Kia Stinger 2022 StarPilot fork. Everything that matters is in two GitHub repos; logs and build outputs are
re-created. Follow the steps in order, report what you did, and stop and ask before anything not listed here.

## 0. Ground rules (carry these over)
- Owner: GitHub user `ervteng`. Commits are authored as the user and committed as Claude:
  `GIT_AUTHOR_NAME=ervteng GIT_AUTHOR_EMAIL=5085265+ervteng@users.noreply.github.com git commit ...`
  Every commit message ends with a `Co-Authored-By:` trailer for the model you are, plus any session link your
  environment asks for. Don't copy the previous session's link.
- Push with `git push -u origin <branch>`. No force-push except on the dev repo when the user asks. No PRs unless asked:
  PRs to FireStar (firestar5683/StarPilot) are on hold until FireStar releases SecretGoodStarPilot.
- Never use bare `git stash` (worktrees share the stash). Use temporary WIP commits.
- `stinger-combined` is the user's daily build. Nothing goes into it without the user's explicit OK.
- The comma connect token is the env var `COMMA_JWT` (value starts with `JWT `, from https://jwt.comma.ai). Never ask
  the user to paste it into chat, never print it; scripts read it from the environment.
- The user prefers: plan first, then delegate implementation to Sonnet subagents; every tuning number must be based on
  measured data (no guesses); reuse existing StarPilot code over new code.

## 1. Repositories
| Repo | Purpose |
|---|---|
| `https://github.com/ervteng/StarPilot` (fork of `firestar5683/StarPilot`) | the code; branches below |
| `https://github.com/ervteng/starpilot_stinger_dev` (this repo) | analysis scripts, log downloaders, x86 test env, drive catalog, plans |

StarPilot branches (all on `origin`):
| Branch | What it is |
|---|---|
| `stinger-combined` | **daily build**: radar/LFA + dashboard speed limit + Auto Stop (ISG) fix + radar speed-bias fix + Testing Ground slot 9 (long tune A/B/C) + slot 10 (stopped-lead tune) + FLM overrides fix |
| `stinger-radar-lfa-clean` | PR candidate: radar decoder, LFA button, stop undershoot fix, test move, speed-bias fix |
| `stinger-radar-lfa` | older working branch (G90 stop tune) |
| `hkg-can-dashboard-speed-limit`, `hkg-isg-steer-fault` | standalone PR candidates |
| `stinger-stock-tune` | slot 9 tunes B (GV70-style) and C (Ioniq 6 as-is), based on StarPilot 823a5d5 |
| `stinger-stopped-lead` | slot 10 stopped-lead obstacle bias, based on StarPilot 823a5d5 |
| `flm-overrides-json-fix` | one-fix branch for FireStar: FLMActiveOverrides was json.loads'ed twice so FLM profiles never applied |
| `stinger-long-tuning` | superseded sunnypilot-port experiment (reference only) |

Optional reference clones (read-only, for comparisons): sunnypilot `https://github.com/sunnypilot/sunnypilot.git`
(branches `master`, `dev`), CarrotPilot `https://github.com/ajouatom/openpilot.git` (branch `carrot-wip`).

## 2. Layout
Pick a base folder (ask the user; default `~/stinger`). Recreate this layout under it:
```
BASE/
  starpilot/                  # git clone of ervteng/StarPilot (main checkout)
  wt/<name>/                  # git worktrees, one per branch you work on (git worktree add)
  starpilot_stinger_dev/      # git clone of ervteng/starpilot_stinger_dev
  starpilot-dev/              # = $STINGER_DEV_HOME: venv/, x86/ (built test worktree), logs/
  ref/                        # optional reference clones
```
Commands:
```bash
BASE=~/stinger; mkdir -p $BASE/wt $BASE/ref && cd $BASE
git clone https://github.com/ervteng/StarPilot starpilot          # large; --filter=blob:none is fine
git clone https://github.com/ervteng/starpilot_stinger_dev
cd starpilot && git fetch origin && \
  git worktree add $BASE/wt/stinger-combined stinger-combined     # add others only as needed
```
StarPilot has submodules-like vendored dirs (opendbc_repo, etc.) already in the tree; no `submodule update` needed
unless a build step says otherwise.

## 3. Environment variables
Add to the shell profile:
```bash
export STINGER_DEV_HOME=$BASE/starpilot-dev
export STARPILOT_DIR=$BASE/starpilot
export COMMA_JWT=...            # the user sets this themselves; do not write it for them
```

## 4. Build the x86 test environment
`starpilot_stinger_dev/setup_x86.sh` builds a Python venv and an x86 worktree of StarPilot (`$STINGER_DEV_HOME/x86`)
with the native modules the tests and simulator need (msgq, params, transformations, pandad, long MPC/acados).
StarPilot ships prebuilt aarch64 (comma device) binaries, so this x86 build is required on a PC.
- It is written for Debian/Ubuntu (apt). On WSL2 it should work as is. On macOS it needs adapting (brew packages for
  zmq/OpenCL/Eigen, clang); report what breaks instead of guessing.
- Run: `STARPILOT_REF=stinger-combined ./setup_x86.sh` from the dev repo. Then verify:
  `./run_tests.sh stinger-combined starpilot/common/tests/test_starpilot_variables.py` -> all pass.
- `run_tests.sh <ref> <pytest args>` checks out `<ref>` (local or origin) detached in the x86 worktree and runs pytest.
  Pure-Python opendbc tests can also run straight from a worktree:
  `PYTHONPATH=$PWD:$PWD/opendbc_repo $STINGER_DEV_HOME/venv/bin/python -m pytest -q -n 8 -p no:cacheprovider opendbc_repo/opendbc/car/hyundai/tests`

Known pre-existing test failures (not ours; also fail on FireStar's StarPilot):
- Hyundai suite: `test_fw_format`, `test_fuzzy_excluded_platforms`, `test_platform_code_ecus_available`,
  `test_hyundai_lkas_button_sets_starpilot_safety_flag` (15 failed incl. subtests).
- `selfdrive/controls/tests/test_starpilot_planner.py::test_force_stop_jerk_scale_is_platform_specific`.
- `selfdrive/controls/tests/test_latcontrol.py`: `test_bolt_2022_2023_low_speed_center_output_limit`,
  `test_palisade_center_output_taper_curve`, `test_genesis_g70_center_chatter_tune`.

## 5. Fix hard-coded paths in the dev repo
Most scripts use `$STINGER_DEV_HOME` / `$STARPILOT_DIR` with `/home/user/...` fallbacks (the cloud paths). A few
default `--starpilot` to `/home/user/wt/stock-tune`: `analysis/accel_lag.py`, `analysis/follow_rates.py`,
`analysis/stock_mimic_check.py`, `analysis/tune_safety_replay.py`. Also `analysis/logs.py`, `setup_x86.sh`,
`run_tests.sh`, `comma_dl.py` have `/home/user` fallbacks. Change those defaults to environment-based ones (e.g.
`$STARPILOT_DIR`, or a worktree under `$BASE/wt`), keep behaviour otherwise identical, commit and push to the dev repo.

## 6. Logs
Drive logs are not in git (~4 GB). Dongle `59fa1d41318b87d3`. Download with the token in the env:
```bash
cd starpilot_stinger_dev
./fetch_routes.py --list                         # catalog in routes/ROUTES.md + routes.json (up to Oct 20)
./fetch_routes.py --tag stock-acc --rlog-only    # stock-ACC reference drives (radar/SCC validation)
./comma_dl.py "59fa1d41318b87d3|00000012--2aada6ce32" all --type qlog   # routes newer than the catalog
```
Routes used most (full names; the counter resets, so short names repeat):
- Stock ACC with rlogs: `0000001d--2ead59d2dd`, `00000028--297af64c9c`, `00000029--b3c2c13819`, `00000034--5f8605e608`.
- openpilot long: `0000000d--5e0d1d244b`, `0000000e--5ebf4636a1` (old build), `0000000f--79cf51e7fa` (tune A),
  `00000011--c7d29db317` (tune B), `00000012--2aada6ce32` (tune C, FLM profile marked applied but not effective).
- Older: `0000000a--a4b564ef5e` (FCW), see ROUTES.md.
Regenerate the catalog for newer routes with the same approach as `routes/` (ask the user first).

## 7. Context: where the project stands
Read `README.md` (script index) and `docs/plans/*.md`. Summary:
- **Slot 9 "Stinger Long Tune"** (Testing Grounds, Galaxy): A = installed, B = GV70-style (braking shaped to stock SCC,
  acceleration as today, urgent braking <= -1.0 not smoothed), C = Ioniq 6 tune as-is. Delay 0.5 s for B/C.
- **Slot 10 "Stinger Stopped Lead"**: B = stopped-lead obstacle bias (stop ~3.4-4.4 m behind instead of 4.3-6.3 m).
  Its early/crawl shape is limited by StarPilot's generic `get_close_lead_brake_cap`; a narrow Stinger-only skip of that
  cap is **paused by the user**; a low-speed crawl fix (CR-V-style cap) was proposed, not started.
- **Lateral**: settings SteerLatAccel 2.75, SteerFriction 0.12, auto steer delay. Analysis (`analysis/lateral_tracking.py`)
  suggests SteerFriction ~0.10 and, via FLM, higher low-speed friction thresholds (~0.40-0.50 at 0/5 m/s). The FLM
  overrides bug is fixed on `stinger-combined`; the user's `cleanup_pass:assertive` profile will now apply.
- Simulator: `analysis/sim_stopped_lead.py` (real planner + MPC on the x86 build; see its docstring and
  `analysis/stopped_lead_results.md`). Replays: `planner_delay_replay.py`, `tune_safety_replay.py`, `stock_mimic_check.py`.
- Open questions the user has not decided: closer stop target, squashing the `-clean` commits, when to PR upstream.

## 8. Done when
- Both repos cloned, `stinger-combined` worktree present, env vars set (token by the user).
- `setup_x86.sh` built; `run_tests.sh stinger-combined <a test file>` passes with only the known failures.
- Hard-coded paths fixed and pushed in the dev repo.
- Logs for the routes in section 6 downloaded (or the user chose to skip).
- A short report to the user of what was done and anything that needed adapting (e.g. macOS).
