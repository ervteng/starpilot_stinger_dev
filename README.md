# starpilot_stinger_dev

Dev tools for the Kia Stinger 2022 work in [ervteng/StarPilot](https://github.com/ervteng/StarPilot)
(`stinger-radar-lfa`): run StarPilot's Python tests on an x86 machine, download drive logs from comma connect,
and analyze stops and the Stinger radar.

Setting this up on a new machine: see [docs/LOCAL_SETUP.md](docs/LOCAL_SETUP.md) (written for an agent). Design plans for
the Stinger tunes are in [docs/plans](docs/plans).

Everything heavy (venv, x86 builds, logs) lives outside this repo in `$STINGER_DEV_HOME`
(default `/home/user/starpilot-dev`) and is recreated on demand.

## Setup

```bash
./setup_x86.sh
```

- Installs the system headers the native builds need (`libzmq3-dev`, `opencl-headers`, `ocl-icd-opencl-dev`, `libeigen3-dev`).
- Creates a Python 3.12 venv from `requirements.txt` (numpy pinned to StarPilot's `uv.lock`; newer numpy trips `-Werror`).
- Adds a git worktree of StarPilot at `$STINGER_DEV_HOME/x86` and builds the native modules the tests import
  (msgq, params, transformations, pandad, longitudinal MPC) for x86. StarPilot ships these prebuilt for the
  comma device (aarch64), so the main checkout is left untouched.

It finds the StarPilot checkout at `$STARPILOT_DIR`, `/home/user/starpilot` or `/home/user/ervteng/starpilot`,
and clones it otherwise. It is safe to re-run.

For a Claude Code cloud environment, put this in the environment's **Setup script** so each new session starts ready:

```bash
git clone https://github.com/ervteng/starpilot_stinger_dev /home/user/starpilot_stinger_dev 2>/dev/null || true
/home/user/starpilot_stinger_dev/setup_x86.sh
```

## Tests

```bash
./run_tests.sh                                   # stinger-radar-lfa: Hyundai car tests + selfdrive/controls/tests
./run_tests.sh 823a5d5 selfdrive/controls/tests/test_leads.py
./run_tests.sh stinger-radar-lfa opendbc_repo/opendbc/car/hyundai/tests -k stinger
```

The first argument is any StarPilot ref; the x86 worktree is switched to it. Upstream already has a few failing
tests on x86 (NON_SCC firmware fingerprints, `HAS_LDA_BUTTON`, Hyundai button sequences, some lateral tunes), so
compare against the base commit before blaming a change.

## Logs

```bash
./comma_dl.py "59fa1d41318b87d3|00000006--38cf647ea8" --list            # what's uploaded
./comma_dl.py "59fa1d41318b87d3|00000006--38cf647ea8" 1,3,5-8           # rlogs
./comma_dl.py "59fa1d41318b87d3|00000006--38cf647ea8" all --type qlog
```

Public routes need nothing. Private routes need a comma JWT (from https://jwt.comma.ai), either injected by the
environment as the header `Authorization: JWT <token>` for `api.commadotai.com`, or exported as `COMMA_JWT`.
The network must allow `api.commadotai.com` and `commadata2.blob.core.windows.net`.

Files land in `$STINGER_DEV_HOME/logs/<dongle>_<route>--<seg>--<rlog|qlog>.zst`.

### Drive catalog

[`routes/ROUTES.md`](routes/ROUTES.md) lists every drive with its build, longitudinal mode (openpilot or stock ACC),
which segments have rlogs, and notes on what each was used for. `fetch_routes.py` re-downloads by name or tag:

```bash
./fetch_routes.py --tag stock-acc --rlog-only      # stock ACC drives with rlogs
./fetch_routes.py --branch stinger-combined        # every drive on a build
./fetch_routes.py 0000000e--5ebf4636a1 --type qlog
./fetch_routes.py --list --tag auto-stop
```

## Analysis

Run with the venv's Python from `analysis/`:

| Script | What it shows |
|---|---|
| `route_scan.py ROUTE [--kind qlog]` | build, CarParams, per-segment speed, openpilot long %, and stops (openpilot vs driver) |
| `stop_report.py ROUTE [--segments 1,5]` | per openpilot stop: entry speed, radar vs model gap, double stop, hold creep and brake sent, stopped-lead vLead/aLeadK artifacts |
| `stop_window.py LOG START END` | timeline: speed, long state, accel commanded vs sent, planner, radar lead, model lead |
| `radar_check.py RLOG [--radar-interface PATH]` | runs StarPilot's Stinger radar decoder over raw CAN: rates, triplet outcomes, flicker, match vs `liveTracks` |
| `drive_profile.py ROUTE...` | per drive: distance, top speed, openpilot long vs stock ACC, engaged %, stops, notable alerts |
| `alerts.py ROUTE... [--match REGEX]` | alert changes (default steering-unavailable and FCW) with car state and route time |
| `auto_stop.py ROUTE...` (rlog) | Auto Stop events: engaged/brake, MDPS unavailable frames, whether carState reported a steering fault |
| `speed_limit.py ROUTE...` | dashboard speed limit presence, whether SLC used it, map agreement, SLC source and settings |
| `highway_brakes.py ROUTE... [--at T]` | openpilot braking episodes above 45 mph with the likely cause; `--at` prints a lead timeline |
| `lead_braking.py --group NAME ROUTE... --group ...` | ego vs lead decel and gap shrink when the lead brakes, e.g. stock ACC vs openpilot |
| `brake_blend.py ROUTE...` (rlog) | brake light and throttle vs requested accel: where the car switches from coasting to braking |
| `radar_vs_factory.py ROUTE... [--starpilot DIR] [--fine]` (rlog, stock ACC) | decoder vs the factory SCC11 lead: distance, relative speed and lateral by range and ego speed |
| `radar_range_rate.py ROUTE... [--starpilot DIR]` (rlog) | decoder relative speed vs the radar's own range rate on centred tracks (any drive) |
| `radar_vs_vision.py ROUTE...` (rlog) | radar lead vs camera lead by range, near-range offset, low-speed lead source |
| `stock_mimic_check.py [ROUTE...] [--starpilot DIR] [--cache F.npz]` (rlog, stock ACC) | Stinger tunes B and C (stinger-stock-tune) vs the factory SCC fed with the factory target: aReqValue error and lower-jerk agreement by speed, lead and braking groups, against today's openpilot |
| `tune_safety_replay.py ROUTE SEG... KIND` / `--scan ROUTE...` | replays logged openpilot braking through tunes A (today), B and C: peak, delay to -2/-3 m/s^2 and extra distance closer to the lead, for one hard brake or every braking episode of a drive |
| `accel_lag.py [ROUTE...] [--carcontrol]` (rlog, openpilot long) | command -> aEgo lag of the sent accel: cross-correlation lag and delay + first-order tau (basis of longitudinalActuatorDelay) |
| `follow_rates.py [--stock ROUTE...] [--openpilot ROUTE...]` (rlog) | rate of change of the accel command while following a lead, factory SCC vs openpilot long, for far-follow and any lead (basis of the planner slew / lead-follow jerk entries) |
| `lateral_tracking.py ROUTE [ROUTE2] [--kind qlog\|rlog] [--segments 0,3-7] [--speed-edges 20,40,60] [--events]` (qlog or rlog, torque lateral) | lateral tracking of the torque controller for one drive, or two compared (second minus first, 95% bootstrap over segments): lateral-related settings in effect, effective friction threshold, desired vs actual lat accel error (bias, RMS, peg%) by speed x lat-accel band x turn-in/unwind/steady x left/right, lag, straight-road output/error sign changes and 0.2-2 / 2-5 Hz power, hard-curve episodes, driver interventions per hour and km with the situation before each (curve side, phase, understeer/oversteer, driver direction), and learned torque params / liveValid / lateral delay per segment |
| `stopped_lead_approaches.py [--source all\|stock-rlog\|stock-qlog\|op] [--op-catalog] [--list] [--cache DIR] [--starpilot DIR]` (rlog stock ACC, qlog stock/openpilot) | approaches to a stationary lead, factory ACC vs openpilot tunes: onset gap/TTC, decel vs speed and gap, fitted decel = k v^2 / (2 (gap - g0)), creep and final gap, closing events by needed decel, with a per-approach list (route + seconds) |
| `manual_stops.py [--logs DIR]` (qlog, every local route) | the driver's own stops behind a stopped / slow / slowing lead with no longitudinal control engaged: coast before braking, onset gap/TTC, decel vs speed, peak and where it falls, creep, final gap, fitted decel = k v^2 / (2 (gap - g0)) |
| `sim_stopped_lead.py scenarios\|run\|validate\|openloop` (x86 build; `validate` needs the logs) | closed-loop longitudinal sim of the approach to a stopped car on the Stinger: the real `LongitudinalPlanner` + MPC, `LongControl`, car tune A/B/C shaping and the measured delay 0.20 s + lag 0.30 s car response, against a radar lead (optionally detected late). Per-run onset gap, peak decel and where, decel at 20/15/10/5/2 mph, creep, final and min gap, jerk; `validate` compares it with real approaches in the logs, `openloop` replays the planner alone on logged inputs |

Routes can be short names (`0000000e--5ebf4636a1`); the dongle defaults to `$STINGER_DONGLE` (59fa1d41318b87d3).
Scripts that decode CAN or run the radar decoder take opendbc from the StarPilot checkout (`--starpilot` to pick
one, e.g. a worktree of another branch to compare decoder versions).

```bash
cd analysis
P=/home/user/starpilot-dev/venv/bin/python
$P route_scan.py "59fa1d41318b87d3|00000006--38cf647ea8"
$P stop_report.py "59fa1d41318b87d3|00000006--38cf647ea8"
$P stop_window.py /home/user/starpilot-dev/logs/59fa1d41318b87d3_00000006--38cf647ea8--5--rlog.zst 18 28
$P auto_stop.py 0000000d--5e0d1d244b
$P highway_brakes.py 0000000e--5ebf4636a1 --at 712
$P lead_braking.py --group stock 00000028--297af64c9c 00000029--b3c2c13819 --group openpilot 0000000e--5ebf4636a1
$P radar_vs_factory.py 00000034--5f8605e608 --fine
$P lateral_tracking.py 00000011--c7d29db317 00000012--2aada6ce32          # first vs second drive; add --events to list interventions
```
