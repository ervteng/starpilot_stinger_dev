# starpilot_stinger_dev

Dev tools for the Kia Stinger 2022 work in [ervteng/StarPilot](https://github.com/ervteng/StarPilot)
(`stinger-radar-lfa`): run StarPilot's Python tests on an x86 machine, download drive logs from comma connect,
and analyze stops and the Stinger radar.

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

```bash
cd analysis
P=/home/user/starpilot-dev/venv/bin/python
$P route_scan.py "59fa1d41318b87d3|00000006--38cf647ea8"
$P stop_report.py "59fa1d41318b87d3|00000006--38cf647ea8"
$P stop_window.py /home/user/starpilot-dev/logs/59fa1d41318b87d3_00000006--38cf647ea8--5--rlog.zst 18 28
```
