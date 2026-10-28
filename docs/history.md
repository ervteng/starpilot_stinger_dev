# Project history (archived handover, frozen 2026-10-26)

This is the full workspace handover as it stood on 2026-10-26, kept for reference. It includes reverted designs and superseded plans. For the current state, see `/home/openpilotstuff/HANDOVER.md` on the workstation. For the rules, see `AGENTS.md` there.

---

## Handover: Kia Stinger 2022 StarPilot workspace (as of 2026-10-23, ~00:00 PDT)

You are taking over work on the user's Kia Stinger 2022 fork of StarPilot (an openpilot fork by FireStar).
The machine is Ubuntu 26.04 on WSL2, running as root.

## 1. Ground rules (from the user and `starpilot_stinger_dev/docs/LOCAL_SETUP.md`)

**Commits and pushes**
- Author commits as the user and commit as the assistant:
  `GIT_AUTHOR_NAME=ervteng GIT_AUTHOR_EMAIL=5085265+ervteng@users.noreply.github.com GIT_COMMITTER_NAME=<you> GIT_COMMITTER_EMAIL=<you> git commit ...`
- End each message with a Co-Authored-By trailer for your model.
- Push with `git push -u origin <branch>`. `gh` is logged in as `ervteng` over HTTPS.
- Never force-push. Never rewrite pushed history; revert instead.
- Never use bare `git stash`, because worktrees share it. Use WIP commits.
- No PRs to FireStar (`firestar5683/StarPilot`) until FireStar releases SecretGoodStarPilot.

**The daily build**
- `stinger-combined` is the user's daily build. Nothing goes into it without the user's explicit OK.
- The user wants Stinger-only changes to be gated to `KIA_STINGER_2022`, not generic.

**The comma token**
- The token is in `/home/openpilotstuff/.comma_jwt` (an `export COMMA_JWT=...` line).
- Never read, print or copy it. Only source it inside scripts that download, for example:
  `bash -c "set -a; . /home/openpilotstuff/.comma_jwt; set +a; ..."`

**How to work**
- Plan first, then delegate implementation to a cheaper model.
- Every tuning number must come from measured data (factory ACC logs, replays). No guessing.
- Reuse existing StarPilot per-car hooks instead of writing new mechanisms.
- Fix acceptance criteria before running evaluations. Don't move them afterwards.

## 2. Layout and environment

Base folder: `/home/openpilotstuff`.

**Repos**

| Path | What it is |
|---|---|
| `starpilot/` | clone of github.com/ervteng/StarPilot (main checkout on branch `StarPilot`) |
| `starpilot_stinger_dev/` | clone of github.com/ervteng/starpilot_stinger_dev: analysis scripts, downloaders, docs |

**Worktrees under `wt/`**

| Worktree | Branch / commit |
|---|---|
| `stinger-combined` | daily build |
| `stock-tune` | branch `stinger-stock-tune`, the slot 9 long-tune branch |
| `long-mpc-lead-filter-sync` | generic version of the filter fix, upstream PR candidate |
| `stinger-far-follow-slew` | far-follow slew experiment, NOT merged |
| `ui-radar-camera-offset` | UI marker offset fix, not merged |
| `clip` | detached at `f6f89ce96`; used for video clips; has its own `.venv` and a dirty build tree |

**`starpilot-dev/`** (this is `$STINGER_DEV_HOME`)
- `venv/`: Python 3.12.
- `x86/`: built test/replay worktree, normally detached at `f6f89ce96`. Restore it there after use.
  `third_party/acados/x86_64/lib/libblasfeo.so` needed `patchelf --clear-execstack` (marked skip-worktree) because the newer glibc rejects an executable stack.
- `logs/`: rlogs and qlogs, about 3 GB.
- `video/`: qcamera files.
- `clipdata*/`: symlink folders for the clip tool.

**Environment**
- Env: `export STINGER_DEV_HOME=/home/openpilotstuff/starpilot-dev STARPILOT_DIR=/home/openpilotstuff/starpilot`. These are also in `~/.bashrc`.
- Python: `$STINGER_DEV_HOME/venv/bin/python`.
- Tests: `starpilot_stinger_dev/run_tests.sh <ref> <pytest paths>`. It checks the ref out in `x86`.
- Known pre-existing failures, which also fail upstream:
  - `test_hyundai_button_sequences` (19)
  - `test_curve_speed_controller` (2)
  - `test_latcontrol` (3)
  - `test_starpilot_planner::test_force_stop_jerk_scale_is_platform_specific`
  - the `test_lateral_mpc` import error
  - 4 Hyundai fingerprint tests

**Downloads**
- `comma_dl.py ROUTE SEGS --type rlog|qlog|qcamera|fcamera`.
- `fetch_routes.py --tag ...`.
- Dongle `59fa1d41318b87d3`.
- Route catalog: `starpilot_stinger_dev/routes/`. It only covers drives up to Oct 20.

## 3. Branch state (pushed to origin)

- **`stinger-combined`** (daily build, at `ed11c71a4`; Gemini's `8e9cbdf84` is REVERTED, see 4b)
  - Slot 9 tunes B/C, slot 10 stopped-lead tune, radar decoder, speed limit, ISG fix and FLM fix.
  - `6ee82e684` → revert `25165dae1` → `623729c81` (4a: Stinger-only `use_model_lead_filter_sync`).
  - `8e9cbdf84` (4b: Slot 9 Tune B highway lead-braking overbraking attenuation: `use_stopped_lead_position`, `get_kia_stinger_2022_dynamic_t_follow`, `get_kia_stinger_2022_close_lead_brake_cap_max_ttc`, and `get_kia_stinger_2022_extrapolate_lead`).
- **`stinger-stock-tune`** (slot 9 only, at `030b74ba6`; `d898d7885` is REVERTED, see 4b)
  - Ends `4b8c569ed`, then `829d580b7` → revert `5b7beb078` → `bd041f20f` (4a) → `d898d7885` (4b).
- **`long-mpc-lead-filter-sync`** (`52724e416`): the generic fix on `StarPilot`, with a test. Kept as an upstream candidate.
- **`stinger-far-follow-slew`** (`8ad9138f3`): far-follow brake slew 1.5 / release 2.2 m/s³ for slot 9 B/C.
  - **NOT merged.** It failed one acceptance criterion: 0.544 m extra gap lost against a 0.5 m limit, in one lead-braking onset (route 15 at 2154 s).
  - It changed almost nothing: the hard-brake count was the same, and it bound on about 1% of far-follow frames.
- **`ui-radar-camera-offset`** (`39aafab7`): lead markers and radar dots drawn at `dRel + RADAR_TO_CAMERA` (big UI and mici UI), with tests. Not merged.
  - On the Stinger the true camera-to-radar distance is about 2.05 m against the 1.52 m constant, so markers still sit about 0.5 m short.
- **Dev repo `starpilot_stinger_dev`, `main` at `1b4057d`** (pushed: path fixes). **Uncommitted work:**
  - `comma_dl.py`: qcamera/fcamera types.
  - `clip_route.py`: wrapper for openpilot `tools/clip/run.py`. It works offline, turns radar tracks on, uses half-speed playback for smooth output, and fixes the seek offset.
  - `analysis/radar_first_seen.py`, `highway_overbrake.py` + results, `follow_closing_compare.py` + results, `scan_mpc_drops.py`, `scan_stale_filter.py`.
  - Opt-in flags added to `planner_delay_replay.py` (`--dump`, `--sync-filters`, `--testing-ground 9:B`), `sim_stopped_lead.py` (`--exp-crv-cap`, `--testing-ground`) and `highway_brakes.py` (one-line cruise field).
  - Ask the user before committing these.

## 4. In flight and next steps

### 4a. DONE and verified: the Stinger-only lead-filter sync

- `stinger-combined` = 6ee82e684 → revert `25165dae1` → `623729c81`.
- `stinger-stock-tune` = 829d580b7 → revert `5b7beb078` → `bd041f20f`.
- Both are pushed. Each branch's net diff from before the fix is only `use_model_lead_filter_sync` (+ hyundai `KIA_STINGER_2022`) plus 2 tests. `long_mpc.py` is identical to upstream.
- Tests: no new failures.
- Replay of route 15 at 1836 s on the new head: peak -0.59 (was -2.0 logged).
- `x86` is back at `f6f89ce96`.

The text below is the original brief, kept for reference.

The user asked that the lead-filter fix apply only to the Stinger (alongside the Corolla), not to all cars. A subagent was told to do this on both `stinger-combined` and `stinger-stock-tune`:
1. `git revert` the generic cherry-pick (6ee82e684 / 829d580b7).
2. Add a new commit making `use_model_lead_filter_sync(CP)` in `selfdrive/controls/lib/longitudinal_vehicle_tunes.py` return True for hyundai `KIA_STINGER_2022` as well as toyota `TOYOTA_COROLLA_TSS2`, with tests.
3. Run tests, replay-verify route 15 at 1836 s, then push with normal fast-forward pushes.

It may or may not have finished. Check:
- `git -C /home/openpilotstuff/starpilot fetch origin && git log --oneline -4 origin/stinger-combined origin/stinger-stock-tune`
- `git diff f6f89ce96 origin/stinger-combined -- selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py` must be empty, so `long_mpc.py` is back to upstream.
- `git diff 4b8c569ed origin/stinger-stock-tune` should touch only the hook and tests.
- `git -C /home/openpilotstuff/starpilot-dev/x86 log -1 --format=%h` must be `f6f89ce96`, with no leftover cherry-pick state.

Finish any missing step yourself, following the same rules.

**Why this fix exists**
- In `long_mpc.py::process_lead`, the raw-path `lead_a_filter` / `lead_v_filter` (initially 0) are only updated on the model-lead path when `sync_model_lead_filters` is on.
- When radar `aLeadK` drops below -0.5, `build_model_lead_trajectory` returns None. The raw path then reads the stale filter.
- Route `00000015--2b757ccb00` at 1836.35 s: vLead 23.5 m/s, filter 2.6 m/s, a phantom brake to -2.0 m/s² (unclipped -5.4).
- With sync on, it's -0.43.

### 4b. REVERTED: it failed closed-loop safety testing. Redo it in pieces.

**Status**
- Reverted on both branches (normal revert commits, no history rewrite, user's OK):
  - `stinger-combined` `8e9cbdf84` → revert `ed11c71a4`
  - `stinger-stock-tune` `d898d7885` → revert `030b74ba6`
- Both trees are now byte-identical to `623729c81` / `bd041f20f`.

**Why it was reverted**
- Test: `analysis/sustained_lead_brake.py`, results in `analysis/sustained_lead_brake_results.md` (both uncommitted). A lead braking steadily to a stop at -1.5..-3.0 m/s² from 45/55/65 mph at 1.5/1.9/2.3 s headway, standard and aggressive, 72 scenarios.
- Acceptance criteria, fixed in advance: no new contact; min gap ≥ before − 0.25 m; peak decel ≤ before + 0.3 m/s².
- Result: 32 of 72 failed.
  - 7 new contacts: aggressive at 55 mph, lead -2.0/-2.5, at every headway. Before stopped about 8-10 m short; after hit at 14-29 mph.
  - Min gap was as much as 9.9 m worse.
  - Peak decel was up to +0.8 m/s² harder (for example 1.90 → 2.66 at 45 mph with the lead at -1.5).
- Cause: `get_kia_stinger_2022_extrapolate_lead` (min tau 1.2) assumes the lead's braking decays within about 2 s. It only falls back to the constant-decel prediction at TTC < 6 s, and its emergency fallback needs headway < 1.8 s, so it never triggers at the user's normal 1.9 s. The car then brakes late and hard.
- The intended benefit is real: when the lead slows briefly (-1 m/s² for 2 s), peak braking drops 30-45%.

**Sim caveat**
- The sim caps planner decel at the personality's logged `minAcceleration` (-0.5 standard / -2.0 aggressive). On the road that relaxes in danger (route 15 reached -3.5), so absolute contact counts are pessimistic.
- The before/after delta is still valid. Fix the sim's min-accel model before trusting absolutes.

**Process issues to avoid next time**
- Ship one mechanism per commit, each through this harness.
- Give every constant a measured-basis comment.
- Save the factory-measurement script and results. The "SCC11, 163 events, ego/lead 0.37" analysis was never saved.
- Don't change behaviour for un-requested cases: the `t_follow` give-up also triggered when merely closing.
- Don't claim slot-9-B-only gating when `use_stopped_lead_position` was ungated.

**Next (needs the user's go-ahead)**
1. `t_follow` give-up only while the lead is braking (the user's original ask).
2. A lead-prediction decay whose fade depends on how long and how hard the lead has been braking.
3. The close-cap TTC change, evaluated separately.

Each must pass the sustained-braking matrix and still show the slowdown benefit.

---

The previous (reverted) text follows, for reference.

### (reverted) attenuate highway lead-braking overbraking (Slot 9 Tune B)

- `stinger-stock-tune` = `d898d7885` (pushed).
- `stinger-combined` = `8e9cbdf84` (pushed, with user's OK).
- `x86` is back at `f6f89ce96`.
- `long_mpc.py` remains 100% identical to upstream StarPilot; all changes live in `selfdrive/controls/lib/longitudinal_vehicle_tunes.py`, `selfdrive/controls/lib/longitudinal_planner.py`, and `selfdrive/controls/tests/test_longitudinal_planner.py` (537 passed).

**What was found and implemented (gated to `KIA_STINGER_2022` Slot 9 Tune B; `use_stopped_lead_position` for `KIA_STINGER_2022`)**
- Factory ACC ground truth from `SCC11` (`ACC_ObjDist` / `ACC_ObjRelSpd` across 4 stock rlog drives: `0000001d`, `00000028`, `00000029`, `00000034`, 163 highway lead-braking events $\ge 15\text{ m/s}$):
  - Median $a_{\text{ego}} / a_{\text{lead}}$ is $0.37\text{–}0.38$ for moderate braking ($-0.4$ to $-1.2\text{ m/s}^2$, $81\text{–}93\%$ stay in throttle-off coast $> -0.6\text{ m/s}^2$), rising to $0.74\text{–}0.83$ for harder braking ($< -1.2\text{ m/s}^2$), with headway compressing by up to $0.20\text{–}0.35\text{ s}$.
- Why changing `get_dynamic_t_follow` alone wasn't enough:
  1. `radard.py` decays `aLeadTau` toward `0.0` whenever $|a_{\text{lead}}| \ge 0.5\text{ m/s}^2$, and StarPilot's `long_mpc.py::extrapolate_lead` sets `exp_weight = 0.0` above 35 mph—projecting 10 seconds of constant deceleration to a dead freeway stop as soon as `aLeadK < -0.5` drops out of `build_model_lead_trajectory`.
  2. `LongitudinalPlanner.get_close_lead_brake_cap` (`CLOSE_LEAD_BRAKE_CAP_MAX_TTC = 25.0 s`) bypasses MPC and ignores `t_follow`, clamping `output_a_target` and `self.a_desired` (`mpc.x0[2]`) at 40–70 m highway gaps.
  3. `LongitudinalPlanner.get_dynamic_t_follow` inflated `effective_t_follow` by $+0.12$ to $+0.32\text{ s}$ on Route 15 lead slowdowns.
- Four hooks added in `longitudinal_vehicle_tunes.py` (Tune B only, highway ramp $16.0 \to 18.0\text{ m/s}$ / $36 \to 40\text{ mph}$):
  1. `use_stopped_lead_position(CP)`: `True` for `KIA_STINGER_2022` (prevents backwards lead integration after stopping in `extrapolate_lead`).
  2. `get_kia_stinger_2022_extrapolate_lead`: Applies exponential decay (`min_tau = 1.2`) on the highway with `closing_fade`, fading back to constant-decel when $v_{\text{lead}} < 6\text{ m/s}$, $\text{TTC} < 6\text{ s}$, or in emergency braking ($\ge 2.8\text{–}3.5\text{ m/s}^2$ at $< 1.8\text{ s}$ headway).
  3. `get_kia_stinger_2022_close_lead_brake_cap_max_ttc`: Lowers highway `max_ttc` from $25.0\text{ s}$ to $5.0\text{ s}$ during moderate braking, ramping back to $25.0\text{ s}$ for emergency braking ($\ge 2.8\text{–}3.2\text{ m/s}^2$ at $\le 1.85\text{ s}$ headway or $\text{TTC} \le 10\text{ s}$).
  4. `get_kia_stinger_2022_dynamic_t_follow`: Gives up to $0.22\text{ s}$ below `base_t_follow` (floored at $\min(\text{base\_t\_follow}, 0.95\text{ s})$), fading out when $\text{TTC} < 6\text{ s}$ or lead decel $\ge 3.0\text{ m/s}^2$.
- Validation summary:
  - Open-loop Route 15 replay: $30\%\text{–}50\%$ peak decel reduction across moderate slowdowns ($451\text{ s}$: $-1.56 \to -0.89$, $714\text{ s}$: $-1.54 \to -0.77$, $836\text{ s}$: $-2.02 \to -1.09$, $859\text{ s}$: $-3.35 \to -2.40$, $1951\text{ s}$: $-1.29 \to -0.73$, $2155\text{ s}$: $-3.39 \to -2.55\text{ m/s}^2$).
  - Closed-loop Route 15 tracks (`sim_stopped_lead.py`): $24\%\text{–}58\%$ softer car decel (`aEgo`) on $451\text{ s}$, $714\text{ s}$, $836\text{ s}$, $1951\text{ s}$; zero regression on $2155\text{ s}$ ($34\text{ mph}$); $0.00\text{ s}$ latency and identical $2.65\text{ m}$ stop gap on the $65\text{ mph}$ $-4.0\text{ m/s}^2$ emergency stop; identical $15 / 25 / 35\text{ mph}$ stopped-lead matrix.

### 4c. Mid-term platform roadmap & Rebase Plan onto `SecretGoodStarPilot`
- **Target branch:** `upstream/SecretGoodStarPilot` (fetched at `202a52c353`).
- If Comma releases a new small driving model later, either shift to **SunnyPilot** or run that model in StarPilot.

#### Architectural Changes Discovered in `upstream/SecretGoodStarPilot` (`202a52c353`) vs `upstream/StarPilot` (`823a5d5674`)
1. **Repo Layout:**
   - `selfdrive/`, `starpilot/`, and `cereal/` moved under `openpilot/` (`openpilot/selfdrive/`, `openpilot/starpilot/`, `openpilot/cereal/`).
2. **`opendbc_repo` & `Testing Ground` Cleanup:**
   - `CarInterface.get_starpilot_params` (`FPCP`), `fp_ret`, `starpilot/common/starpilot_variables.py`, and `starpilot/common/testing_grounds.py` were completely removed.
   - `CarState.update()` returns only standard `structs.CarState()`.
3. **Longitudinal Overhaul (Why Planner/MPC Hacks Are No Longer Needed):**
   - `selfdrive/controls/lib/longitudinal_vehicle_tunes.py` was eliminated.
   - In `openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py::process_lead`, the internal `lead_v_filter` and `lead_a_filter` were deleted upstream—when `build_model_lead_trajectory` yields, MPC reads `lead.vLead` and `lead.aLeadK` directly every cycle. Thus **4a (`623729c814`, `use_model_lead_filter_sync`) is obsolete/fixed upstream**.
   - `longitudinal_planner.py` removed `get_close_lead_brake_cap` (`25 s` TTC bypass), `get_dynamic_t_follow` give-up/inflation, `get_far_lead_coast_headway_floor`, and `obstacle_bias`, replacing them with small bounded modules in `openpilot/starpilot/longitudinal/` (`lead_approach.py`, `lead_behavior.py` with `inside_gap_cap` bounded to `-0.65 m/s²`, `force_stop.py`, `vehicle_policy.py`). Thus **Slot 10 (`adee799780`, `3a792e2259`) is superseded** and should not be ported unless testing on `SecretGoodStarPilot` shows a need.
4. **Already Upstreamed on `SecretGoodStarPilot`:**
   - **Lateral Control:** `KIA_STINGER_2022` lateral torque profile (`get_kia_stinger_2022_center_taper_scale` and `get_kia_stinger_2022_friction_threshold`) is already upstreamed in `openpilot/starpilot/lateral/hkg_vehicle_policy.py` and `hkg_vehicle_shaping.py`.
   - **LFA Steering Wheel Button (`0x391` `BCM_PO_11`):** `opendbc_repo/opendbc/car/hyundai/interface.py` lines 238–241 automatically sets `HyundaiFlags.HAS_LDA_BUTTON` when `0x391 in fingerprint[0]`, and `carstate.py` lines 389–390 reads `cp.vl["BCM_PO_11"]["LDA_BTN"]`. (Just ensure `BCM_PO_11` has `math.nan` frequency in `carstate.py::get_can_parsers` for empty-fingerprint tests.)
   - **`FLMActiveOverrides` JSON fix (`f6f89ce960`):** Obsolete (`starpilot_variables.py` deleted; handled in `openpilot/starpilot/flm/gm_evidence.py`).

#### Review of the port plan (Claude, 2026-10-25; verified against `upstream/SecretGoodStarPilot`)
**Confirmed in the code**
- Layout moved under `openpilot/`.
- Testing Grounds, `longitudinal_vehicle_tunes.py` and `starpilot_variables.py` are gone.
- `long_mpc.process_lead` has no `lead_v_filter` / `lead_a_filter`, so 4a is obsolete. `extrapolate_lead` is plain exponential, with no >35 mph constant-decel blend.
- The planner add-ons are gone (close-lead cap, `get_dynamic_t_follow`, far-lead coast floor). The planner is 466 lines plus `openpilot/starpilot/longitudinal/*`.
- The Stinger lateral profile and LFA-button auto-detect are upstream.
- Still needed: the ISG mask (`carstate.py:244`), the radar decoder (`radarUnavailable` will be true), `JerkLowerLimit` (hard-coded 5.0 at `hyundaican.py:179`) and the classic-CAN `LimitTracker`.
- Per-car `*_longitudinal.py` modules exist. Panda torque limits are unchanged (384/3/7).

**Corrections and gaps**
1. The tip has moved to `4b30638588` (2026-10-25), 9 commits past `202a52c353`. Branch from the current tip and re-check the four items.
2. `longitudinalActuatorDelay = 0.5` is already the Hyundai default on SG (`interface.py:302`). Do NOT port Tune B's delay; port only the car-side braking shaping.
3. Tune B was fitted against the OLD planner. On SG there's no A/B switch, so re-validate with before/after drives plus the sustained-braking sim matrix (`analysis/sustained_lead_brake.py`). Keep the "≤ -1.0 passes through unshaped" behaviour.
4. **The analysis/test toolchain must be ported first** (`setup_x86.sh`, `run_tests.sh`, `sim_stopped_lead.py`, `planner_delay_replay.py` and the others import old paths). Otherwise nothing can be validated on SG.
5. UI marker offset: the SG mici UI already adds `RADAR_TO_CAMERA`, so the user's comma four is covered. The big UI `onroad/model_renderer.py:187` (used by clips) still uses raw `dRel`; porting it is optional.
6. Slot 10: don't port, but measure the stopped-lead final gap on the new planner first. The user's goal is about 4 m.
7. Keep panda safety untouched (comma ban risk; see section 1 and the torque-limit discussion).

**Agreed order** (user OK 2026-10-25). One commit per step with tests, on new branches; `stinger-combined` untouched.
1. **Toolchain port, switchable between the old and SG builds (DONE 2026-10-25; dev-repo branch `toolchain-sg`, NOT pushed, `main` untouched at `1b4057d`).**
   - **Commits:**
     - `77dcc07`: snapshot of the 21 previously uncommitted analysis files, unchanged.
     - `6685a76`: the selector.
     - `87a973e`: the planner tools on SG.
     - `5714715`: README "Toolchains" section.
     - `b046f36`: empty value means old.
   - **Switching:** `STINGER_TOOLCHAIN=old` (default; also when unset or empty) or `sg`. It's resolved in `toolchain.sh` (shell) and `analysis/toolchain.py` (Python).
     - old = `$DEV/x86` @ `f6f89ce96` + `$DEV/venv` (untouched).
     - sg = `$DEV/x86-sg` (detached worktree @ `4b30638588`) + `$DEV/venv-sg` (uv from SG's lock).
     - Run Python tools with `./run_py.sh analysis/x.py ...` so the right venv is used.
   - **Old toolchain verified identical before and after:**
     - planner tests: 529 passed;
     - `planner_delay_replay` at 1836 s: byte-identical;
     - sim run: identical;
     - `highway_brakes`: `cmp` identical.
   - **SG verified:**
     - tests: 4615 passed serially (`-n 0`); the 41 failures with xdist are serialisation-environmental;
     - sim `run` / `scenarios` (car tune A only) work;
     - `planner_delay_replay` works: event 1836 peaks at -0.41 on the SG planner, against -2.01 logged;
     - log-only tools are identical between toolchains.
   - **Under sg:**
     - Logs are read with an ID-remapped copy of the OLD cereal schema (`$DEV/schema-old-remap/`), because SG's schema isn't wire-compatible with the old logs.
     - Old-only features exit with a clear message: car tunes B/C, slot 9/10, `--testing-ground`, `--sync-filters`, `stock_mimic_check`, `tune_safety_replay`, `sustained_lead_brake run`, the `radar_*` tools (they need the Stinger decoder, so steps 3/5 will re-enable them), and `brake_blend`.
     - Not yet ported to the SG planner API: sim `--t-follow/--jerks/--min-accel`, `validate`, `openloop`; replay `--dump` / `--approach`.
   - **Possible future base:** a newer SG tip can go in a fresh `x86-sg` worktree, or re-check out `x86-sg` and re-run `STINGER_TOOLCHAIN=sg ./setup_x86.sh`.
   - **Merged:** `toolchain-sg` was fast-forwarded into dev-repo `main` and PUSHED (`b046f36`, 2026-10-25).
2. **ISG fault fix: DONE and verified (2026-10-25).** Branch `sg-hyundai-isg-steer-fault` is pushed to the fork. It's one commit, `3fb831a8f7`, on SG base `32360730ba`.
   - **Diff:** only `carstate.py` (+15) and `test_hyundai.py` (+46/-1); the safety/panda diff is empty.
   - **New tests:** 3. Two of them fail without the fix; the steer-at-standstill guard test now uses `HYUNDAI_PALISADE_2023` + `0x2AA`, because on SG `0x2AA` only sets `steerAtStandstill` for blended cars.
   - **Base vs branch** (serial): 5146 vs 5149 passed, 0 failures.
   - **`x86-sg`** was left at `3fb831a8f7`; re-check out the next base before building on it.
   - **Gotcha:** agent shells may not source `~/.bashrc`. Always pass `STINGER_DEV_HOME=/home/openpilotstuff/starpilot-dev` explicitly, or `toolchain.sh` falls back to `~/stinger/starpilot-dev` and `setup_x86.sh` tries to clone there. (That happened once; the stray clones were deleted.)
   - The original in-flight notes follow, for reference.
   - **What it was doing:**
     - re-fetching `upstream/SecretGoodStarPilot` (tip was `32360730ba`) and moving `x86-sg` to it, then re-running `STINGER_TOOLCHAIN=sg ./setup_x86.sh`;
     - creating worktree `wt/sg-hyundai-isg-steer-fault` on branch `sg-hyundai-isg-steer-fault`;
     - porting the fix and tests;
     - running the Hyundai and car-interface tests serially (base versus branch);
     - pushing the branch (no PR, no merge).
   - **The next session must verify it:**
     - `git -C /home/openpilotstuff/starpilot fetch origin && git log --oneline -3 origin/sg-hyundai-isg-steer-fault`;
     - the diff touches only `opendbc_repo/opendbc/car/hyundai/carstate.py` and the tests; no `opendbc_repo/opendbc/safety` or `panda` changes;
     - tests pass serially (`STINGER_TOOLCHAIN=sg ./run_tests.sh <sha> opendbc_repo/opendbc/car/hyundai/tests -n 0`);
     - `x86` is still at `f6f89ce96`.
   - If the branch is missing or incomplete, redo it from the description above.
   - **Working-style notes from the user:**
     - Use Haiku agents for mechanical jobs (clips, downloads, reruns) and Sonnet for implementation.
     - SG base (user, 2026-10-25, replaces "always fetch the latest SG before branching"): do NOT move to a new SG tip for each change. Branch new SG work from the current SG base (the commit `sg-stinger-combined` is built on) and keep `x86-sg` where it is. The user moves the base to a newer SG tip themselves once in a while.
3. **Radar decoder: DONE and verified (2026-10-25).** SG base is now `d33f5b550c`.
   - **`sg-kia-stinger-2022-radar`** (pushed, upstream PR candidate): one commit `b8689f645f`. Touches only hyundai `radar_interface.py`, `interface.py` (`radarUnavailable` from all 30 bus-1 addresses 0x238–0x255 at length 8) and new `tests/test_stinger_radar.py`. No safety/panda changes. Gated on `KIA_STINGER_2022`. SG's radard takes every `radarTracks` point, so the Stinger is treated like any other classic-CAN Hyundai radar.
   - **Tests** (hyundai + opendbc car tests, serial): base 5230 / branch 5248 / combined 5251 passed, 0 failed.
   - **Equivalence** against the old daily-build decoder (`analysis/stinger_radar_equiv.py`): routes 15, 28, 0e, 247k emissions, 0 mismatches (exactly identical, same trackIds and canError frames). A real fingerprint gives `radarUnavailable=False`.
   - **Radar tools under sg:** merged into dev-repo `main` and pushed (`82bcd47`, fast-forward from `sg-radar-tools`). radar_vs_factory (routes 28, 34), radar_vs_vision, radar_range_rate, radar_first_seen and radar_check give output identical to the old toolchain.
   - **Follow-up `08faae5bf0`** (pushed): dropped the `deprecated.measured/aRel/yvRel` fields (only set for parity with the old fork; SG's radard never reads them). Tests: 5248 passed, 0 failed. Merged into `sg-stinger-combined`.
   - **`sg-stinger-combined`** (pushed): `d33f5b550c` + merge of `sg-hyundai-isg-steer-fault` (`05e7238402`) + merge of the radar branch (`13e84bf47a`). Future SG feature branches get merged in the same way. `x86-sg` is at `13e84bf47a`; `x86` is still at `f6f89ce96`.
4. **Classic-CAN dashboard speed limit: DONE (2026-10-25), merged into `sg-stinger-combined` (`f316b2a48e`, pushed; combined tests 5625 passed, 0 failed).** Branch `sg-hkg-can-dashboard-speed-limit`, one commit `229372a7ee` on `d33f5b550c`. It touches `dashboard_speed_limit.py` (`hyundai_can_sign`: 1..254 is VALID; 0 and 255 are UNKNOWN, not ABSENT), hyundai `carstate.py` (+33) and a new test file. Navi_HU and LKAS12 are subscribed with NaN on classic non-blended cars. LKAS12 is not subscribed for the Stinger (its 0x53E is 8 bytes, not LKAS12) or the Ray EV.
   - **Data:** on the Stinger, Navi_HU runs at 5 Hz and the cluster is in mph. Raw values seen: 0, 15–65, and 255 (route 15 only).
   - **Equivalence** with the old fork: 1.31M frames on 7 routes, 0 mismatches on warm frames.
   - **Strict criterion B missed on cold starts:** at a cold start, SG stays UNKNOWN for 1.0–1.2 s (319 frames in 3 segments) while the parser learns the message timeout. The limit was one message period. Every SG `parser_expiry` sign source (Ford, Honda, Toyota, HKG CAN-FD) has the same delay, because the parser learns the rate after more than 1 s of samples. The user accepted it.
   - **canValid:** identical to base. **Tests:** 5588 passed on base vs 5604 on the branch, 0 failures.
   - **Dev-repo script:** `analysis/dashboard_limit_equiv.py`, merged into dev-repo `main` and pushed (`d735af4`).
   - **Upstream risk:** SG's CANParser doesn't check message length, so a classic car whose bus-2 0x53E isn't LKAS12 (other than the Stinger and Ray EV) would decode garbage TSR values.
5. **Tune B car-side shaping: evaluated, NOT ported (2026-10-25).** Eval script `analysis/tune_b_sg_eval.py` and results file `analysis/tune_b_sg_eval_results.md` are on dev-repo branch `tune-b-sg-eval` (`dfc5f52`, not pushed). It's an open-loop replay of the SG planner on routes 15, 0e and 0d; route 15 was a Tune B drive.
   - **The fixed rule says NECESSARY**, via criterion (b) in the no-lead bucket only: SG p95 falling rate 2.5 vs factory 1.0 m/s³. That bucket is only 25 s of data, and its top events are all in `0000000e` seg 25. There the SG command steps from about +0.95 to -0.42 at 19 m/s with no lead, which looks like a replay artifact (missing SG cruise and speed-limit inputs).
   - **Lead < 60 m** (93% of frames): SG is already no faster than the factory (p95 2.0 vs 2.5), and B would bind on 0.19% of frames. Criterion (a): 1.04% against the 5% threshold.
   - **Recommendation:** drive `sg-stinger-combined`, then re-run the eval on real SG logs before deciding. Whether the car acts on the JerkLowerLimit value itself is still unknown.

#### Concrete 4-Commit Port Plan onto `upstream/SecretGoodStarPilot` (Gemini's original; see the review above)
Create a new branch/worktree from `upstream/SecretGoodStarPilot` (`202a52c353`) first (do not touch `stinger-combined` without user's explicit OK) and port only these 4 commits:

1. **Commit 1 — Kia Stinger 2022+ Radar Track Decoder (`df45123546` + `4ff10b969b` + `7b8efbbef7` + `b7b3a2e491` squashed)**
   - **Why needed:** `SecretGoodStarPilot` removed `RadarTrackConfig` / `RADAR_LIVE_LONGITUDINAL_CAR` and sets `ret.radarUnavailable = radar_start not in fingerprint[1] or Bus.radar not in DBC[ret.carFingerprint]` (`opendbc_repo/opendbc/car/hyundai/interface.py` line 289), which evaluates to `True` on `KIA_STINGER_2022`.
   - **What to do:**
     - Port `Stinger2022CandidateObject` and `Stinger2022ObjectDecoder` (including the `-0.7 m/s` stopped-lead speed undershoot fix from `4ff10b969b` and the `0.14 m/s` radar ego-speed offset ramp from `7b8efbbef7`) into `opendbc_repo/opendbc/car/hyundai/radar_interface.py`.
     - Hook `Stinger2022ObjectDecoder` into `RadarInterface` for `CAR.KIA_STINGER_2022` (subscribing to `WHL_SPD11` on bus 0 and `0x238–0x255` triplets on bus 1 at 50 Hz).
     - Update `opendbc_repo/opendbc/car/hyundai/interface.py` so `ret.radarUnavailable` is `False` when Stinger 2022 radar messages are present on bus 1.
     - Add unit tests in `opendbc_repo/opendbc/car/hyundai/tests/test_stinger_radar.py`.

2. **Commit 2 — Hyundai Idle Stop and Go (ISG) Engine-Off Steering Fault Fix (`9387109950`)**
   - **Why needed:** `opendbc_repo/opendbc/car/hyundai/carstate.py` line 244 on `SecretGoodStarPilot` still sets `ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0` without masking ISG engine-stop (`LVR12.CF_Lvr_IsgState == 1`).
   - **What to do:** Port `9387109950` (`LVR12` subscription on bus 0, standstill ISG mask + `isg_steer_grace_frames = 20` on restart) and its unit tests.

3. **Commit 3 — Kia Stinger 2022 SCC12 Braking Actuation (Tune B as Default)**
   - **Why needed:** User confirmed they want **Tune B** (the stock-fitted braking rate/jerk shaping in `SCC12` from `0a3f622e68` / `52310c70bf`). On `SecretGoodStarPilot`, `Testing Ground` is gone, `hyundaican.py` line 172 still hardcodes `"JerkLowerLimit": 5.0`, and `carcontroller.py` passes un-slewed `accel` to `create_acc_commands`.
   - **What to do:**
     - Follow `SecretGoodStarPilot`'s clean per-car module pattern (`opendbc_repo/opendbc/car/hyundai/gv70_longitudinal.py` and `ioniq6_longitudinal.py`): create `opendbc_repo/opendbc/car/hyundai/stinger_longitudinal.py` implementing Tune B's braking-side decel slew and dynamic `JerkLowerLimit` for `CAR.KIA_STINGER_2022`.
     - Add optional `lower_jerk=5.0` parameter (or pass from `stinger_longitudinal`) in `opendbc_repo/opendbc/car/hyundai/hyundaican.py::create_acc_commands` and invoke it in `carcontroller.py` when `self.CP.carFingerprint == CAR.KIA_STINGER_2022`.
     - Port the Tune B unit tests into `opendbc_repo/opendbc/car/hyundai/tests/test_stinger_stock_tune.py`.

4. **Commit 4 — HKG Classic CAN Dashboard Speed Limit (`3df2f9d20c`, `28f4c8cf4f` adapted to `LimitTracker`)**
   - **Why needed:** Checked `upstream/SecretGoodStarPilot` — it has a brand-new `LimitTracker` (`opendbc_repo/opendbc/car/dashboard_speed_limit.py`) that feeds `self.dashboard_limit` in `carstate.py`, which `openpilot/selfdrive/car/card.py` (line 1109) automatically publishes as `slcDashboardObservation`! However, `opendbc_repo/opendbc/car/hyundai/carstate.py` ONLY wires up `self.dashboard_limit` in `update_canfd()` (lines 599–605 using `FR_CMR_02_100ms` and `hyundai_canfd_sign`). Classic CAN (`update()`) never updates `self.dashboard_limit`, and `openpilot/starpilot/speed_limits/physical_actions.py` / `slc_transport_capable` may also gate which cars enable SLC dashboard transport.
   - **What to do:**
     - Add a helper in `opendbc_repo/opendbc/car/dashboard_speed_limit.py` (or `carstate.py`) for classic HKG CAN (`Navi_HU` `0x544` `SpeedLim_Nav_Clu` and `LKAS12` `0x53E` `CF_Lkas_TsrSpeed_Display_Clu`, using `parser_expiry`), skipping CAN-FD blended cars just like `28f4c8cf4f`.
     - Subscribe to `Navi_HU` (bus 0, `math.nan`) and `LKAS12` (bus 2, `math.nan`) in `carstate.py::get_can_parsers`, and call `self.dashboard_limit.update(...)` in `CarState.update()`.
     - Check `slc_transport_capable` in `openpilot/selfdrive/car/card.py` to make sure classic Hyundai CAN cars (or `KIA_STINGER_2022`) are admitted to publish `slcDashboardObservation`, and add unit tests.

### 4d. Other open findings (not started)
- **Lead marker on empty road at low speed:** at low speed openpilot sometimes takes an adjacent-lane car as the lead. Route 0e, 1480-1491 s: radar track 2016 at 1.5-4 m ahead, 1.1-2.0 m to the right, while the camera lead was in-lane at 7-12 m.
  - That is a possible cause of early stops and creeping. Check how often it happens and which lead-selection rule admits it.
- **Steady highway following:** openpilot holds 1.86 s versus the factory's 1.71 s at distance setting 3, which is the setting the user uses.
  - The suspected extra comes from the far-lead coast floor (1.75 s default, `get_far_lead_coast_headway_floor`) or the dynamic `t_follow` above. A closed-loop test is needed.
- **Slot 10 stopped-lead tune:** it only fixes the final stop distance. Its approach shape is blocked by StarPilot's generic `get_close_lead_brake_cap`; a cap-skip experiment is paused by the user.
  - A CR-V-style crawl cap was tested in the sim and does nothing.
- **Camera-to-radar distance:** the Stinger is about 2.05 m against the generic 1.52 m. Only camera-only leads are affected (about 0.5 m). Optional calibration, not done.

## 5. Useful scripts (in `starpilot_stinger_dev/analysis/`, run from that folder with the env above)
- `highway_brakes.py ROUTE [--at T]`: braking episodes and timelines.
- `highway_overbrake.py --group NAME ROUTES...`: commanded versus kinematically needed decel.
- `follow_closing_compare.py`: factory versus openpilot steady time gap and closing events.
- `lead_braking.py --group NAME ROUTES...`: response to lead braking.
- `follow_rates.py`: factory versus openpilot accel change rates.
- `planner_delay_replay.py ROUTE --segments A-B --events T [--dump T0 T1 --dump-full --sync-filters --testing-ground 9:B]`: replays the real planner from `x86` on rlogs, with a line trace.
- `sim_stopped_lead.py scenarios|run|validate`: closed-loop sim using the real planner.
- `tune_safety_replay.py`: extra distance and delay scoring.
- **Video clips:**
  - Command: `/home/openpilotstuff/wt/clip/.venv/bin/python /home/openpilotstuff/starpilot_stinger_dev/clip_route.py 59fa1d41318b87d3/<route> -s S -e E -q low -f 60 -d <clipdata dir> -o out.mp4`.
  - Data dir layout: `<dongle>|<route>--<seg>/{rlog.zst,qlog.zst,qcamera.ts}` symlinks.
  - Output is the big comma 3/3X UI (the user has a comma four, which uses the mici UI).

## 2026-10-27 to 2026-10-28: SG drive 1, HUD fixes, radar PR merged, old build retired
- Three SG base moves on 2026-10-27 (`12dab8c31d` → `7dc37f190f` → `0422970f2d` → `05d14fc86e` → `efe875ffd6`), then to Dom `c55ef78274` on 2026-10-28; every `range-diff` unchanged.
- Drive 1 on `sg-stinger-combined` ran stock ACC (alpha long unset). Fresh-install defaults to know: Alpha Longitudinal off, `LKASButtonControl` 0 (Off), `HideLeadMarker` 1.
- Torque learner (`torqued`): needs every steer-torque bucket at its minimum (100/300/500/500/500/500/300/100) plus 4000 points before `valid`; drive 1 reached 76% with only the outer buckets short. Learned ~3.05 latAccelFactor / ~0.08 friction vs CP 2.75 / 0.120, inside the ±30%/±50% clamps. The points are cached every 60 s (`LiveTorqueParameters`) and restored when the CP torque values and version match; a calibration reset clears them.
- Radar decoder vs factory SCC11 (routes 00000000, 28): distance within 0.1 m under 15 m, vRel within ±0.25 m/s. Beyond 30 m SCC11 reads 2–5% long against its own relative speed; the decoder's distance slope matches its vRel (0.99–1.00 at 30–60 m), so the offset is in SCC11. Script `starpilot-dev/scratch/range_scale.py`.
- Radar PR #259 (squashed `9736d887b3`) opened as draft against Dom on 2026-10-27, merged by FireStar 2026-10-28. Review fixes before the PR: a line over the 160-column lint limit, and objects decoded in an error cycle leaking into the next good cycle.
- HUD wheel icon: with stock ACC, the old build and SG send identical LKAS11/LFAHDA_MFC during AOL-only driving and the panda forwarding is the same; with openpilot long, SG gated the AOL icon to the Elantra 2021. Fixed for the Stinger (`sg-hyundai-classic-long-aol-dash-icon`).
- Dash/HUD lead: SG sent real SCC11/SCC14 lead data only for the G90; widened to the Stinger (`sg-kia-stinger-2022-can-lead-display`), validated by replay against the old build's sent frames on route 0e.
- Retired on 2026-10-28: `stinger-combined`, `stinger-stock-tune`, `stinger-stopped-lead`, `sg-kia-stinger-2022-radar`, `kia-stinger-2022-radar` (archive tags pushed, branches deleted), the old toolchain (`x86`, `venv`).
