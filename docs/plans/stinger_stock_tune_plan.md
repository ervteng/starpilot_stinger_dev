# Plan v3: Kia Stinger 2022 stock-like tunes from existing StarPilot code (Testing Ground slot 9: A/B/C)

Goal: two Stinger-only longitudinal tunes that make openpilot's SCC12/SCC14 output look like the factory SCC,
each built by REUSING an existing StarPilot tune, with full-strength urgent braking kept.
- A = installed tune (no change, byte-identical to today)
- B = GV70 Electrified style (fixed SCC jerk limits + urgent bypass, `get_hyundai_canfd_scc_jerk_limits` /
  `shape_hyundai_canfd_scc_accel` in selfdrive/controls/lib/longitudinal_vehicle_tunes.py)
- C = EV6 GT-Line / Ioniq 6 style (`update_ioniq_6_longitudinal_tuning` + `Ioniq6LongitudinalTuningState` +
  the egmp wiring in carcontroller.update)

Do NOT port or copy sunnypilot code. Do NOT write a new controller when an existing function can take a Stinger flag
or branch. Reuse > new code; new code only where the existing function cannot express the stock behavior.

## Repo / branch
- Create worktree /home/user/wt/stock-tune on a NEW branch `stinger-stock-tune` from origin/StarPilot (commit 823a5d5):
  `cd /home/user/ervteng/starpilot && git fetch origin StarPilot && git worktree add -b stinger-stock-tune /home/user/wt/stock-tune 823a5d5`
  (submodules: opendbc_repo is a normal directory in this repo — check; if it is a submodule, init it.)
- Reference only (do not merge/cherry-pick wholesale): branch `stinger-long-tuning` in /home/user/wt/long-tune has an
  earlier attempt (sunnypilot port + custom state machine). Its Testing Ground slot 9 wiring
  (starpilot/common/testing_grounds.py, starpilot/common/tests/test_testing_grounds.py, values.py ID) and the CAN
  `create_acc_commands` jerk plumbing in hyundaican.py are fine to reuse in spirit; everything sunnypilot-derived
  (longitudinal_tuning.py, LongitudinalTuner) must NOT come along.
- Git author is configured in the repo (author ervteng, committer Claude) — verify `git config user.name` in the new
  worktree; if missing, set user.name "ervteng", user.email "5085265+ervteng@users.noreply.github.com" locally.
- Commit (one or a few commits). Push with `git push -u origin stinger-stock-tune`. Never touch stinger-combined.
- Commit message trailer (required, exactly):
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Xb1xauXRxxPYDoUiEdA8e

## Measured stock behavior (factory SCC, 4 stock-ACC drives, ~260k active samples)
- SCC14 JerkUpperLimit: 0.5 m/s^3 97% of the time. Accelerating from near standstill: v<1 m/s median 0.7 (p90 1.1);
  v 1-3 median 0.5 (p90 0.9); above 3 m/s 0.5.
- SCC14 JerkLowerLimit: 2.5 (75%) or 0.5 (25%). 0.5 = no lead (100% of no-lead time) or far lead (60-90 m: 72% at 0.5;
  >90 m: 98%; <60 m: 3-5%). Rises to ~2.7-4.4 only when aReqRaw < ~-2.25 and closing fast.
- SCC12 aReqValue ramps toward aReqRaw at those limits (rising median 0.5 m/s^3, falling median 0.5, p1 -2.6);
  aReqValue != aReqRaw only 6% of the time.
- Comfort bands: small (<= 0.1), one-sided. GV70 and Ioniq tunes send 0; keep 0 (not worth new code).
- Command -> aEgo lag on this car ~0.5 s (default CP.longitudinalActuatorDelay is 0.15).

## Gate (EV6 GT-Line helper pattern)
- values.py: `KIA_STINGER_2022_LONG_TUNING_TESTING_GROUND_ID = "9"` next to KIA_EV6_GT_LINE_LONG_TUNING_TESTING_GROUND_ID,
  plus a helper modeled on `kia_ev6_gt_line_longitudinal_tuning`, e.g.
  `kia_stinger_2022_longitudinal_tuning(car_fingerprint, variant_b_active, variant_c_active) -> "B" | "C" | None`
  (or two bool helpers — pick whatever reads most like the EV6 helper). Stinger 2022 + openpilot long only.
- testing_grounds.py: slot 9 "Stinger Long Tune", aLabel "A - Installed tune", bLabel "B - Stock-like (GV70 style)",
  cLabel "C - Stock-like (EV6/Ioniq style)", description saying it targets the factory cruise feel on the Kia Stinger
  2022 and that urgent braking is never smoothed. Use `testing_ground.use(ID, "B")` / `use(ID, "C")` like existing
  call sites (look at how slot variants are queried in testing_grounds.py).
- Read the gate where the existing EV6 GT-Line code reads it (EV6 reads testing_ground in update() each frame and in
  apply_post_fingerprint_params). Mid-drive changes must not cause a jump: either latch the variant in
  CarController.__init__ (preferred, simplest) or reset state like reset_ev6_gt_line_longitudinal_tuning does.

## B: GV70-style (reuse get_hyundai_canfd_scc_jerk_limits / shape_hyundai_canfd_scc_accel)
- Add a Stinger branch to `get_hyundai_canfd_scc_jerk_limits` with KIA_STINGER_2022_SCC_* constants in
  longitudinal_vehicle_tunes.py, same structure as the GV70 branch: upper, lower, urgent (stopping or
  accel <= URGENT_ACCEL) -> lower 5.0. URGENT_ACCEL = -1.0 (same as GV70) and URGENT jerk 5.0: fixed, safety.
- To match stock you may add OPTIONAL keyword args (default None -> ignored, so the GV70 path is unchanged):
  `v_ego` for the speed-based upper jerk (1.0 / 0.7 / 0.5 at 0 / 1 / 3 m/s) and `lead_distance`/`lead_visible` for
  the lower jerk (2.5 with a lead < 60 m, 0.5 with no lead / lead > 90 m, interpolated 60-90 m; never below 2.5 when
  accel < -0.5 so openpilot's own braking for curves/stops without a lead still ramps at 2.5).
- The gate for B must live where the Stinger branch is chosen: pass it in (e.g. an optional kwarg or check CP + a
  flag the caller supplies). GV70 and every other car must produce identical results.
- `shape_hyundai_canfd_scc_accel` is used unchanged: it rate-limits at HYUNDAI_CANFD_SCC_FREQUENCY = 50 Hz, and the CAN
  path sends SCC12 at 50 Hz too (check create_can_msgs: frame % 2), so call it there with the same args the CAN-FD
  GV70 block uses (self.accel_last = last SENT accel at 50 Hz — verify the CAN path's accel_last semantics match).
- Wire into carcontroller.create_can_msgs (plain CAN branch, not blended), mirroring the GV70 block in the CAN-FD path
  line for line.

## C: EV6/Ioniq-style (reuse update_ioniq_6_longitudinal_tuning)
- Add a `kia_stinger_2022: bool = False` kwarg to `update_ioniq_6_longitudinal_tuning` (same pattern as its `ev9`
  kwarg). Under the flag, with KIA_STINGER_2022_* constants:
  - jerk_upper capped at the stock speed curve (1.0 / 0.7 / 0.5 at 0 / 1 / 3 m/s); the 0.6 MIN_JERK floor
    (0.5 * 1.2) must not lift it above 0.5 — cap after the floor.
  - jerk_lower capped at 2.5 (the dynamic lower jerk already ranges ~0.6 -> scaled max from the accel error, which
    resembles stock's 0.5 / 2.5 split; check and tune the cap/floor to match stock).
  - Disable EV-specific pieces: launch hold (launch_active never set), stop brake cap, stop-hold/stop-release jerk
    tweaks. Stopping state passes openpilot's command through (desired = accel_cmd clipped), like B and like today.
  - Urgent: accel_cmd <= URGENT_ACCEL (-1.0, share the constant with B) -> no smoothing, jerk_lower 5.0.
- Wiring: prefer extending the existing egmp block in CarController.update rather than a parallel block:
  include the Stinger C gate in the `use_egmp_dynamic_long_tuning` condition (keep `egmp_dynamic_longitudinal_tuning`
  for the EV cars or add the Stinger there — whichever keeps other cars identical), pass `kia_stinger_2022=True`, and
  give the Stinger its own `use_egmp_smoothed_accel` rule: smoothed unless accel_cmd <= URGENT_ACCEL (unlike the EV6
  rule, which skips smoothing whenever braking harder than the smoothed value — stock ramps braking at ~2.5).
  The existing non-smoothed fallback (CAN-FD decel step clip) is acceptable as the urgent path if it is effectively
  instant (check IONIQ_6_CANFD_SCC_DECEL_STEP); otherwise pass accel_cmd through.
- Update runs every 5 frames as upstream. CAN create_acc_commands gets jerk_upper/jerk_lower from
  self._ioniq_6_long_tuning (as the CAN-FD acc_kwargs do).

## Shared
- hyundaican.create_acc_commands: add `lower_jerk=5.0` kwarg (default = today's SCC14 JerkLowerLimit). aReqRaw =
  aReqValue = shaped accel (as GV70/Ioniq do). Comfort bands stay 0. Blended paths untouched.
- Planner (longitudinal_vehicle_tunes.py), applied when B or C is active:
  - `get_lead_follow_jerk_scale(CP)`: Stinger entry, start at 1.25 (Elantra value).
  - `get_far_follow_output_slew_rates(CP)`: Stinger entry, start at 2.0 brake / 1.25 release (Niro EV values).
  These take only CP; gate by importing testing_ground there (check for circular imports) or another clean route.
  Document the choice in the commit message.
- Actuator delay: `apply_post_fingerprint_params` -> `apply_kia_stinger_2022_longitudinal_params(CP)` setting
  longitudinalActuatorDelay = 0.5 when B or C (EV6 GT-Line helper pattern).

## Tests (new file opendbc_repo/opendbc/car/hyundai/tests/test_stinger_stock_tune.py; do not grow test_hyundai.py)
- Gate: A/B/C x Stinger 2022 / other car / no op long; latch at startup.
- B: jerk limits (speed, lead, braking), urgent -> 5.0, GV70 results unchanged, shape rate.
- C: caps, urgent pass-through, launch hold/stop cap off, Ioniq 6 / EV6 / EV9 results unchanged (compare
  update_ioniq_6_longitudinal_tuning with and without the new kwarg defaulted).
- CAN SCC12/SCC14 carry the values; non-Stinger car and Stinger slot A send exactly the previous bytes.
- Delay 0.5 only for B/C; planner entries only for B/C (selfdrive/controls/tests — add to an existing tune test file
  if one covers these functions, e.g. grep for get_lead_follow_jerk_scale tests).
- starpilot/common/tests/test_testing_grounds.py slot 9 A/B/C.

## Commands
- opendbc tests: cd /home/user/wt/stock-tune && PYTHONPATH=$PWD:$PWD/opendbc_repo /home/user/starpilot-dev/venv/bin/python -m pytest -q -n 8 -p no:cacheprovider opendbc_repo/opendbc/car/hyundai/tests
  Known upstream failures (must stay exactly these 4, nothing new): test_fw_format, test_fuzzy_excluded_platforms,
  test_platform_code_ecus_available, test_hyundai_lkas_button_sets_starpilot_safety_flag.
- starpilot/ and selfdrive/ tests need compiled modules: /home/user/starpilot_stinger_dev/run_tests.sh stinger-stock-tune <test paths>
  (run after committing; it checks out the ref in an x86 build worktree). Run the touched test files plus
  selfdrive/controls/tests/test_longitudinal_planner.py and test_starpilot_planner.py.
- Style: 2-space indent, match surrounding code, comments only where non-obvious.
