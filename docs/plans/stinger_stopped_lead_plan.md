# Plan: Kia Stinger 2022 stopped-lead tune (StarPilot)

## Goal (agreed with the user)
Approaching a stopped (or nearly stopped) lead from city speeds:
- start braking ~5 s before reaching the stopped car (driver data: TTC at brake onset 4.7 s at 8-15 mph, 5.9 s at 15-25 mph),
- brake roughly evenly at ~1.2-1.6 m/s^2 (stock ACC: near-constant decel, CV 0.12; follow stops 1.1-1.3 m/s^2 at 15-5 mph; driver peaks 1.5-1.9),
- short crawl at the end (~1 s below 3 mph; driver 1.0-1.1 s, stock 1-2.5 s),
- stop ~4 m behind the lead (driver 3.7-4.2 m, stock 3.1-3.7 m; openpilot today 5.4-7.4 m),
- still brake as hard as needed when a stopped lead is detected late (gap already inside the safe stopping distance).
Data sources: /home/user/starpilot_stinger_dev/analysis/manual_stops.py (driver, 54 stopped/slow-lead stops),
analysis/stopped_lead_approaches.py (stock ACC + openpilot). Numbers above come from those scripts.

## Mechanism (reuse, not new controller)
Reuse StarPilot's per-car stopped-lead obstacle bias: selfdrive/controls/lib/longitudinal_vehicle_tunes.py
`get_stopped_lead_obstacle_bias` dispatcher (Prius / CR-V 5G / EV9 entries) -> lead_obstacle_bias passed to the MPC
(selfdrive/controls/lib/longitudinal_planner.py ~2366-2386, long_mpc.py ~976: `lead_obstacle -= bias`). It is already
skipped for modelV2 shouldStop / redLight / forcingStop / standstill, so Force Stops and red lights are unaffected.
- Add a `get_kia_stinger_2022_stopped_lead_obstacle_bias(CP, lead, v_ego, t_follow?)` modeled on the EV9/Prius ones, and
  a branch in the dispatcher. Gate: CP.carFingerprint KIA_STINGER_2022 (brand hyundai) AND the Testing Ground slot below.
- Shape: make the MPC's desired gap for a stopped lead follow a constant-decel curve to a ~4 m stop point instead of
  v^2/(2*COMFORT_BRAKE) + t_follow*v + STOP_DISTANCE. Steady-state target: bias(v) = MPC_desired(v) - (v^2/(2*a_t) + g0)
  with the sign convention of `lead_obstacle -= bias` (check it: positive bias moves the obstacle closer = earlier/
  farther stop; we need both signs: positive (earlier) at higher speed, negative (closer final stop, ~ -2 m) at low
  speed). Initial values from data: a_t = 1.4 m/s^2, g0 = 4.0 m. If the dispatcher/MPC path clamps negative bias, find a
  minimal way to allow a bounded negative bias for this car only (bounded by g0 >= 3.5 m), or report.
- Activity window like the existing entries: lead vLead <= 1.0 m/s, closing >= 0.15 m/s, |yRel| <= 1.75 m, dRel <= ~80 m
  with a ramp, radar leads (and vision leads with modelProb >= 0.9 if consistent with the others), v_ego <= ~22 m/s.
  The bias must never make braking weaker when the gap is already inside the safe distance for the current speed
  (late detection): e.g. fade the positive/negative bias to 0 when dRel < v^2/(2*3.0) + g0, and clamp |bias| so the
  obstacle never moves farther than the measured lead minus g0.
- Only if the closed-loop sim still shows a long low-speed creep: add a Stinger entry modeled on the existing low-speed
  final-stop shaping (CR-V `get_honda_crv_5g_low_speed_stopped_lead_cap` LVT ~387, or Santa Fe 2022 final stop cap in
  longcontrol_vehicle_tunes.py). Not before measuring.
- All tunables: named KIA_STINGER_2022_STOPPED_LEAD_* constants with a one-line data-basis comment.

## Toggle
New Testing Ground slot (next free id; follow slot 9 / EV6 GT-Line patterns in starpilot/common/testing_grounds.py and
values.py): "Stinger Stopped Lead", A = installed behaviour, B = Stinger stopped-lead tune. Independent of slot 9 (tune
A/B/C). Read once at process start (latched), like slot 9.

## Validation
1. Unit tests mirroring test_longitudinal_planner.py Prius/CR-V/EV9 stopped-lead tests: gate on/off, other cars
   unchanged, inactive for moving leads / standstill / red light / forcing stop, late-detection case never weaker.
2. Closed-loop sim (analysis/sim_stopped_lead.py in the dev repo, being built/validated now; uses the built x86 tree
   /home/user/starpilot-dev/x86 via /home/user/starpilot_stinger_dev/run_tests.sh <ref> to check out a ref): approaches
   from 15/20/25/30/35 mph, lead visible early and late detection at 60/40/30 m, Standard and Aggressive, car tune A/B,
   delay 0.15/0.5. Targets: onset TTC ~4.5-6 s; decel in the middle of the approach 1.2-1.6, CV <= ~0.25; peak <= ~1.9 in
   ordinary stops; crawl below 3 mph <= ~1.5 s; final gap 3.5-4.5 m; late detection: min gap and peak no worse than
   baseline (baseline = installed). Tune a_t, g0 and the ramp within the data range only.
3. Compare against the driver stops and stock stops (distributions above).

## Branch / process
- Branch `stinger-stopped-lead` from origin/StarPilot (823a5d5), worktree /home/user/wt/stopped-lead. Later cherry-pick
  onto stinger-combined only after the user agrees.
- Author ervteng, committer Claude; trailer:
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_014Xb1xauXRxxPYDoUiEdA8e
- Tests: opendbc hyundai suite (4 known failures only); run_tests.sh for selfdrive/controls/tests/test_longitudinal_planner.py,
  test_starpilot_planner.py, starpilot/common/tests/test_testing_grounds.py (test_force_stop_jerk_scale_is_platform_specific
  is a known failure).
