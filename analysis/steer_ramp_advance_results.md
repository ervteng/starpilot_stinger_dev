# Steering ramp advance: how much earlier would the request have to start (Kia Stinger 2022, +3 / -7 per 10 ms, cap 384)

Definitions, models and usage are in the docstring of analysis/steer_ramp_advance.py. Analysis only: the panda safety limits are not touched.

## 0. Signals, signs and fitted response

- rlog routes: 7, 222 segments, 1307336 frames (100 Hz).
- Sign relations (from data): des vs request opposite; des vs modelV2 desiredCurvature*v^2 same; plan curvature vs desiredCurvature same.
- liveDelay.lateralDelay while lateral active: median 0.322 s (p10 0.312, p90 0.348). modeld plans for lat_action_t = lag + 0.1 (LAT_SMOOTH_SECONDS) + 0.05 + 0.025 -> tau0 = 0.497 s; controlsd uses lat_delay = lag + 0.1.
- Plan vs action check (frames with |des| > 1): s_pl * a_plan(tau0) vs modelV2 action * v^2: corr 0.99, median ratio 0.97 (1.00 = the action is the plan evaluated at tau0). Sign of carControl.actuators.curvature vs modelV2 action: same.
- Static response fit on 38 tight turn-ins (turn-signed actual lat accel on turn-signed sent torque delayed by d, least squares through the origin, tight unpressed frames): the best-fitting d is 0.54 s (g = 1.56 m/s^2 per full torque); the model uses the measured liveDelay d = 0.32 s, g = 1.52 m/s^2 per full torque (0.00396 per unit; controlsState.curvature based g_a = 1.52). Reference: steer_torque_limit_results.md measured 2.26 m/s^2 per full torque at v > 8 m/s (model B2 uses 2.26); the low-speed gain is lower because the steering-angle based lat accel at 2-9 m/s also depends on speed and the cap saturates it.
- Tight-turn episodes: 54; turn-ins (lobes with a tight frame, peak |des| >= 0.5): 43.

- Limiter check (driver clip from carState.steeringTorque, then the +3/-7 step, started from the logged sent 1.5 s before each turn-in): the unshifted simulation reproduces the logged sent torque exactly in 98.27% of 33324 window frames (max deviation 51 units).
- Turn-ins analysed: 41 (dropped 2 with peak request < 100 units); with no steeringPressed from 0.5 s before the turn-in to 0.5 s after the request peak and a des start inside the data: 11 ('clean'). 30 of 41 turn-ins contain a driver press: in tight turns the driver intervenes often, so the unpressed subset is small and the all-turn-ins numbers are shown next to it.

## 1. How the controller uses the lateral delay (read-only, /home/openpilotstuff/wt/stinger-combined)

- `selfdrive/locationd/lagd.py:236-237`: `liveDelay.lateralDelay` = StarPilot `SteerDelay` param when `UseAutoSteerDelay` is off (`starpilot/common/starpilot_variables.py:808-810`, clamp 0.01-1.0 s; the UI value is `steerActuatorDelay + 0.2`, `starpilot/common/lateral_delay.py`), else the live estimate (0.32-0.35 s on this car).
- `selfdrive/modeld/modeld.py:1295-1296, 1362`: `lat_delay = liveDelay.lateralDelay + lat_smooth_seconds` (LAT_SMOOTH_SECONDS 0.1, or developer param `LatSmoothSeconds`, `modeld.py:87, 91-95`, `params_keys.h:535`); the action is taken `lat_action_t = lat_delay + DT_MDL + DT_MDL/2` ahead (about 0.5 s) -> `modelV2.action.desiredCurvature` (`modeld.py:399-455`, smoothed with `smooth_value`).
- `selfdrive/controls/controlsd.py:800` `lat_delay = liveDelay.lateralDelay + 0.1` goes to `LatControlTorque.update`. There (`latcontrol_torque.py:246, 278-286`): feed-forward input = `desired_curvature * v^2` (the model action, NOT delayed); the PID setpoint is the action delayed by lat_delay plus a jerk term (`expected_lateral_accel + desired_lateral_jerk * lat_delay`). So the feed-forward already leads the car's delay by about 0.4-0.5 s; the +3/10 ms ramp (0 to 384 takes 1.28 s) is what is not covered.
- Existing low-speed anticipation: `controlsd.py:303-368` TURN_LEAD_* (plan probed at a constant 1.3 s distance, blinker turns only, 3-7 m/s, capped at 0.12 1/m) and the curvature hold / pre-wind before it (`CURVATURE_HOLD_*`, `get_plan_turn_onset_dist`). These are blinker-gated, so they do not cover un-signalled tight curves.
- The panda limits (`opendbc hyundai carcontroller`: STEER_MAX 384, DELTA_UP 3, DELTA_DOWN 7) are not part of any of that and stay as they are.

## 2. Timeline of tight low-speed turn-ins

t_c = des crosses 0.3 m/s^2 heading to the peak (= 0 below). Median (p75) of each event in seconds relative to t_c. t_a: the controller's desired curvature * v^2 (feed-forward input) crosses 0.3; t_r0: start of the final rise of the request (last frame below 10% of its peak); t_rp: request reaches 98% of its peak (<= 384); t_sp: sent reaches 98% of that peak (missing when it does not before the lobe ended + 2 s); t_err: first turn*(des - act) > 0.3. Only turn-ins with peak request >= 100 units.

| event | 0-10 mph | 10-20 mph | 0-20 mph | 0-20 clean |
|---|---|---|---|---|
| turn-ins | 18 | 20 | 41 | 11 |
| t_a feed-forward input >= 0.3 | -0.04 (0.00) | -0.05 (-0.02) | -0.05 (-0.01) | -0.03 (0.00) |
| t_r0 request starts final rise | 0.37 (0.50) | -0.04 (0.22) | 0.01 (0.38) | -0.98 (-0.34) |
| t_rp request peak | 0.54 (0.76) | 0.42 (0.90) | 0.46 (0.76) | 0.08 (0.75) |
| t_sp sent peak | 1.63 (1.69) | 1.95 (2.43) | 1.66 (2.17) | 1.95 (2.43) |
| t_err tracking error > 0.3 | 1.20 (2.19) | 0.48 (0.76) | 0.50 (1.32) | 0.50 (0.93) |
| lag t_sp - t_rp | 0.99 (1.18) | 1.54 (1.91) | 1.07 (1.54) | 1.63 (2.13) |
| ramp needed = peak / 3 per 10 ms | 1.28 (1.28) | 1.28 (1.28) | 1.28 (1.28) | 1.28 (1.28) |
| request ramp duration t_rp - t_r0 | 0.26 (0.75) | 0.60 (1.57) | 0.45 (1.40) | 1.51 (1.58) |
| peak request (units) | 384 (384) | 384 (384) | 384 (384) | 384 (384) |
| sent never reaches 98% of peak | 28% | 25% | 24% | 27% |
| t_err missing (never behind by 0.3) | 17% | 0% | 7% | 9% |

## 3. Required advance Delta

Smallest Delta in 0.1 s steps such that, after the same driver clip and +3/-7 limiter, sent' stays within 19 units (5%) of the torque the request definitely needs: the suffix-minimum envelope of the driver-clipped request over [i_c, t_rp + 0.3 s] (it ignores dips and one-frame spikes of the noisy request). '>1.5' = not met within 1.5 s, 'needs none' = the unshifted limiter already met it. Two ways to advance the request: **pure shift** x'(t) = x(t + Delta) (as specified; it also advances every release and every reversal) and **rise-only** x'(t) = max of x over [t, t + Delta] (the build-up comes Delta early, the release stays where it is: what an anticipatory feed-forward that only reads the plan ahead on the way in would do). Small n: the tight-turn set is a few dozen turn-ins, and the request often oscillates between the two torque limits while the driver fights it, which a pure shift cannot follow.

### 3.1 pure shift (as specified)

| group | turn-ins | min Delta median | p75 | p90 | needs none | > 1.5 s | met at 0.3 | at 0.5 | at 0.8 | at 1.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0-10 mph | 18 | >1.5 | >1.5 | >1.5 | 11% | 67% | 11% | 11% | 17% | 22% |
| 10-20 mph | 20 | 1.1 | >1.5 | >1.5 | 20% | 40% | 35% | 35% | 45% | 50% |
| 0-20 mph | 41 | >1.5 | >1.5 | >1.5 | 15% | 54% | 24% | 24% | 32% | 37% |
| clean 0-10 | 4 | >1.5 | >1.5 | >1.5 | 25% | 75% | 25% | 25% | 25% | 25% |
| clean 10-20 | 6 | 0.2 | 0.3 | >1.5 | 33% | 17% | 83% | 83% | 83% | 83% |
| clean 0-20 | 11 | 0.2 | >1.5 | >1.5 | 27% | 36% | 64% | 64% | 64% | 64% |

By peak radius (1 / max |curvature| over the turn-in), 0-20 mph:

| group | turn-ins | min Delta median | p75 | p90 | needs none | > 1.5 s | met at 0.3 | at 0.5 | at 0.8 | at 1.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| < 10 m | 14 | >1.5 | >1.5 | >1.5 | 7% | 57% | 7% | 7% | 21% | 29% |
| 10-20 m | 16 | 1.4 | >1.5 | >1.5 | 19% | 44% | 38% | 38% | 44% | 44% |
| 20-50 m | 11 | >1.5 | >1.5 | >1.5 | 18% | 64% | 27% | 27% | 27% | 36% |

### 3.2 rise-only advance

| group | turn-ins | min Delta median | p75 | p90 | needs none | > 1.5 s | met at 0.3 | at 0.5 | at 0.8 | at 1.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0-10 mph | 18 | 0.8 | >1.5 | >1.5 | 11% | 28% | 17% | 28% | 61% | 67% |
| 10-20 mph | 20 | 0.8 | 1.2 | >1.5 | 20% | 15% | 40% | 40% | 55% | 65% |
| 0-20 mph | 41 | 0.8 | 1.2 | >1.5 | 15% | 24% | 29% | 34% | 56% | 63% |
| clean 0-10 | 4 | >1.5 | >1.5 | >1.5 | 25% | 50% | 50% | 50% | 50% | 50% |
| clean 10-20 | 6 | 0.1 | 0.2 | >1.5 | 33% | 17% | 83% | 83% | 83% | 83% |
| clean 0-20 | 11 | 0.2 | >1.5 | >1.5 | 27% | 27% | 73% | 73% | 73% | 73% |

By peak radius (1 / max |curvature| over the turn-in), 0-20 mph:

| group | turn-ins | min Delta median | p75 | p90 | needs none | > 1.5 s | met at 0.3 | at 0.5 | at 0.8 | at 1.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| < 10 m | 14 | 0.8 | 1.1 | 1.2 | 7% | 7% | 14% | 14% | 57% | 71% |
| 10-20 m | 16 | 0.8 | >1.5 | >1.5 | 19% | 31% | 38% | 50% | 56% | 56% |
| 20-50 m | 11 | 0.8 | >1.5 | >1.5 | 18% | 36% | 36% | 36% | 55% | 64% |

Strict criterion for the pure shift (every frame of the driver-clipped request must be matched within 19 units, no envelope):

| group | turn-ins | min Delta median | p75 | p90 | needs none | > 1.5 s | met at 0.3 | at 0.5 | at 0.8 | at 1.0 |
|---|---|---|---|---|---|---|---|---|---|---|
| 0-20 mph | 41 | >1.5 | >1.5 | >1.5 | 5% | 63% | 10% | 10% | 20% | 27% |
| clean 0-20 | 11 | >1.5 | >1.5 | >1.5 | 9% | 55% | 27% | 27% | 36% | 45% |

10 of 41 turn-ins are not met within 1.5 s even by the rise-only advance; in those the request reverses between the torque limits within the window or the driver clips it (7 of them contain a press), so no earlier start of the same request profile helps.

Peak request of the turn-ins: 100-192 0, 192-288 1, 288-380 0, 380-384 (cap) 40 (every turn-in that matters wants the full 384, so the ramp needs 1.28 s from zero).

## 4. Behind time and alerts removed by an advance of Delta

Behind = tight, unpressed, < 20 mph, turn*(des-act) > 0.3, inside a turn-in window [t_c, lobe end]. Four response models (turn-in only; none of them can credit the 384 cap):
- A request matching: removed if sent was > 19 units short of the driver-clipped request (the rate limit, not the cap or the driver) AND sent' is within 19 units.
- B static gain with delay: act' = act + g (sent' - sent)(t - d), d = 0.32 s (liveDelay), g = 1.52 m/s^2 per full torque (fitted on these turn-ins).
- B2 the same with g = 2.26 m/s^2 per full torque (steer_torque_limit_results.md, v > 8 m/s).
- C response shift (optimistic bound): the car's response moves with the torque profile, act'(t) = act(t + s), s = min(Delta, how much earlier sent' reaches 98% of the peak request than sent did). It assumes the wheel angle keeps integrating the torque (true under a pegged request at 2-9 m/s, where the wheel turns 100-280 deg/s), so the lead persists after the ramp.

### 4.1 Behind time removed, pure shift (share of ALL behind seconds in the band; A / B / B2 / C)

'in turn-in windows' = share of behind time inside an analysed turn-in window (the rest is in dropped low-demand lobes or outside any lobe); 'cap-bound' = share of those frames where sent was not short of the driver-clipped request by more than 19 units (at the 384 cap, or the request itself is low), so advancing the ramp changes nothing there.

| band mph | behind s | in turn-in windows | cap-bound share | Delta 0.3 | 0.5 | 0.8 | 1.0 |
|---|---|---|---|---|---|---|---|
| 0-10 | 13.9 | 99% | 64% | 10% / 14% / 24% / 12% | 13% / 21% / 27% / 19% | 11% / 18% / 23% / 28% | 9% / 15% / 20% / 28% |
| 10-20 | 36.9 | 94% | 69% | 1% / 3% / 5% / 6% | 2% / 5% / 6% / 13% | 2% / 6% / 7% / 18% | 3% / 6% / 7% / 19% |
| 0-20 | 50.8 | 95% | 68% | 3% / 6% / 10% / 7% | 5% / 9% / 12% / 15% | 5% / 9% / 11% / 21% | 4% / 8% / 10% / 21% |

Every Delta in the grid (pure shift, 0-20 mph):

| Delta s | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.1 | 1.2 | 1.3 | 1.4 | 1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A removed | 1% | 2% | 3% | 4% | 5% | 5% | 5% | 5% | 5% | 4% | 4% | 4% | 4% | 4% | 4% |
| B removed | 3% | 4% | 6% | 8% | 9% | 9% | 9% | 9% | 9% | 8% | 8% | 8% | 8% | 7% | 7% |
| B2 removed | 3% | 6% | 10% | 11% | 12% | 12% | 11% | 11% | 11% | 10% | 10% | 10% | 9% | 9% | 8% |
| C removed | 3% | 4% | 7% | 12% | 15% | 19% | 20% | 21% | 22% | 21% | 21% | 20% | 20% | 20% | 20% |

### 4.2 Behind time removed, rise-only (share of ALL behind seconds in the band; A / B / B2 / C)

'in turn-in windows' = share of behind time inside an analysed turn-in window (the rest is in dropped low-demand lobes or outside any lobe); 'cap-bound' = share of those frames where sent was not short of the driver-clipped request by more than 19 units (at the 384 cap, or the request itself is low), so advancing the ramp changes nothing there.

| band mph | behind s | in turn-in windows | cap-bound share | Delta 0.3 | 0.5 | 0.8 | 1.0 |
|---|---|---|---|---|---|---|---|
| 0-10 | 13.9 | 99% | 64% | 13% / 15% / 24% / 10% | 16% / 21% / 27% / 16% | 19% / 23% / 27% / 17% | 19% / 23% / 27% / 17% |
| 10-20 | 36.9 | 94% | 69% | 2% / 4% / 6% / 8% | 2% / 6% / 7% / 12% | 3% / 6% / 7% / 13% | 3% / 6% / 7% / 14% |
| 0-20 | 50.8 | 95% | 68% | 5% / 7% / 11% / 9% | 6% / 10% / 12% / 13% | 7% / 11% / 12% / 14% | 7% / 11% / 12% / 15% |

Every Delta in the grid (rise-only, 0-20 mph):

| Delta s | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.1 | 1.2 | 1.3 | 1.4 | 1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A removed | 2% | 3% | 5% | 6% | 6% | 7% | 7% | 7% | 7% | 7% | 7% | 7% | 7% | 7% | 7% |
| B removed | 3% | 5% | 7% | 8% | 10% | 11% | 11% | 11% | 11% | 11% | 11% | 11% | 11% | 11% | 11% |
| B2 removed | 4% | 7% | 11% | 12% | 12% | 12% | 12% | 12% | 12% | 12% | 12% | 12% | 12% | 12% | 12% |
| C removed | 2% | 4% | 9% | 12% | 13% | 14% | 14% | 14% | 15% | 15% | 14% | 15% | 15% | 15% | 15% |

### 4.3 The 21 steerSaturated alerts

For each alert the turn-in containing it is analysed like the episodes (same limiter). The PRED predicate (selfdrived.py L769-783) is re-evaluated in [alert - 0.1, alert + 2.0 s]: A: a PRED frame survives unless the rate limit was what held sent short and sent' is within 19 units; B / B2 / C: undershoot recomputed with controlsState.curvature * v^2 moved by the model (the 'output at max' condition is kept as logged, i.e. the controller is assumed to stay saturated: conservative). The alert is removed when no PRED frame survives. Rise-only advance; the pure shift is summarised in the count table. 'peak req' = turn peak request, 'min Delta' = section 3 criterion (rise-only).

Timeline per alert (seconds relative to t_c, des crossing 0.3 of the turn-in that contains the alert):

| route | t s | mph | alert at | t_a | t_r0 | t_rp request peak | t_sp sent peak | t_err | plan lead any tau (rate or chord) |  |
|---|---|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 1716 | 20 | +3.70 | -0.07 | -0.36 | 1.64 | 2.51 | 0.30 | 5.27 | press |
| 0000000d--5e0d1d244b | 1815 | 10 | +0.65 | -0.02 | -1.52 | 0.06 | 5.36 | 0.49 | -1.03 |  |
| 0000000d--5e0d1d244b | 1820 | 17 | +5.33 | -0.02 | -1.52 | 0.06 | 5.36 | 0.49 | -1.04 |  |
| 0000000e--5ebf4636a1 | 511 | 19 | +2.56 | -0.02 | -0.94 | 0.46 | 1.24 | -0.01 | 0.69 | press |
| 0000000e--5ebf4636a1 | 3050 | 30 | +3.59 | -0.02 | -0.61 | 1.76 | never | 0.40 | 1.82 | press |
| 0000001d--2ead59d2dd | 774 | 14 | +1.30 | 0.00 | -1.05 | 0.52 | 2.43 | 0.47 | -0.12 |  |
| 0000001d--2ead59d2dd | 777 | 16 | +4.62 | 0.00 | -1.05 | 0.52 | 2.43 | 0.47 | -0.12 |  |
| 0000001d--2ead59d2dd | 894 | 40 | +5.88 | -0.14 | 0.17 | 3.78 | never | 2.72 | 5.09 | press |
| 0000001d--2ead59d2dd | 1465 | 14 | +3.37 | -0.07 | 0.22 | 0.62 | 1.46 | 0.62 | 2.10 | press |
| 0000001d--2ead59d2dd | 1579 | 10 | +0.64 | -0.08 | -1.52 | 0.04 | 2.17 | 0.48 | -0.92 |  |
| 0000001d--2ead59d2dd | 1812 | 10 | +1.54 | -0.05 | -0.98 | 0.27 | 1.90 | 0.76 | 0.48 |  |
| 0000001d--2ead59d2dd | 1835 | 6 | +1.50 | -0.01 | -1.52 | -0.01 | -0.01 | 0.93 | -0.24 |  |
| 00000028--297af64c9c | 128 | 19 | +2.29 | -0.01 | -1.52 | 0.90 | 6.71 | 0.75 | 0.65 | press |
| 00000028--297af64c9c | 897 | 25 | +3.51 | -0.01 | -0.10 | 3.51 | 4.86 | 1.86 | 3.52 |  |
| 00000028--297af64c9c | 964 | 26 | +2.69 | 0.00 | 0.01 | 0.01 | 3.11 | 0.00 | 0.01 | press |
| 00000028--297af64c9c | 3294 | 13 | +2.76 | -0.04 | -1.46 | -0.01 | 1.53 | -0.01 | -0.16 | press |
| 00000028--297af64c9c | 3980 | 14 | +2.17 | -0.04 | -0.37 | 0.23 | 2.11 | -0.01 | 0.33 | press |
| 00000029--b3c2c13819 | 2819 | 54 | +2.19 | -0.05 | -0.18 | 2.07 | 2.64 | 2.07 | 5.08 |  |
| 00000029--b3c2c13819 | 2924 | 22 | +0.75 | -0.06 | -0.34 | 0.75 | 1.04 | 0.50 | 1.98 |  |
| 00000029--b3c2c13819 | 3297 | 9 | +0.45 | -0.12 | -1.22 | -0.01 | 0.50 | -0.01 | -0.76 |  |
| 00000034--5f8605e608 | 4132 | 16 | +2.31 | -0.03 | -0.82 | 1.66 | 1.95 | 2.06 | 1.56 |  |

| route | t s | mph | peak req | min Delta | removed @0.3 (A/B/B2/C) | @0.5 | @0.8 | @1.0 |
|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 1716 | 20 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000000d--5e0d1d244b | 1815 | 10 | 384 | 0.1 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000000d--5e0d1d244b | 1820 | 17 | 384 | 0.1 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000000e--5ebf4636a1 | 511 | 19 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000000e--5ebf4636a1 | 3050 | 30 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 774 | 14 | 384 | 0.1 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 777 | 16 | 384 | 0.1 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 894 | 40 | 384 | >1.5 | n/n/n/y | n/n/n/y | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 1465 | 14 | 384 | 0.8 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 1579 | 10 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 1812 | 10 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 0000001d--2ead59d2dd | 1835 | 6 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000028--297af64c9c | 128 | 19 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000028--297af64c9c | 897 | 25 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000028--297af64c9c | 964 | 26 | 384 | 0.0 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000028--297af64c9c | 3294 | 13 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000028--297af64c9c | 3980 | 14 | 384 | >1.5 | n/n/n/y | n/n/n/y | n/n/n/y | n/n/n/y |
| 00000029--b3c2c13819 | 2819 | 54 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/y | n/n/n/y |
| 00000029--b3c2c13819 | 2924 | 22 | 384 | 0.2 | n/y/y/y | n/y/y/y | n/y/y/y | n/y/y/y |
| 00000029--b3c2c13819 | 3297 | 9 | 384 | >1.5 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |
| 00000034--5f8605e608 | 4132 | 16 | 384 | 0.2 | n/n/n/n | n/n/n/n | n/n/n/n | n/n/n/n |

Alerts removed of the 21 analysable ones, by Delta:

| Delta s | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.1 | 1.2 | 1.3 | 1.4 | 1.5 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| rise-only A | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| rise-only B | 0 | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| rise-only B2 | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| rise-only C | 1 | 2 | 3 | 3 | 3 | 2 | 2 | 3 | 3 | 3 | 3 | 3 | 3 | 2 | 2 |
| shift A | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| shift B | 0 | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| shift B2 | 0 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 | 1 |
| shift C | 2 | 3 | 3 | 4 | 4 | 3 | 2 | 3 | 3 | 3 | 3 | 3 | 3 | 2 | 1 |

Share of the PRED frames (alert predicate true, 100 Hz, in the alert windows) that survive with the rise-only advance: the alerts come 0.5-6 s after t_c, mostly while sent already sits at the cap and the car still undershoots, so the predicate stays true.

| model | Delta 0.3 | 0.5 | 0.8 | 1.0 |
|---|---|---|---|---|
| A | 99% | 99% | 99% | 99% |
| B | 96% | 96% | 96% | 96% |
| B2 | 96% | 96% | 96% | 96% |
| C | 93% | 90% | 89% | 89% |

Min Delta for the alert turn-ins (rise-only): median 0.2, p75 >1.5, p90 >1.5 s; share needing > 1.5 s: 38%; needing none: 29%.
Min Delta for the alert turn-ins (pure shift): median 0.2, p75 >1.5, p90 >1.5 s; share needing > 1.5 s: 38%; needing none: 29%.

## 5. Is the advance knowable? What the model plan shows ahead of time

a_plan(t, tau) = plan curvature at tau seconds ahead of the latest modelV2 * v_ego^2; 'rate' = orientationRate.z / velocity.x, 'chord' = circle through the car (tangent to its heading) and the plan position at tau (the probe TURN_LEAD uses). tau0 = 0.50 s is the look-ahead modeld already uses for the action, so a_plan(tau0) is today's feed-forward input. Plan crossing time = start of the last run, going back from the des peak, in which turn*a_plan >= 0.3 m/s^2. Lead = t_r0 - plan crossing (positive = the plan showed the turn before the request started its final rise; this is the anticipation available to start the ramp early); lead_c = t_c - plan crossing.

Plan crossing relative to the controller (seconds, median / p25 / p10; positive = plan earlier). 'today' = a_plan(tau0); 'plan(tau0 + e)' = the same plan read e seconds further ahead (what a larger look-ahead would feed); 'any tau' = the earliest crossing over tau in {tau0, 0.5, 1, 1.5, 2, 3, 4, 5 s}, rate or chord; 'any tau (rate)' without the chord probe. n/a = the plan never showed >= 0.3 at that tau by the des peak.

| set | n | lead today | lead_c today | lead plan(tau0 + 1.0) | lead any tau (rate) | lead any tau (rate or chord) |
|---|---|---|---|---|---|---|
| 0-10 mph | 18 | 0.28 / -0.83 / -1.37 (n/a 1) | -0.04 / -0.33 / -0.76 (n/a 1) | 0.10 / -0.61 / -0.92 (n/a 4) | 0.28 / -0.32 / -0.88 (n/a 1) | 0.67 / -0.23 / -0.45 (n/a 0) |
| 10-20 mph | 23 | -0.37 / -1.03 / -1.49 (n/a 0) | 0.13 / -0.02 / -0.10 (n/a 0) | 0.39 / -0.13 / -0.94 (n/a 1) | 0.65 / 0.18 / -0.80 (n/a 0) | 0.65 / 0.38 / -0.80 (n/a 0) |
| 0-20 mph | 41 | -0.08 / -0.93 / -1.49 (n/a 1) | 0.04 / -0.07 / -0.35 (n/a 1) | 0.31 / -0.51 / -0.95 (n/a 5) | 0.44 / -0.17 / -0.90 (n/a 1) | 0.65 / -0.16 / -0.76 (n/a 0) |
| 0-20 clean | 11 | -1.03 / -1.42 / -1.49 (n/a 1) | 0.02 / -0.03 / -0.06 (n/a 1) | -0.82 / -1.19 / -1.26 (n/a 2) | -0.06 / -0.93 / -1.08 (n/a 1) | -0.12 / -0.50 / -0.95 (n/a 0) |

Realized advance of a raised look-ahead = plan crossing today - plan crossing of a_plan(tau0 + e) (median / p25 / p10 seconds; n = turn-ins where both exist). The needed Delta (section 3) is the target. 'share >= 0.8 e' = how often the plan delivers at least 80% of the extension.

| set | e (s) | n | realized advance | share >= 0.8 e |
|---|---|---|---|---|
| 0-10 mph | +0.2 | 16 | 0.00 / 0.00 / -0.05 | 12% |
|  | +0.4 | 16 | 0.05 / -0.01 / -0.08 | 19% |
|  | +0.6 | 16 | 0.07 / -0.06 / -0.13 | 12% |
|  | +0.8 | 16 | 0.07 / -0.06 / -0.22 | 6% |
|  | +1.0 | 14 | 0.10 / 0.00 / -0.12 | 7% |
|  | +1.5 | 10 | 0.14 / -0.02 / -0.06 | 0% |
|  | +2.0 | 5 | -0.05 / -0.10 / -0.13 | 0% |
| 10-20 mph | +0.2 | 23 | 0.05 / 0.00 / 0.00 | 26% |
|  | +0.4 | 23 | 0.15 / 0.00 / 0.00 | 39% |
|  | +0.6 | 23 | 0.23 / 0.00 / -0.20 | 39% |
|  | +0.8 | 22 | 0.40 / 0.00 / 0.00 | 32% |
|  | +1.0 | 22 | 0.41 / 0.00 / 0.00 | 36% |
|  | +1.5 | 21 | 0.54 / 0.00 / 0.00 | 29% |
|  | +2.0 | 18 | 0.68 / 0.00 / -0.12 | 17% |
| 0-20 mph | +0.2 | 39 | 0.05 / 0.00 / -0.01 | 21% |
|  | +0.4 | 39 | 0.10 / 0.00 / -0.05 | 31% |
|  | +0.6 | 39 | 0.10 / 0.00 / -0.17 | 28% |
|  | +0.8 | 38 | 0.16 / 0.00 / -0.18 | 21% |
|  | +1.0 | 36 | 0.21 / 0.00 / -0.10 | 25% |
|  | +1.5 | 31 | 0.39 / 0.00 / -0.05 | 19% |
|  | +2.0 | 23 | 0.52 / 0.00 / -0.14 | 13% |

Available anticipation against the needed Delta, per turn-in (needed Delta from section 3.2, rise-only advance with the envelope criterion; turn-ins that need none are dropped; '> 1.5' counts as unmet):

| set | n | needed Delta median | anticipation (rate, any tau) median / p10 | plan sees >= Delta | plan sees >= Delta/2 | anticipation (rate+chord) median / p10 | plan sees >= Delta |
|---|---|---|---|---|---|---|---|
| all 0-20 mph | 35 | 0.8 (> 1.5 s: 10) | 0.57 / -0.32 | 34% | 43% | 0.82 / -0.26 | 46% |
| 0-10 mph | 16 | 0.7 (> 1.5 s: 5) | 0.31 / -0.47 | 31% | 31% | 0.82 / -0.32 | 50% |
| 10-20 mph | 19 | 0.8 (> 1.5 s: 5) | 0.69 / -0.13 | 37% | 53% | 0.85 / -0.13 | 42% |
| clean | 8 | 0.2 (> 1.5 s: 3) | 0.01 / -0.94 | 25% | 25% | -0.06 / -0.85 | 25% |

How good is the plan as a predictor? For every look-ahead extension e, compare a_plan(t, tau0 + e) with the controller's own feed-forward input a_ff at t + e (a_ff = actuators.curvature * v^2, i.e. what the command will be e seconds later). Baseline = today's command a_ff(t). Frames in the turn-in windows [i_c - 0.5 s, i_rp], turn space; error = mean |prediction - a_ff(t + e)| (m/s^2) and share of frames where the plan lands within 0.3 of it.

| extension e (s) | frames | plan(tau0+e) error vs a_ff(t+e) | today's a_ff(t) error | plan within 0.3 | today's within 0.3 |
|---|---|---|---|---|---|
| +0.2 | 5065 | 0.21 | 0.18 | 79% | 83% |
| +0.4 | 5010 | 0.29 | 0.36 | 63% | 57% |
| +0.6 | 4990 | 0.40 | 0.53 | 47% | 37% |
| +0.8 | 4990 | 0.51 | 0.67 | 37% | 26% |
| +1.0 | 4990 | 0.61 | 0.78 | 31% | 21% |
| +1.5 | 4990 | 0.82 | 1.01 | 27% | 16% |
| +2.0 | 4990 | 0.96 | 1.14 | 24% | 13% |

## 6. Side effects

### 6a. Unwind and S-bends (shifted request, same limiter), turn-ins of 0-20 mph

S-bends (opposite lobe with peak > 1 m/s^2 within 3 s of the lobe end): 7 of 41 turn-ins (17%).

| Delta s | pure shift: early unwind s median (p90) | turn-ins with > 0.3 s | S-bend counter-torque s median (p90) | S-bends with > 0.2 s | extra torque vs original, median units (p90) | rise-only: early unwind s (p90) | rise-only: S-bend counter-torque s (p90) | rise-only: extra torque, median units (p90) |
|---|---|---|---|---|---|---|---|---|
| 0.2 | 0.45 (0.89) | 66% | 0.00 (0.12) | 0% | 86 (140) | 0.00 (0.00) | 0.00 (0.00) | 110 (171) |
| 0.3 | 0.60 (1.25) | 83% | 0.00 (0.25) | 14% | 104 (210) | 0.00 (0.00) | 0.00 (0.00) | 149 (220) |
| 0.5 | 0.88 (1.55) | 88% | 0.00 (0.49) | 29% | 150 (285) | 0.00 (0.00) | 0.00 (0.00) | 202 (285) |
| 0.8 | 1.20 (1.93) | 90% | 0.00 (0.73) | 43% | 237 (304) | 0.00 (0.00) | 0.00 (0.00) | 251 (379) |
| 1.0 | 1.31 (2.36) | 93% | 0.00 (0.78) | 43% | 276 (379) | 0.00 (0.00) | 0.00 (0.00) | 294 (396) |
| 1.5 | 1.47 (2.57) | 93% | 0.00 (0.85) | 43% | 343 (401) | 0.00 (0.00) | 0.00 (0.00) | 384 (420) |

(Early unwind = frames after the des peak where the shifted sent' is more than 40 units below the driver-clipped request while the unshifted one was not: a literal shift of the whole request releases the turn early. Counter-torque = in S-bends, after the first lobe ended, sent' pushes against the first turn by more than 40 units beyond the request. These are the costs of a perfect-foresight shift applied to everything; an advance applied only to the turn-in (look-ahead raised on the rise, not on the unwind) would not have the unwind cost.)

### 6b. Gentle non-tight curves: how big is a shift of Delta in lat accel on frames that were never in trouble?

Frames: lateral active, unpressed, not tight, 0.3 < |des| < 1.5. Share of frames where |des(t + Delta) - des(t)| > X (this is how much earlier a perfect-prediction advance would command than today), by speed band.

| Delta s | 0-10 mph (X = 0.3 / 0.5 / 1.0 m/s^2) | 10-20 mph (X = 0.3 / 0.5 / 1.0 m/s^2) | 20-40 mph (X = 0.3 / 0.5 / 1.0 m/s^2) | 40+ mph (X = 0.3 / 0.5 / 1.0 m/s^2) |
|---|---|---|---|---|
| 0.2 | 2% / 0% / 0% | 3% / 2% / 0% | 2% / 0% / 0% | 2% / 0% / 0% |
| 0.3 | 14% / 1% / 0% | 6% / 2% / 1% | 5% / 1% / 0% | 5% / 1% / 0% |
| 0.5 | 21% / 3% / 0% | 15% / 4% / 2% | 13% / 4% / 0% | 14% / 4% / 0% |
| 0.8 | 28% / 3% / 1% | 25% / 11% / 2% | 24% / 11% / 1% | 22% / 8% / 1% |
| 1.0 | 34% / 4% / 1% | 32% / 15% / 2% | 30% / 14% / 2% | 22% / 8% / 1% |

### 6c. Tracking error on non-tight frames if the look-ahead were raised at low speed only

Frames: lateral active, unpressed, not tight (de-facto gentle and straight driving), by band. Baseline error = |des - act| as logged. Shift model: the car's response moves e earlier, act'(t) = act(t + e), measured against the unchanged reference des(t) (perfect plan; upper bound of the earliness cost). Plan model: act'(t) = act(t) + [a_plan(t - d, tau0 + e) - a_plan(t - d, tau0)] with d = 0.32 s (what the plan actually says; plan in the des sign convention). Mean |error| m/s^2 / share > 0.3 m/s^2.

| band | baseline | shift +0.2 | shift +0.4 | shift +0.8 | plan +0.2 | plan +0.4 | plan +0.8 | plan +1.5 |
|---|---|---|---|---|---|---|---|---|
| 0-10 mph | 0.010 / 0.1% | 0.007 / 0.1% | 0.007 / 0.0% | 0.013 / 0.2% | 0.009 / 0.1% | 0.009 / 0.0% | 0.009 / 0.0% | 0.010 / 0.1% |
| 10-20 mph | 0.026 / 0.5% | 0.020 / 0.3% | 0.021 / 0.4% | 0.030 / 1.0% | 0.023 / 0.4% | 0.023 / 0.4% | 0.026 / 0.8% | 0.035 / 1.7% |
| 20-40 mph | 0.060 / 2.4% | 0.046 / 1.8% | 0.049 / 2.3% | 0.079 / 5.0% | 0.050 / 2.0% | 0.050 / 2.1% | 0.066 / 4.3% | 0.102 / 7.5% |
| 40+ mph | 0.089 / 1.9% | 0.063 / 1.2% | 0.083 / 3.4% | 0.148 / 12.1% | 0.076 / 1.5% | 0.075 / 1.7% | 0.093 / 4.0% | 0.134 / 10.3% |

## 7. Hardest turn-ins (0-20 mph, peak request >= 250, highest rise-only min Delta first)

| route | route t s | mph | radius m | peak des | peak req | t_rp - t_c | t_sp - t_c | ramp needed s | min Delta shift / rise-only | request ramp s | plan lead any tau s |  |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 1713 | 26 | 18 | 3.35 | 384 | 1.64 | 2.51 | 1.28 | >1.5 / >1.5 | 2.00 | 5.07 | press |
| 0000000e--5ebf4636a1 | 508 | 20 | 25 | 2.99 | 384 | 0.46 | 1.24 | 1.28 | >1.5 / >1.5 | 1.40 | 0.69 | press |
| 0000001d--2ead59d2dd | 1802 | 5 | 5 | 1.65 | 384 | 0.54 | 1.63 | 1.28 | >1.5 / >1.5 | 0.16 | 0.53 | press |
| 00000028--297af64c9c | 3291 | 12 | 20 | 1.54 | 384 | -0.01 | 1.53 | 1.28 | >1.5 / >1.5 | 1.45 | -0.16 | press |
| 00000028--297af64c9c | 3362 | 9 | 15 | 1.48 | 384 | 0.01 | 1.48 | 1.28 | >1.5 / >1.5 | 0.00 | -0.19 | press |
| 00000028--297af64c9c | 3977 | 15 | 19 | 2.65 | 384 | 0.23 | 2.11 | 1.28 | >1.5 / >1.5 | 0.60 | 0.33 | press |
| 00000029--b3c2c13819 | 3296 | 9 | 10 | 1.94 | 384 | -0.01 | 0.50 | 1.28 | >1.5 / >1.5 | 1.21 | -0.76 |  |
| 00000029--b3c2c13819 | 3391 | 8 | 11 | 1.20 | 384 | -0.01 | 0.75 | 1.28 | >1.5 / >1.5 | 0.75 | -0.32 | press |
| 00000034--5f8605e608 | 4340 | 14 | 24 | 0.98 | 384 | 1.71 | never | 1.28 | >1.5 / >1.5 | 2.48 | 0.60 |  |
| 00000028--297af64c9c | 3602 | 8 | 30 | 0.53 | 275 | 0.08 | never | 0.92 | >1.5 / >1.5 | 0.07 | 0.01 |  |
| 0000000d--5e0d1d244b | 2090 | 11 | 13 | 1.61 | 384 | 0.24 | 1.42 | 1.28 | 1.4 / 1.2 | 0.08 | 0.53 | press |
| 0000001d--2ead59d2dd | 1816 | 5 | 5 | 1.15 | 384 | 0.76 | 1.94 | 1.28 | 1.2 / 1.2 | 0.06 | 7.07 | press |
| 00000029--b3c2c13819 | 3388 | 11 | 9 | 2.30 | 384 | 0.42 | never | 1.28 | >1.5 / 1.2 | 0.07 | 0.99 | press |
| 00000028--297af64c9c | 625 | 14 | 12 | 3.19 | 384 | 0.23 | 1.31 | 1.28 | 1.1 / 1.1 | 0.16 | 0.43 | press |
| 00000029--b3c2c13819 | 3293 | 11 | 8 | 2.44 | 384 | 0.57 | 1.68 | 1.28 | >1.5 / 1.1 | 0.13 | 1.57 | press |
| 00000034--5f8605e608 | 4330 | 17 | 20 | 2.68 | 384 | 1.64 | never | 1.28 | >1.5 / 1.0 | 0.20 | 1.88 | press |
| 00000034--5f8605e608 | 4350 | 6 | 5 | 1.71 | 384 | 0.28 | 1.48 | 1.28 | 1.3 / 1.0 | 0.05 | 0.81 | press |
| 00000028--297af64c9c | 3973 | 12 | 10 | 2.85 | 384 | 0.33 | 2.22 | 1.28 | >1.5 / 0.9 | 0.15 | 1.10 | press |
| 0000000d--5e0d1d244b | 2085 | 20 | 24 | 2.07 | 384 | 0.54 | never | 1.28 | 0.9 / 0.8 | 0.42 | 0.49 | press |
| 0000000d--5e0d1d244b | 2163 | 5 | 5 | 1.04 | 384 | 0.88 | 2.12 | 1.28 | >1.5 / 0.8 | 0.02 | 2.38 | press |
| 0000000e--5ebf4636a1 | 493 | 19 | 24 | 3.66 | 384 | 7.65 | 8.51 | 1.28 | >1.5 / 0.8 | 0.31 | 7.35 | press |
| 0000001d--2ead59d2dd | 1462 | 15 | 14 | 3.11 | 384 | 0.62 | 1.46 | 1.28 | 0.8 / 0.8 | 0.40 | 2.10 | press |
| 0000001d--2ead59d2dd | 1735 | 9 | 9 | 2.05 | 384 | 0.58 | 1.58 | 1.28 | 1.0 / 0.8 | 0.21 | 0.82 | press |
| 00000028--297af64c9c | 3395 | 9 | 8 | 2.41 | 384 | 0.76 | 1.66 | 1.28 | >1.5 / 0.8 | 0.26 | 1.26 | press |
| 00000028--297af64c9c | 3285 | 9 | 9 | 2.33 | 384 | 2.11 | 3.11 | 1.28 | >1.5 / 0.7 | 0.22 | 2.66 | press |

## 8. Summary of this run

- Needed Delta, rise-only advance, 41 turn-ins 0-20 mph (peak request 384 in all but one): median 0.8 s, p75 1.2, p90 >1.5; met within 0.3 / 0.5 / 0.8 / 1.0 s: 29% / 34% / 56% / 63%; not met within 1.5 s: 24%.
- Needed Delta, pure shift, 41 turn-ins 0-20 mph (peak request 384 in all but one): median >1.5 s, p75 >1.5, p90 >1.5; met within 0.3 / 0.5 / 0.8 / 1.0 s: 24% / 24% / 32% / 37%; not met within 1.5 s: 54%.
- The request itself rises in a median 0.45 s (t_rp - t_r0) while the ramp limit needs 1.28 s from zero to 384: that gap (~0.8 s) is the needed Delta; sent reaches the peak a median 1.07 s after the request.
- Plan anticipation (turn visible in the plan before the request starts its final rise; any tau <= 5 s): rate 0.44 s median / -0.17 p25 / -0.90 p10, rate or chord 0.65 / -0.16 / -0.76. Today's a_plan(tau0) crosses 0.3 0.04 s before des does (it already is the feed-forward input).
- Among the 35 turn-ins that need an advance, the plan (rate or chord, any tau) showed the turn at least the needed Delta ahead in 46% and at least half of it in 54%.
- Behind time removed (rise-only, share of all 0-20 mph behind seconds, A/B/B2/C): 0.3: 5%/7%/11%/9%; 0.5: 6%/10%/12%/13%; 0.8: 7%/11%/12%/14%; 1.0: 7%/11%/12%/15%.
- Behind time removed (pure shift, share of all 0-20 mph behind seconds, A/B/B2/C): 0.3: 3%/6%/10%/7%; 0.5: 5%/9%/12%/15%; 0.8: 5%/9%/11%/21%; 1.0: 4%/8%/10%/21%.
- Alerts removed of 21 (rise-only, A/B/B2/C): 0.3: 0/1/1/3; 0.5: 0/1/1/3; 0.8: 0/1/1/3; 1.0: 0/1/1/3.
- Alerts removed of 21 (pure shift, A/B/B2/C): 0.3: 0/1/1/3; 0.5: 0/1/1/4; 0.8: 0/1/1/3; 1.0: 0/1/1/3.
- Gentle curves, share of non-tight frames where a pure shift of 0.5 s changes des by > 0.3 m/s^2: 0-10 mph 21%, 10-20 mph 15%, 20-40 mph 13%, 40+ mph 14%.
- Non-tight tracking error (mean |des - act| m/s^2 / share > 0.3), baseline vs plan model +0.8 s: 0-10 mph 0.010/0.1% -> 0.009/0.0%, 10-20 mph 0.026/0.5% -> 0.026/0.8%, 20-40 mph 0.060/2.4% -> 0.066/4.3%, 40+ mph 0.089/1.9% -> 0.093/4.0%.

