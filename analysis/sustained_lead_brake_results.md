# Sustained lead braking: slot 9 B planner change, closed-loop A/B

Before = x86 at `623729c81` (623729c81), After = `8e9cbdf84` (8e9cbdf84). Both: Testing Ground 9:B forced, car tune B, actuator delay 0.5, lead filter fast, vision-model lead = constant speed (sim default), radar lead from radard's Track (see below).

Slot 9 B evidence (from the sim process, printed after planner construction):

- before: `{"extrapolate_lead": "extrapolate_lead", "stock_tune": "n/a (function absent)", "latch": {}, "has_close_cap_hook": false, "far_follow_slew": [0.0, 0.0], "x86_head": "623729c81"}`
- after: `{"extrapolate_lead": "get_kia_stinger_2022_extrapolate_lead", "stock_tune": "B", "latch": {"tune": "B"}, "has_close_cap_hook": true, "far_follow_slew": [0.0, 0.0], "x86_head": "8e9cbdf84"}`

(The before commit has no Stinger hooks in the planner at all, so its extrapolate_lead is upstream and there is no stock_tune function; the 9:B car-side tune is --car-tune B in both.)

Lead model: radard `Track` (KF1D + aLeadTau rule) fed the true lead speed at 20 Hz, no radar noise or latency. aLeadK is therefore the Kalman estimate (lags the true accel by a few hundred ms), aLeadTau starts at 0.6, decays x0.9/step (rc 0.45 s) once |aLeadK| >= 0.5 and recovers x1.1/step otherwise. Not modelled: radar noise/dropouts, a real vision-model lead prediction (constant-speed lead used), experimental mode, AEB/driver.

Scenario: ego = lead = v0, gap = headway x v0, cruise set speed v0 + 10 mph, 5 s steady, lead accel ramps in over 0.5 s to a and holds until stopped. Peak decel = hardest applied accel (car response) after onset. t-a = seconds from lead brake onset to applied accel first <= -0.5/-1/-2 ('-' = never). Min time gap = min(gap / v_ego) after onset while v_ego > 2 m/s. Run ends at contact (gap < 0.3 m): then min gap ~0, final gap 0 and peak decel is only up to the impact. Criteria: (1) no new contact, (2) min gap >= before - 0.25 m, (3) peak decel <= before + 0.3. Primary cases are `standard` (logged minAcceleration -0.5) and `aggressive` (-2.0). The standard -0.5 floor binds on a hard-braking lead so BEFORE already crashes in most standard scenarios; two supplementary standard cases with a deeper floor (-1.0, -2.0) are included and reported separately.

## standard, 45 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.50 / 1.50 | no / no | 5.3 / 7.4 | 1.40 / 1.02 | 1.90 / 2.66 | 1.8 / 2.8 | 2.5 / 3.8 | - / 4.9 | 5.3 / 7.4 | P | P | F | FAIL |
| 1.5 | -2.0 | 1.50 / 1.50 | no / no | 7.4 / 5.4 | 1.37 / 1.07 | 2.81 / 3.30 | 1.6 / 2.2 | 2.1 / 2.9 | 3.0 / 3.7 | 7.4 / 5.4 | P | F | F | FAIL |
| 1.5 | -2.5 | 1.50 / 1.50 | no / no | 8.3 / 5.1 | 1.35 / 1.11 | 3.45 / 3.45 | 1.5 / 2.0 | 1.8 / 2.4 | 2.5 / 2.9 | 8.3 / 5.1 | P | F | P | FAIL |
| 1.5 | -3.0 | 1.50 / 1.50 | no / no | 5.5 / 0.3 | 1.35 / 0.50 | 3.45 / 3.45 | 1.3 / 1.7 | 1.7 / 2.0 | 2.0 / 2.5 | 5.5 / 0.3 | P | F | P | FAIL |
| 1.9 | -1.5 | 1.81 / 1.78 | no / no | 5.5 / 7.3 | 1.51 / 1.16 | 2.05 / 2.67 | 2.0 / 2.7 | 3.0 / 3.7 | 5.4 / 5.0 | 5.5 / 7.3 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.81 / 1.78 | no / no | 7.6 / 5.3 | 1.52 / 1.24 | 3.15 / 3.30 | 1.8 / 2.2 | 2.5 / 2.8 | 3.4 / 3.7 | 7.6 / 5.3 | P | F | P | FAIL |
| 1.9 | -2.5 | 1.81 / 1.78 | no / no | 8.1 / 5.0 | 1.55 / 1.28 | 3.45 / 3.45 | 1.6 / 1.9 | 2.1 / 2.4 | 2.7 / 3.0 | 8.1 / 5.0 | P | F | P | FAIL |
| 1.9 | -3.0 | 1.81 / 1.78 | no / no | 5.8 / 0.4 | 1.59 / 0.53 | 3.45 / 3.45 | 1.5 / 1.8 | 1.8 / 2.1 | 2.2 / 2.5 | 5.8 / 0.4 | P | F | P | FAIL |
| 2.3 | -1.5 | 2.09 / 2.07 | no / no | 5.4 / 7.1 | 1.73 / 1.34 | 2.03 / 2.63 | 2.0 / 2.7 | 3.0 / 3.6 | 5.8 / 5.1 | 5.4 / 7.1 | P | P | F | FAIL |
| 2.3 | -2.0 | 2.09 / 2.07 | no / no | 7.7 / 5.1 | 1.72 / 1.43 | 3.23 / 3.27 | 1.8 / 2.2 | 2.5 / 2.8 | 3.6 / 3.7 | 7.7 / 5.1 | P | F | P | FAIL |
| 2.3 | -2.5 | 2.09 / 2.07 | no / no | 8.3 / 5.3 | 1.77 / 1.50 | 3.45 / 3.45 | 1.6 / 2.0 | 2.2 / 2.4 | 2.8 / 3.0 | 8.3 / 5.3 | P | F | P | FAIL |
| 2.3 | -3.0 | 2.09 / 2.07 | no / no | 5.7 / 1.4 | 1.83 / 0.96 | 3.45 / 3.45 | 1.5 / 1.8 | 1.9 / 2.1 | 2.2 / 2.6 | 5.7 / 1.4 | P | F | P | FAIL |

## standard, 55 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | CONTACT 45mph / CONTACT 47mph | 0.1 / 0.3 | 0.01 / 0.01 | 1.85 / 0.51 | 6.1 / 8.1 | 7.9 / - | - / - | 0.1 / 0.3 | P | P | P | PASS |
| 1.5 | -2.0 | 1.49 / 1.49 | CONTACT 46mph / CONTACT 49mph | 0.3 / 0.1 | 0.01 / 0.00 | 1.67 / 0.55 | 4.4 / 6.5 | 6.2 / - | - / - | 0.3 / 0.1 | P | P | P | PASS |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 46mph / CONTACT 49mph | 0.0 / 0.3 | 0.00 / 0.01 | 2.18 / 0.62 | 3.4 / 5.4 | 4.9 / - | 6.3 / - | 0.0 / 0.3 | P | P | P | PASS |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 45mph / CONTACT 50mph | 0.0 / 0.0 | 0.00 / 0.00 | 2.32 / 0.71 | 3.0 / 4.7 | 4.1 / - | 5.7 / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.78 | CONTACT 46mph / CONTACT 49mph | 0.3 / 0.0 | 0.01 / 0.00 | 1.54 / 0.50 | 6.4 / 8.4 | 8.4 / - | - / - | 0.3 / 0.0 | P | F | P | FAIL |
| 1.9 | -2.0 | 1.80 / 1.78 | CONTACT 46mph / CONTACT 50mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.52 / 0.58 | 4.7 / 6.6 | 6.4 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.9 | -2.5 | 1.80 / 1.78 | CONTACT 46mph / CONTACT 51mph | 0.3 / 0.2 | 0.01 / 0.01 | 1.76 / 0.67 | 3.7 / 5.5 | 5.1 / - | - / - | 0.3 / 0.2 | P | P | P | PASS |
| 1.9 | -3.0 | 1.80 / 1.78 | CONTACT 46mph / CONTACT 51mph | 0.1 / 0.0 | 0.01 / 0.00 | 2.10 / 0.77 | 3.1 / 4.8 | 4.3 / - | 6.1 / - | 0.1 / 0.0 | P | P | P | PASS |
| 2.3 | -1.5 | 2.11 / 2.10 | CONTACT 46mph / CONTACT 50mph | 0.0 / 0.2 | 0.00 / 0.01 | 1.42 / 0.52 | 6.7 / 8.6 | 8.8 / - | - / - | 0.0 / 0.2 | P | P | P | PASS |
| 2.3 | -2.0 | 2.11 / 2.10 | CONTACT 47mph / CONTACT 51mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.45 / 0.64 | 4.9 / 6.7 | 6.7 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -2.5 | 2.11 / 2.10 | CONTACT 46mph / CONTACT 51mph | 0.1 / 0.2 | 0.01 / 0.01 | 1.81 / 0.75 | 4.1 / 5.6 | 5.3 / - | - / - | 0.1 / 0.2 | P | P | P | PASS |
| 2.3 | -3.0 | 2.11 / 2.10 | CONTACT 46mph / CONTACT 52mph | 0.0 / 0.0 | 0.00 / 0.00 | 2.13 / 0.89 | 3.5 / 4.8 | 4.6 / - | 6.3 / - | 0.0 / 0.0 | P | P | P | PASS |

## standard, 65 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | CONTACT 56mph / CONTACT 57mph | 0.1 / 0.1 | 0.01 / 0.00 | 0.72 / 0.50 | 7.2 / - | - / - | - / - | 0.1 / 0.1 | P | P | P | PASS |
| 1.5 | -2.0 | 1.49 / 1.49 | CONTACT 56mph / CONTACT 58mph | 0.0 / 0.0 | 0.00 / 0.00 | 0.99 / 0.50 | 5.1 / - | - / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 56mph / CONTACT 59mph | 0.0 / 0.3 | 0.00 / 0.01 | 1.24 / 0.50 | 4.1 / - | 5.8 / - | - / - | 0.0 / 0.3 | P | P | P | PASS |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 56mph / CONTACT 60mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.45 / 0.53 | 3.5 / 5.7 | 4.8 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.79 | CONTACT 56mph / CONTACT 58mph | 0.2 / 0.0 | 0.01 / 0.00 | 0.80 / 0.50 | 7.3 / - | - / - | - / - | 0.2 / 0.0 | P | P | P | PASS |
| 1.9 | -2.0 | 1.80 / 1.79 | CONTACT 57mph / CONTACT 59mph | 0.2 / 0.0 | 0.01 / 0.00 | 1.08 / 0.50 | 5.3 / - | 7.7 / - | - / - | 0.2 / 0.0 | P | P | P | PASS |
| 1.9 | -2.5 | 1.80 / 1.79 | CONTACT 56mph / CONTACT 60mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.34 / 0.54 | 4.2 / 6.6 | 5.8 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.9 | -3.0 | 1.80 / 1.79 | CONTACT 56mph / CONTACT 61mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.57 / 0.61 | 3.6 / 5.7 | 4.9 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -1.5 | 2.14 / 2.12 | CONTACT 57mph / CONTACT 59mph | 0.0 / 0.0 | 0.00 / 0.00 | 0.87 / 0.50 | 7.6 / - | - / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -2.0 | 2.14 / 2.12 | CONTACT 57mph / CONTACT 60mph | 0.2 / 0.0 | 0.01 / 0.00 | 1.16 / 0.53 | 5.5 / 7.9 | 7.7 / - | - / - | 0.2 / 0.0 | P | P | P | PASS |
| 2.3 | -2.5 | 2.14 / 2.12 | CONTACT 57mph / CONTACT 61mph | 0.3 / 0.0 | 0.01 / 0.00 | 1.43 / 0.63 | 4.5 / 6.5 | 6.0 / - | - / - | 0.3 / 0.0 | P | F | P | FAIL |
| 2.3 | -3.0 | 2.14 / 2.12 | CONTACT 57mph / CONTACT 62mph | 0.0 / 0.3 | 0.00 / 0.01 | 1.68 / 0.72 | 3.9 / 5.6 | 5.1 / - | - / - | 0.0 / 0.3 | P | P | P | PASS |

## aggressive, 45 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.47 / 1.47 | no / no | 5.6 / 10.4 | 1.42 / 1.11 | 1.92 / 2.42 | 1.3 / 2.2 | 2.0 / 2.8 | - / 5.2 | 5.6 / 10.4 | P | P | F | FAIL |
| 1.5 | -2.0 | 1.47 / 1.47 | no / no | 5.5 / 8.3 | 1.40 / 1.16 | 2.77 / 3.04 | 1.2 / 2.0 | 1.5 / 2.2 | 3.2 / 3.7 | 5.5 / 8.3 | P | P | P | PASS |
| 1.5 | -2.5 | 1.47 / 1.47 | no / no | 5.5 / 5.3 | 1.39 / 1.18 | 3.45 / 3.45 | 1.1 / 1.7 | 1.4 / 1.8 | 2.3 / 2.7 | 5.5 / 5.3 | P | P | P | PASS |
| 1.5 | -3.0 | 1.47 / 1.47 | no / no | 7.8 / 3.1 | 1.38 / 1.17 | 3.45 / 3.45 | 1.0 / 1.5 | 1.3 / 1.6 | 1.9 / 2.2 | 7.8 / 3.1 | P | F | P | FAIL |
| 1.9 | -1.5 | 1.73 / 1.71 | no / no | 5.7 / 10.3 | 1.63 / 1.23 | 1.92 / 2.45 | 1.2 / 2.1 | 1.9 / 2.8 | - / 5.2 | 5.7 / 10.3 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.73 / 1.71 | no / no | 5.6 / 8.3 | 1.62 / 1.30 | 2.77 / 3.08 | 1.2 / 1.8 | 1.5 / 2.2 | 3.2 / 3.4 | 5.6 / 8.3 | P | P | F | FAIL |
| 1.9 | -2.5 | 1.73 / 1.71 | no / no | 5.5 / 5.6 | 1.61 / 1.35 | 3.45 / 3.44 | 1.0 / 1.7 | 1.3 / 1.8 | 2.4 / 2.5 | 5.5 / 5.6 | P | P | P | PASS |
| 1.9 | -3.0 | 1.73 / 1.71 | no / no | 9.2 / 3.3 | 1.61 / 1.32 | 3.45 / 3.45 | 1.0 / 1.5 | 1.2 / 1.6 | 1.9 / 2.2 | 9.2 / 3.3 | P | F | P | FAIL |
| 2.3 | -1.5 | 2.03 / 2.00 | no / no | 5.7 / 10.3 | 1.88 / 1.46 | 1.90 / 2.33 | 1.2 / 2.0 | 1.8 / 2.7 | - / 5.1 | 5.7 / 10.3 | P | P | F | FAIL |
| 2.3 | -2.0 | 2.03 / 2.00 | no / no | 5.5 / 8.6 | 1.88 / 1.50 | 2.76 / 3.03 | 1.1 / 1.8 | 1.5 / 2.2 | 3.3 / 3.2 | 5.5 / 8.6 | P | P | P | PASS |
| 2.3 | -2.5 | 2.03 / 2.00 | no / no | 5.6 / 7.0 | 1.87 / 1.58 | 3.45 / 3.44 | 1.0 / 1.6 | 1.3 / 1.9 | 2.5 / 2.5 | 5.6 / 7.0 | P | P | P | PASS |
| 2.3 | -3.0 | 2.03 / 2.00 | no / no | 7.1 / 4.3 | 1.88 / 1.56 | 3.45 / 3.45 | 1.0 / 1.5 | 1.2 / 1.7 | 2.0 / 2.2 | 7.1 / 4.3 | P | F | P | FAIL |

## aggressive, 55 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.47 / 1.47 | no / no | 5.1 / 4.0 | 1.16 / 0.72 | 1.86 / 2.00 | 1.5 / 2.5 | 2.0 / 3.5 | - / - | 5.1 / 4.0 | P | F | P | FAIL |
| 1.5 | -2.0 | 1.47 / 1.47 | no / CONTACT 15mph | 5.5 / 0.3 | 1.31 / 0.04 | 2.88 / 2.96 | 1.2 / 2.0 | 1.6 / 2.6 | 4.7 / 6.8 | 5.5 / 0.3 | F | F | P | FAIL |
| 1.5 | -2.5 | 1.47 / 1.47 | no / CONTACT 16mph | 0.6 / 0.3 | 0.66 / 0.04 | 3.44 / 3.45 | 1.2 / 1.8 | 1.4 / 2.2 | 4.5 / 4.5 | 0.6 / 0.3 | F | F | P | FAIL |
| 1.5 | -3.0 | 1.47 / 1.47 | CONTACT 4mph / CONTACT 24mph | 0.3 / 0.1 | 0.22 / 0.01 | 3.45 / 3.45 | 1.0 / 1.7 | 1.3 / 1.8 | 3.4 / 3.9 | 0.3 / 0.1 | P | P | P | PASS |
| 1.9 | -1.5 | 1.75 / 1.73 | no / no | 4.6 / 5.5 | 1.26 / 0.98 | 1.87 / 2.71 | 1.3 / 2.5 | 2.0 / 3.6 | - / 6.3 | 4.6 / 5.5 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.75 / 1.73 | no / CONTACT 16mph | 9.9 / 0.2 | 1.51 / 0.04 | 2.80 / 3.23 | 1.2 / 2.2 | 1.5 / 2.8 | 5.3 / 6.5 | 9.9 / 0.2 | F | F | F | FAIL |
| 1.9 | -2.5 | 1.75 / 1.73 | no / CONTACT 14mph | 10.2 / 0.3 | 1.50 / 0.05 | 3.45 / 3.45 | 1.1 / 1.9 | 1.3 / 2.2 | 3.8 / 4.7 | 10.2 / 0.3 | F | F | P | FAIL |
| 1.9 | -3.0 | 1.75 / 1.73 | CONTACT 8mph / CONTACT 26mph | 0.1 / 0.0 | 0.04 / 0.00 | 3.45 / 3.45 | 1.0 / 1.7 | 1.2 / 1.9 | 3.7 / 4.3 | 0.1 / 0.0 | P | P | P | PASS |
| 2.3 | -1.5 | 2.05 / 2.03 | no / no | 5.2 / 4.4 | 1.59 / 0.77 | 1.92 / 2.44 | 1.2 / 2.8 | 1.9 / 3.8 | - / 7.6 | 5.2 / 4.4 | P | F | F | FAIL |
| 2.3 | -2.0 | 2.05 / 2.03 | no / CONTACT 29mph | 7.8 / 0.2 | 1.72 / 0.01 | 2.70 / 2.00 | 1.2 / 2.4 | 1.5 / 3.0 | 5.9 / - | 7.8 / 0.2 | F | F | P | FAIL |
| 2.3 | -2.5 | 2.05 / 2.03 | no / CONTACT 22mph | 5.5 / 0.1 | 1.78 / 0.01 | 3.45 / 3.45 | 1.0 / 2.1 | 1.3 / 2.5 | 4.1 / 5.2 | 5.5 / 0.1 | F | F | P | FAIL |
| 2.3 | -3.0 | 2.05 / 2.03 | CONTACT 5mph / CONTACT 27mph | 0.2 / 0.1 | 0.10 / 0.01 | 3.45 / 3.45 | 1.0 / 1.8 | 1.2 / 2.0 | 3.9 / 4.8 | 0.2 / 0.1 | P | P | P | PASS |

## aggressive, 65 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.47 / 1.46 | no / no | 4.0 / 4.0 | 0.79 / 0.48 | 1.97 / 2.00 | 1.5 / 2.4 | 2.0 / 4.1 | - / - | 4.0 / 4.0 | P | P | P | PASS |
| 1.5 | -2.0 | 1.47 / 1.46 | CONTACT 17mph / CONTACT 34mph | 0.2 / 0.3 | 0.03 / 0.02 | 2.00 / 2.00 | 1.3 / 2.0 | 1.7 / 3.1 | - / - | 0.2 / 0.3 | P | P | P | PASS |
| 1.5 | -2.5 | 1.47 / 1.46 | CONTACT 29mph / CONTACT 40mph | 0.0 / 0.2 | 0.00 / 0.01 | 2.32 / 2.00 | 1.2 / 1.8 | 1.5 / 2.5 | 7.8 / - | 0.0 / 0.2 | P | P | P | PASS |
| 1.5 | -3.0 | 1.47 / 1.46 | CONTACT 35mph / CONTACT 42mph | 0.3 / 0.2 | 0.02 / 0.01 | 2.88 / 2.51 | 1.1 / 1.7 | 1.3 / 2.0 | 5.8 / 6.2 | 0.3 / 0.2 | P | P | P | PASS |
| 1.9 | -1.5 | 1.76 / 1.75 | no / no | 4.1 / 0.5 | 0.91 / 0.10 | 2.00 / 2.00 | 1.4 / 2.6 | 2.0 / 4.3 | - / - | 4.1 / 1.4 | P | F | P | FAIL |
| 1.9 | -2.0 | 1.76 / 1.75 | CONTACT 16mph / CONTACT 37mph | 0.3 / 0.2 | 0.04 / 0.01 | 2.06 / 2.00 | 1.2 / 2.2 | 1.6 / 3.3 | 13.1 / - | 0.3 / 0.2 | P | P | P | PASS |
| 1.9 | -2.5 | 1.76 / 1.75 | CONTACT 27mph / CONTACT 42mph | 0.2 / 0.1 | 0.01 / 0.01 | 2.65 / 2.00 | 1.1 / 1.9 | 1.4 / 2.8 | 8.2 / - | 0.2 / 0.1 | P | P | P | PASS |
| 1.9 | -3.0 | 1.76 / 1.75 | CONTACT 33mph / CONTACT 42mph | 0.2 / 0.2 | 0.02 / 0.01 | 3.04 / 2.74 | 1.0 / 1.8 | 1.3 / 2.0 | 6.2 / 6.6 | 0.2 / 0.2 | P | P | P | PASS |
| 2.3 | -1.5 | 2.08 / 2.06 | no / CONTACT 26mph | 4.2 / 0.3 | 1.04 / 0.02 | 2.00 / 2.00 | 1.3 / 2.9 | 2.0 / 4.7 | - / - | 4.2 / 0.3 | F | F | P | FAIL |
| 2.3 | -2.0 | 2.08 / 2.06 | CONTACT 14mph / CONTACT 39mph | 0.1 / 0.2 | 0.02 / 0.01 | 2.67 / 2.00 | 1.2 / 2.5 | 1.5 / 3.7 | 12.4 / - | 0.1 / 0.2 | P | P | P | PASS |
| 2.3 | -2.5 | 2.08 / 2.06 | CONTACT 25mph / CONTACT 43mph | 0.0 / 0.0 | 0.00 / 0.00 | 3.02 / 2.07 | 1.2 / 2.2 | 1.4 / 3.0 | 8.2 / 8.1 | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -3.0 | 2.08 / 2.06 | CONTACT 31mph / CONTACT 42mph | 0.2 / 0.1 | 0.01 / 0.00 | 3.41 / 2.98 | 1.1 / 1.8 | 1.3 / 2.0 | 6.6 / 6.9 | 0.2 / 0.1 | P | P | P | PASS |

## standard (min accel -1.0), 45 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.50 / 1.50 | no / no | 8.2 / 9.2 | 1.45 / 1.05 | 1.82 / 2.60 | 1.4 / 2.5 | 2.3 / 3.6 | - / 5.1 | 8.2 / 9.2 | P | P | F | FAIL |
| 1.5 | -2.0 | 1.50 / 1.50 | no / no | 8.3 / 5.6 | 1.41 / 1.09 | 2.73 / 3.25 | 1.2 / 2.0 | 1.9 / 2.8 | 3.1 / 3.8 | 8.3 / 5.6 | P | F | F | FAIL |
| 1.5 | -2.5 | 1.50 / 1.50 | no / no | 9.1 / 5.3 | 1.38 / 1.14 | 3.45 / 3.45 | 1.2 / 1.8 | 1.7 / 2.3 | 2.5 / 2.9 | 9.1 / 5.3 | P | F | P | FAIL |
| 1.5 | -3.0 | 1.50 / 1.50 | no / no | 5.1 / 1.2 | 1.38 / 0.89 | 3.45 / 3.45 | 1.1 / 1.5 | 1.6 / 2.0 | 2.0 / 2.5 | 5.1 / 1.2 | P | F | P | FAIL |
| 1.9 | -1.5 | 1.81 / 1.78 | no / no | 8.0 / 8.9 | 1.66 / 1.20 | 1.87 / 2.60 | 1.3 / 2.4 | 2.5 / 3.5 | - / 5.2 | 8.0 / 8.9 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.81 / 1.78 | no / no | 8.3 / 5.5 | 1.59 / 1.27 | 3.03 / 3.24 | 1.2 / 2.0 | 2.2 / 2.7 | 3.7 / 3.7 | 8.3 / 5.5 | P | F | P | FAIL |
| 1.9 | -2.5 | 1.81 / 1.78 | no / no | 8.7 / 5.3 | 1.60 / 1.32 | 3.45 / 3.45 | 1.1 / 1.8 | 1.9 / 2.3 | 2.8 / 3.0 | 8.7 / 5.3 | P | F | P | FAIL |
| 1.9 | -3.0 | 1.81 / 1.78 | no / no | 5.5 / 1.1 | 1.63 / 0.87 | 3.45 / 3.45 | 1.0 / 1.5 | 1.7 / 2.0 | 2.2 / 2.5 | 5.5 / 1.1 | P | F | P | FAIL |
| 2.3 | -1.5 | 2.09 / 2.07 | no / no | 7.6 / 8.4 | 1.89 / 1.40 | 1.86 / 2.55 | 1.3 / 2.3 | 2.5 / 3.4 | - / 5.2 | 7.6 / 8.4 | P | P | F | FAIL |
| 2.3 | -2.0 | 2.09 / 2.07 | no / no | 8.2 / 5.3 | 1.81 / 1.47 | 3.11 / 3.20 | 1.2 / 2.0 | 2.2 / 2.8 | 3.8 / 3.8 | 8.2 / 5.3 | P | F | P | FAIL |
| 2.3 | -2.5 | 2.09 / 2.07 | no / no | 8.6 / 5.6 | 1.84 / 1.54 | 3.45 / 3.45 | 1.0 / 1.8 | 1.9 / 2.3 | 2.8 / 3.0 | 8.6 / 5.6 | P | F | P | FAIL |
| 2.3 | -3.0 | 2.09 / 2.07 | no / no | 8.2 / 2.2 | 1.88 / 1.19 | 3.45 / 3.45 | 1.0 / 1.5 | 1.7 / 2.0 | 2.2 / 2.6 | 8.2 / 2.2 | P | F | P | FAIL |

## standard (min accel -1.0), 55 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | CONTACT 21mph / CONTACT 31mph | 0.2 / 0.2 | 0.02 / 0.01 | 2.18 / 2.52 | 1.5 / 2.5 | 6.0 / 6.6 | 9.8 / 7.7 | 0.2 / 0.2 | P | P | F | FAIL |
| 1.5 | -2.0 | 1.49 / 1.49 | CONTACT 18mph / CONTACT 35mph | 0.3 / 0.2 | 0.03 / 0.02 | 3.21 / 3.41 | 1.3 / 2.0 | 5.0 / 5.8 | 5.8 / 6.3 | 0.3 / 0.2 | P | P | P | PASS |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 32mph / CONTACT 41mph | 0.2 / 0.0 | 0.02 / 0.00 | 3.45 / 3.32 | 1.2 / 1.8 | 4.5 / 5.4 | 5.2 / 5.8 | 0.2 / 0.0 | P | P | P | PASS |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 37mph / CONTACT 44mph | 0.0 / 0.2 | 0.00 / 0.01 | 3.43 / 2.98 | 1.1 / 1.7 | 3.8 / 5.2 | 4.8 / 5.5 | 0.0 / 0.2 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.78 | no / CONTACT 34mph | 0.5 / 0.1 | 0.33 / 0.01 | 2.20 / 2.67 | 1.4 / 2.6 | 6.2 / 7.2 | 8.7 / 7.9 | 0.5 / 0.1 | F | F | F | FAIL |
| 1.9 | -2.0 | 1.80 / 1.78 | CONTACT 19mph / CONTACT 40mph | 0.2 / 0.0 | 0.02 / 0.00 | 3.35 / 3.28 | 1.2 / 2.2 | 5.5 / 6.6 | 6.2 / 7.1 | 0.2 / 0.0 | P | P | P | PASS |
| 1.9 | -2.5 | 1.80 / 1.78 | CONTACT 32mph / CONTACT 44mph | 0.1 / 0.1 | 0.01 / 0.01 | 3.45 / 2.72 | 1.2 / 1.9 | 4.8 / 6.2 | 5.7 / 6.6 | 0.1 / 0.1 | P | P | P | PASS |
| 1.9 | -3.0 | 1.80 / 1.78 | CONTACT 38mph / CONTACT 46mph | 0.0 / 0.0 | 0.00 / 0.00 | 3.42 / 1.79 | 1.1 / 1.8 | 4.0 / 5.4 | 5.3 / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -1.5 | 2.11 / 2.10 | CONTACT 7mph / CONTACT 37mph | 0.3 / 0.3 | 0.10 / 0.02 | 2.33 / 3.22 | 1.3 / 3.0 | 6.8 / 7.9 | 8.8 / 8.6 | 0.3 / 0.3 | P | P | F | FAIL |
| 2.3 | -2.0 | 2.11 / 2.10 | CONTACT 18mph / CONTACT 44mph | 0.2 / 0.0 | 0.03 / 0.00 | 3.45 / 2.91 | 1.2 / 2.5 | 6.0 / 7.4 | 6.6 / 7.8 | 0.2 / 0.0 | P | P | P | PASS |
| 2.3 | -2.5 | 2.11 / 2.10 | CONTACT 33mph / CONTACT 46mph | 0.0 / 0.0 | 0.00 / 0.00 | 3.45 / 1.51 | 1.1 / 2.2 | 5.2 / 6.4 | 6.1 / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -3.0 | 2.11 / 2.10 | CONTACT 38mph / CONTACT 47mph | 0.0 / 0.0 | 0.00 / 0.00 | 3.43 / 1.56 | 1.0 / 1.8 | 4.4 / 5.8 | 5.7 / - | 0.0 / 0.0 | P | P | P | PASS |

## standard (min accel -1.0), 65 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | CONTACT 43mph / CONTACT 49mph | 0.1 / 0.0 | 0.01 / 0.00 | 1.34 / 1.00 | 1.7 / 2.5 | 9.9 / - | - / - | 0.1 / 0.0 | P | P | P | PASS |
| 1.5 | -2.0 | 1.49 / 1.49 | CONTACT 48mph / CONTACT 52mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.13 / 1.00 | 1.4 / 2.0 | 7.8 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 51mph / CONTACT 54mph | 0.2 / 0.0 | 0.01 / 0.00 | 1.32 / 1.00 | 1.2 / 1.8 | 5.7 / - | - / - | 0.2 / 0.0 | P | P | P | PASS |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 52mph / CONTACT 54mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.53 / 1.00 | 1.2 / 1.7 | 4.6 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.79 | CONTACT 42mph / CONTACT 51mph | 0.3 / 0.2 | 0.02 / 0.01 | 1.48 / 1.00 | 1.5 / 2.7 | 10.2 / - | - / - | 0.3 / 0.2 | P | P | P | PASS |
| 1.9 | -2.0 | 1.80 / 1.79 | CONTACT 48mph / CONTACT 53mph | 0.0 / 0.2 | 0.00 / 0.01 | 1.22 / 1.00 | 1.3 / 2.2 | 7.8 / - | - / - | 0.0 / 0.2 | P | P | P | PASS |
| 1.9 | -2.5 | 1.80 / 1.79 | CONTACT 51mph / CONTACT 55mph | 0.0 / 0.3 | 0.00 / 0.01 | 1.43 / 1.00 | 1.2 / 2.0 | 5.9 / - | - / - | 0.0 / 0.3 | P | P | P | PASS |
| 1.9 | -3.0 | 1.80 / 1.79 | CONTACT 52mph / CONTACT 55mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.65 / 1.00 | 1.1 / 1.8 | 4.8 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -1.5 | 2.14 / 2.12 | CONTACT 42mph / CONTACT 52mph | 0.2 / 0.3 | 0.01 / 0.01 | 1.60 / 1.00 | 1.5 / 3.1 | 10.6 / - | - / - | 0.2 / 0.3 | P | P | P | PASS |
| 2.3 | -2.0 | 2.14 / 2.12 | CONTACT 48mph / CONTACT 54mph | 0.0 / 0.3 | 0.00 / 0.01 | 1.30 / 1.00 | 1.3 / 2.5 | 8.1 / - | - / - | 0.0 / 0.3 | P | P | P | PASS |
| 2.3 | -2.5 | 2.14 / 2.12 | CONTACT 50mph / CONTACT 56mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.54 / 1.00 | 1.2 / 2.2 | 6.2 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |
| 2.3 | -3.0 | 2.14 / 2.12 | CONTACT 51mph / CONTACT 56mph | 0.0 / 0.0 | 0.00 / 0.00 | 1.77 / 1.00 | 1.2 / 1.9 | 4.9 / - | - / - | 0.0 / 0.0 | P | P | P | PASS |

## standard (min accel -2.0), 45 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.50 / 1.50 | no / no | 5.6 / 10.3 | 1.46 / 1.15 | 1.84 / 2.40 | 1.4 / 2.5 | 2.0 / 3.1 | - / 5.2 | 5.6 / 10.3 | P | P | F | FAIL |
| 1.5 | -2.0 | 1.50 / 1.50 | no / no | 5.7 / 8.3 | 1.45 / 1.22 | 2.63 / 2.96 | 1.2 / 2.0 | 1.6 / 2.3 | 3.2 / 3.8 | 5.7 / 8.3 | P | P | F | FAIL |
| 1.5 | -2.5 | 1.50 / 1.50 | no / no | 5.6 / 5.5 | 1.44 / 1.24 | 3.45 / 3.44 | 1.2 / 1.8 | 1.4 / 1.9 | 2.3 / 2.8 | 5.6 / 5.5 | P | P | P | PASS |
| 1.5 | -3.0 | 1.50 / 1.50 | no / no | 8.9 / 3.6 | 1.43 / 1.24 | 3.45 / 3.45 | 1.0 / 1.5 | 1.3 / 1.7 | 1.9 / 2.3 | 8.9 / 3.6 | P | F | P | FAIL |
| 1.9 | -1.5 | 1.81 / 1.78 | no / no | 5.6 / 10.2 | 1.72 / 1.30 | 1.84 / 2.43 | 1.3 / 2.4 | 2.0 / 3.1 | - / 5.3 | 5.6 / 10.2 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.81 / 1.78 | no / no | 5.6 / 8.3 | 1.72 / 1.37 | 2.72 / 3.03 | 1.2 / 2.0 | 1.5 / 2.4 | 3.7 / 3.6 | 5.6 / 8.3 | P | P | F | FAIL |
| 1.9 | -2.5 | 1.81 / 1.78 | no / no | 5.6 / 5.5 | 1.71 / 1.42 | 3.45 / 3.44 | 1.1 / 1.8 | 1.4 / 2.0 | 2.5 / 2.8 | 5.6 / 5.5 | P | P | P | PASS |
| 1.9 | -3.0 | 1.81 / 1.78 | no / no | 10.3 / 3.5 | 1.70 / 1.39 | 3.45 / 3.45 | 1.0 / 1.5 | 1.3 / 1.8 | 2.0 / 2.3 | 10.3 / 3.5 | P | F | P | FAIL |
| 2.3 | -1.5 | 2.09 / 2.07 | no / no | 5.7 / 10.2 | 1.97 / 1.52 | 1.81 / 2.35 | 1.3 / 2.3 | 1.9 / 3.1 | - / 5.2 | 5.7 / 10.2 | P | P | F | FAIL |
| 2.3 | -2.0 | 2.09 / 2.07 | no / no | 5.6 / 8.5 | 1.98 / 1.58 | 2.71 / 2.98 | 1.2 / 2.0 | 1.5 / 2.4 | 3.8 / 3.4 | 5.6 / 8.5 | P | P | P | PASS |
| 2.3 | -2.5 | 2.09 / 2.07 | no / no | 5.7 / 5.6 | 1.97 / 1.64 | 3.45 / 3.42 | 1.0 / 1.8 | 1.3 / 2.0 | 2.6 / 2.8 | 5.7 / 5.6 | P | P | P | PASS |
| 2.3 | -3.0 | 2.09 / 2.07 | no / no | 5.7 / 4.3 | 1.97 / 1.64 | 3.45 / 3.45 | 1.0 / 1.5 | 1.2 / 1.8 | 2.0 / 2.4 | 5.7 / 4.3 | P | F | P | FAIL |

## standard (min accel -2.0), 55 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | no / no | 8.3 / 4.0 | 1.28 / 0.75 | 1.95 / 2.00 | 1.5 / 2.5 | 2.0 / 3.8 | - / - | 8.3 / 4.0 | P | F | P | FAIL |
| 1.5 | -2.0 | 1.49 / 1.49 | no / no | 0.5 / 8.0 | 0.72 / 0.98 | 2.52 / 3.21 | 1.3 / 2.0 | 1.7 / 2.8 | 8.2 / 4.9 | 0.5 / 8.0 | P | P | F | FAIL |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 22mph / CONTACT 21mph | 0.2 / 0.2 | 0.02 / 0.02 | 2.00 / 3.45 | 1.2 / 1.8 | 1.5 / 2.2 | - / 5.1 | 0.2 / 0.2 | P | P | F | FAIL |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 19mph / CONTACT 23mph | 0.2 / 0.3 | 0.02 / 0.03 | 3.45 / 3.45 | 1.1 / 1.7 | 1.3 / 1.9 | 4.2 / 3.9 | 0.2 / 0.3 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.78 | no / no | 4.5 / 5.6 | 1.20 / 1.03 | 1.97 / 2.66 | 1.4 / 2.6 | 2.0 / 3.9 | - / 6.6 | 4.5 / 5.6 | P | P | F | FAIL |
| 1.9 | -2.0 | 1.80 / 1.78 | no / no | 3.2 / 7.1 | 1.43 / 1.03 | 2.46 / 3.41 | 1.2 / 2.2 | 1.6 / 3.0 | 6.7 / 5.3 | 3.2 / 7.1 | P | P | F | FAIL |
| 1.9 | -2.5 | 1.80 / 1.78 | no / CONTACT 16mph | 2.1 / 0.1 | 1.23 / 0.02 | 3.44 / 3.45 | 1.2 / 1.9 | 1.4 / 2.5 | 5.0 / 4.8 | 2.1 / 0.1 | F | F | P | FAIL |
| 1.9 | -3.0 | 1.80 / 1.78 | no / CONTACT 22mph | 2.7 / 0.1 | 1.32 / 0.01 | 3.45 / 3.45 | 1.1 / 1.8 | 1.3 / 2.0 | 3.6 / 4.2 | 2.7 / 0.1 | F | F | P | FAIL |
| 2.3 | -1.5 | 2.11 / 2.10 | no / no | 5.7 / 3.1 | 1.64 / 0.67 | 1.87 / 2.29 | 1.3 / 3.0 | 2.0 / 4.2 | - / 8.9 | 5.7 / 3.3 | P | F | F | FAIL |
| 2.3 | -2.0 | 2.11 / 2.10 | no / CONTACT 22mph | 5.6 / 0.2 | 1.88 / 0.02 | 2.92 / 3.15 | 1.2 / 2.5 | 1.5 / 3.2 | 5.6 / 7.2 | 5.6 / 0.2 | F | F | P | FAIL |
| 2.3 | -2.5 | 2.11 / 2.10 | no / CONTACT 17mph | 2.6 / 0.1 | 1.42 / 0.02 | 3.43 / 3.45 | 1.1 / 2.2 | 1.4 / 2.7 | 5.3 / 5.2 | 2.6 / 0.1 | F | F | P | FAIL |
| 2.3 | -3.0 | 2.11 / 2.10 | no / CONTACT 22mph | 4.2 / 0.2 | 1.68 / 0.02 | 3.45 / 3.45 | 1.0 / 1.8 | 1.3 / 2.0 | 3.8 / 4.6 | 4.2 / 0.2 | F | F | P | FAIL |

## standard (min accel -2.0), 65 mph

| hw (s) | a lead | hw@onset b/a | contact b/a | min gap b/a (m) | min tgap b/a (s) | peak decel b/a | t-0.5 b/a | t-1.0 b/a | t-2.0 b/a | final gap b/a (m) | C1 | C2 | C3 | result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.5 | -1.5 | 1.49 / 1.49 | no / no | 4.1 / 4.0 | 0.82 / 0.49 | 1.97 / 2.00 | 1.7 / 2.5 | 2.2 / 4.5 | - / - | 4.1 / 4.0 | P | P | P | PASS |
| 1.5 | -2.0 | 1.49 / 1.49 | CONTACT 15mph / CONTACT 33mph | 0.3 / 0.2 | 0.04 / 0.02 | 2.29 / 2.00 | 1.4 / 2.0 | 1.7 / 3.3 | 12.1 / - | 0.3 / 0.2 | P | P | P | PASS |
| 1.5 | -2.5 | 1.49 / 1.49 | CONTACT 26mph / CONTACT 39mph | 0.3 / 0.0 | 0.03 / 0.00 | 2.79 / 2.00 | 1.2 / 1.8 | 1.5 / 2.7 | 7.6 / - | 0.3 / 0.0 | P | F | P | FAIL |
| 1.5 | -3.0 | 1.49 / 1.49 | CONTACT 34mph / CONTACT 41mph | 0.2 / 0.1 | 0.01 / 0.01 | 2.83 / 2.00 | 1.2 / 1.7 | 1.3 / 2.0 | 6.0 / - | 0.2 / 0.1 | P | P | P | PASS |
| 1.9 | -1.5 | 1.80 / 1.79 | no / no | 4.2 / 0.3 | 0.96 / 0.08 | 2.00 / 2.00 | 1.5 / 2.7 | 2.0 / 4.8 | - / - | 4.2 / 1.0 | P | F | P | FAIL |
| 1.9 | -2.0 | 1.80 / 1.79 | CONTACT 13mph / CONTACT 35mph | 0.2 / 0.1 | 0.03 / 0.01 | 2.54 / 2.00 | 1.3 / 2.2 | 1.7 / 3.6 | 12.2 / - | 0.2 / 0.1 | P | P | P | PASS |
| 1.9 | -2.5 | 1.80 / 1.79 | CONTACT 25mph / CONTACT 41mph | 0.1 / 0.0 | 0.01 / 0.00 | 2.70 / 2.02 | 1.2 / 2.0 | 1.5 / 3.0 | 8.5 / 8.2 | 0.1 / 0.0 | P | P | P | PASS |
| 1.9 | -3.0 | 1.80 / 1.79 | CONTACT 31mph / CONTACT 41mph | 0.2 / 0.2 | 0.01 / 0.01 | 3.35 / 2.40 | 1.1 / 1.8 | 1.3 / 2.0 | 6.2 / 6.9 | 0.2 / 0.2 | P | P | P | PASS |
| 2.3 | -1.5 | 2.14 / 2.12 | no / CONTACT 24mph | 4.3 / 0.2 | 1.12 / 0.02 | 2.00 / 2.00 | 1.5 / 3.1 | 2.0 / 5.1 | - / - | 4.3 / 0.2 | F | F | P | FAIL |
| 2.3 | -2.0 | 2.14 / 2.12 | CONTACT 11mph / CONTACT 38mph | 0.1 / 0.3 | 0.02 / 0.02 | 2.80 / 2.00 | 1.3 / 2.5 | 1.7 / 3.9 | 12.6 / - | 0.1 / 0.3 | P | P | P | PASS |
| 2.3 | -2.5 | 2.14 / 2.12 | CONTACT 24mph / CONTACT 42mph | 0.2 / 0.0 | 0.02 / 0.00 | 2.84 / 2.17 | 1.2 / 2.2 | 1.5 / 3.2 | 9.2 / 8.2 | 0.2 / 0.0 | P | P | P | PASS |
| 2.3 | -3.0 | 2.14 / 2.12 | CONTACT 27mph / CONTACT 39mph | 0.0 / 0.3 | 0.00 / 0.02 | 3.45 / 3.21 | 1.1 / 1.9 | 1.3 / 2.0 | 6.4 / 6.8 | 0.0 / 0.3 | P | P | P | PASS |

## Overall summary

### PRIMARY (standard -0.5 + aggressive -2.0)

Scenarios: 72; failing at least one criterion: 32. Overall: FAIL.

- C1 (new contact) failures: 7
- C2 (min gap worse by > 0.25 m) failures: 24
- C3 (peak decel harder by > 0.3) failures: 11
- Contacts: before 36, after 43 (both: 36)
- delta min gap (after-before): mean -0.87 m, min -9.87, max +4.77; delta peak decel: mean -0.27, min -1.61, max +0.84

Worst 5 by (after - before) min gap:

| scenario | min gap before | after | delta | peak decel b/a | contact b/a |
|---|---|---|---|---|---|
| aggressive 55 mph hw 1.9 a -2.5 | 10.16 | 0.28 | -9.87 | 3.45 / 3.45 | no / CONTACT 14mph |
| aggressive 55 mph hw 1.9 a -2.0 | 9.85 | 0.25 | -9.61 | 2.80 / 3.23 | no / CONTACT 16mph |
| aggressive 55 mph hw 2.3 a -2.0 | 7.81 | 0.19 | -7.61 | 2.70 / 2.00 | no / CONTACT 29mph |
| aggressive 45 mph hw 1.9 a -3.0 | 9.24 | 3.32 | -5.92 | 3.45 / 3.45 | no / no |
| aggressive 55 mph hw 2.3 a -2.5 | 5.53 | 0.09 | -5.44 | 3.45 / 3.45 | no / CONTACT 22mph |

Worst 5 by (after - before) peak decel:

| scenario | peak before | after | delta | min gap b/a |
|---|---|---|---|---|
| aggressive 55 mph hw 1.9 a -1.5 | 1.87 | 2.71 | +0.84 | 4.62 / 5.54 |
| standard 45 mph hw 1.5 a -1.5 | 1.90 | 2.66 | +0.76 | 5.33 / 7.43 |
| standard 45 mph hw 1.9 a -1.5 | 2.05 | 2.67 | +0.62 | 5.48 / 7.28 |
| standard 45 mph hw 2.3 a -1.5 | 2.03 | 2.63 | +0.60 | 5.36 / 7.08 |
| aggressive 45 mph hw 1.9 a -1.5 | 1.92 | 2.45 | +0.53 | 5.67 / 10.27 |

Failing scenarios:

| scenario | failed | min gap b/a | peak decel b/a | contact b/a |
|---|---|---|---|---|
| standard 45 mph hw 1.5 a -1.5 | C3 | 5.33 / 7.43 | 1.90 / 2.66 | no / no |
| standard 45 mph hw 1.5 a -2.0 | C2,C3 | 7.36 / 5.36 | 2.81 / 3.30 | no / no |
| standard 45 mph hw 1.5 a -2.5 | C2 | 8.35 / 5.09 | 3.45 / 3.45 | no / no |
| standard 45 mph hw 1.5 a -3.0 | C2 | 5.54 / 0.32 | 3.45 / 3.45 | no / no |
| standard 45 mph hw 1.9 a -1.5 | C3 | 5.48 / 7.28 | 2.05 / 2.67 | no / no |
| standard 45 mph hw 1.9 a -2.0 | C2 | 7.63 / 5.29 | 3.15 / 3.30 | no / no |
| standard 45 mph hw 1.9 a -2.5 | C2 | 8.09 / 5.00 | 3.45 / 3.45 | no / no |
| standard 45 mph hw 1.9 a -3.0 | C2 | 5.77 / 0.39 | 3.45 / 3.45 | no / no |
| standard 45 mph hw 2.3 a -1.5 | C3 | 5.36 / 7.08 | 2.03 / 2.63 | no / no |
| standard 45 mph hw 2.3 a -2.0 | C2 | 7.66 / 5.13 | 3.23 / 3.27 | no / no |
| standard 45 mph hw 2.3 a -2.5 | C2 | 8.31 / 5.28 | 3.45 / 3.45 | no / no |
| standard 45 mph hw 2.3 a -3.0 | C2 | 5.68 / 1.43 | 3.45 / 3.45 | no / no |
| standard 55 mph hw 1.9 a -1.5 | C2 | 0.28 / 0.00 | 1.54 / 0.50 | CONTACT 46mph / CONTACT 49mph |
| standard 65 mph hw 2.3 a -2.5 | C2 | 0.28 / 0.00 | 1.43 / 0.63 | CONTACT 57mph / CONTACT 61mph |
| aggressive 45 mph hw 1.5 a -1.5 | C3 | 5.58 / 10.35 | 1.92 / 2.42 | no / no |
| aggressive 45 mph hw 1.5 a -3.0 | C2 | 7.79 / 3.06 | 3.45 / 3.45 | no / no |
| aggressive 45 mph hw 1.9 a -1.5 | C3 | 5.67 / 10.27 | 1.92 / 2.45 | no / no |
| aggressive 45 mph hw 1.9 a -2.0 | C3 | 5.58 / 8.30 | 2.77 / 3.08 | no / no |
| aggressive 45 mph hw 1.9 a -3.0 | C2 | 9.24 / 3.32 | 3.45 / 3.45 | no / no |
| aggressive 45 mph hw 2.3 a -1.5 | C3 | 5.70 / 10.29 | 1.90 / 2.33 | no / no |
| aggressive 45 mph hw 2.3 a -3.0 | C2 | 7.13 / 4.28 | 3.45 / 3.45 | no / no |
| aggressive 55 mph hw 1.5 a -1.5 | C2 | 5.08 / 3.98 | 1.86 / 2.00 | no / no |
| aggressive 55 mph hw 1.5 a -2.0 | C1,C2 | 5.52 / 0.29 | 2.88 / 2.96 | no / CONTACT 15mph |
| aggressive 55 mph hw 1.5 a -2.5 | C1,C2 | 0.61 / 0.27 | 3.44 / 3.45 | no / CONTACT 16mph |
| aggressive 55 mph hw 1.9 a -1.5 | C3 | 4.62 / 5.54 | 1.87 / 2.71 | no / no |
| aggressive 55 mph hw 1.9 a -2.0 | C1,C2,C3 | 9.85 / 0.25 | 2.80 / 3.23 | no / CONTACT 16mph |
| aggressive 55 mph hw 1.9 a -2.5 | C1,C2 | 10.16 / 0.28 | 3.45 / 3.45 | no / CONTACT 14mph |
| aggressive 55 mph hw 2.3 a -1.5 | C2,C3 | 5.23 / 4.39 | 1.92 / 2.44 | no / no |
| aggressive 55 mph hw 2.3 a -2.0 | C1,C2 | 7.81 / 0.19 | 2.70 / 2.00 | no / CONTACT 29mph |
| aggressive 55 mph hw 2.3 a -2.5 | C1,C2 | 5.53 / 0.09 | 3.45 / 3.45 | no / CONTACT 22mph |
| aggressive 65 mph hw 1.9 a -1.5 | C2 | 4.13 / 0.50 | 2.00 / 2.00 | no / no |
| aggressive 65 mph hw 2.3 a -1.5 | C1,C2 | 4.22 / 0.28 | 2.00 / 2.00 | no / CONTACT 26mph |

### SUPPLEMENTARY standard (min accel -1.0)

Scenarios: 36; failing at least one criterion: 15. Overall: FAIL.

- C1 (new contact) failures: 1
- C2 (min gap worse by > 0.25 m) failures: 10
- C3 (peak decel harder by > 0.3) failures: 7
- Contacts: before 23, after 24 (both: 23)
- delta min gap (after-before): mean -0.85 m, min -5.96, max +0.94; delta peak decel: mean -0.21, min -1.93, max +0.89

Worst 5 by (after - before) min gap:

| scenario | min gap before | after | delta | peak decel b/a | contact b/a |
|---|---|---|---|---|---|
| standard (min accel -1.0) 45 mph hw 2.3 a -3.0 | 8.16 | 2.20 | -5.96 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -3.0 | 5.52 | 1.15 | -4.38 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.5 a -3.0 | 5.07 | 1.21 | -3.86 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.5 a -2.5 | 9.09 | 5.33 | -3.76 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -2.5 | 8.68 | 5.27 | -3.41 | 3.45 / 3.45 | no / no |

Worst 5 by (after - before) peak decel:

| scenario | peak before | after | delta | min gap b/a |
|---|---|---|---|---|
| standard (min accel -1.0) 55 mph hw 2.3 a -1.5 | 2.33 | 3.22 | +0.89 | 0.29 / 0.29 |
| standard (min accel -1.0) 45 mph hw 1.5 a -1.5 | 1.82 | 2.60 | +0.78 | 8.22 / 9.16 |
| standard (min accel -1.0) 45 mph hw 1.9 a -1.5 | 1.87 | 2.60 | +0.73 | 7.96 / 8.86 |
| standard (min accel -1.0) 45 mph hw 2.3 a -1.5 | 1.86 | 2.55 | +0.69 | 7.56 / 8.38 |
| standard (min accel -1.0) 45 mph hw 1.5 a -2.0 | 2.73 | 3.25 | +0.52 | 8.31 / 5.63 |

Failing scenarios:

| scenario | failed | min gap b/a | peak decel b/a | contact b/a |
|---|---|---|---|---|
| standard (min accel -1.0) 45 mph hw 1.5 a -1.5 | C3 | 8.22 / 9.16 | 1.82 / 2.60 | no / no |
| standard (min accel -1.0) 45 mph hw 1.5 a -2.0 | C2,C3 | 8.31 / 5.63 | 2.73 / 3.25 | no / no |
| standard (min accel -1.0) 45 mph hw 1.5 a -2.5 | C2 | 9.09 / 5.33 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.5 a -3.0 | C2 | 5.07 / 1.21 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -1.5 | C3 | 7.96 / 8.86 | 1.87 / 2.60 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -2.0 | C2 | 8.29 / 5.49 | 3.03 / 3.24 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -2.5 | C2 | 8.68 / 5.27 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 1.9 a -3.0 | C2 | 5.52 / 1.15 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 2.3 a -1.5 | C3 | 7.56 / 8.38 | 1.86 / 2.55 | no / no |
| standard (min accel -1.0) 45 mph hw 2.3 a -2.0 | C2 | 8.22 / 5.29 | 3.11 / 3.20 | no / no |
| standard (min accel -1.0) 45 mph hw 2.3 a -2.5 | C2 | 8.62 / 5.57 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 45 mph hw 2.3 a -3.0 | C2 | 8.16 / 2.20 | 3.45 / 3.45 | no / no |
| standard (min accel -1.0) 55 mph hw 1.5 a -1.5 | C3 | 0.22 / 0.18 | 2.18 / 2.52 | CONTACT 21mph / CONTACT 31mph |
| standard (min accel -1.0) 55 mph hw 1.9 a -1.5 | C1,C2,C3 | 0.55 / 0.08 | 2.20 / 2.67 | no / CONTACT 34mph |
| standard (min accel -1.0) 55 mph hw 2.3 a -1.5 | C3 | 0.29 / 0.29 | 2.33 / 3.22 | CONTACT 7mph / CONTACT 37mph |

### SUPPLEMENTARY standard (min accel -2.0)

Scenarios: 36; failing at least one criterion: 22. Overall: FAIL.

- C1 (new contact) failures: 6
- C2 (min gap worse by > 0.25 m) failures: 13
- C3 (peak decel harder by > 0.3) failures: 10
- Contacts: before 11, after 17 (both: 11)
- delta min gap (after-before): mean -0.31 m, min -6.81, max +7.44; delta peak decel: mean +0.04, min -0.95, max +1.45

Worst 5 by (after - before) min gap:

| scenario | min gap before | after | delta | peak decel b/a | contact b/a |
|---|---|---|---|---|---|
| standard (min accel -2.0) 45 mph hw 1.9 a -3.0 | 10.30 | 3.49 | -6.81 | 3.45 / 3.45 | no / no |
| standard (min accel -2.0) 55 mph hw 2.3 a -2.0 | 5.65 | 0.23 | -5.41 | 2.92 / 3.15 | no / CONTACT 22mph |
| standard (min accel -2.0) 45 mph hw 1.5 a -3.0 | 8.88 | 3.56 | -5.32 | 3.45 / 3.45 | no / no |
| standard (min accel -2.0) 55 mph hw 1.5 a -1.5 | 8.29 | 4.00 | -4.29 | 1.95 / 2.00 | no / no |
| standard (min accel -2.0) 65 mph hw 2.3 a -1.5 | 4.31 | 0.19 | -4.12 | 2.00 / 2.00 | no / CONTACT 24mph |

Worst 5 by (after - before) peak decel:

| scenario | peak before | after | delta | min gap b/a |
|---|---|---|---|---|
| standard (min accel -2.0) 55 mph hw 1.5 a -2.5 | 2.00 | 3.45 | +1.45 | 0.21 / 0.19 |
| standard (min accel -2.0) 55 mph hw 1.9 a -2.0 | 2.46 | 3.41 | +0.95 | 3.21 / 7.09 |
| standard (min accel -2.0) 55 mph hw 1.9 a -1.5 | 1.97 | 2.66 | +0.69 | 4.52 / 5.65 |
| standard (min accel -2.0) 55 mph hw 1.5 a -2.0 | 2.52 | 3.21 | +0.68 | 0.51 / 7.96 |
| standard (min accel -2.0) 45 mph hw 1.9 a -1.5 | 1.84 | 2.43 | +0.60 | 5.65 / 10.22 |

Failing scenarios:

| scenario | failed | min gap b/a | peak decel b/a | contact b/a |
|---|---|---|---|---|
| standard (min accel -2.0) 45 mph hw 1.5 a -1.5 | C3 | 5.61 / 10.29 | 1.84 / 2.40 | no / no |
| standard (min accel -2.0) 45 mph hw 1.5 a -2.0 | C3 | 5.65 / 8.27 | 2.63 / 2.96 | no / no |
| standard (min accel -2.0) 45 mph hw 1.5 a -3.0 | C2 | 8.88 / 3.56 | 3.45 / 3.45 | no / no |
| standard (min accel -2.0) 45 mph hw 1.9 a -1.5 | C3 | 5.65 / 10.22 | 1.84 / 2.43 | no / no |
| standard (min accel -2.0) 45 mph hw 1.9 a -2.0 | C3 | 5.63 / 8.29 | 2.72 / 3.03 | no / no |
| standard (min accel -2.0) 45 mph hw 1.9 a -3.0 | C2 | 10.30 / 3.49 | 3.45 / 3.45 | no / no |
| standard (min accel -2.0) 45 mph hw 2.3 a -1.5 | C3 | 5.72 / 10.19 | 1.81 / 2.35 | no / no |
| standard (min accel -2.0) 45 mph hw 2.3 a -3.0 | C2 | 5.75 / 4.31 | 3.45 / 3.45 | no / no |
| standard (min accel -2.0) 55 mph hw 1.5 a -1.5 | C2 | 8.29 / 4.00 | 1.95 / 2.00 | no / no |
| standard (min accel -2.0) 55 mph hw 1.5 a -2.0 | C3 | 0.51 / 7.96 | 2.52 / 3.21 | no / no |
| standard (min accel -2.0) 55 mph hw 1.5 a -2.5 | C3 | 0.21 / 0.19 | 2.00 / 3.45 | CONTACT 22mph / CONTACT 21mph |
| standard (min accel -2.0) 55 mph hw 1.9 a -1.5 | C3 | 4.52 / 5.65 | 1.97 / 2.66 | no / no |
| standard (min accel -2.0) 55 mph hw 1.9 a -2.0 | C3 | 3.21 / 7.09 | 2.46 / 3.41 | no / no |
| standard (min accel -2.0) 55 mph hw 1.9 a -2.5 | C1,C2 | 2.15 / 0.13 | 3.44 / 3.45 | no / CONTACT 16mph |
| standard (min accel -2.0) 55 mph hw 1.9 a -3.0 | C1,C2 | 2.75 / 0.14 | 3.45 / 3.45 | no / CONTACT 22mph |
| standard (min accel -2.0) 55 mph hw 2.3 a -1.5 | C2,C3 | 5.75 / 3.13 | 1.87 / 2.29 | no / no |
| standard (min accel -2.0) 55 mph hw 2.3 a -2.0 | C1,C2 | 5.65 / 0.23 | 2.92 / 3.15 | no / CONTACT 22mph |
| standard (min accel -2.0) 55 mph hw 2.3 a -2.5 | C1,C2 | 2.55 / 0.13 | 3.43 / 3.45 | no / CONTACT 17mph |
| standard (min accel -2.0) 55 mph hw 2.3 a -3.0 | C1,C2 | 4.18 / 0.21 | 3.45 / 3.45 | no / CONTACT 22mph |
| standard (min accel -2.0) 65 mph hw 1.5 a -2.5 | C2 | 0.29 / 0.00 | 2.79 / 2.00 | CONTACT 26mph / CONTACT 39mph |
| standard (min accel -2.0) 65 mph hw 1.9 a -1.5 | C2 | 4.18 / 0.34 | 2.00 / 2.00 | no / no |
| standard (min accel -2.0) 65 mph hw 2.3 a -1.5 | C1,C2 | 4.31 / 0.19 | 2.00 / 2.00 | no / CONTACT 24mph |

### ALL CASES

Scenarios: 144; failing at least one criterion: 69. Overall: FAIL.

- C1 (new contact) failures: 14
- C2 (min gap worse by > 0.25 m) failures: 47
- C3 (peak decel harder by > 0.3) failures: 28
- Contacts: before 70, after 84 (both: 70)
- delta min gap (after-before): mean -0.72 m, min -9.87, max +7.44; delta peak decel: mean -0.18, min -1.93, max +1.45

## Control scenarios (lead -1.0 m/s2 for 2 s, then holds speed; headway 1.9 s)

Final gap/speed are at the end of the run (15 s after the lead stops slowing). Criteria applied for reference.

| case | mph | min gap b/a | min tgap b/a | peak decel b/a | t-0.5 b/a | t-1.0 b/a | final gap b/a | final v ego mph b/a | C1 C2 C3 |
|---|---|---|---|---|---|---|---|---|---|
| standard | 45 | 32.0 / 28.1 | 1.70 / 1.54 | 0.84 / 0.52 | 2.3 / 3.6 | - / - | 33.2 / 30.7 | 40.7 / 39.9 | PFP |
| standard | 55 | 34.5 / 30.8 | 1.52 / 1.36 | 0.50 / 0.37 | - / - | - / - | 35.5 / 31.6 | 49.8 / 49.6 | PFP |
| standard | 65 | 41.3 / 38.2 | 1.52 / 1.41 | 0.50 / 0.37 | - / - | - / - | 42.0 / 38.7 | 59.9 / 59.8 | PFP |
| aggressive | 45 | 30.8 / 26.8 | 1.65 / 1.47 | 1.09 / 0.72 | 1.5 / 2.5 | 2.7 / - | 30.8 / 28.3 | 41.1 / 40.3 | PFP |
| aggressive | 55 | 34.7 / 27.6 | 1.53 / 1.22 | 0.76 / 0.42 | 1.6 / - | - / - | 34.7 / 28.1 | 50.5 / 49.8 | PFP |
| aggressive | 65 | 41.0 / 34.6 | 1.51 / 1.27 | 0.75 / 0.40 | 1.8 / - | - / - | 41.0 / 34.9 | 60.6 / 60.0 | PFP |
| standard (min accel -1.0) | 45 | 32.8 / 28.6 | 1.74 / 1.56 | 0.97 / 0.60 | 1.7 / 2.9 | - / - | 34.1 / 31.1 | 41.0 / 40.0 | PFP |
| standard (min accel -1.0) | 55 | 36.2 / 30.8 | 1.60 / 1.36 | 0.75 / 0.37 | 1.8 / - | - / - | 37.7 / 31.6 | 49.8 / 49.6 | PFP |
| standard (min accel -1.0) | 65 | 42.8 / 38.2 | 1.58 / 1.41 | 0.73 / 0.37 | 1.9 / - | - / - | 43.5 / 38.7 | 60.1 / 59.8 | PFP |
| standard (min accel -2.0) | 45 | 32.8 / 28.6 | 1.74 / 1.56 | 0.98 / 0.60 | 1.7 / 2.9 | - / - | 34.1 / 31.1 | 41.0 / 40.0 | PFP |
| standard (min accel -2.0) | 55 | 36.2 / 30.8 | 1.60 / 1.36 | 0.75 / 0.37 | 1.8 / - | - / - | 37.7 / 31.6 | 49.8 / 49.6 | PFP |
| standard (min accel -2.0) | 65 | 42.8 / 38.2 | 1.58 / 1.41 | 0.73 / 0.37 | 1.9 / - | - / - | 43.5 / 38.7 | 60.1 / 59.8 | PFP |
