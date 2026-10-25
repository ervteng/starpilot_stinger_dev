# Steering torque limit events (Kia Stinger 2022, STEER_MAX 384, +3 up / -7 down per 10 ms)

Definitions, the torque chain and usage are in the docstring of analysis/steer_limit_events.py.

## 0. Signals and validation

- rlog routes: 7, 222 segments, 1307336 frames at 100 Hz; lateral-active (v >= 1.0 m/s): 3.01 h, of which steeringPressed 8.2%.
- requested = carControl.actuators.torque * 384: max |requested| in lateral-active frames = 384.0 (controller output is clipped at +-1, so a request above 384 is never seen); max |sent| = 384.
- carOutput.actuatorsOutput.torqueOutputCan exists and holds the post-limit apply_torque; carControl.actuators.torque is the pre-limit wish (the carOutput row k answers the carControl logged one row earlier; the script pairs them that way). Both are logged at 100 Hz.
- Limiter reconstruction (driver clip then rate clip from last sent, using carState.steeringTorque): reproduces the sent value exactly in 99.865% of 1083001 lateral-active frames.
  Mismatching frames: 1461.
- LKAS11 echo (can src 128, ActToi = 1) vs carOutput torqueOutputCan, 2 sample segments, 9347 frames: equal to the carOutput value k rows back (k = 0..5; the can echo is sampled at an arbitrary instant): 0: 40.7%, 1: 22.4%, 2: 16.5%, 3: 13.3%, 4: 11.3%, 5: 9.5%; equal to any of the last 20 carOutput rows: 68.1% (max |sent| 124). Weak check only: a can event batches several 10 ms frames and the parser keeps the last one, so exact timing alignment is not recoverable here; carOutput.torqueOutputCan is the value the controller passes to create_lkas11, so it is the sent torque.
- At any excess (> 0) the rate limiter is binding in 16.54% of base frames; RATE_UP3 class counts frames where the wish is more than one full step (3) beyond what the step allows; RATE_UP needs a deficit >= 40 units.

## 1. Summary by speed band (rlogs, 100 Hz)

Episodes use the material classes (CAP, RATE_UP, RATE_DOWN, DRIVER at deficit >= 40 units, SAT). Time shares are of lateral-active, unpressed time in the band (DRIVER: of lateral-active time including pressed). Classes overlap (SAT is nearly always inside RATE_UP/CAP).

| band mph | lat-active min | episodes | CAP | RATE_UP (>=40) | RATE_UP3 (>3) | RATE_DOWN (>=40) | RATE_DOWN3 (>7) | SAT | DRIVER (>=40) | DRIVER3 (any) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0-10 | 30.2 | 294 | 0.43% | 4.54% | 19.04% | 0.61% | 2.41% | 2.15% | 6.31% | 7.39% |
| 10-20 | 26.4 | 86 | 0.80% | 1.78% | 8.31% | 0.50% | 1.30% | 2.95% | 4.04% | 5.27% |
| 20-40 | 24.9 | 76 | 0.25% | 1.01% | 4.34% | 0.16% | 0.56% | 1.45% | 3.59% | 4.94% |
| 40+ | 84.2 | 299 | 0.01% | 0.81% | 6.14% | 0.04% | 0.34% | 0.16% | 2.32% | 3.79% |

Lateral-active minutes and episode counts per route:

| route | segments | lat-active min (unpressed) | episodes |
|---|---|---|---|
| 0000000d--5e0d1d244b | 9 | 5.5 | 24 |
| 0000000e--5ebf4636a1 | 26 | 22.9 | 153 |
| 00000015--2b757ccb00 | 51 | 41.7 | 112 |
| 0000001d--2ead59d2dd | 21 | 11.3 | 91 |
| 00000028--297af64c9c | 47 | 32.6 | 102 |
| 00000029--b3c2c13819 | 31 | 25.7 | 78 |
| 00000034--5f8605e608 | 37 | 26.1 | 195 |

### Episodes with an audible alert or driver intervention: what bound just before

Binding class = the classes seen in the episode's first 1 s (or, for the alert, the 1 s before the alert).

| band | episode had | binding in first 1 s | episodes |
|---|---|---|---|
| 0-10 | alert | rate-up | 2 |
| 0-10 | alert+press | driver-limit | 3 |
| 0-10 | alert+press | rate-up | 3 |
| 0-10 | alert+press | saturation-only | 1 |
| 0-10 | press | cap | 4 |
| 0-10 | press | driver-limit | 31 |
| 0-10 | press | rate+cap | 4 |
| 0-10 | press | rate-down | 3 |
| 0-10 | press | rate-up | 32 |
| 10-20 | alert | rate-up | 1 |
| 10-20 | alert+press | cap | 1 |
| 10-20 | alert+press | driver-limit | 1 |
| 10-20 | alert+press | rate+cap | 2 |
| 10-20 | alert+press | rate-up | 8 |
| 10-20 | press | driver-limit | 15 |
| 10-20 | press | rate+cap | 3 |
| 10-20 | press | rate-down | 1 |
| 10-20 | press | rate-up | 23 |
| 20-40 | alert | rate+cap | 1 |
| 20-40 | alert | rate-up | 1 |
| 20-40 | alert+press | driver-limit | 2 |
| 20-40 | alert+press | rate-up | 4 |
| 20-40 | press | driver-limit | 31 |
| 20-40 | press | rate-down | 3 |
| 20-40 | press | rate-up | 22 |
| 40+ | alert | rate-up | 2 |
| 40+ | alert+press | driver-limit | 4 |
| 40+ | alert+press | rate+cap | 1 |
| 40+ | alert+press | rate-up | 2 |
| 40+ | press | driver-limit | 154 |
| 40+ | press | rate-down | 6 |
| 40+ | press | rate-up | 80 |
| 40+ | press | saturation-only | 1 |

Total episodes 755; with an audible alert during or within 1 s after: 39; with steeringPressed during/within 1 s after: 445.

## 2. Tight low-speed turns: why does the car fall behind?

Frames: lateral active, unpressed, v < 20 mph, and (|desired lat accel| > 1.5 m/s^2 or desired radius < 50 m, i.e. |curvature| > 0.02). Behind = sign(desired)*(desired - actual) > 0.3 m/s^2. Categories are exclusive, in this priority: RATE_UP+SAT (wish at the +-1 ceiling and the ramp holding it back), RATE_UP only (wish below ceiling), CAP (sent >= 380 without a rate-up bind), SAT-only (|out| >= 0.98 but sent < 380 and no rate bind), RATE_DOWN, neither (controller not asking for more, and no limit binding).

Primary: RATE_UP = RATE_UP3 as specified (any wish more than 3 units beyond the step).

**0-20 mph**: tight-turn time 1.9 min; behind (> 0.3) 0.85 min = 44.4% of tight-turn time.

| binding | behind seconds | share of behind time | median |output| | median |wish - sent| (units) |
|---|---|---|---|---|
| RATE_UP+SAT | 21.6 | 42.6% | 1.00 | 98 |
| RATE_UP only | 0.2 | 0.3% | 0.94 | 122 |
| CAP | 14.8 | 29.1% | 1.00 | 0 |
| SAT-only | 14.1 | 27.9% | 1.00 | 81 |
| RATE_DOWN | 0.0 | 0.0% | - | - |
| neither | 0.1 | 0.1% | 0.97 | 150 |

Of the 'neither' behind frames, 6 frames (100.0%) had a RATE_UP/CAP/SAT flag within the previous 0.5 s (a tail of an episode that just ended).

**0-10 mph**: tight-turn time 0.8 min; behind (> 0.3) 0.23 min = 27.6% of tight-turn time.

| binding | behind seconds | share of behind time | median |output| | median |wish - sent| (units) |
|---|---|---|---|---|
| RATE_UP+SAT | 6.7 | 48.1% | 1.00 | 102 |
| RATE_UP only | 0.0 | 0.0% | - | - |
| CAP | 4.4 | 31.3% | 1.00 | 0 |
| SAT-only | 2.9 | 20.5% | 1.00 | 68 |
| RATE_DOWN | 0.0 | 0.0% | - | - |
| neither | 0.0 | 0.0% | - | - |

Of the 'neither' behind frames, 0 frames (0.0%) had a RATE_UP/CAP/SAT flag within the previous 0.5 s (a tail of an episode that just ended).

**10-20 mph**: tight-turn time 1.1 min; behind (> 0.3) 0.61 min = 57.6% of tight-turn time.

| binding | behind seconds | share of behind time | median |output| | median |wish - sent| (units) |
|---|---|---|---|---|
| RATE_UP+SAT | 14.9 | 40.5% | 1.00 | 96 |
| RATE_UP only | 0.2 | 0.5% | 0.94 | 122 |
| CAP | 10.4 | 28.3% | 1.00 | 0 |
| SAT-only | 11.3 | 30.6% | 1.00 | 85 |
| RATE_DOWN | 0.0 | 0.0% | - | - |
| neither | 0.1 | 0.2% | 0.97 | 150 |

Of the 'neither' behind frames, 6 frames (100.0%) had a RATE_UP/CAP/SAT flag within the previous 0.5 s (a tail of an episode that just ended).

Sensitivity: same 0-20 mph table counting RATE_UP only when the deficit is >= 40 units (~0.29 m/s^2); the small-deficit tail of each ramp moves into SAT-only:

**0-20 mph, material (>= 40 units) rate limit**: tight-turn time 1.9 min; behind (> 0.3) 0.85 min = 44.4% of tight-turn time.

| binding | behind seconds | share of behind time | median |output| | median |wish - sent| (units) |
|---|---|---|---|---|
| RATE_UP+SAT | 11.0 | 21.7% | 1.00 | 128 |
| RATE_UP only | 0.1 | 0.3% | 0.94 | 120 |
| CAP | 14.8 | 29.2% | 1.00 | 0 |
| SAT-only | 24.7 | 48.7% | 1.00 | 73 |
| RATE_DOWN | 0.0 | 0.0% | - | - |
| neither | 0.1 | 0.2% | 0.96 | 148 |

Of the 'neither' behind frames, 9 frames (100.0%) had a RATE_UP/CAP/SAT flag within the previous 0.5 s (a tail of an episode that just ended).

Behind frames in tight turns, controller |output| distribution (0-20 mph):

| |output| bin | share of behind frames |
|---|---|
| 0.00-0.25 | 0.0% |
| 0.25-0.50 | 0.0% |
| 0.50-0.75 | 0.0% |
| 0.75-0.90 | 0.1% |
| 0.90-0.98 | 0.3% |
| 0.98-1.01 | 99.5% |

| |sent| bin | share of behind frames |
|---|---|
| 0-96 | 0.2% |
| 96-192 | 7.2% |
| 192-288 | 26.3% |
| 288-380 | 37.0% |
| 380-385 | 29.2% |

## 3. Top 30 episodes by tracking error x duration (integral of |desired - actual| dt, m/s^2 * s)

Route time = 60 s * segment + time since the first event of the segment (approximate, a second or two of offset). v, desired/actual are episode mean speed and the |desired|/|actual| at the peak-desired frame. Radius = 1/max |desired curvature| (carControl.actuators.curvature). Class column lists classes in order of first occurrence with seconds active.

| route | time m:s | dur s | mph | des/act m/s^2 | peak err | err*dur | radius m | max angle/rate deg | classes | peak deficit | audible alert (offset s) | pressed s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 28:33.7 | 6.5 | 19 | 3.44/2.02 | 1.58 | 4.88 | 18 | 80/100 | DRIVER 2.37s > RATE_UP 0.98s > SAT 4.03s > CAP 1.72s > RATE_DOWN 0.43s | 227 | steerSaturated/warning@+2.7 | 1.3 |
| 0000001d--2ead59d2dd | 12:52.1 | 6.5 | 15 | 3.01/1.74 | 1.28 | 4.05 | 17 | 95/108 | RATE_UP 1.05s > SAT 4.33s > RATE_DOWN 0.22s > DRIVER 2.90s > CAP 0.11s | 237 | steerSaturated/warning@+1.6, steerSaturated/warning@+4.9 | 0.6 |
| 00000028--297af64c9c | 16:01.7 | 8.4 | 25 | 4.09/3.18 | 0.98 | 3.92 | 33 | 60/48 | DRIVER 4.99s > SAT 5.61s > RATE_UP 1.16s > RATE_DOWN 0.54s > CAP 0.73s | 208 | steerSaturated/warning@+2.7 | 1.3 |
| 0000000e--5ebf4636a1 | 50:36.3 | 8.0 | 33 | 3.63/2.80 | 1.08 | 3.12 | 66 | 35/60 | DRIVER 5.46s > RATE_UP 2.06s > SAT 3.04s > RATE_DOWN 0.26s > CAP 0.22s | 236 | - | 3.0 |
| 0000000d--5e0d1d244b | 30:14.0 | 6.0 | 13 | 1.91/1.34 | 1.02 | 3.02 | 17 | 90/140 | RATE_UP 1.34s > DRIVER 4.05s > SAT 4.90s > RATE_DOWN 0.06s > CAP 0.04s | 244 | steerSaturated/warning@+0.9, steerSaturated/warning@+5.6 | 0.4 |
| 00000028--297af64c9c | 54:47.9 | 6.9 | 12 | 2.33/1.92 | 1.14 | 2.59 | 9 | 206/168 | SAT 4.29s > CAP 0.68s > DRIVER 3.58s > RATE_DOWN 0.45s > RATE_UP 1.21s | 255 | steerSaturated/warning@+5.6 | 1.7 |
| 0000000e--5ebf4636a1 | 50:47.2 | 4.3 | 30 | 3.25/2.17 | 1.09 | 2.57 | 57 | 33/60 | DRIVER 3.47s > RATE_UP 0.92s > SAT 2.50s > RATE_DOWN 0.19s | 205 | steerSaturated/warning@+2.8 | 1.1 |
| 00000028--297af64c9c | 14:56.6 | 5.1 | 25 | 3.84/3.45 | 1.08 | 2.48 | 34 | 68/68 | DRIVER 2.97s > RATE_UP 1.04s > SAT 4.38s > RATE_DOWN 0.13s > CAP 0.97s | 217 | steerSaturated/warning@+0.4 | 0.2 |
| 0000000e--5ebf4636a1 | 8:28.5 | 3.4 | 20 | 2.99/2.21 | 1.16 | 2.45 | 25 | 94/88 | DRIVER 1.07s > RATE_UP 0.77s > SAT 2.83s > CAP 1.07s > RATE_DOWN 0.19s | 206 | steerSaturated/warning@+2.4 | 0.6 |
| 0000001d--2ead59d2dd | 24:23.2 | 5.8 | 15 | 3.11/1.79 | 1.31 | 2.38 | 15 | 119/140 | SAT 2.67s > CAP 2.06s > DRIVER 0.82s > RATE_DOWN 0.41s > RATE_UP 0.50s | 185 | steerSaturated/warning@+2.0 | 0.8 |
| 0000001d--2ead59d2dd | 14:51.0 | 4.1 | 40 | 3.36/2.25 | 1.41 | 2.37 | 96 | 27/100 | DRIVER 3.13s > RATE_UP 0.68s > SAT 2.20s > RATE_DOWN 0.23s | 215 | steerSaturated/warning@+3.2 | 1.4 |
| 00000028--297af64c9c | 66:14.3 | 6.7 | 15 | 2.85/2.55 | 1.00 | 2.22 | 15 | 144/280 | RATE_UP 1.75s > SAT 4.38s > RATE_DOWN 0.83s > CAP 1.97s > DRIVER 1.00s | 239 | steerSaturated/warning@+5.3 | 0.4 |
| 00000029--b3c2c13819 | 48:02.5 | 6.2 | 46 | 3.94/4.24 | 0.75 | 2.08 | 115 | 24/48 | SAT 3.37s > RATE_DOWN 0.19s > DRIVER 3.84s > RATE_UP 1.33s > CAP 0.06s | 209 | - | 0.6 |
| 00000034--5f8605e608 | 68:49.8 | 6.4 | 16 | 2.63/2.16 | 1.27 | 1.95 | 19 | 100/208 | RATE_UP 1.43s > SAT 4.01s > CAP 2.28s > DRIVER 1.73s > RATE_DOWN 0.16s | 321 | steerSaturated/warning@+1.9 | 1.1 |
| 00000029--b3c2c13819 | 54:54.0 | 2.8 | 8 | 2.44/1.53 | 0.99 | 1.77 | 8 | 199/228 | RATE_UP 1.70s > SAT 2.67s > DRIVER 0.05s > RATE_DOWN 0.53s > CAP 0.25s | 378 | steerSaturated/warning@+2.5 | near |
| 00000028--297af64c9c | 2:21.7 | 4.0 | 16 | 2.36/1.32 | 1.11 | 1.73 | 11 | 128/168 | RATE_UP 1.11s > SAT 2.13s > CAP 0.18s > DRIVER 2.72s > RATE_DOWN 0.17s | 258 | - | 1.3 |
| 0000000e--5ebf4636a1 | 8:24.1 | 2.4 | 17 | 3.64/2.92 | 0.87 | 1.62 | 15 | 140/100 | CAP 1.01s > SAT 2.30s > DRIVER 0.88s > RATE_UP 0.22s > RATE_DOWN 0.22s | 162 | - | - |
| 0000001d--2ead59d2dd | 30:09.3 | 7.4 | 9 | 1.25/0.65 | 0.63 | 1.59 | 9 | 290/288 | RATE_UP 1.14s > DRIVER 4.63s > SAT 3.46s > RATE_DOWN 0.49s > CAP 0.34s | 278 | steerSaturated/warning@+2.4 | 2.1 |
| 0000001d--2ead59d2dd | 30:31.9 | 10.5 | 6 | 1.25/0.67 | 0.59 | 1.51 | 7 | 238/140 | DRIVER 5.35s > SAT 4.61s > RATE_UP 0.47s > CAP 2.86s > RATE_DOWN 0.22s | 209 | steerSaturated/warning@+3.0 | 4.6 |
| 00000028--297af64c9c | 10:26.0 | 3.3 | 16 | 2.92/1.95 | 1.01 | 1.50 | 13 | 137/280 | CAP 1.14s > SAT 1.61s > DRIVER 0.20s > RATE_DOWN 0.58s > RATE_UP 0.38s | 85 | - | - |
| 00000028--297af64c9c | 2:05.4 | 3.7 | 18 | 1.61/0.89 | 0.81 | 1.47 | 45 | 35/68 | DRIVER 2.40s > RATE_DOWN 0.18s > RATE_UP 0.94s > SAT 1.99s | 225 | pedalPressed/userDisable@+2.1, steerSaturated/warning@+2.3 | 0.5 |
| 0000001d--2ead59d2dd | 26:18.4 | 2.5 | 11 | 2.29/1.44 | 0.86 | 1.38 | 16 | 107/88 | RATE_UP 0.29s > SAT 2.31s > DRIVER 0.86s > CAP 0.17s | 73 | steerSaturated/warning@+0.8 | near |
| 00000028--297af64c9c | 50:31.3 | 4.0 | 50 | 2.63/1.93 | 1.19 | 1.37 | 202 | 11/60 | DRIVER 2.06s > RATE_UP 0.98s > RATE_DOWN 0.05s | 229 | - | 1.4 |
| 0000000d--5e0d1d244b | 34:51.5 | 3.3 | 9 | 1.46/0.62 | 0.84 | 1.28 | 13 | 125/108 | RATE_UP 0.58s > SAT 2.85s > CAP 0.26s > DRIVER 1.56s > RATE_DOWN 0.38s | 204 | - | 0.0 |
| 00000034--5f8605e608 | 12:01.8 | 2.6 | 74 | 3.26/2.78 | 1.24 | 1.27 | 322 | 10/40 | DRIVER 1.35s > RATE_UP 1.01s > SAT 0.86s > RATE_DOWN 0.32s | 250 | - | 0.8 |
| 0000001d--2ead59d2dd | 14:35.7 | 3.7 | 36 | 2.73/1.88 | 0.99 | 1.25 | 96 | 20/28 | RATE_DOWN 0.02s > DRIVER 2.70s > RATE_UP 0.54s > SAT 0.93s | 185 | - | 1.9 |
| 0000000d--5e0d1d244b | 34:46.3 | 2.4 | 15 | 2.07/1.30 | 1.03 | 1.22 | 24 | 65/60 | RATE_UP 0.92s > SAT 1.68s > DRIVER 1.70s > RATE_DOWN 0.20s | 246 | - | 0.5 |
| 0000001d--2ead59d2dd | 30:02.9 | 4.7 | 8 | 1.65/1.11 | 0.75 | 1.16 | 5 | 328/260 | RATE_UP 2.28s > SAT 2.43s > CAP 0.49s > RATE_DOWN 0.93s > DRIVER 0.02s | 338 | - | - |
| 0000000e--5ebf4636a1 | 50:16.0 | 3.1 | 42 | 2.64/1.93 | 1.03 | 1.11 | 132 | 20/60 | DRIVER 2.02s > RATE_UP 1.28s > SAT 0.83s > RATE_DOWN 0.13s | 249 | - | 1.3 |
| 0000001d--2ead59d2dd | 26:22.2 | 2.1 | 16 | 2.86/2.39 | 0.94 | 1.10 | 16 | 139/220 | CAP 0.65s > SAT 0.91s > RATE_DOWN 0.52s > RATE_UP 0.84s | 268 | - | - |

`pressed s` = seconds of steeringPressed inside the episode; `near` = pressed only within 1 s after it.

### Top 30 episodes with no steeringPressed frame inside the episode (pure limit episodes, best for replay)

| route | time m:s | dur s | mph | des/act m/s^2 | peak err | err*dur | radius m | max angle/rate deg | classes | peak deficit | audible alert (offset s) | pressed s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 00000029--b3c2c13819 | 54:54.0 | 2.8 | 8 | 2.44/1.53 | 0.99 | 1.77 | 8 | 199/228 | RATE_UP 1.70s > SAT 2.67s > DRIVER 0.05s > RATE_DOWN 0.53s > CAP 0.25s | 378 | steerSaturated/warning@+2.5 | near |
| 0000000e--5ebf4636a1 | 8:24.1 | 2.4 | 17 | 3.64/2.92 | 0.87 | 1.62 | 15 | 140/100 | CAP 1.01s > SAT 2.30s > DRIVER 0.88s > RATE_UP 0.22s > RATE_DOWN 0.22s | 162 | - | - |
| 00000028--297af64c9c | 10:26.0 | 3.3 | 16 | 2.92/1.95 | 1.01 | 1.50 | 13 | 137/280 | CAP 1.14s > SAT 1.61s > DRIVER 0.20s > RATE_DOWN 0.58s > RATE_UP 0.38s | 85 | - | - |
| 0000001d--2ead59d2dd | 26:18.4 | 2.5 | 11 | 2.29/1.44 | 0.86 | 1.38 | 16 | 107/88 | RATE_UP 0.29s > SAT 2.31s > DRIVER 0.86s > CAP 0.17s | 73 | steerSaturated/warning@+0.8 | near |
| 0000001d--2ead59d2dd | 30:02.9 | 4.7 | 8 | 1.65/1.11 | 0.75 | 1.16 | 5 | 328/260 | RATE_UP 2.28s > SAT 2.43s > CAP 0.49s > RATE_DOWN 0.93s > DRIVER 0.02s | 338 | - | - |
| 0000001d--2ead59d2dd | 26:22.2 | 2.1 | 16 | 2.86/2.39 | 0.94 | 1.10 | 16 | 139/220 | CAP 0.65s > SAT 0.91s > RATE_DOWN 0.52s > RATE_UP 0.84s | 268 | - | - |
| 0000001d--2ead59d2dd | 16:18.6 | 3.0 | 62 | 1.02/0.65 | 0.54 | 0.81 | 796 | 4/48 | RATE_UP 1.31s > RATE_DOWN 0.11s | 109 | - | - |
| 0000001d--2ead59d2dd | 18:57.2 | 2.0 | 57 | 0.75/0.34 | 0.60 | 0.64 | 1097 | 4/40 | RATE_UP 0.99s > RATE_DOWN 0.04s | 82 | - | near |
| 0000001d--2ead59d2dd | 28:56.1 | 0.9 | 9 | 1.77/0.98 | 0.79 | 0.62 | 9 | 174/40 | SAT 0.90s > CAP 0.78s | 39 | - | near |
| 0000000e--5ebf4636a1 | 8:32.9 | 1.8 | 21 | 2.88/2.73 | 0.50 | 0.40 | 26 | 101/160 | CAP 0.23s > SAT 0.22s > RATE_DOWN 0.41s > RATE_UP 0.38s | 69 | - | - |
| 00000028--297af64c9c | 56:16.8 | 3.3 | 11 | 2.07/1.88 | 0.33 | 0.39 | 10 | 214/260 | RATE_UP 0.78s > SAT 0.84s > CAP 0.62s > RATE_DOWN 0.63s | 118 | - | - |
| 0000001d--2ead59d2dd | 16:56.3 | 1.0 | 57 | 0.67/0.02 | 0.68 | 0.39 | 1170 | 3/20 | RATE_UP 0.44s > RATE_DOWN 0.09s | 75 | - | - |
| 00000029--b3c2c13819 | 46:58.9 | 1.6 | 54 | 4.01/3.84 | 0.49 | 0.35 | 151 | 20/20 | DRIVER 0.38s > RATE_UP 0.47s > SAT 1.28s > CAP 0.69s | 173 | steerSaturated/warning@+0.4 | near |
| 0000001d--2ead59d2dd | 18:11.7 | 0.9 | 57 | 0.73/0.34 | 0.70 | 0.33 | 1038 | 3/28 | RATE_UP 0.45s > RATE_DOWN 0.01s | 87 | - | - |
| 00000029--b3c2c13819 | 48:44.1 | 0.9 | 22 | 2.19/1.96 | 0.52 | 0.31 | 44 | 52/68 | RATE_UP 0.31s > SAT 0.66s > CAP 0.30s > DRIVER 0.01s | 71 | steerSaturated/warning@+0.2 | - |
| 0000001d--2ead59d2dd | 18:29.6 | 0.9 | 58 | 0.71/0.49 | 0.51 | 0.28 | 1184 | 4/48 | RATE_UP 0.52s > RATE_DOWN 0.05s | 69 | - | - |
| 0000000d--5e0d1d244b | 36:05.3 | 1.6 | 3 | 0.79/0.57 | 0.25 | 0.26 | 5 | 380/108 | RATE_UP 0.82s > SAT 1.43s > CAP 0.10s > RATE_DOWN 0.56s | 382 | - | near |
| 0000001d--2ead59d2dd | 0:56.7 | 0.6 | 40 | 1.48/0.78 | 0.70 | 0.24 | 236 | 8/8 | RATE_UP 0.39s > RATE_DOWN 0.01s | 150 | - | - |
| 00000029--b3c2c13819 | 48:46.9 | 1.9 | 17 | 1.97/1.71 | 0.28 | 0.23 | 30 | 73/60 | DRIVER 0.59s > RATE_DOWN 0.04s > SAT 0.85s > RATE_UP 0.18s > CAP 0.02s | 102 | - | near |
| 0000001d--2ead59d2dd | 18:16.3 | 0.5 | 57 | 0.71/0.12 | 0.63 | 0.22 | 1059 | 2/28 | RATE_UP 0.48s | 88 | - | - |
| 0000000e--5ebf4636a1 | 40:38.8 | 6.1 | 7 | 0.09/0.06 | 0.10 | 0.20 | 165 | 18/108 | RATE_UP 2.77s > RATE_DOWN 0.32s | 117 | - | - |
| 0000001d--2ead59d2dd | 18:31.0 | 0.4 | 58 | 0.65/0.19 | 0.54 | 0.17 | 1265 | 2/20 | RATE_UP 0.43s | 76 | - | - |
| 0000000d--5e0d1d244b | 28:03.9 | 0.5 | 58 | 1.19/0.72 | 0.55 | 0.16 | 608 | 3/20 | RATE_UP 0.49s | 86 | - | - |
| 0000001d--2ead59d2dd | 12:39.0 | 0.4 | 20 | 1.01/0.51 | 0.50 | 0.15 | 93 | 20/40 | RATE_UP 0.36s | 150 | - | - |
| 00000028--297af64c9c | 2:01.8 | 1.0 | 15 | 0.46/0.24 | 0.22 | 0.15 | 108 | 18/28 | RATE_DOWN 0.03s > RATE_UP 0.68s | 148 | lkasEnable/warning@+0.0, pcmEnable/enable@+0.0, steerTempUnavailable/softDisable@+0.5 | - |
| 0000000e--5ebf4636a1 | 40:45.5 | 4.9 | 7 | 0.08/0.04 | 0.08 | 0.15 | 157 | 15/88 | RATE_DOWN 0.25s > RATE_UP 2.44s | 121 | - | - |
| 00000034--5f8605e608 | 24:26.8 | 0.6 | 66 | 0.58/0.76 | 0.39 | 0.14 | 1752 | 4/40 | RATE_UP 0.11s | 51 | - | near |
| 00000028--297af64c9c | 22:02.9 | 0.4 | 69 | 1.86/1.43 | 0.46 | 0.12 | 520 | 5/20 | RATE_UP 0.35s | 78 | - | - |
| 0000001d--2ead59d2dd | 2:33.8 | 0.3 | 28 | 0.93/1.00 | 0.56 | 0.11 | 246 | 19/80 | RATE_DOWN 0.18s > RATE_UP 0.12s | 72 | - | - |
| 0000001d--2ead59d2dd | 18:42.8 | 0.3 | 54 | 0.49/0.06 | 0.43 | 0.11 | 1345 | 1/28 | RATE_UP 0.33s | 57 | - | - |

### Every episode with a steering-related audible alert (steerSaturated, goat*Saturated, steerTempUnavailable*, steerTimeLimit)

22 episodes.

| route | time m:s | dur s | mph | des/act m/s^2 | peak err | err*dur | radius m | max angle/rate deg | classes | peak deficit | audible alert (offset s) | pressed s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 28:33.7 | 6.5 | 19 | 3.44/2.02 | 1.58 | 4.88 | 18 | 80/100 | DRIVER 2.37s > RATE_UP 0.98s > SAT 4.03s > CAP 1.72s > RATE_DOWN 0.43s | 227 | steerSaturated/warning@+2.7 | 1.3 |
| 0000000d--5e0d1d244b | 30:14.0 | 6.0 | 13 | 1.91/1.34 | 1.02 | 3.02 | 17 | 90/140 | RATE_UP 1.34s > DRIVER 4.05s > SAT 4.90s > RATE_DOWN 0.06s > CAP 0.04s | 244 | steerSaturated/warning@+0.9, steerSaturated/warning@+5.6 | 0.4 |
| 0000000e--5ebf4636a1 | 8:28.5 | 3.4 | 20 | 2.99/2.21 | 1.16 | 2.45 | 25 | 94/88 | DRIVER 1.07s > RATE_UP 0.77s > SAT 2.83s > CAP 1.07s > RATE_DOWN 0.19s | 206 | steerSaturated/warning@+2.4 | 0.6 |
| 0000000e--5ebf4636a1 | 50:47.2 | 4.3 | 30 | 3.25/2.17 | 1.09 | 2.57 | 57 | 33/60 | DRIVER 3.47s > RATE_UP 0.92s > SAT 2.50s > RATE_DOWN 0.19s | 205 | steerSaturated/warning@+2.8 | 1.1 |
| 0000001d--2ead59d2dd | 12:52.1 | 6.5 | 15 | 3.01/1.74 | 1.28 | 4.05 | 17 | 95/108 | RATE_UP 1.05s > SAT 4.33s > RATE_DOWN 0.22s > DRIVER 2.90s > CAP 0.11s | 237 | steerSaturated/warning@+1.6, steerSaturated/warning@+4.9 | 0.6 |
| 0000001d--2ead59d2dd | 14:51.0 | 4.1 | 40 | 3.36/2.25 | 1.41 | 2.37 | 96 | 27/100 | DRIVER 3.13s > RATE_UP 0.68s > SAT 2.20s > RATE_DOWN 0.23s | 215 | steerSaturated/warning@+3.2 | 1.4 |
| 0000001d--2ead59d2dd | 24:23.2 | 5.8 | 15 | 3.11/1.79 | 1.31 | 2.38 | 15 | 119/140 | SAT 2.67s > CAP 2.06s > DRIVER 0.82s > RATE_DOWN 0.41s > RATE_UP 0.50s | 185 | steerSaturated/warning@+2.0 | 0.8 |
| 0000001d--2ead59d2dd | 26:18.4 | 2.5 | 11 | 2.29/1.44 | 0.86 | 1.38 | 16 | 107/88 | RATE_UP 0.29s > SAT 2.31s > DRIVER 0.86s > CAP 0.17s | 73 | steerSaturated/warning@+0.8 | near |
| 0000001d--2ead59d2dd | 30:09.3 | 7.4 | 9 | 1.25/0.65 | 0.63 | 1.59 | 9 | 290/288 | RATE_UP 1.14s > DRIVER 4.63s > SAT 3.46s > RATE_DOWN 0.49s > CAP 0.34s | 278 | steerSaturated/warning@+2.4 | 2.1 |
| 0000001d--2ead59d2dd | 30:31.9 | 10.5 | 6 | 1.25/0.67 | 0.59 | 1.51 | 7 | 238/140 | DRIVER 5.35s > SAT 4.61s > RATE_UP 0.47s > CAP 2.86s > RATE_DOWN 0.22s | 209 | steerSaturated/warning@+3.0 | 4.6 |
| 00000028--297af64c9c | 0:10.7 | 2.4 | 12 | 0.29/0.03 | 0.26 | 0.17 | 150 | 6/28 | RATE_UP 0.48s > RATE_DOWN 0.01s > DRIVER 0.23s | 107 | lkasDisable/permanent@+3.2, pedalPressed/userDisable@+0.4, steerTempUnavailableSilent/warning@+0.6 | 1.2 |
| 00000028--297af64c9c | 2:01.8 | 1.0 | 15 | 0.46/0.24 | 0.22 | 0.15 | 108 | 18/28 | RATE_DOWN 0.03s > RATE_UP 0.68s | 148 | lkasEnable/warning@+0.0, pcmEnable/enable@+0.0, steerTempUnavailable/softDisable@+0.5 | - |
| 00000028--297af64c9c | 2:05.4 | 3.7 | 18 | 1.61/0.89 | 0.81 | 1.47 | 45 | 35/68 | DRIVER 2.40s > RATE_DOWN 0.18s > RATE_UP 0.94s > SAT 1.99s | 225 | pedalPressed/userDisable@+2.1, steerSaturated/warning@+2.3 | 0.5 |
| 00000028--297af64c9c | 14:55.9 | 0.1 | 25 | 1.90/1.62 | 0.28 | 0.03 | 69 | 32/20 | DRIVER 0.03s | 0 | steerSaturated/warning@+1.0 | - |
| 00000028--297af64c9c | 14:56.6 | 5.1 | 25 | 3.84/3.45 | 1.08 | 2.48 | 34 | 68/68 | DRIVER 2.97s > RATE_UP 1.04s > SAT 4.38s > RATE_DOWN 0.13s > CAP 0.97s | 217 | steerSaturated/warning@+0.4 | 0.2 |
| 00000028--297af64c9c | 16:01.7 | 8.4 | 25 | 4.09/3.18 | 0.98 | 3.92 | 33 | 60/48 | DRIVER 4.99s > SAT 5.61s > RATE_UP 1.16s > RATE_DOWN 0.54s > CAP 0.73s | 208 | steerSaturated/warning@+2.7 | 1.3 |
| 00000028--297af64c9c | 54:47.9 | 6.9 | 12 | 2.33/1.92 | 1.14 | 2.59 | 9 | 206/168 | SAT 4.29s > CAP 0.68s > DRIVER 3.58s > RATE_DOWN 0.45s > RATE_UP 1.21s | 255 | steerSaturated/warning@+5.6 | 1.7 |
| 00000028--297af64c9c | 66:14.3 | 6.7 | 15 | 2.85/2.55 | 1.00 | 2.22 | 15 | 144/280 | RATE_UP 1.75s > SAT 4.38s > RATE_DOWN 0.83s > CAP 1.97s > DRIVER 1.00s | 239 | steerSaturated/warning@+5.3 | 0.4 |
| 00000029--b3c2c13819 | 46:58.9 | 1.6 | 54 | 4.01/3.84 | 0.49 | 0.35 | 151 | 20/20 | DRIVER 0.38s > RATE_UP 0.47s > SAT 1.28s > CAP 0.69s | 173 | steerSaturated/warning@+0.4 | near |
| 00000029--b3c2c13819 | 48:44.1 | 0.9 | 22 | 2.19/1.96 | 0.52 | 0.31 | 44 | 52/68 | RATE_UP 0.31s > SAT 0.66s > CAP 0.30s > DRIVER 0.01s | 71 | steerSaturated/warning@+0.2 | - |
| 00000029--b3c2c13819 | 54:54.0 | 2.8 | 8 | 2.44/1.53 | 0.99 | 1.77 | 8 | 199/228 | RATE_UP 1.70s > SAT 2.67s > DRIVER 0.05s > RATE_DOWN 0.53s > CAP 0.25s | 378 | steerSaturated/warning@+2.5 | near |
| 00000034--5f8605e608 | 68:49.8 | 6.4 | 16 | 2.63/2.16 | 1.27 | 1.95 | 19 | 100/208 | RATE_UP 1.43s > SAT 4.01s > CAP 2.28s > DRIVER 1.73s > RATE_DOWN 0.16s | 321 | steerSaturated/warning@+1.9 | 1.1 |

## 4. Alerts

All selfdriveState / starpilotSelfdriveState alert rising edges and onroadEvents / starpilotOnroadEvents names across the rlog routes (a name listed under 'audible' has alertSound != none on the display message).

Audible alerts (selfdriveState `sd`, starpilotSelfdriveState `sp`):

| source | alertType | alertSound | count |
|---|---|---|---|
| sd | pedalPressed/userDisable | disengage | 30 |
| sd | pcmEnable/enable | engage | 23 |
| sd | steerSaturated/warning | promptRepeat | 21 |
| sd | buttonEnable/enable | engage | 10 |
| sp | lkasEnable/warning | engage | 9 |
| sd | driverDistracted1/permanent | preAlert | 7 |
| sd | steerTempUnavailableSilent/warning | prompt | 7 |
| sp | lkasDisable/permanent | disengage | 5 |
| sd | driverDistracted2/permanent | promptDistracted | 3 |
| sd | pedalPressed/noEntry | refuse | 2 |
| sp | trafficModeActive/warning | prompt | 2 |
| sd | buttonCancel/userDisable | disengage | 1 |
| sp | trafficModeInactive/warning | prompt | 1 |
| sd | steerTempUnavailable/softDisable | warningSoft | 1 |
| sd | laneChangeBlocked/warning | prompt | 1 |

Steering-related event names (onroadEvents `ev`, starpilotOnroadEvents `spev`):

| source | event | count |
|---|---|---|
| ev | steerSaturated | 35 |
| ev | steerTempUnavailableSilent | 8 |
| ev | steerTempUnavailable | 1 |

### What precedes each steerSaturated / steering alert (rlogs)

29 steering alerts. Binding class in the 1 s before, by speed band:

| band | binding class | alerts |
|---|---|---|
| 0-10 | cap | 1 |
| 0-10 | none | 6 |
| 0-10 | rate-up | 3 |
| 10-20 | cap | 2 |
| 10-20 | rate+cap | 4 |
| 10-20 | rate-up | 7 |
| 20-40 | rate-up | 4 |
| 20-40 | saturation-only | 1 |
| 40+ | rate-up | 1 |

steerSaturated only (21 alerts), binding class in the 1 s before: rate-up 13, rate+cap 4, cap 3, saturation-only 1. Median seconds in that 1 s window: RATE_UP3 0.61, CAP 0.00, SAT 1.01.

| route | time | alert | sound | mph | max |des| 1 s before | max |out| | max |sent| | binding | RATE_UP s | RATE_UP3 s | CAP s | SAT s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000000d--5e0d1d244b | 28:36.4 | steerSaturated/warning | promptRepeat | 20 | 3.44 | 1.00 | 384 | cap | 0.00 | 0.00 | 1.01 | 1.01 |
| 0000000d--5e0d1d244b | 30:14.9 | steerSaturated/warning | promptRepeat | 10 | 1.00 | 1.00 | 375 | rate-up | 0.03 | 0.64 | 0.00 | 0.61 |
| 0000000d--5e0d1d244b | 30:19.6 | steerSaturated/warning | promptRepeat | 17 | 1.82 | 1.00 | 371 | rate-up | 0.58 | 0.75 | 0.00 | 1.01 |
| 0000000e--5ebf4636a1 | 8:30.9 | steerSaturated/warning | promptRepeat | 19 | 2.77 | 1.00 | 384 | rate+cap | 0.21 | 0.62 | 0.21 | 1.01 |
| 0000000e--5ebf4636a1 | 50:50.0 | steerSaturated/warning | promptRepeat | 30 | 3.25 | 1.00 | 323 | rate-up | 0.14 | 0.48 | 0.00 | 1.01 |
| 0000001d--2ead59d2dd | 12:53.7 | steerSaturated/warning | promptRepeat | 14 | 1.25 | 1.00 | 336 | rate-up | 0.25 | 0.79 | 0.00 | 0.06 |
| 0000001d--2ead59d2dd | 12:57.1 | steerSaturated/warning | promptRepeat | 16 | 3.01 | 1.00 | 384 | rate+cap | 0.16 | 0.38 | 0.06 | 1.01 |
| 0000001d--2ead59d2dd | 14:54.1 | steerSaturated/warning | promptRepeat | 40 | 3.36 | 1.00 | 351 | rate-up | 0.27 | 0.61 | 0.00 | 1.01 |
| 0000001d--2ead59d2dd | 24:25.2 | steerSaturated/warning | promptRepeat | 14 | 2.72 | 1.00 | 384 | cap | 0.00 | 0.00 | 1.01 | 1.01 |
| 0000001d--2ead59d2dd | 26:10.7 | steerTempUnavailableSilent/warning | prompt | -0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |
| 0000001d--2ead59d2dd | 26:19.1 | steerSaturated/warning | promptRepeat | 10 | 0.98 | 1.00 | 362 | rate-up | 0.11 | 0.62 | 0.00 | 0.62 |
| 0000001d--2ead59d2dd | 30:11.7 | steerSaturated/warning | promptRepeat | 10 | 1.03 | 1.00 | 282 | rate-up | 0.01 | 0.40 | 0.00 | 1.01 |
| 0000001d--2ead59d2dd | 30:34.8 | steerSaturated/warning | promptRepeat | 6 | 1.09 | 1.00 | 384 | cap | 0.00 | 0.00 | 1.01 | 1.01 |
| 0000001d--2ead59d2dd | 30:59.6 | steerTempUnavailableSilent/warning | prompt | 0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |
| 0000001d--2ead59d2dd | 32:01.6 | steerTempUnavailableSilent/warning | prompt | 0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |
| 00000028--297af64c9c | 0:11.3 | steerTempUnavailableSilent/warning | prompt | 15 | 0.29 | 0.44 | 156 | rate-up | 0.24 | 0.57 | 0.00 | 0.00 |
| 00000028--297af64c9c | 2:02.3 | steerTempUnavailable/softDisable | warningSoft | 15 | 0.43 | 0.45 | 156 | rate-up | 0.29 | 0.49 | 0.00 | 0.00 |
| 00000028--297af64c9c | 2:07.7 | steerSaturated/warning | promptRepeat | 19 | 1.61 | 1.00 | 332 | rate-up | 0.57 | 0.69 | 0.00 | 1.01 |
| 00000028--297af64c9c | 14:56.9 | steerSaturated/warning | promptRepeat | 25 | 2.50 | 1.00 | 285 | rate-up | 0.03 | 0.41 | 0.00 | 0.03 |
| 00000028--297af64c9c | 16:04.3 | steerSaturated/warning | promptRepeat | 26 | 3.25 | 1.00 | 336 | saturation-only | 0.00 | 0.41 | 0.00 | 0.26 |
| 00000028--297af64c9c | 54:53.5 | steerSaturated/warning | promptRepeat | 13 | 1.50 | 1.00 | 370 | rate-up | 0.08 | 0.41 | 0.00 | 1.01 |
| 00000028--297af64c9c | 66:19.6 | steerSaturated/warning | promptRepeat | 14 | 1.97 | 1.00 | 384 | rate+cap | 0.46 | 0.81 | 0.07 | 1.01 |
| 00000028--297af64c9c | 76:37.2 | steerTempUnavailableSilent/warning | prompt | 0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |
| 00000029--b3c2c13819 | 46:59.4 | steerSaturated/warning | promptRepeat | 54 | 2.98 | 1.00 | 274 | rate-up | 0.16 | 0.39 | 0.00 | 0.15 |
| 00000029--b3c2c13819 | 48:44.3 | steerSaturated/warning | promptRepeat | 22 | 1.55 | 1.00 | 313 | rate-up | 0.21 | 0.68 | 0.00 | 0.02 |
| 00000029--b3c2c13819 | 54:56.6 | steerSaturated/warning | promptRepeat | 9 | 1.02 | 1.00 | 363 | rate-up | 0.94 | 1.01 | 0.00 | 1.01 |
| 00000029--b3c2c13819 | 58:44.9 | steerTempUnavailableSilent/warning | prompt | 0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |
| 00000034--5f8605e608 | 68:51.7 | steerSaturated/warning | promptRepeat | 16 | 2.09 | 1.00 | 384 | rate+cap | 0.43 | 0.61 | 0.27 | 0.68 |
| 00000034--5f8605e608 | 70:03.9 | steerTempUnavailableSilent/warning | prompt | 0 | 0.00 | 0.00 | 0 | none | 0.00 | 0.00 | 0.00 | 0.00 |

### The StarPilot alert predicate (PRED) vs actual alerts

PRED frames (lat active, no press in 2 s, |desired a| > 1, undershooting x1.2, |out| > 0.99 or saturated flag): 1798 (18 s). Alerts fired: 21 steerSaturated.

| band mph | PRED frames | seconds | also RATE_UP | also CAP | also SAT |
|---|---|---|---|---|---|
| 0-10 | 330 | 3.3 | 7.9% | 42.4% | 100.0% |
| 10-20 | 906 | 9.1 | 7.2% | 22.8% | 100.0% |
| 20-40 | 559 | 5.6 | 18.4% | 12.9% | 100.0% |
| 40+ | 3 | 0.0 | 100.0% | 0.0% | 100.0% |

## 5. qlog drives (10 Hz): cap, saturation, alerts

**Rate limiting cannot be measured at 10 Hz** (the +3/-7 per 10 ms step is invisible when only every 10th frame is logged); only the cap (sent >= 380), controller saturation and alerts are reported. The qlog drives have no modelV2, so PRED uses torqueState desired/actual lateral accel. qlog carOutput/carControl are decimated samples, not every frame.

| route | segs | lat-active min | % CAP (sent >= 380) | % SAT | PRED samples | audible alerts | steering alerts | steering alert types |
|---|---|---|---|---|---|---|---|---|
| 00000016--1fe83bd745 | 10 | 2.5 | 0.13% | 2.54% | 0 | 10 | 0 | - |
| 00000017--0c40138e36 | 9 | 4.4 | 0.57% | 2.53% | 23 | 9 | 1 | steerSaturated/warningx1 |
| 00000011--c7d29db317 | 28 | 14.4 | 0.19% | 1.22% | 27 | 13 | 2 | steerSaturated/warningx2 |
| 00000012--2aada6ce32 | 40 | 32.2 | 0.09% | 0.66% | 2 | 12 | 1 | steerSaturated/warningx1 |
| 00000013--6677db1144 | 11 | 2.1 | 0.23% | 3.37% | 0 | 8 | 0 | - |
| 00000014--62bceb20db | 37 | 26.8 | 0.32% | 1.67% | 32 | 18 | 5 | steerSaturated/warningx5 |

By speed band:

| route | band mph | lat-active min | % CAP | % SAT | max |sent| |
|---|---|---|---|---|---|
| 00000016--1fe83bd745 | 0-10 | 0.8 | 0.00% | 2.57% | 276 |
| 00000016--1fe83bd745 | 10-20 | 1.0 | 0.33% | 4.18% | 384 |
| 00000016--1fe83bd745 | 20-40 | 0.7 | 0.00% | 0.23% | 267 |
| 00000017--0c40138e36 | 0-10 | 1.8 | 0.09% | 2.90% | 381 |
| 00000017--0c40138e36 | 10-20 | 1.6 | 1.42% | 3.55% | 384 |
| 00000017--0c40138e36 | 20-40 | 0.9 | 0.00% | 0.00% | 169 |
| 00000011--c7d29db317 | 0-10 | 1.1 | 0.00% | 2.86% | 347 |
| 00000011--c7d29db317 | 10-20 | 1.5 | 1.38% | 4.02% | 384 |
| 00000011--c7d29db317 | 20-40 | 4.5 | 0.15% | 1.87% | 384 |
| 00000011--c7d29db317 | 40+ | 7.3 | 0.00% | 0.00% | 210 |
| 00000012--2aada6ce32 | 0-10 | 14.7 | 0.09% | 0.85% | 384 |
| 00000012--2aada6ce32 | 10-20 | 5.7 | 0.03% | 0.47% | 384 |
| 00000012--2aada6ce32 | 20-40 | 4.6 | 0.18% | 0.58% | 384 |
| 00000012--2aada6ce32 | 40+ | 7.3 | 0.07% | 0.46% | 384 |
| 00000013--6677db1144 | 0-10 | 1.4 | 0.24% | 4.50% | 384 |
| 00000013--6677db1144 | 10-20 | 0.7 | 0.23% | 1.37% | 384 |
| 00000013--6677db1144 | 20-40 | 0.0 | 0.00% | 0.00% | 27 |
| 00000014--62bceb20db | 0-10 | 1.1 | 1.65% | 6.92% | 384 |
| 00000014--62bceb20db | 10-20 | 1.8 | 3.35% | 9.69% | 384 |
| 00000014--62bceb20db | 20-40 | 5.2 | 0.10% | 3.69% | 384 |
| 00000014--62bceb20db | 40+ | 18.7 | 0.00% | 0.00% | 269 |

Steering alerts in qlog drives (time, type, mph, desired accel, |out|, |sent| at the nearest sample, tracking error):

| route | time | alert | sound | mph | max |des| 1 s before | max |out| | max |sent| | peak err |
|---|---|---|---|---|---|---|---|---|
| 00000017--0c40138e36 | 12:33.5 | steerSaturated/warning | promptRepeat | 12 | 1.12 | 1.00 | 282 | 0.55 |
| 00000011--c7d29db317 | 10:50.3 | steerSaturated/warning | promptRepeat | 27 | 2.53 | 1.00 | 309 | 0.55 |
| 00000011--c7d29db317 | 38:16.2 | steerSaturated/warning | promptRepeat | 10 | 1.47 | 1.00 | 346 | 0.89 |
| 00000012--2aada6ce32 | 52:31.9 | steerSaturated/warning | promptRepeat | 41 | 3.37 | 1.00 | 384 | 0.99 |
| 00000014--62bceb20db | 4:37.5 | steerSaturated/warning | promptRepeat | 20 | 1.99 | 1.00 | 367 | 0.43 |
| 00000014--62bceb20db | 4:56.3 | steerSaturated/warning | promptRepeat | 15 | 2.44 | 1.00 | 384 | 0.93 |
| 00000014--62bceb20db | 6:44.1 | steerSaturated/warning | promptRepeat | 25 | 2.88 | 0.99 | 373 | 0.48 |
| 00000014--62bceb20db | 54:30.8 | steerSaturated/warning | promptRepeat | 27 | 3.73 | 1.00 | 377 | 1.69 |
| 00000014--62bceb20db | 54:38.4 | steerSaturated/warning | promptRepeat | 25 | 3.04 | 1.00 | 356 | 1.45 |

Audible alerts in qlog drives:

| route | source | alertType | count |
|---|---|---|---|
| 00000011--c7d29db317 | sd | buttonEnable/enable | 5 |
| 00000011--c7d29db317 | sd | laneChangeBlocked/warning | 1 |
| 00000011--c7d29db317 | sd | pedalPressed/userDisable | 5 |
| 00000011--c7d29db317 | sd | steerSaturated/warning | 2 |
| 00000011--c7d29db317 | sp | lkasDisable/permanent | 1 |
| 00000011--c7d29db317 | sp | lkasEnable/warning | 1 |
| 00000012--2aada6ce32 | sd | buttonEnable/enable | 5 |
| 00000012--2aada6ce32 | sd | driverDistracted1/permanent | 1 |
| 00000012--2aada6ce32 | sd | pedalPressed/userDisable | 5 |
| 00000012--2aada6ce32 | sd | steerSaturated/warning | 1 |
| 00000012--2aada6ce32 | sp | lkasDisable/permanent | 1 |
| 00000012--2aada6ce32 | sp | lkasEnable/warning | 2 |
| 00000012--2aada6ce32 | sp | trafficModeActive/warning | 2 |
| 00000012--2aada6ce32 | sp | trafficModeInactive/warning | 2 |
| 00000013--6677db1144 | sd | buttonEnable/enable | 4 |
| 00000013--6677db1144 | sd | driverDistracted1/permanent | 1 |
| 00000013--6677db1144 | sd | pedalPressed/userDisable | 3 |
| 00000013--6677db1144 | sp | lkasEnable/warning | 1 |
| 00000014--62bceb20db | sd | buttonEnable/enable | 6 |
| 00000014--62bceb20db | sd | commIssueAvgFreq/softDisable | 1 |
| 00000014--62bceb20db | sd | pedalPressed/userDisable | 6 |
| 00000014--62bceb20db | sd | steerSaturated/warning | 5 |
| 00000014--62bceb20db | sp | lkasDisable/permanent | 1 |
| 00000014--62bceb20db | sp | lkasEnable/warning | 1 |
| 00000016--1fe83bd745 | sd | buttonEnable/enable | 5 |
| 00000016--1fe83bd745 | sd | pedalPressed/userDisable | 5 |
| 00000016--1fe83bd745 | sp | lkasDisable/permanent | 3 |
| 00000016--1fe83bd745 | sp | lkasEnable/warning | 2 |
| 00000016--1fe83bd745 | sp | trafficModeActive/warning | 1 |
| 00000016--1fe83bd745 | sp | trafficModeInactive/warning | 1 |
| 00000017--0c40138e36 | sd | buttonCancel/userDisable | 1 |
| 00000017--0c40138e36 | sd | buttonEnable/enable | 4 |
| 00000017--0c40138e36 | sd | pedalPressed/userDisable | 3 |
| 00000017--0c40138e36 | sd | steerSaturated/warning | 1 |
| 00000017--0c40138e36 | sp | lkasDisable/permanent | 1 |
| 00000017--0c40138e36 | sp | lkasEnable/warning | 1 |

