# Stinger steering torque limit

STEER_MAX 384 (KIA_STINGER_2022 default CarControllerParams); rlog routes: stock 0000001d--2ead59d2dd, 00000028--297af64c9c, 00000029--b3c2c13819, 00000034--5f8605e608; openpilot-lateral 0000000d--5e0d1d244b, 0000000e--5ebf4636a1, 00000015--2b757ccb00; qlog routes 0000000f--79cf51e7fa, 00000011--c7d29db317, 00000012--2aada6ce32, 00000013--6677db1144, 00000014--62bceb20db, 00000015--2b757ccb00, 00000016--1fe83bd745, 00000017--0c40138e36.

## 1. Factory LKAS request (camera LKAS11, can src 2)

| route | rlog segs | can samples | openpilot latActive | openpilot LKAS11 ActToi=1 | camera ActToi=1 samples | camera torque != 0 samples | max |camera StrToqReq| | camera LKAS11 frames seen on bus 0 |
|---|---|---|---|---|---|---|---|---|
| 0000001d--2ead59d2dd | 21 | 124110 | 61% | 61% | 0 | 0 | 0 | 1008 |
| 00000028--297af64c9c | 47 | 278848 | 77% | 77% | 0 | 0 | 0 | 1042 |
| 00000029--b3c2c13819 | 31 | 179836 | 94% | 94% | 0 | 0 | 0 | 1065 |
| 00000034--5f8605e608 | 37 | 218576 | 91% | 91% | 72 | 0 | 0 | 983 |

Factory-steering samples (camera ActToi = 1 or camera torque != 0): 72 of 801370.
Camera ActToi = 1 samples all have torque [0.0] and EPS ToiActive in them: 0% (openpilot latActive in them: 17%).

**No factory LKAS torque request exists in these logs**: the camera's LKAS11 frame is alive (counter increments) but its CR_Lkas_StrToqReq is 0 in every sample. In all four 'stock' drives openpilot lateral was active for most of the time and was the sender of the torque that the EPS acted on (EPS ToiActive = 1 follows openpilot's ActToi, not the camera's). The factory maximum (<384 or not) therefore cannot be measured from these logs; sections 2 and 3 are the proxy.

## 2. openpilot's request and the EPS response (rlog routes)

| route | segs | lat-active samples | lat-active min | p50 | p90 | p95 | p99 | p99.9 | max | % >= 376 (98%) | % = 384 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000001d--2ead59d2dd | 21 | 70527 | 12 | 32 | 138 | 218 | 384 | 384 | 384 | 1.46 | 1.37 |
| 00000028--297af64c9c | 47 | 198838 | 33 | 27 | 85 | 127 | 304 | 384 | 384 | 0.44 | 0.39 |
| 00000029--b3c2c13819 | 31 | 163903 | 27 | 22 | 78 | 114 | 264 | 384 | 384 | 0.23 | 0.21 |
| 00000034--5f8605e608 | 37 | 197337 | 33 | 20 | 93 | 124 | 198 | 384 | 384 | 0.14 | 0.13 |
| 0000000d--5e0d1d244b | 9 | 35759 | 6 | 29 | 117 | 214 | 344 | 384 | 384 | 0.58 | 0.55 |
| 0000000e--5ebf4636a1 | 26 | 122384 | 20 | 33 | 99 | 127 | 280 | 384 | 384 | 0.54 | 0.52 |
| 00000015--2b757ccb00 | 51 | 235797 | 39 | 26 | 70 | 88 | 131 | 182 | 274 | 0.00 | 0.00 |

Pooled: 1024545 lateral-active samples (2.85 h). Share at/within 2% of 384 (>= 376.3): 0.33% ; exactly 384: 0.31%. Sample counts: |req| >= 300: 7803, >= 350: 4900, >= 380: 3327.

Top sent values (|StrToqReq|) in lateral-active samples:

| |request| | samples |
|---|---|
| 384 | 3194 |
| 383 | 52 |
| 382 | 27 |
| 381 | 33 |
| 380 | 21 |
| 379 | 15 |
| 378 | 37 |
| 377 | 53 |

Max |request| by speed band:

| band mph | samples | max | p99.9 | p99 | % >= 376 |
|---|---|---|---|---|---|
| 0-20 | 308302 | 384 | 384 | 370 | 0.95 |
| 20-40 | 161545 | 384 | 384 | 286 | 0.27 |
| 40-60 | 233764 | 384 | 357 | 167 | 0.03 |
| 60+ | 320934 | 312 | 228 | 163 | 0.00 |

EPS response, steady-state samples (request range <= 24 over 0.3 s, ToiActive, no driver press, v > 2.0 m/s): 493265 samples. OutTq is signed with the request (so positive = EPS output follows request); units are DBC-scaled MDPS12 CR_Mdps_OutTq. Gain = median OutTq / bin-mean request.

| request bin | n | mean request | median OutTq | p10 .. p90 | gain (OutTq/request) | median mph |
|---|---|---|---|---|---|---|
| 0-32 | 347135 | 13 | 0.50 | -0.20 .. 2.60 | 0.0376 | 44 |
| 32-64 | 107587 | 44 | 1.80 | 0.90 .. 3.60 | 0.0410 | 32 |
| 64-96 | 25268 | 75 | 2.90 | 2.00 .. 4.30 | 0.0384 | 19 |
| 96-128 | 7846 | 109 | 4.00 | 3.10 .. 5.50 | 0.0368 | 31 |
| 128-160 | 1967 | 140 | 4.80 | 3.40 .. 6.40 | 0.0342 | 35 |
| 160-192 | 722 | 172 | 6.00 | 4.90 .. 7.60 | 0.0348 | 30 |
| 192-224 | 273 | 207 | 7.10 | 5.90 .. 8.70 | 0.0343 | 22 |
| 224-256 | 171 | 234 | 7.80 | 7.20 .. 9.50 | 0.0334 | 22 |
| 256-288 | 70 | 267 | 9.10 | 8.30 .. 11.21 | 0.0341 | 25 |
| 288-320 | 34 | 303 | 10.40 | 9.86 .. 11.11 | 0.0343 | 14 |
| 320-352 | 111 | 343 | 12.10 | 11.30 .. 12.90 | 0.0353 | 16 |
| 352-384 | 370 | 368 | 13.10 | 12.10 .. 15.00 | 0.0356 | 25 |
| 384-384 | 1711 | 384 | 14.10 | 13.00 .. 15.90 | 0.0367 | 15 |

- request 20-200: through-origin gain 0.0419, fit OutTq = 0.0322 * request +0.55 (n=236389)
- request 200-320: through-origin gain 0.0348, fit OutTq = 0.0353 * request -0.13 (n=472)
- request 320-384: through-origin gain 0.0370, fit OutTq = 0.0494 * request -4.73 (n=2192)

EPS faults at lateral-active samples (any route above):

| condition | CF_Mdps_ToiUnavail samples | CF_Mdps_ToiFlt samples | carState.steerFaultTemporary samples |
|---|---|---|---|
| all lateral-active | 0 / 1024545 | 0 / 1024545 | 0 / 1024545 |
| |request| >= 300 | 0 / 7803 | 0 / 7803 | 0 / 7803 |
| |request| >= 376 | 0 / 3465 | 0 / 3465 | 0 / 3465 |

Fault episodes starting while lateral active: 3.
Max |request| in the 0.5 s before each: 0, 0, 0 ... (median 0); speeds mph: 15, 15, 72

## 3. Torque controller saturation and achieved lateral accel (rlog 100 Hz, all rlog routes)

| route | segs | lat-active samples | lat-active min | % |out| >= 0.98 | % saturated flag | peg runs >= 0.5 s | peg run seconds |
|---|---|---|---|---|---|---|---|
| 0000001d--2ead59d2dd | 21 | 65277 | 11 | 4.43 | 0.00 | 19 | 27 |
| 00000028--297af64c9c | 47 | 186758 | 31 | 1.70 | 0.04 | 17 | 27 |
| 00000029--b3c2c13819 | 31 | 152047 | 25 | 0.70 | 0.13 | 8 | 8 |
| 00000034--5f8605e608 | 37 | 156304 | 26 | 0.60 | 0.00 | 4 | 6 |
| 0000000d--5e0d1d244b | 9 | 32698 | 5 | 4.55 | 0.26 | 7 | 13 |
| 0000000e--5ebf4636a1 | 26 | 117049 | 19 | 1.00 | 0.00 | 5 | 11 |
| 00000015--2b757ccb00 | 51 | 230715 | 38 | 0.03 | 0.00 | 0 | 0 |

Saturation by speed band (pooled, lateral-active, unpressed):

| band mph | samples | % pegged | % sat flag | |desired| p99 | |desired| max |
|---|---|---|---|---|---|
| 0-20 | 286224 | 2.75 | 0.00 | 1.95 | 3.64 |
| 20-40 | 149435 | 1.45 | 0.11 | 2.98 | 4.17 |
| 40-60 | 220659 | 0.28 | 0.09 | 1.77 | 4.06 |
| 60+ | 284530 | 0.05 | 0.00 | 1.56 | 3.35 |

Pegged samples by |desired| lateral accel (m/s^2):

| |desired| | samples | % pegged | median |actual| |
|---|---|---|---|
| 0-0.5 | 818797 | 0.2 | 0.05 |
| 0.5-1 | 85083 | 1.6 | 0.72 |
| 1-1.5 | 21891 | 6.7 | 1.17 |
| 1.5-2 | 6904 | 21.9 | 1.59 |
| 2-2.5 | 2937 | 38.7 | 1.83 |
| 2.5-3 | 2521 | 66.7 | 2.17 |
| 3-+ | 2715 | 84.5 | 2.79 |

Peg runs (|output| >= 0.98 same sign >= 0.5 s): actual lateral accel signed with the output, by mean speed of the run.

| band mph | runs | total s | median of run-median actual | p90 | max run-median | max instantaneous | median desired |
|---|---|---|---|---|---|---|---|
| 0-20 | 44 | 66 | 0.86 | 2.01 | 2.81 | 2.92 | 1.45 |
| 20-40 | 11 | 19 | 2.48 | 2.92 | 3.14 | 3.48 | 3.17 |
| 40-60 | 5 | 6 | 2.87 | 3.33 | 3.50 | 4.17 | 3.25 |
| 60+ | 0 | - | - | - | - | - | - |

- output sign +: 30 runs, median actual 0.93, p90 2.12, max 2.63 m/s^2
- output sign -: 30 runs, median actual 1.82, p90 2.93, max 3.50 m/s^2

Achieved lateral accel in the direction of the request (-actual * sign(output)) per unit normalized output; binned medians, lateral-active, unpressed, v > 8 m/s, |desired - actual| < 0.5:

| |output| bin | n | mean |output| | median achieved accel m/s^2 | accel per unit output |
|---|---|---|---|---|
| 0.1-0.2 | 128797 | 0.137 | 0.05 | 0.34 |
| 0.2-0.3 | 29111 | 0.242 | 0.59 | 2.46 |
| 0.3-0.4 | 10539 | 0.341 | 0.71 | 2.07 |
| 0.4-0.5 | 4543 | 0.443 | 0.88 | 1.99 |
| 0.5-0.6 | 2367 | 0.547 | 1.13 | 2.07 |
| 0.6-0.7 | 1419 | 0.645 | 1.38 | 2.15 |
| 0.7-0.8 | 863 | 0.750 | 1.71 | 2.27 |
| 0.8-0.9 | 436 | 0.839 | 2.00 | 2.38 |
| 0.9-0.98 | 571 | 0.940 | 2.76 | 2.94 |
| 0.98-1.01 | 886 | 0.999 | 2.78 | 2.79 |

Linear fit over 0.2 <= |output| <= 0.9 (n=49278, signed): actual = 2.257 * output -0.086  (slope = implied LAT_ACCEL_FACTOR incl. friction offset 2.26 vs 2.7499 configured, 2.789 user; zero-crossing at output +0.038). Extrapolated lateral accel at |output| = 1.0: 2.26; at 0.98: 2.21 m/s^2. Torque factor in params.toml MAX_LAT_ACCEL_MEASURED = 1.8497.

Pegged samples with tracking error < 0.5 (n=886) median achieved accel 2.78 m/s^2; all pegged samples (any error, v > 8 m/s, n=3531) median 2.27, p90 3.18, max 4.34.

## 3. Torque controller saturation and achieved lateral accel (qlog 10 Hz, qlog routes)

| route | segs | lat-active samples | lat-active min | % |out| >= 0.98 | % saturated flag | peg runs >= 0.5 s | peg run seconds |
|---|---|---|---|---|---|---|---|
| 0000000f--79cf51e7fa | 37 | 15350 | 26 | 1.41 | 0.00 | 11 | 15 |
| 00000011--c7d29db317 | 28 | 8400 | 14 | 1.24 | 0.17 | 7 | 8 |
| 00000012--2aada6ce32 | 40 | 16924 | 28 | 0.65 | 0.00 | 6 | 6 |
| 00000013--6677db1144 | 11 | 935 | 2 | 3.96 | 0.00 | 2 | 2 |
| 00000014--62bceb20db | 37 | 15872 | 27 | 1.61 | 0.20 | 16 | 20 |
| 00000015--2b757ccb00 | 46 | 21199 | 35 | 0.03 | 0.00 | 0 | 0 |
| 00000016--1fe83bd745 | 10 | 1327 | 2 | 2.64 | 0.00 | 2 | 2 |
| 00000017--0c40138e36 | 9 | 2378 | 4 | 2.82 | 0.00 | 5 | 6 |

Saturation by speed band (pooled, lateral-active, unpressed):

| band mph | samples | % pegged | % sat flag | |desired| p99 | |desired| max |
|---|---|---|---|---|---|
| 0-20 | 28619 | 1.86 | 0.00 | 1.40 | 3.17 |
| 20-40 | 15706 | 1.69 | 0.29 | 2.93 | 4.16 |
| 40-60 | 26027 | 0.13 | 0.00 | 1.68 | 3.61 |
| 60+ | 12033 | 0.00 | 0.00 | 1.23 | 2.25 |

Pegged samples by |desired| lateral accel (m/s^2):

| |desired| | samples | % pegged | median |actual| |
|---|---|---|---|
| 0-0.5 | 72777 | 0.2 | 0.05 |
| 0.5-1 | 6645 | 1.5 | 0.69 |
| 1-1.5 | 1526 | 4.9 | 1.13 |
| 1.5-2 | 705 | 17.2 | 1.60 |
| 2-2.5 | 375 | 37.3 | 1.87 |
| 2.5-3 | 191 | 58.1 | 2.15 |
| 3-+ | 166 | 84.3 | 2.49 |

Peg runs (|output| >= 0.98 same sign >= 0.5 s): actual lateral accel signed with the output, by mean speed of the run.

| band mph | runs | total s | median of run-median actual | p90 | max run-median | max instantaneous | median desired |
|---|---|---|---|---|---|---|---|
| 0-20 | 32 | 35 | 0.48 | 1.54 | 2.24 | 2.31 | 1.02 |
| 20-40 | 15 | 21 | 2.05 | 2.77 | 3.62 | 4.07 | 2.94 |
| 40-60 | 2 | 3 | 2.29 | 2.62 | 2.70 | 2.95 | 3.12 |
| 60+ | 0 | - | - | - | - | - | - |

- output sign +: 16 runs, median actual 0.67, p90 1.48, max 2.70 m/s^2
- output sign -: 33 runs, median actual 1.41, p90 2.28, max 3.62 m/s^2

Achieved lateral accel in the direction of the request (-actual * sign(output)) per unit normalized output; binned medians, lateral-active, unpressed, v > 8 m/s, |desired - actual| < 0.5:

| |output| bin | n | mean |output| | median achieved accel m/s^2 | accel per unit output |
|---|---|---|---|---|
| 0.1-0.2 | 12192 | 0.137 | 0.03 | 0.21 |
| 0.2-0.3 | 2223 | 0.238 | 0.54 | 2.27 |
| 0.3-0.4 | 661 | 0.343 | 0.76 | 2.22 |
| 0.4-0.5 | 296 | 0.442 | 0.82 | 1.85 |
| 0.5-0.6 | 165 | 0.549 | 1.10 | 2.00 |
| 0.6-0.7 | 131 | 0.645 | 1.64 | 2.54 |
| 0.7-0.8 | 65 | 0.750 | 1.79 | 2.38 |
| 0.8-0.9 | 62 | 0.849 | 1.85 | 2.17 |
| 0.9-0.98 | 37 | 0.939 | 2.18 | 2.32 |
| 0.98-1.01 | 58 | 0.998 | 2.48 | 2.49 |

Linear fit over 0.2 <= |output| <= 0.9 (n=3603, signed): actual = 2.354 * output -0.076  (slope = implied LAT_ACCEL_FACTOR incl. friction offset 2.35 vs 2.7499 configured, 2.789 user; zero-crossing at output +0.032). Extrapolated lateral accel at |output| = 1.0: 2.35; at 0.98: 2.31 m/s^2. Torque factor in params.toml MAX_LAT_ACCEL_MEASURED = 1.8497.

Pegged samples with tracking error < 0.5 (n=58) median achieved accel 2.48 m/s^2; all pegged samples (any error, v > 8 m/s, n=308) median 2.04, p90 3.06, max 4.07.

## 4. liveTorqueParameters per qlog route

Configured: params.toml KIA_STINGER LAT_ACCEL_FACTOR 2.7499, friction 0.1205 (MAX_LAT_ACCEL_MEASURED 1.8497); user settings 2.789 / 0.113.

| route | ltp samples | liveValid samples | factor first -> last valid | factor first-quarter -> last-quarter mean | factor slope /h | friction first -> last | friction q1 -> q4 mean | friction slope /h | last buckets / calPerc / useParams |
|---|---|---|---|---|---|---|---|---|---|
| 0000000f--79cf51e7fa | 8700 | 8700 (100%) | 2.750 -> 2.750 | 2.750 -> 2.750 | -0.000 | 0.120 -> 0.120 | 0.120 -> 0.120 | -0.000 | 11051 / cal 100% / use 1 |
| 00000011--c7d29db317 | 6583 | 6583 (100%) | 2.750 -> 2.750 | 2.750 -> 2.750 | 0.000 | 0.120 -> 0.120 | 0.120 -> 0.120 | 0.000 | 11056 / cal 100% / use 1 |
| 00000012--2aada6ce32 | 9450 | 9450 (100%) | 2.750 -> 2.750 | 2.750 -> 2.750 | 0.000 | 0.120 -> 0.120 | 0.120 -> 0.120 | 0.000 | 11059 / cal 100% / use 1 |
| 00000013--6677db1144 | 2390 | 2390 (100%) | 2.789 -> 2.789 | 2.789 -> 2.789 | 0.000 | 0.113 -> 0.113 | 0.113 -> 0.113 | -0.000 | 11059 / cal 100% / use 1 |
| 00000014--62bceb20db | 8653 | 8653 (100%) | 2.789 -> 2.789 | 2.789 -> 2.789 | 0.000 | 0.113 -> 0.113 | 0.113 -> 0.113 | -0.000 | 11075 / cal 100% / use 1 |
| 00000015--2b757ccb00 | 11016 | 11016 (100%) | 2.789 -> 2.789 | 2.789 -> 2.789 | 0.000 | 0.113 -> 0.113 | 0.113 -> 0.113 | -0.000 | 11089 / cal 100% / use 1 |
| 00000016--1fe83bd745 | 2174 | 2174 (100%) | 2.789 -> 2.789 | 2.789 -> 2.789 | 0.000 | 0.113 -> 0.113 | 0.113 -> 0.113 | 0.000 | 11089 / cal 100% / use 1 |
| 00000017--0c40138e36 | 2136 | 2136 (100%) | 2.789 -> 2.789 | 2.789 -> 2.789 | -0.000 | 0.113 -> 0.113 | 0.113 -> 0.113 | -0.000 | 11089 / cal 100% / use 1 |

Range of the filtered values and the raw per-sample estimates (min .. median .. max) per route:

| route | filtered factor min .. max | raw factor | filtered friction min .. max | raw friction | bucket points first -> last |
|---|---|---|---|---|---|
| 0000000f--79cf51e7fa | 2.7499 .. 2.7499 | 2.830 .. 2.975 .. 3.079 | 0.1205 .. 0.1205 | 0.078 .. 0.087 .. 0.095 | 10980 -> 11051 |
| 00000011--c7d29db317 | 2.7499 .. 2.7499 | 2.807 .. 2.885 .. 3.021 | 0.1205 .. 0.1205 | 0.082 .. 0.100 .. 0.111 | 11051 -> 11056 |
| 00000012--2aada6ce32 | 2.7499 .. 2.7499 | 2.706 .. 2.786 .. 2.949 | 0.1205 .. 0.1205 | 0.084 .. 0.091 .. 0.106 | 11056 -> 11059 |
| 00000013--6677db1144 | 2.7890 .. 2.7890 | 2.617 .. 2.778 .. 2.900 | 0.1130 .. 0.1130 | 0.081 .. 0.091 .. 0.104 | 11059 -> 11059 |
| 00000014--62bceb20db | 2.7890 .. 2.7890 | 2.713 .. 2.795 .. 2.897 | 0.1130 .. 0.1130 | 0.080 .. 0.090 .. 0.099 | 11059 -> 11075 |
| 00000015--2b757ccb00 | 2.7890 .. 2.7890 | 2.720 .. 2.826 .. 2.960 | 0.1130 .. 0.1130 | 0.087 .. 0.098 .. 0.106 | 11075 -> 11089 |
| 00000016--1fe83bd745 | 2.7890 .. 2.7890 | 2.667 .. 2.782 .. 2.904 | 0.1130 .. 0.1130 | 0.089 .. 0.098 .. 0.109 | 11089 -> 11089 |
| 00000017--0c40138e36 | 2.7890 .. 2.7890 | 2.728 .. 2.782 .. 2.846 | 0.1130 .. 0.1130 | 0.092 .. 0.098 .. 0.104 | 11089 -> 11089 |

