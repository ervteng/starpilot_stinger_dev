# Highway over-braking vs kinematically needed decel

Produced by `highway_overbrake.py` (defaults: --min-speed 20 m/s = 45 mph, --threshold -0.8, --gap-model headway, --al-source leadk, --al-smooth 1.0) on qlogs, groups B-today (00000013, 00000014, 00000015), B-earlier (00000011), A (0000000f, 0000000d, 0000000e), C (00000012) and stock (0000001d, 00000028, 00000029, 00000034; stock braking is carState aEgo, engaged = cruiseState.enabled). Episodes come from `highway_brakes.py`. Needed decel = lead decel plus (v - vL)^2 / (2 (d - D)), with D = tFollow*v + 6 m (see script docstring); over-braking = peak command (stock: aEgo) beyond needed by > 0.5 m/s^2 and ratio > 1.5, measured before any lead switch. "need (aL=0)" columns assume the lead holds speed. The "cut-in rule" entry in the lead-switch column means only the highway_brakes onset heuristic fired (no dRel jump seen).

### B-today: per-episode (peak cmd before any lead switch)

| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak cmd | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 00000014--62bceb20db | 734 | 55 | 41.1 | 3.1 | -2.71 | 1.35 | 3.73 | 469.59 | 3.49 | 0.0 | 467.04 | 0.0 | - | - | - | - | lead braking |
| 00000014--62bceb20db | 872 | 61 | 59.2 | 0.6 | -0.53 | 1.28 | 0.65 | 1.15 | 0.94 | 0.8 | 0.04 | 18.9 | - | - | - | - | other |
| 00000014--62bceb20db | 904 | 58 | 40.8 | 3.0 | -0.30 | 1.32 | 6.33 | 6.33 | 0.93 | 0.1 | 5.98 | 0.2 | +0.3s (earlier -0.3s) | - | - | - | cut-in / new lead |
| 00000014--62bceb20db | 947 | 52 | 31.1 | 3.6 | 0.49 | 1.38 | 6.61 | 6.61 | 1.05 | 0.2 | 6.61 | 0.2 | cut-in rule (earlier -0.5s) | Y | - | - | cut-in / new lead |
| 00000014--62bceb20db | 982 | 58 | 30.2 | 3.2 | 0.55 | 1.32 | 5.16 | 5.16 | 1.00 | 0.2 | 5.16 | 0.2 | cut-in rule (earlier +0.0s) | Y | - | - | cut-in / new lead |
| 00000014--62bceb20db | 1027 | 66 | 54.8 | 1.4 | -0.48 | 1.24 | 0.55 | 0.55 | 1.52 | 2.7 | 0.08 | 19.2 | cut-in rule | - | - | **OVER** | cut-in / new lead |
| 00000014--62bceb20db | 1042 | 58 | 45.3 | 3.6 | -1.99 | 1.31 | 2.90 | 8.83 | 1.93 | 0.2 | 7.79 | 0.2 | cut-in rule | - | - | - | cut-in / new lead |
| 00000014--62bceb20db | 1076 | 58 | 9.0 | 0.9 | 0.48 | 1.32 | 0.39 | 0.39 | 1.41 | 3.6 | 0.39 | 3.6 | cut-in rule (earlier -0.2s) | Y | - | **OVER** | cut-in / new lead |
| 00000014--62bceb20db | 1116 | 57 | 40.0 | 1.4 | -0.49 | 1.33 | 4.99 | 4.99 | 0.87 | 0.2 | 4.39 | 0.2 | - | - | - | - | other |
| 00000014--62bceb20db | 1235 | 55 | 45.0 | 0.9 | -0.58 | 1.35 | 0.59 | 0.61 | 1.49 | 2.4 | 0.06 | 23.4 | - | - | - | **OVER** | other |
| 00000014--62bceb20db | 1249 | 52 | 21.6 | 0.5 | 0.18 | 1.38 | 0.13 | 0.13 | 0.99 | 7.6 | 0.13 | 7.6 | cut-in rule (earlier +0.0s) | Y | - | **OVER** | cut-in / new lead |
| 00000014--62bceb20db | 1469 | 61 | 47.2 | 3.1 | -0.94 | 1.29 | 1.70 | 3.49 | 1.51 | 0.4 | 2.84 | 0.5 | +2.8s (earlier +0.0s) | - | - | - | closing on slower lead |
| 00000014--62bceb20db | 1638 | 59 | 60.6 | 4.1 | -0.74 | 1.31 | 1.15 | 1.84 | 1.49 | 0.8 | 0.80 | 1.9 | cut-in rule | - | Y | - | cut-in / new lead |
| 00000015--2b757ccb00 | 391 | 69 | 65.0 | 3.0 | -1.01 | 1.20 | 1.72 | 1.72 | 1.21 | 0.7 | 0.21 | 5.7 | cut-in rule | - | - | - | cut-in / new lead |
| 00000015--2b757ccb00 | 399 | 60 | 65.2 | 6.7 | -1.03 | 1.30 | 1.25 | 1.66 | 1.72 | 1.0 | 0.93 | 1.8 | +1.3s (earlier -0.7s) | - | - | - | closing on slower lead |
| 00000015--2b757ccb00 | 452 | 49 | 37.2 | 1.5 | -1.21 | 1.41 | 8.22 | 8.22 | 1.48 | 0.2 | 7.50 | 0.2 | - | - | - | - | other |
| 00000015--2b757ccb00 | 476 | 51 | 43.8 | 1.3 | -0.24 | 1.39 | 0.68 | 0.68 | 0.83 | 1.2 | 0.13 | 6.2 | cut-in rule | - | - | - | cut-in / new lead |
| 00000015--2b757ccb00 | 525 | 50 | 39.8 | 3.1 | -1.98 | 1.40 | 3.38 | 33.00 | 2.36 | 0.1 | 31.06 | 0.1 | cut-in rule | - | - | - | cut-in / new lead |
| 00000015--2b757ccb00 | 575 | 52 | 40.3 | 2.0 | -1.03 | 1.38 | 2.11 | 2.11 | 1.24 | 0.6 | 1.24 | 1.0 | cut-in rule | - | - | - | cut-in / new lead +curve |
| 00000015--2b757ccb00 | 714 | 61 | 44.5 | 3.0 | -1.94 | 1.29 | 3.12 | 3.12 | 1.38 | 0.4 | 1.42 | 1.0 | cut-in rule | - | - | - | cut-in / new lead |
| 00000015--2b757ccb00 | 837 | 64 | 35.9 | 2.2 | -2.20 | 1.25 | 3.94 | 5.24 | 1.21 | 0.2 | 3.24 | 0.4 | +0.5s | Y | - | - | lead braking |
| 00000015--2b757ccb00 | 840 | 57 | 72.9 | -0.9 | 2.56 | 1.33 | -0.00 | -0.00 | 1.18 | 23.5 | -0.00 | 23.5 | - (earlier -1.2s) | - | - | **OVER** | other |
| 00000015--2b757ccb00 | 860 | 53 | 46.0 | 4.9 | -3.25 | 1.37 | 3.61 | 5.02 | 3.49 | 0.7 | 2.25 | 1.5 | cut-in rule | - | - | - | cut-in / new lead |
| 00000015--2b757ccb00 | 921 | 51 | 38.5 | 1.3 | -0.71 | 1.39 | 1.72 | 2.04 | 1.27 | 0.6 | 1.32 | 1.0 | - | - | - | - | other |

### B-earlier: per-episode (peak cmd before any lead switch)

| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak cmd | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 00000011--c7d29db317 | 389 | 53 | 30.4 | 1.1 | -0.89 | 1.36 | 1.45 | 1.45 | 0.90 | 0.6 | 0.59 | 1.5 | - | Y | - | - | other |
| 00000011--c7d29db317 | 399 | 58 | 54.2 | 2.0 | -1.24 | 1.32 | 2.50 | 2.50 | 1.29 | 0.5 | 0.15 | 8.8 | cut-in rule | - | - | - | cut-in / new lead |
| 00000011--c7d29db317 | 557 | 51 | 39.7 | 1.6 | -1.34 | 1.39 | 1.77 | 3.10 | 2.31 | 0.7 | 1.16 | 2.0 | - | - | - | - | lead braking +curve |
| 00000011--c7d29db317 | 698 | 61 | 94.1 | -1.2 | 0.03 | 1.29 | -0.00 | -0.00 | 0.96 | 19.2 | -0.00 | 19.2 | +0.3s (earlier -0.5s) | - | - | **OVER** | other |
| 00000011--c7d29db317 | 833 | 61 | 27.5 | 4.2 | 0.67 | 1.28 | 8.70 | 8.70 | 1.15 | 0.1 | 8.70 | 0.1 | cut-in rule (earlier +0.0s) | Y | - | - | cut-in / new lead |
| 00000011--c7d29db317 | 886 | 50 | 46.0 | 2.5 | -0.74 | 1.40 | 1.09 | 1.80 | 1.35 | 0.8 | 0.45 | 3.0 | cut-in rule | - | - | - | cut-in / new lead |
| 00000011--c7d29db317 | 1056 | 46 | 23.7 | 2.5 | -0.15 | 1.24 | 3.30 | 3.30 | 1.12 | 0.3 | 3.15 | 0.4 | +0.5s (earlier +0.0s) | Y | - | - | cut-in / new lead |

### A: per-episode (peak cmd before any lead switch)

| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak cmd | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000000f--79cf51e7fa | 457 | 51 | 60.0 | 3.6 | -1.16 | 1.55 | 2.16 | 2.16 | 1.30 | 0.6 | 0.36 | 3.6 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000f--79cf51e7fa | 459 | 47 | 59.1 | 0.4 | 1.12 | 1.58 | 0.00 | 0.00 | 1.07 | 21.4 | 0.00 | 21.4 | cut-in rule | - | - | **OVER** | cut-in / new lead |
| 0000000f--79cf51e7fa | 654 | 64 | 66.7 | 3.8 | -1.39 | 1.45 | 1.28 | 5.36 | 2.93 | 0.5 | 2.85 | 1.0 | +1.7s | - | - | - | cut-in / new lead |
| 0000000f--79cf51e7fa | 715 | 50 | 52.1 | 3.1 | -1.36 | 1.56 | 1.72 | 1.72 | 1.23 | 0.7 | 0.43 | 2.9 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000f--79cf51e7fa | 1546 | 60 | 38.8 | 2.1 | -0.93 | 1.09 | 1.48 | 1.59 | 1.15 | 0.7 | 0.90 | 1.3 | cut-in rule (earlier -1.5s) | - | - | - | cut-in / new lead |
| 0000000f--79cf51e7fa | 1725 | 65 | 47.0 | 4.9 | -0.21 | 1.05 | 1.36 | 1.36 | 0.82 | 0.6 | 1.13 | 0.7 | +0.3s (earlier -0.3s) | - | - | - | cut-in / new lead |
| 0000000d--5e0d1d244b | 286 | 61 | 42.3 | 5.4 | -1.28 | 1.09 | 3.29 | 111.50 | 2.71 | 0.0 | 110.29 | 0.0 | cut-in rule (earlier -0.5s) | - | Y | - | cut-in / new lead |
| 0000000d--5e0d1d244b | 307 | 62 | 25.2 | 1.8 | -0.04 | 1.07 | 1.84 | 1.84 | 0.84 | 0.5 | 1.71 | 0.5 | +0.3s (earlier -0.7s) | Y | - | - | cut-in / new lead |
| 0000000d--5e0d1d244b | 574 | 62 | 77.9 | 4.6 | -1.20 | 1.27 | 1.78 | 1.78 | 1.34 | 0.8 | 0.35 | 3.8 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000d--5e0d1d244b | 727 | 61 | 38.3 | 2.2 | -0.94 | 1.29 | 3.46 | 3.85 | 1.08 | 0.3 | 2.84 | 0.4 | +0.5s | Y | - | - | cut-in / new lead |
| 0000000d--5e0d1d244b | 733 | 59 | 40.4 | 2.3 | -0.29 | 1.31 | 2.90 | 2.90 | 0.84 | 0.3 | 2.61 | 0.3 | +0.5s (earlier +0.0s) | Y | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 306 | 52 | 32.5 | -0.2 | -0.52 | 1.38 | 0.49 | 0.53 | 1.15 | 2.2 | -0.00 | 23.0 | - | - | - | **OVER** | other +curve |
| 0000000e--5ebf4636a1 | 319 | 54 | 39.5 | 3.0 | -2.27 | 1.36 | 6.94 | 40.04 | 1.98 | 0.0 | 38.08 | 0.1 | - | - | - | - | lead braking |
| 0000000e--5ebf4636a1 | 340 | 55 | 27.2 | 3.6 | 0.39 | 1.35 | 6.62 | 6.62 | 1.14 | 0.2 | 6.62 | 0.2 | - (earlier +0.0s) | Y | - | - | closing on slower lead |
| 0000000e--5ebf4636a1 | 372 | 58 | 35.3 | 0.6 | -0.67 | 1.31 | 0.86 | 0.86 | 0.85 | 1.0 | 0.20 | 4.2 | - | Y | - | - | other |
| 0000000e--5ebf4636a1 | 416 | 58 | 47.3 | 2.5 | -1.39 | 1.32 | 1.48 | 2.10 | 1.45 | 0.7 | 0.74 | 2.0 | +1.7s | - | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 431 | 54 | 43.6 | 1.9 | -0.70 | 1.36 | 1.12 | 1.30 | 1.04 | 0.8 | 0.61 | 1.7 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 483 | 60 | 47.5 | 3.1 | -1.49 | 1.30 | 2.38 | 2.47 | 1.54 | 0.6 | 0.92 | 1.7 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 532 | 56 | 36.0 | 1.3 | -0.53 | 1.34 | 1.48 | 1.48 | 0.88 | 0.6 | 0.88 | 1.0 | - | Y | - | - | other |
| 0000000e--5ebf4636a1 | 556 | 59 | 43.6 | 3.1 | -2.15 | 1.30 | 3.07 | 20.42 | 2.43 | 0.1 | 18.60 | 0.1 | cut-in rule | - | - | - | cut-in / new lead +curve |
| 0000000e--5ebf4636a1 | 590 | 46 | 42.8 | 4.4 | -3.39 | 1.43 | 3.59 | 5.48 | 3.50 | 0.6 | 2.44 | 1.4 | cut-in rule | - | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 614 | 59 | 44.0 | 4.6 | -0.27 | 1.31 | 3.25 | 6.02 | 0.80 | 0.1 | 5.74 | 0.1 | +0.5s (earlier -0.5s) | - | - | - | cut-in / new lead |
| 0000000e--5ebf4636a1 | 707 | 64 | 0.0 | 28.8 | 0.00 | 1.26 | 0.00 | 0.00 | 1.05 | 21.0 | 0.00 | 21.0 | +1.3s | - | - | - | no lead | (no lead)
| 0000000e--5ebf4636a1 | 712 | 62 | 77.8 | 3.6 | -5.77 | 1.28 | 4.86 | 5.23 | 3.45 | 0.7 | 0.28 | 12.4 | +0.8s (earlier -0.2s) | - | - | - | cut-in / new lead +vCruise below speed |

### C: per-episode (peak cmd before any lead switch)

| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak cmd | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 00000012--2aada6ce32 | 94 | 60 | 51.8 | 2.1 | -1.25 | 1.10 | 1.39 | 1.39 | 0.94 | 0.7 | 0.14 | 6.9 | +0.2s (earlier +0.0s) | - | - | - | lead braking +vCruise below speed +curve |
| 00000012--2aada6ce32 | 280 | 52 | 59.7 | 2.4 | -0.97 | 1.17 | 0.91 | 0.98 | 0.83 | 0.8 | 0.11 | 7.3 | - (earlier -1.2s) | - | - | - | closing on slower lead |
| 00000012--2aada6ce32 | 291 | 52 | 53.3 | 1.3 | -0.35 | 1.17 | 0.31 | 2.47 | 2.16 | 0.9 | 0.26 | 8.2 | cut-in rule | - | - | - | cut-in / new lead +curve |
| 00000012--2aada6ce32 | 513 | 52 | 42.5 | 1.4 | -0.56 | 1.18 | 0.49 | 0.64 | 1.73 | 2.7 | 0.12 | 14.8 | - | - | - | **OVER** | other |
| 00000012--2aada6ce32 | 558 | 51 | 36.8 | 1.0 | -0.56 | 1.18 | 0.78 | 0.78 | 0.84 | 1.1 | 0.12 | 6.8 | - | - | - | - | other |
| 00000012--2aada6ce32 | 633 | 62 | 56.1 | 0.6 | -1.12 | 1.08 | 0.75 | 1.87 | 1.78 | 1.0 | 0.14 | 13.1 | +4.2s | - | - | - | other +curve |
| 00000012--2aada6ce32 | 732 | 61 | 0.0 | 27.1 | 0.00 | 1.09 | 0.00 | 0.00 | 0.87 | 17.5 | 0.00 | 17.5 | - | - | - | - | no lead +vCruise below speed | (no lead)
| 00000012--2aada6ce32 | 792 | 50 | 39.8 | 0.6 | -0.93 | 1.19 | 0.92 | 4.42 | 3.45 | 0.8 | 0.97 | 3.5 | - | - | - | - | other +curve |

### stock: per-episode (peak aEgo before any lead switch)

| route | t (s) | mph | gap m | closing m/s | aLeadK raw | tFollow | need@onset | peak need | peak aEgo | ratio | need (aL=0) | ratio (aL=0) | lead switch dt | inside gap | exp | over | cause |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0000001d--2ead59d2dd | 636 | 46 | 64.5 | 1.3 | -0.53 | 1.45 | 0.51 | 0.54 | 1.04 | 1.9 | 0.05 | 20.8 | +0.5s (earlier -0.8s) | - | - | **OVER** | cut-in / new lead |
| 00000028--297af64c9c | 528 | 63 | 37.2 | 2.6 | -0.06 | 1.45 | 3.46 | 5.14 | 1.30 | 0.3 | 5.10 | 0.3 | cut-in rule | Y | - | - | cut-in / new lead |
| 00000028--297af64c9c | 1017 | 60 | 39.2 | 1.6 | -0.38 | 1.45 | 1.53 | 3.05 | 2.28 | 0.7 | 2.71 | 0.8 | cut-in rule | Y | - | - | cut-in / new lead |
| 00000028--297af64c9c | 1144 | 54 | 26.0 | 1.1 | -0.03 | 1.45 | 0.71 | 1.05 | 0.89 | 0.8 | 0.99 | 0.9 | - | Y | - | - | gap too short |
| 00000028--297af64c9c | 1210 | 55 | 29.9 | 2.3 | -1.40 | 1.45 | 3.57 | 4.74 | 2.04 | 0.4 | 3.27 | 0.6 | - | Y | - | - | closing on slower lead |
| 00000028--297af64c9c | 1299 | 51 | 16.7 | 0.2 | 0.08 | 1.45 | 0.02 | 0.02 | 0.86 | 17.2 | 0.02 | 17.2 | cut-in rule | Y | - | **OVER** | cut-in / new lead |
| 00000029--b3c2c13819 | 128 | 46 | 25.6 | 0.5 | -0.05 | 1.45 | 0.19 | 0.19 | 0.92 | 4.9 | 0.16 | 5.9 | - | Y | - | **OVER** | other |
| 00000029--b3c2c13819 | 290 | 54 | 30.3 | 3.5 | -1.69 | 1.45 | 7.68 | 39.80 | 2.09 | 0.1 | 39.63 | 0.1 | - | Y | - | - | lead braking |
| 00000029--b3c2c13819 | 353 | 49 | 23.1 | 1.6 | 0.05 | 1.45 | 1.34 | 1.34 | 1.10 | 0.8 | 1.32 | 0.8 | - | Y | - | - | closing on slower lead |
| 00000029--b3c2c13819 | 382 | 55 | 14.8 | 1.2 | -0.11 | 1.45 | 0.97 | 0.97 | 1.42 | 1.5 | 0.74 | 1.9 | cut-in rule | Y | - | - | cut-in / new lead |
| 00000029--b3c2c13819 | 740 | 58 | 43.4 | 3.2 | -0.11 | 1.45 | 5.28 | 5.28 | 1.03 | 0.2 | 5.18 | 0.2 | cut-in rule | Y | - | - | cut-in / new lead |
| 00000029--b3c2c13819 | 1066 | 60 | 34.0 | 2.4 | -0.38 | 1.45 | 3.34 | 4.49 | 1.84 | 0.4 | 4.07 | 0.5 | - | Y | - | - | closing on slower lead |
| 00000029--b3c2c13819 | 1263 | 69 | 86.6 | 8.5 | -0.35 | 1.45 | 1.36 | 1.88 | 1.66 | 0.9 | 1.38 | 1.2 | +0.5s (earlier +0.0s) | - | - | - | cut-in / new lead |
| 00000029--b3c2c13819 | 1278 | 67 | 103.6 | 6.1 | -0.24 | 1.45 | 0.49 | 0.72 | 2.10 | 2.9 | 0.43 | 4.9 | +1.0s (earlier -1.2s) | - | - | **OVER** | cut-in / new lead |
| 00000029--b3c2c13819 | 1390 | 63 | 57.0 | 5.3 | -0.08 | 1.45 | 1.41 | 2.23 | 2.38 | 1.1 | 2.19 | 1.1 | - (earlier -1.8s) | - | - | - | closing on slower lead |
| 00000034--5f8605e608 | 270 | 69 | 48.4 | 0.6 | -0.04 | 1.45 | 0.26 | 2.09 | 1.49 | 0.7 | 2.02 | 0.7 | cut-in rule | Y | - | - | cut-in / new lead |
| 00000034--5f8605e608 | 295 | 69 | 21.6 | 0.8 | -0.09 | 1.45 | 0.37 | 1.01 | 0.95 | 0.9 | 0.95 | 1.0 | cut-in rule (earlier -0.8s) | Y | - | - | cut-in / new lead |
| 00000034--5f8605e608 | 742 | 69 | 35.0 | 2.1 | -0.04 | 1.45 | 2.24 | 4.03 | 1.08 | 0.3 | 3.95 | 0.3 | cut-in rule (earlier -0.8s) | Y | - | - | cut-in / new lead |
| 00000034--5f8605e608 | 1653 | 70 | 41.7 | 0.3 | -0.02 | 1.45 | 0.08 | 1.50 | 0.95 | 0.6 | 1.50 | 0.6 | cut-in rule (earlier -1.0s) | Y | - | - | cut-in / new lead |
| 00000034--5f8605e608 | 1725 | 67 | 62.3 | 1.7 | 0.03 | 1.45 | 0.11 | 0.11 | 0.82 | 7.2 | 0.11 | 7.2 | cut-in rule | - | - | **OVER** | cut-in / new lead |

### Summary (episodes with a lead; ratio and excess = peak command (stock: aEgo) minus needed, before lead switch)

| group | n episodes | n with lead | n lead switch | n over-braking | ratio median | ratio p90 | excess median m/s^2 | ratio (aL=0) median |
|---|---|---|---|---|---|---|---|---|
| B-today | 24 | 24 | 18 | 5 | 0.61 | 3.35 | -1.20 | 0.99 |
| B-earlier | 7 | 7 | 5 | 1 | 0.62 | 8.12 | -0.79 | 1.98 |
| A | 24 | 23 | 19 | 2 | 0.60 | 0.95 | -0.93 | 1.28 |
| C | 8 | 7 | 4 | 1 | 0.88 | 1.74 | -0.15 | 7.28 |
| stock | 20 | 20 | 14 | 5 | 0.83 | 5.18 | -0.23 | 0.87 |

### Worst 5 over-braking episodes (by excess)

| group | route | t (s) | mph | gap m | peak need | peak cmd | excess | ratio | cause | replay |
|---|---|---|---|---|---|---|---|---|---|---|
| stock | 00000029--b3c2c13819 | 1278 | 67 | 103.6 | 0.72 | 2.10 | 1.38 | 2.9 | cut-in / new lead | `highway_brakes.py 00000029--b3c2c13819 --at 1278` |
| B-today | 00000015--2b757ccb00 | 840 | 57 | 72.9 | -0.00 | 1.18 | 1.18 | 23.5 | other | `highway_brakes.py 00000015--2b757ccb00 --at 840` |
| C | 00000012--2aada6ce32 | 513 | 52 | 42.5 | 0.64 | 1.73 | 1.09 | 2.7 | other | `highway_brakes.py 00000012--2aada6ce32 --at 513` |
| A | 0000000f--79cf51e7fa | 459 | 47 | 59.1 | 0.00 | 1.07 | 1.06 | 21.4 | cut-in / new lead | `highway_brakes.py 0000000f--79cf51e7fa --at 459` |
| B-today | 00000014--62bceb20db | 1076 | 58 | 9.0 | 0.39 | 1.41 | 1.02 | 3.6 | cut-in / new lead | `highway_brakes.py 00000014--62bceb20db --at 1076` |
