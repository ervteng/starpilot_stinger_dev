# Drive catalog

Every drive from the Stinger (dongle `59fa1d41318b87d3`) from 2026-09-17 on, generated from comma connect.
`routes.json` holds the same data for `fetch_routes.py`.

- **Route counters reset** (Sep 25 and Oct 18), so short names like `0000000b` repeat. Always use the full
  `name--hash`.
- **longitudinal**: `openpilot-long` = openpilot longitudinal; `stock-acc` = factory SCC with cruise engaged;
  `stock-capable-unused` = openpilot long off but cruise never engaged. Classified from the startup carParams
  and the qlogs present when generated (sometimes only 1-2 segments).
- **rlogs**: segments with full rlogs uploaded (needed for raw CAN / radar work). Everything has qlogs.

## Fetching

```bash
./fetch_routes.py --tag stock-acc --rlog-only            # stock ACC drives with rlogs, rlog + qlog
./fetch_routes.py --tag stinger-combined                 # by tag or branch
./fetch_routes.py 0000000e--5ebf4636a1 0000002b--66bdc92988 --type qlog
./fetch_routes.py --list --tag auto-stop
```

## Routes used in the analysis

| Route | Start (PDT) | Segs | Build | Longitudinal | rlogs | Notes |
|---|---|---|---|---|---|---|
| `0000000f--3473db42b0` | 2026-09-20 11:58 | 14 | release-mici `6a17f75c` | openpilot-long (0%) | - | sunnypilot release-mici, camera-only long, HyundaiLongitudinalTuning Off, experimental mode on, standard. Near-miss ~271 s (seg 4): lead braked 19->0 mph, camera aLeadK only -0.3..-1.0, openpilot reached -1.8 m/s^2, driver braked at 9 m closing 5 m/s. |
| `0000001b--e606a33a74` | 2026-09-24 09:16 | 48 | release-mici `6a17f75c` | openpilot-long (0%) | - | sunnypilot release-mici, camera-only long, tuning Off, experimental off, standard. Close call ~241 s (seg 4): lead braked 13->0 mph, openpilot -2.3 m/s^2, driver braked at 3.9 m. |
| `00000002--b47e891d5f` | 2026-09-25 12:52 | 11 | StarPilot `2a12dbd0` | openpilot-long (30%) | 0-10 | Auto Stop steering-fault root cause: rlog segs 9-10 show LVR12 CF_Lvr_IsgState -> MDPS ToiUnavail -> steerTempUnavailableSilent. |
| `00000003--57168e519c` | 2026-09-25 13:04 | 18 | StarPilot `2a12dbd0` | openpilot-long (59%) | 0 | rlog seg 0: 0x544 Navi_HU on bus 0 within 0.03 s of startup. Moving steerTempUnavailable soft-disables in segs 8 and 15 (not yet investigated). |
| `00000004--d9c8338110` | 2026-09-25 18:15 | 53 | StarPilot `2a12dbd0` | openpilot-long (82%) | 0 | rlog seg 0: 0x544 on bus 0 at 0.12 s; 0x391 (LFA button) and 0x53E (8 bytes, not LKAS12) present. |
| `00000012--fa118cae70` | 2026-09-28 18:41 | 10 | Dom `8d01d881` | openpilot-long (16%) | 0-9 | Early stinger_shadow_logger / radar validation rlogs. |
| `0000001d--2ead59d2dd` | 2026-09-30 18:47 | 21 | Dom `bf1b916d` | stock-acc (46%) | 0-20 | Stock ACC with rlogs: speed-bias fix validation, long-range distance step, stock highway lead-braking comparison. |
| `00000023--3f5d2c1b16` | 2026-09-30 22:53 | 36 | Dom `42d4a5b2` | stock-acc (88%) | - | Stock ACC, qlogs only. |
| `00000028--297af64c9c` | 2026-10-13 17:48 | 47 | Dom `3ccae7b8` | stock-acc (79%) | 0-46 | Stock ACC reference drive (Gemini's). Decoder vs factory SCC11 lead, speed-bias fit, FCA11/SCC12 AEB fields (never active), bus-1 contents. |
| `00000029--b3c2c13819` | 2026-10-14 09:30 | 31 | Dom `b220a37f` | stock-acc (89%) | 0-30 | Stock ACC with rlogs (mostly highway): speed-bias fix validation, stock lead-braking comparison. |
| `0000002b--66bdc92988` | 2026-10-14 16:13 | 54 | Dom `b220a37f` | stock-acc (86%) | - | Stock ACC, long highway drive, qlogs only. Moving steerTempUnavailable soft-disable seg 51. |
| `00000034--5f8605e608` | 2026-10-16 09:09 | 37 | Dom `b220a37f` | stock-acc (79%) | 0-36 | Stock ACC with rlogs (mostly highway): speed-bias fix validation. |
| `00000035--fcfb8f4e99` | 2026-10-16 10:01 | 74 | Dom `b220a37f` | stock-acc (82%) | - | Stock ACC, 74 segs, qlogs only. |
| `00000038--99b9b4ae8e` | 2026-10-17 13:35 | 61 | stinger-radar-lfa `a5b024a6` | openpilot-long (70%) | 0-60 | Old stop tune with radar (before the G90 port). |
| `0000003d--5535441901` | 2026-10-17 23:26 | 12 | stinger-radar-lfa `a5b024a6` | openpilot-long (70%) | 0-11 | Public route. |
| `00000000--c3b9b96ad1` | 2026-10-18 15:01 | 5 | stinger-radar-lfa `fdfe8e23` | openpilot-long (9%) | - | First drive after the Oct 18 settings reset: SteerLatAccel 0.0 -> clamped to 1.375, doubled steering torque. |
| `00000005--8264b7d4f0` | 2026-10-19 15:07 | 12 | stinger-radar-lfa `f087104e` | openpilot-long (62%) | 0-11 | Steering oscillation analysis (SteerLatAccel). rlog seg 9 has an Auto Stop engine restart. |
| `00000006--38cf647ea8` | 2026-10-19 16:19 | 11 | stinger-radar-lfa `f087104e` | openpilot-long (63%) | 0-10 | First drive with the decoder changes; reversing-lead (stop undershoot) investigation. |
| `00000007--705b36fece` | 2026-10-19 18:44 | 24 | stinger-radar-lfa `eb7ecd28` | openpilot-long (21%) | - | G90 stop tune (stinger-radar-lfa eb7ecd2). |
| `00000008--b0f7c14716` | 2026-10-19 21:29 | 11 | stinger-radar-lfa `eb7ecd28` | openpilot-long (35%) | - | G90 stop tune (stinger-radar-lfa eb7ecd2). |
| `0000000a--a4b564ef5e` | 2026-10-20 08:46 | 39 | stinger-radar-lfa `eb7ecd28` | openpilot-long (91%) | - | G90 build. Openpilot FCW at seg 25 (lead braked 57->38 mph; -3.5 m/s^2, closest gap 10.7 m). Highway braking analysis. |
| `0000000b--4ec3463e2b` | 2026-10-20 17:02 | 2 | stinger-radar-lfa `eb7ecd28` | openpilot-long (0%) | - | 1.5 min idle, no driving. |
| `0000000c--8c7821740b` | 2026-10-20 17:05 | 5 | stinger-combined `93871099` | openpilot-long (0%) | - | First stinger-combined drive (9387109), not engaged. Dashboard speed limit check. |
| `0000000d--5e0d1d244b` | 2026-10-20 17:16 | 19 | stinger-combined `93871099` | openpilot-long (88%) | 10-18 | stinger-combined 9387109: Auto Stop fix validated in rlog segs 15-18, dashboard speed limit 99%, highway braking (cut-in seg ~4). |
| `0000000e--5ebf4636a1` | 2026-10-20 17:38 | 35 | stinger-combined `93871099` | openpilot-long (92%) | 3-28 | stinger-combined 9387109: highway traffic jam hard brake ~712 s (seg 11), Auto Stop seg 3, radar vs vision offsets, out-of-sample speed-bias check. |

## All routes

| Route | Start (PDT) | Segs | Build | Longitudinal | rlogs |
|---|---|---|---|---|---|
| `00000001--97a0ab2a9f` | 2026-09-17 15:13 | 15 | release-mici `6a17f75c` | stock-capable-unused (0%) | - |
| `00000003--2041a858cd` | 2026-09-17 17:39 | 8 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `00000004--82e41031d9` | 2026-09-17 17:57 | 13 | release-mici `6a17f75c` | openpilot-long (27%) | - |
| `00000007--09d019e731` | 2026-09-18 18:29 | 10 | release-mici `6a17f75c` | stock-capable-unused (0%) | - |
| `0000000b--36ae41adcc` | 2026-09-18 19:38 | 2 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000000d--d675980b5f` | 2026-09-18 23:13 | 16 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000000e--334ca4004b` | 2026-09-20 11:04 | 15 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000000f--3473db42b0` | 2026-09-20 11:58 | 14 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `00000010--420bf4cd99` | 2026-09-21 13:45 | 11 | release-mici `6a17f75c` | stock-capable-unused (0%) | - |
| `00000012--c9d8ad98fc` | 2026-09-22 08:52 | 4 | release-mici `6a17f75c` | stock-capable-unused (0%) | - |
| `00000013--c09c96b6b2` | 2026-09-22 08:56 | 40 | release-mici `6a17f75c` | stock-acc (7%) | - |
| `00000014--cb06fd8d0c` | 2026-09-22 11:03 | 2 | release-mici `6a17f75c` | stock-capable-unused (0%) | - |
| `00000016--9fa9b9d151` | 2026-09-22 17:56 | 6 | release-mici `6a17f75c` | openpilot-long (50%) | - |
| `00000018--7e9c97b1ac` | 2026-09-22 20:06 | 1 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `00000019--72bc60fe97` | 2026-09-22 20:07 | 1 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000001a--35482e63af` | 2026-09-22 20:08 | 34 | release-mici `6a17f75c` | openpilot-long (70%) | - |
| `0000001b--e606a33a74` | 2026-09-24 09:16 | 48 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000001c--202f4bdf5d` | 2026-09-24 15:08 | 2 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `0000001e--3d11b5b5d6` | 2026-09-24 15:11 | 52 | release-mici `6a17f75c` | openpilot-long (0%) | - |
| `00000000--a033848cef` | 2026-09-25 11:33 | 15 | StarPilot `2a12dbd0` | stock-capable-unused (0%) | - |
| `00000001--b1c5bf23f3` | 2026-09-25 11:47 | 21 | StarPilot `2a12dbd0` | openpilot-long (0%) | - |
| `00000002--b47e891d5f` | 2026-09-25 12:52 | 11 | StarPilot `2a12dbd0` | openpilot-long (30%) | 0-10 |
| `00000003--57168e519c` | 2026-09-25 13:04 | 18 | StarPilot `2a12dbd0` | openpilot-long (59%) | 0 |
| `00000004--d9c8338110` | 2026-09-25 18:15 | 53 | StarPilot `2a12dbd0` | openpilot-long (82%) | 0 |
| `00000005--3d16185212` | 2026-09-25 21:47 | 27 | StarPilot `2a12dbd0` | openpilot-long (50%) | - |
| `00000006--e8e7a5b4b8` | 2026-09-25 23:48 | 22 | StarPilot `2a12dbd0` | openpilot-long (72%) | - |
| `00000007--12de5a9ebe` | 2026-09-26 13:42 | 24 | StarPilot `2a12dbd0` | openpilot-long (65%) | - |
| `00000008--d614cda160` | 2026-09-26 14:26 | 29 | StarPilot `2a12dbd0` | openpilot-long (87%) | - |
| `00000009--b4c501a09f` | 2026-09-26 15:43 | 19 | StarPilot `2a12dbd0` | openpilot-long (62%) | - |
| `0000000a--5d70970aea` | 2026-09-26 16:05 | 2 | StarPilot `2a12dbd0` | openpilot-long (0%) | - |
| `0000000b--b1ff9cb5b9` | 2026-09-26 17:38 | 34 | StarPilot `2a12dbd0` | openpilot-long (39%) | - |
| `0000000d--589d37fcb6` | 2026-09-26 18:18 | 23 | StarPilot `2a12dbd0` | openpilot-long (57%) | - |
| `0000000e--302c49a370` | 2026-09-27 12:47 | 5 | StarPilot `2a12dbd0` | openpilot-long (0%) | - |
| `0000000f--043ec62787` | 2026-09-27 12:56 | 13 | StarPilot `2a12dbd0` | openpilot-long (28%) | - |
| `00000010--f19579589d` | 2026-09-27 13:55 | 25 | StarPilot `2a12dbd0` | openpilot-long (18%) | - |
| `00000011--958a4f4239` | 2026-09-28 17:09 | 15 | Dom `8d01d881` | openpilot-long (66%) | - |
| `00000012--fa118cae70` | 2026-09-28 18:41 | 10 | Dom `8d01d881` | openpilot-long (16%) | 0-9 |
| `00000013--1dee345bd5` | 2026-09-28 19:15 | 8 | Dom `8d01d881` | openpilot-long (24%) | - |
| `00000014--b007190867` | 2026-09-29 12:58 | 37 | Dom `faf5b533` | openpilot-long (56%) | - |
| `00000015--6e8a636bae` | 2026-09-29 13:38 | 16 | Dom `faf5b533` | openpilot-long (29%) | - |
| `00000016--b561df0e16` | 2026-09-29 14:15 | 7 | Dom `faf5b533` | openpilot-long (0%) | - |
| `00000017--d84f118ce3` | 2026-09-29 16:45 | 46 | Dom `faf5b533` | openpilot-long (30%) | - |
| `00000018--cd6ae716bf` | 2026-09-30 10:19 | 27 | Dom `bf1b916d` | openpilot-long (85%) | - |
| `00000019--a442ae859d` | 2026-09-30 10:46 | 8 | Dom `bf1b916d` | openpilot-long (0%) | - |
| `0000001b--ca5f606d61` | 2026-09-30 17:41 | 2 | Dom `bf1b916d` | stock-capable-unused (0%) | - |
| `0000001c--8038c5b788` | 2026-09-30 17:43 | 9 | Dom `bf1b916d` | stock-capable-unused (0%) | - |
| `0000001d--2ead59d2dd` | 2026-09-30 18:47 | 21 | Dom `bf1b916d` | stock-acc (46%) | 0-20 |
| `0000001e--552bf479e7` | 2026-09-30 22:05 | 5 | Dom `bf1b916d` | stock-capable-unused (0%) | - |
| `0000001f--d329339126` | 2026-09-30 22:13 | 1 | Dom `bf1b916d` | unknown (0%) | - |
| `00000020--c23a593a3c` | 2026-09-30 22:13 | 8 | Dom `42d4a5b2` | stock-capable-unused (0%) | - |
| `00000022--a0e97fae8d` | 2026-09-30 22:46 | 7 | Dom `42d4a5b2` | stock-capable-unused (0%) | - |
| `00000023--3f5d2c1b16` | 2026-09-30 22:53 | 36 | Dom `42d4a5b2` | stock-acc (88%) | - |
| `00000024--c157d1aa98` | 2026-10-11 20:29 | 14 | Dom `83f914fc` | stock-acc (1%) | 0-13 |
| `00000025--d6de5490c3` | 2026-10-11 21:21 | 1 | Dom `83f914fc` | stock-capable-unused (0%) | - |
| `00000026--d88af29f29` | 2026-10-11 21:42 | 17 | Dom `83f914fc` | stock-capable-unused (0%) | - |
| `00000027--95ce23ee58` | 2026-10-13 09:51 | 4 | Dom `3ccae7b8` | stock-acc (3%) | 0-3 |
| `00000028--297af64c9c` | 2026-10-13 17:48 | 47 | Dom `3ccae7b8` | stock-acc (79%) | 0-46 |
| `00000029--b3c2c13819` | 2026-10-14 09:30 | 31 | Dom `b220a37f` | stock-acc (89%) | 0-30 |
| `0000002a--9e80623643` | 2026-10-14 13:15 | 6 | Dom `b220a37f` | stock-capable-unused (0%) | - |
| `0000002b--66bdc92988` | 2026-10-14 16:13 | 54 | Dom `b220a37f` | stock-acc (86%) | - |
| `0000002d--4057c8b1e2` | 2026-10-15 10:55 | 2 | Dom `b220a37f` | openpilot-long (0%) | - |
| `0000002e--2b8b57f853` | 2026-10-15 10:57 | 33 | Dom `b220a37f` | openpilot-long (88%) | - |
| `0000002f--80fea0ab0e` | 2026-10-15 15:55 | 50 | Dom `b220a37f` | openpilot-long (92%) | - |
| `00000030--261b737529` | 2026-10-15 16:45 | 2 | Dom `b220a37f` | stock-capable-unused (0%) | - |
| `00000032--9efb5c0a74` | 2026-10-15 16:47 | 9 | Dom `b220a37f` | stock-acc (29%) | - |
| `00000033--f7e46742e3` | 2026-10-16 08:32 | 34 | Dom `b220a37f` | stock-acc (18%) | - |
| `00000034--5f8605e608` | 2026-10-16 09:09 | 37 | Dom `b220a37f` | stock-acc (79%) | 0-36 |
| `00000035--fcfb8f4e99` | 2026-10-16 10:01 | 74 | Dom `b220a37f` | stock-acc (82%) | - |
| `00000036--d0d092a03d` | 2026-10-17 10:00 | 10 | stinger-radar-lfa `a5b024a6` | stock-acc (18%) | - |
| `00000037--6c52a9db6d` | 2026-10-17 13:25 | 10 | stinger-radar-lfa `a5b024a6` | stock-acc (19%) | - |
| `00000038--99b9b4ae8e` | 2026-10-17 13:35 | 61 | stinger-radar-lfa `a5b024a6` | openpilot-long (70%) | 0-60 |
| `00000039--957778f296` | 2026-10-17 18:42 | 2 | stinger-radar-lfa `a5b024a6` | openpilot-long (0%) | - |
| `0000003b--e4b2bdfa67` | 2026-10-17 18:44 | 9 | stinger-radar-lfa `a5b024a6` | openpilot-long (11%) | - |
| `0000003c--37afc888fe` | 2026-10-17 19:20 | 51 | stinger-radar-lfa `a5b024a6` | openpilot-long (83%) | - |
| `0000003d--5535441901` | 2026-10-17 23:26 | 12 | stinger-radar-lfa `a5b024a6` | openpilot-long (70%) | 0-11 |
| `00000000--c3b9b96ad1` | 2026-10-18 15:01 | 5 | stinger-radar-lfa `fdfe8e23` | openpilot-long (9%) | - |
| `00000003--c4851468f1` | 2026-10-18 15:17 | 14 | stinger-radar-lfa `fdfe8e23` | openpilot-long (61%) | - |
| `00000004--51331d19d5` | 2026-10-18 16:23 | 18 | stinger-radar-lfa `fdfe8e23` | openpilot-long (63%) | - |
| `00000005--8264b7d4f0` | 2026-10-19 15:07 | 12 | stinger-radar-lfa `f087104e` | openpilot-long (62%) | 0-11 |
| `00000006--38cf647ea8` | 2026-10-19 16:19 | 11 | stinger-radar-lfa `f087104e` | openpilot-long (63%) | 0-10 |
| `00000007--705b36fece` | 2026-10-19 18:44 | 24 | stinger-radar-lfa `eb7ecd28` | openpilot-long (21%) | - |
| `00000008--b0f7c14716` | 2026-10-19 21:29 | 11 | stinger-radar-lfa `eb7ecd28` | openpilot-long (35%) | - |
| `0000000a--a4b564ef5e` | 2026-10-20 08:46 | 39 | stinger-radar-lfa `eb7ecd28` | openpilot-long (91%) | - |
| `0000000b--4ec3463e2b` | 2026-10-20 17:02 | 2 | stinger-radar-lfa `eb7ecd28` | openpilot-long (0%) | - |
| `0000000c--8c7821740b` | 2026-10-20 17:05 | 5 | stinger-combined `93871099` | openpilot-long (0%) | - |
| `0000000d--5e0d1d244b` | 2026-10-20 17:16 | 19 | stinger-combined `93871099` | openpilot-long (88%) | 10-18 |
| `0000000e--5ebf4636a1` | 2026-10-20 17:38 | 35 | stinger-combined `93871099` | openpilot-long (92%) | 3-28 |
