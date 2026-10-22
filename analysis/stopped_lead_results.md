# Stinger stopped-lead tune (Testing Ground slot 10): sim results (stinger-combined 3a792e2)

Sim: analysis/sim_stopped_lead.py, `scenarios --personality standard,aggressive --delay 0.5 --car-tune B,C --stopped-lead-tune A,B`, tables by analysis/stopped_lead_ab.py.

Baseline contacts (A, 25 mph detected at 30 m, 30 mph at 40 m): partly a sim artifact. The sim primed the MPC lead_v filter (rc 0.8 s) to v_ego+10 at start; long_mpc
recreates it with rc 0 after any drive that went above ~51 mph, so `--lead-filter fast` is the usual state and `primed` the fresh-planner worst case
(25 mph/30 m: 14 mph impact primed, 2 mph fast). What remains is real planner behaviour: StarPilot's trackingLead filter needs ~0.5 s, the first plan is capped near -0.5 and
ramps, plus 0.2 s delay + 0.3 s lag; 25 mph from 30 m is marginal (about 28 m of the 30 m are consumed before full braking).

Final constants: TARGET_DECEL 1.4 (unchanged; 1.2 and 1.6 raised the peak to 2.5-2.8 and made late cases worse), FINAL_GAP_M 4.0 -> 4.25, SAFE_DECEL 3.0 -> 1.5, LATE_FADE_M 1.5 -> 4.0.
With 3.0/1.5 the tune stopped late-detected leads 1-2 m closer than A (16 of 60 cases outside tolerance); 1.5/4.0 gives 0 (fast) / 1 (primed, 0.2 m).
Final gaps 3.3-4.4 m (A: 4.3-6.3 m). Note: B min gap < A min gap is by design wherever A stops 5-6 m behind; compared with min(A, 3 m) all cases pass; impact speed B <= A in every case.

What binds the approach and the creep: planner.get_close_lead_brake_cap (L~2646) forces a_target <= max(accel_min, -required) = about -0.5 for the whole approach (TTC < 25 s) in both A and B, so the
MPC bias never gets to act: onset TTC ~10 s, mid decel 0.3-0.9, crawl at 3 mph with only ~0.4 decel -> creep 2-3.7 s.
Proposal (not implemented): skip get_close_lead_brake_cap for the Stinger when the slot-10 bias is active and the lead is stopped (< 1 m/s). Experiment `--exp-no-close-cap`
(Standard, car tune B): onset TTC 5.1-6.6 s, mid decel 1.07-1.46, peak 1.43-1.55 (<= 2.08 at 35 mph), creep 1.1-1.5 s, final gap 3.4-4.1 m; late-detection min gap/impact still no worse than A.
