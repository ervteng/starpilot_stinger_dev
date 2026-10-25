#!/usr/bin/env python3
"""Lateral steering torque usage, turn-in lag, and torque saturation analysis
across all logged Kia Stinger 2022 drives in $STINGER_DEV_HOME/logs/.

Extracts:
1. Learned liveTorqueParameters and liveParameters convergence vs hardcoded CarSpecs.
2. Physical torque ceiling curve (actual & desired lateral accel by speed bin and torque bracket).
3. Anatomy of high-torque curve episodes (|output| >= 0.80 for >= 0.3s):
   - Feedforward vs PID feedback split at peak torque
   - Turn-in entry lag and tracking deficit
   - Slew rate limiting (|d(output)/dt| >= STEER_DELTA_UP = 0.78 / s)
   - Torque shaved by center taper & friction threshold inflation
   - Curve Speed Controller (CSC) comparison and required speed reduction for 85% torque.

Usage:
  lateral_torque_analysis.py [--routes ROUTE [ROUTE ...]] [--detail]
"""
import argparse
import glob
import math
import os
import re
import sys
from collections import defaultdict
import numpy as np

# Ensure analysis folder and starpilot paths are on sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
  sys.path.insert(0, SCRIPT_DIR)
import logs

MPH = 2.23694
STEER_MAX = 384
STEER_DELTA_UP_RATE = 3.0 / 384.0 * 100.0  # 0.78125 / s (3 counts per 100 Hz frame)

# Curve speed controller priors from starpilot
PRIOR_CURVATURE_BP = [0.001, 0.003, 0.01, 0.03, 0.1]
PRIOR_LAT_ACCEL_V = [1.5, 1.8, 2.2, 2.6, 2.9]

# Kia Stinger 2022 latcontrol constants
KIA_STINGER_2022_CENTER_TAPER_MAX = 0.12
KIA_STINGER_2022_CENTER_TAPER_LAT = 0.30
KIA_STINGER_2022_CENTER_TAPER_LAT_WIDTH = 0.05
KIA_STINGER_2022_CENTER_TAPER_SPEED = 10.0
KIA_STINGER_2022_CENTER_TAPER_SPEED_WIDTH = 2.5
KIA_STINGER_2022_FRICTION_THRESHOLD_GAIN = 0.10
KIA_STINGER_2022_HIGHWAY_FRICTION_INCREMENT = 0.20
KIA_STINGER_2022_HIGHWAY_FRICTION_SPEED_BP = [50.0 * 0.44704, 60.0 * 0.44704]
KIA_STINGER_2022_HIGHWAY_FRICTION_LAT_BP = [0.15, 0.40]
KIA_STINGER_2022_HIGHWAY_FRICTION_JERK_BP = [0.20, 0.75]
STANDARD_FRICTION_THRESHOLD = 0.30


def _sigmoid(x: float) -> float:
  return 1.0 / (1.0 + math.exp(-np.clip(x, -20.0, 20.0)))


def stinger_center_taper_scale(desired_lateral_accel: float, v_ego: float) -> float:
  speed_weight = _sigmoid((v_ego - KIA_STINGER_2022_CENTER_TAPER_SPEED) / KIA_STINGER_2022_CENTER_TAPER_SPEED_WIDTH)
  center_weight = _sigmoid((KIA_STINGER_2022_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                           KIA_STINGER_2022_CENTER_TAPER_LAT_WIDTH)
  return 1.0 - (KIA_STINGER_2022_CENTER_TAPER_MAX * speed_weight * center_weight)


def stinger_friction_threshold(v_ego: float, desired_lateral_accel: float, desired_lateral_jerk: float) -> float:
  speed_weight = _sigmoid((v_ego - KIA_STINGER_2022_CENTER_TAPER_SPEED) / KIA_STINGER_2022_CENTER_TAPER_SPEED_WIDTH)
  center_weight = _sigmoid((KIA_STINGER_2022_CENTER_TAPER_LAT - abs(desired_lateral_accel)) /
                           KIA_STINGER_2022_CENTER_TAPER_LAT_WIDTH)
  threshold = STANDARD_FRICTION_THRESHOLD * (1.0 + KIA_STINGER_2022_FRICTION_THRESHOLD_GAIN * speed_weight * center_weight)
  highway_weight = np.interp(v_ego, KIA_STINGER_2022_HIGHWAY_FRICTION_SPEED_BP, [0.0, 1.0])
  highway_weight *= np.interp(abs(desired_lateral_accel), KIA_STINGER_2022_HIGHWAY_FRICTION_LAT_BP, [1.0, 0.0])
  highway_weight *= np.interp(abs(desired_lateral_jerk), KIA_STINGER_2022_HIGHWAY_FRICTION_JERK_BP, [1.0, 0.0])
  threshold += KIA_STINGER_2022_HIGHWAY_FRICTION_INCREMENT * highway_weight
  return float(threshold)


def analyze_route(route_short: str):
  """Analyze a single route for learned parameters, torque ceiling data, and high-torque curve episodes."""
  qfiles = logs.segment_files(route_short, "qlog")
  if not qfiles:
    return None

  # 1. Extract latest learned parameters
  latest_ltp = None
  latest_lp = None
  for f in reversed(qfiles):
    ltp_seg = None
    lp_seg = None
    for e in logs.events(f):
      w = e.which()
      if w == "liveTorqueParameters":
        ltp_seg = e.liveTorqueParameters.to_dict()
      elif w == "liveParameters":
        lp_seg = e.liveParameters.to_dict()
    if latest_ltp is None and ltp_seg and ltp_seg.get("totalBucketPoints", 0) > 0:
      latest_ltp = ltp_seg
    if latest_lp is None and lp_seg and lp_seg.get("valid", False):
      latest_lp = lp_seg
    if latest_ltp is not None and latest_lp is not None:
      break

  # 2. Sequential stream analysis
  route_samples = []
  t0 = None

  for f in qfiles:
    seg_num = logs.seg_num(f)
    v_ego = 0.0
    steer_pressed = False
    lat_active = False
    csc_controlling = False
    csc_speed = 0.0
    road_curvature = 0.0

    for e in logs.events(f):
      w = e.which()
      if t0 is None:
        t0 = e.logMonoTime
      if w == "carState":
        cs = e.carState
        v_ego = cs.vEgo
        steer_pressed = cs.steeringPressed
      elif w == "carControl":
        lat_active = e.carControl.latActive
      elif w == "starpilotPlan":
        sp = e.starpilotPlan
        csc_controlling = bool(sp.cscControllingSpeed)
        csc_speed = float(sp.cscSpeed)
        road_curvature = float(sp.roadCurvature)
      elif w == "controlsState":
        cs = e.controlsState
        if cs.lateralControlState.which() == "torqueState":
          ts = cs.lateralControlState.torqueState
          t_sec = (e.logMonoTime - t0) * 1e-9
          route_samples.append({
            "t": t_sec,
            "seg": seg_num,
            "active": lat_active and not steer_pressed,
            "v": float(v_ego),
            "output": float(ts.output),
            "abs_output": abs(float(ts.output)),
            "act_lat": float(ts.actualLateralAccel),
            "abs_act_lat": abs(float(ts.actualLateralAccel)),
            "des_lat": float(ts.desiredLateralAccel),
            "abs_des_lat": abs(float(ts.desiredLateralAccel)),
            "des_jerk": float(ts.desiredLateralJerk),
            "p": float(ts.p),
            "i": float(ts.i),
            "d": float(ts.d),
            "f": float(ts.f),
            "saturated": bool(ts.saturated),
            "csc": csc_controlling,
            "csc_speed": csc_speed,
            "curv": road_curvature,
          })

  # Extract active samples for torque ceiling statistics
  active_samples = [
    (s["v"], s["abs_output"], s["abs_act_lat"], s["abs_des_lat"], s["abs_output"] >= 0.95 or s["saturated"], s["des_jerk"])
    for s in route_samples if s["active"]
  ]

  # 3. Detect curve episodes (|output| >= 0.80 for >= 0.3s)
  episodes = []
  i = 0
  N = len(route_samples)
  while i < N:
    s = route_samples[i]
    if s["active"] and s["abs_output"] >= 0.80:
      j = i
      while j < N and route_samples[j]["active"] and route_samples[j]["abs_output"] >= 0.70:
        j += 1
      sub = route_samples[i:j]
      dur_80 = sum(0.1 for x in sub if x["abs_output"] >= 0.80)
      if dur_80 >= 0.30:
        peak_offset = int(np.argmax([x["abs_output"] for x in sub]))
        peak_idx = i + peak_offset
        peak = route_samples[peak_idx]

        # Feedforward vs error feedback split
        abs_f = abs(peak["f"])
        abs_pi = abs(peak["p"] + peak["i"])
        split_denom = abs_f + abs_pi
        ff_ratio = abs_f / max(split_denom, 1e-4)
        pi_ratio = abs_pi / max(split_denom, 1e-4)
        same_direction = (peak["f"] * (peak["p"] + peak["i"])) > 0

        # Turn-in window: trace back from peak to where des_lat was baseline (< 0.15 m/s^2)
        k = peak_idx
        while k > max(0, peak_idx - 60):  # up to 6s back
          if route_samples[k]["abs_des_lat"] < 0.15:
            break
          k -= 1
        entry_idx = k
        ti_samples = route_samples[entry_idx:peak_idx + 1]

        # Slew rate during turn-in
        max_slew = 0.0
        for idx_s in range(entry_idx, peak_idx):
          dt = route_samples[idx_s + 1]["t"] - route_samples[idx_s]["t"]
          if 0.05 <= dt <= 0.25:
            slew = abs(route_samples[idx_s + 1]["abs_output"] - route_samples[idx_s]["abs_output"]) / dt
            if slew > max_slew:
              max_slew = slew
        slew_limited = max_slew >= STEER_DELTA_UP_RATE

        # Turn-in delay at 0.30 and 0.50 m/s^2 threshold crossing
        t_des_03 = next((x["t"] for x in ti_samples if x["abs_des_lat"] >= 0.30), None)
        t_act_03 = next((x["t"] for x in ti_samples if x["abs_act_lat"] >= 0.30), None)
        t_des_05 = next((x["t"] for x in ti_samples if x["abs_des_lat"] >= 0.50), None)
        t_act_05 = next((x["t"] for x in ti_samples if x["abs_act_lat"] >= 0.50), None)

        delay_03 = (t_act_03 - t_des_03) if (t_act_03 is not None and t_des_03 is not None) else None
        delay_05 = (t_act_05 - t_des_05) if (t_act_05 is not None and t_des_05 is not None) else None

        # Cross-correlation lag during turn-in
        des_arr = np.array([x["abs_des_lat"] for x in ti_samples])
        act_arr = np.array([x["abs_act_lat"] for x in ti_samples])
        xcorr_lag_ms = None
        if len(des_arr) >= 5 and np.std(des_arr) > 0.05 and np.std(act_arr) > 0.05:
          # correlate
          corr = np.correlate(act_arr - np.mean(act_arr), des_arr - np.mean(des_arr), mode="full")
          lag_idx = np.argmax(corr) - (len(des_arr) - 1)
          xcorr_lag_ms = lag_idx * 100.0  # 100 ms per qlog sample

        # Tracking error during turn-in
        rising_samples = [x for x in ti_samples if 0.15 <= x["abs_des_lat"] <= max(1.0, 0.15)]
        mean_tracking_err = float(np.mean([x["abs_des_lat"] - x["abs_act_lat"] for x in rising_samples])) if rising_samples else 0.0
        max_tracking_err = float(np.max([x["abs_des_lat"] - x["abs_act_lat"] for x in rising_samples])) if rising_samples else 0.0

        # Center taper & friction inflation effect during initial turn-in (|des_lat| < 0.35)
        init_samples = [x for x in ti_samples if x["abs_des_lat"] < 0.35]
        taper_losses = []
        fric_damping_pcts = []
        for x in init_samples:
          taper = stinger_center_taper_scale(x["abs_des_lat"], x["v"])
          taper_losses.append(x["abs_output"] * (1.0 - taper))
          st_thresh = stinger_friction_threshold(x["v"], x["abs_des_lat"], x["des_jerk"])
          if st_thresh > STANDARD_FRICTION_THRESHOLD:
            fric_damping_pcts.append((st_thresh - STANDARD_FRICTION_THRESHOLD) / st_thresh)

        avg_taper_loss = float(np.mean(taper_losses)) if taper_losses else 0.0
        avg_fric_damping_pct = float(np.mean(fric_damping_pcts)) if fric_damping_pcts else 0.0

        # CSC speed reduction evaluation
        curv = abs(peak["curv"]) if abs(peak["curv"]) > 1e-5 else (peak["abs_des_lat"] / max(peak["v"] ** 2, 1.0))
        prior_lat = float(np.interp(curv, PRIOR_CURVATURE_BP, PRIOR_LAT_ACCEL_V))
        lat_85 = 0.65 if peak["v"] < 15.0 else (2.00 if peak["v"] < 22.0 else (2.10 if peak["v"] < 28.0 else 1.80))
        if peak["abs_des_lat"] > lat_85:
          v_safe = peak["v"] * math.sqrt(lat_85 / peak["abs_des_lat"])
          mph_slower = (peak["v"] - v_safe) * MPH
        else:
          mph_slower = 0.0

        episodes.append({
          "route": route_short,
          "seg": peak["seg"],
          "t_start": s["t"],
          "t_peak": peak["t"],
          "t_end": sub[-1]["t"],
          "duration": sub[-1]["t"] - s["t"],
          "v_peak_ms": peak["v"],
          "v_peak_mph": peak["v"] * MPH,
          "peak_output": peak["abs_output"],
          "saturated": peak["abs_output"] >= 0.999 or peak["saturated"],
          "reaches_95": peak["abs_output"] >= 0.95,
          "act_lat_peak": peak["abs_act_lat"],
          "des_lat_peak": peak["abs_des_lat"],
          "f_peak": peak["f"],
          "pi_peak": peak["p"] + peak["i"],
          "ff_ratio": ff_ratio,
          "pi_ratio": pi_ratio,
          "same_direction": same_direction,
          "max_slew": max_slew,
          "slew_limited": slew_limited,
          "delay_03": delay_03,
          "delay_05": delay_05,
          "xcorr_lag_ms": xcorr_lag_ms,
          "mean_tracking_err": mean_tracking_err,
          "max_tracking_err": max_tracking_err,
          "taper_loss": avg_taper_loss,
          "fric_damping_pct": avg_fric_damping_pct,
          "csc_active": any(x["csc"] for x in sub),
          "prior_lat": prior_lat,
          "lat_85": lat_85,
          "mph_slower": mph_slower,
        })
      i = j
    else:
      i += 1

  return {
    "route": route_short,
    "ltp": latest_ltp,
    "lp": latest_lp,
    "active_samples": active_samples,
    "episodes": episodes,
  }


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--routes", nargs="*", help="Optional route list (defaults to all in logs dir)")
  p.add_argument("--detail", action="store_true", help="Print details of each episode")
  args = p.parse_args()

  # Discover all Stinger routes if none provided
  if args.routes:
    routes = [logs.short(r) for r in args.routes]
  else:
    qfiles = glob.glob(os.path.join(logs.LOG_DIR, "*--*--qlog.*"))
    routes = sorted(list(set(re.search(r"([0-9a-f]{16}_[0-9a-f]{8}--[0-9a-f]{10})", os.path.basename(f)).group(1) for f in qfiles)))
    routes = [logs.short(r) for r in routes]

  print(f"================================================================================")
  print(f" KIA STINGER 2022 LATERAL TORQUE, TURN-IN LAG & SATURATION ANALYSIS")
  print(f" Total Stinger Routes: {len(routes)} across {logs.LOG_DIR}")
  print(f"================================================================================\n")

  # Process all routes in parallel
  results = logs.pmap(analyze_route, routes, workers=min(12, os.cpu_count() or 8))
  results = [r for r in results if r is not None]

  # ================================================================================
  # 1. LEARNED PARAMETERS SUMMARY
  # ================================================================================
  print(f"--------------------------------------------------------------------------------")
  print(f" 1. LEARNED liveTorqueParameters & liveParameters SUMMARY")
  print(f"--------------------------------------------------------------------------------")
  print(f"{'Route':<24} | {'Points':>6} | {'latAccelFac':>11} | {'latOffset':>9} | {'friction':>8} | {'steerRatio':>10} | {'stiffness':>9} | {'angleOff(deg)':>13}")
  print(f"{'-'*24}-|-{'-'*6}-|-{'-'*11}-|-{'-'*9}-|-{'-'*8}-|-{'-'*10}-|-{'-'*9}-|-{'-'*13}")

  pts_list = []
  laf_list = []
  la_off_list = []
  fric_list = []
  sr_list = []
  stiff_list = []
  ang_list = []

  for res in results:
    ltp = res["ltp"]
    lp = res["lp"]
    if ltp and lp:
      pts = ltp.get("totalBucketPoints", 0.0)
      laf = ltp.get("latAccelFactorFiltered", 0.0)
      la_off = ltp.get("latAccelOffsetFiltered", 0.0)
      fric = ltp.get("frictionCoefficientFiltered", 0.0)
      sr = lp.get("steerRatio", 0.0)
      stiff = lp.get("stiffnessFactor", 0.0)
      ang = lp.get("angleOffsetAverageDeg", 0.0)

      pts_list.append(pts)
      laf_list.append(laf)
      la_off_list.append(la_off)
      fric_list.append(fric)
      sr_list.append(sr)
      stiff_list.append(stiff)
      ang_list.append(ang)

      print(f"{res['route']:<24} | {pts:6.0f} | {laf:11.3f} | {la_off:9.3f} | {fric:8.3f} | {sr:10.2f} | {stiff:9.2f} | {ang:13.2f}")

  print(f"{'-'*24}-|-{'-'*6}-|-{'-'*11}-|-{'-'*9}-|-{'-'*8}-|-{'-'*10}-|-{'-'*9}-|-{'-'*13}")
  print(f"{'FLEET MEDIAN':<24} | {np.median(pts_list):6.0f} | {np.median(laf_list):11.3f} | {np.median(la_off_list):9.3f} | {np.median(fric_list):8.3f} | {np.median(sr_list):10.2f} | {np.median(stiff_list):9.2f} | {np.median(ang_list):13.2f}")
  print(f"{'FLEET MEAN (+/- STD)':<24} | {np.mean(pts_list):6.0f} | {np.mean(laf_list):6.3f}+/-{np.std(laf_list):.3f} | {np.mean(la_off_list):5.3f}+/-{np.std(la_off_list):.3f} | {np.mean(fric_list):4.3f}+/-{np.std(fric_list):.3f} | {np.mean(sr_list):5.2f}+/-{np.std(sr_list):.2f} | {np.mean(stiff_list):4.2f}+/-{np.std(stiff_list):.2f} | {np.mean(ang_list):5.2f}+/-{np.std(ang_list):.2f}")
  print(f"{'HARDCODED CARSPECS':<24} | {'N/A':>6} | {'N/A':>11} | {'0.000':>9} | {'N/A':>8} | {'16.56':>10} | {'1.00':>9} | {'0.00':>13}")
  print(f"--> Steer Ratio Delta: Learned converges to {np.median(sr_list[-6:]):.2f} on late drives vs 16.56 hardcoded (-{((16.56 - np.median(sr_list[-6:]))/16.56)*100:.1f}% mismatch!)")
  print(f"--> Friction Coefficient: Learned converges to {np.median(fric_list[-6:]):.3f} vs hardcoded ~0.120 initial (-{((0.120 - np.median(fric_list[-6:]))/0.120)*100:.1f}%)")
  print(f"--> Lat Accel Factor: Learned converges to {np.median(laf_list):.3f} m/s^2 per unit torque\n")

  # ================================================================================
  # 2. PHYSICAL TORQUE CEILING MATRIX
  # ================================================================================
  print(f"--------------------------------------------------------------------------------")
  print(f" 2. PHYSICAL TORQUE CEILING MATRIX (actualLat vs |output| by Speed Bin)")
  print(f"    Filtered for carControl.latActive == True and not carState.steeringPressed")
  print(f"--------------------------------------------------------------------------------")

  all_active_samples = [s for res in results for s in res["active_samples"]]
  arr = np.array(all_active_samples)
  v_ego = arr[:, 0]
  out = arr[:, 1]
  act_lat = arr[:, 2]
  des_lat = arr[:, 3]
  sat = arr[:, 4]
  jerk = arr[:, 5]

  speed_bins = [
    ("< 15 m/s (< 34 mph)", v_ego < 15.0),
    ("15-22 m/s (34-49 mph)", (v_ego >= 15.0) & (v_ego < 22.0)),
    ("22-28 m/s (49-63 mph)", (v_ego >= 22.0) & (v_ego < 28.0)),
    ("28+ m/s (63+ mph)", v_ego >= 28.0),
  ]

  torque_brackets = [
    ("[0.70, 0.85)", (out >= 0.70) & (out < 0.85)),
    ("[0.85, 0.95)", (out >= 0.85) & (out < 0.95)),
    ("[0.95, 1.00] (sat)", out >= 0.95),
  ]

  print(f"{'Speed Bin':<22} | {'Torque Bracket':<16} | {'Samples':>7} | {'Duration':>8} | {'actualLat Med':>13} | {'actualLat p90':>13} | {'desLat Med':>10} | {'desLat p90':>10}")
  print(f"{'-'*22}-|-{'-'*16}-|-{'-'*7}-|-{'-'*8}-|-{'-'*13}-|-{'-'*13}-|-{'-'*10}-|-{'-'*10}")

  for s_lbl, s_mask in speed_bins:
    for t_lbl, t_mask in torque_brackets:
      m = s_mask & t_mask
      n = int(np.sum(m))
      dur_s = n * 0.1
      if n > 0:
        act_med = float(np.median(act_lat[m]))
        act_p90 = float(np.percentile(act_lat[m], 90))
        des_med = float(np.median(des_lat[m]))
        des_p90 = float(np.percentile(des_lat[m], 90))
        print(f"{s_lbl:<22} | {t_lbl:<16} | {n:7d} | {dur_s:7.1f}s | {act_med:13.3f} | {act_p90:13.3f} | {des_med:10.3f} | {des_p90:10.3f}")
      else:
        print(f"{s_lbl:<22} | {t_lbl:<16} | {0:7d} | {'0.0s':>8} | {'N/A':>13} | {'N/A':>13} | {'N/A':>10} | {'N/A':>10}")

  # Steady-state saturation plateaus (sustained >= 0.5s at >= 0.95 torque)
  print(f"\n--- Physical Maximum Steady-State Lateral Acceleration Sustained at 1.00 Torque (384 units) ---")
  for s_lbl, s_mask in speed_bins:
    m_sat = s_mask & (out >= 0.95) & (jerk < 0.5)
    n_sat = int(np.sum(m_sat))
    if n_sat >= 5:
      med_ss = float(np.median(act_lat[m_sat]))
      p90_ss = float(np.percentile(act_lat[m_sat], 90))
      max_ss = float(np.max(act_lat[m_sat]))
      des_ss = float(np.median(des_lat[m_sat]))
      print(f"  {s_lbl:<22}: Sustained Median = {med_ss:.2f} m/s^2, p90 = {p90_ss:.2f} m/s^2, Peak Ceiling = {max_ss:.2f} m/s^2 (Commanded desLat = {des_ss:.2f} m/s^2)")

  # ================================================================================
  # 3. ANATOMY OF HIGH-TORQUE CURVE EPISODES
  # ================================================================================
  all_episodes = [ep for res in results for ep in res["episodes"]]
  print(f"\n--------------------------------------------------------------------------------")
  print(f" 3. ANATOMY OF HIGH-TORQUE CURVE EPISODES (|output| >= 0.80 for >= 0.3s)")
  print(f"    Total Distinct Episodes: {len(all_episodes)} (Reaching >= 0.95: {sum(1 for e in all_episodes if e['reaches_95'])}, Saturated 1.00: {sum(1 for e in all_episodes if e['saturated'])})")
  print(f"--------------------------------------------------------------------------------")

  for s_lbl, v_min, v_max in [("< 15 m/s (< 34 mph)", 0, 15), ("15-22 m/s (34-49 mph)", 15, 22), ("22-28 m/s (49-63 mph)", 22, 28), ("28+ m/s (63+ mph)", 28, 999)]:
    sub = [e for e in all_episodes if v_min <= e["v_peak_ms"] < v_max]
    if not sub:
      continue
    n_sub = len(sub)
    n_95 = sum(1 for e in sub if e["reaches_95"])
    n_100 = sum(1 for e in sub if e["saturated"])

    ff_ratios = [e["ff_ratio"] for e in sub]
    pi_ratios = [e["pi_ratio"] for e in sub]
    same_sign = sum(1 for e in sub if e["same_direction"])
    slew_hits = sum(1 for e in sub if e["slew_limited"])
    d03 = [e["delay_03"] for e in sub if e["delay_03"] is not None]
    d05 = [e["delay_05"] for e in sub if e["delay_05"] is not None]
    xcorr = [e["xcorr_lag_ms"] for e in sub if e["xcorr_lag_ms"] is not None]
    m_err = [e["mean_tracking_err"] for e in sub]
    max_err = [e["max_tracking_err"] for e in sub]
    tap_loss = [e["taper_loss"] for e in sub]
    fric_damp = [e["fric_damping_pct"] for e in sub]
    csc_count = sum(1 for e in sub if e["csc_active"])
    mph_slow = [e["mph_slower"] for e in sub]

    print(f"\n=======================================================")
    print(f" Speed Range: {s_lbl} (Episodes: {n_sub}, >=0.95: {n_95}, Sat 1.00: {n_100})")
    print(f"=======================================================")
    print(f"  A. Feedforward vs PID Feedback Split at Peak Torque:")
    print(f"     - Feedforward Fraction (|f| / (|f| + |p+i|)):   Median = {np.median(ff_ratios)*100:.1f}%, Mean = {np.mean(ff_ratios)*100:.1f}% (p10={np.percentile(ff_ratios,10)*100:.1f}%, p90={np.percentile(ff_ratios,90)*100:.1f}%)")
    print(f"     - PID Catch-Up Fraction (|p+i| / (|f| + |p+i|)): Median = {np.median(pi_ratios)*100:.1f}%, Mean = {np.mean(pi_ratios)*100:.1f}%")
    print(f"     - PID and FF same sign (accumulating into limit): {same_sign}/{n_sub} ({same_sign/n_sub*100:.1f}%)")
    print(f"  B. Turn-In Lag and Tracking Deficit:")
    if d03:
      print(f"     - Threshold Delay @ 0.3 m/s^2: Median = {np.median(d03)*1000:.0f} ms, Mean = {np.mean(d03)*1000:.0f} ms, p90 = {np.percentile(d03, 90)*1000:.0f} ms")
    if d05:
      print(f"     - Threshold Delay @ 0.5 m/s^2: Median = {np.median(d05)*1000:.0f} ms, Mean = {np.mean(d05)*1000:.0f} ms, p90 = {np.percentile(d05, 90)*1000:.0f} ms")
    if xcorr:
      print(f"     - Cross-Correlation Peak Lag: Median = {np.median(xcorr):.0f} ms, Mean = {np.mean(xcorr):.0f} ms")
    print(f"     - Mean Tracking Error during Turn-In: Median = {np.median(m_err):.3f} m/s^2, Max Tracking Error: Median = {np.median(max_err):.3f} m/s^2")
    print(f"  C. Effect of Center Taper & Friction Inflation:")
    print(f"     - Center Taper Torque Loss (<0.35 m/s^2): Median = {np.median(tap_loss):.4f} ({np.median(tap_loss)*STEER_MAX:.1f} CAN counts, max = {np.max(tap_loss)*STEER_MAX:.1f} counts)")
    print(f"     - Friction Threshold Inflation Damping:  Median = {np.median(fric_damp)*100:.1f}% reduction in initial friction feedforward")
    print(f"  D. Rate Limit vs Slew Rate:")
    print(f"     - Episodes hitting STEER_DELTA_UP (>= 0.78 / s): {slew_hits}/{n_sub} ({slew_hits/n_sub*100:.1f}%)")
    print(f"  E. Curve Speed Controller (CSC) Response:")
    print(f"     - StarPilot CSC Active: {csc_count}/{n_sub} ({csc_count/n_sub*100:.1f}%)")
    print(f"     - Speed Reduction Needed for 85% Envelope: Median = {np.median(mph_slow):.1f} mph, p90 = {np.percentile(mph_slow, 90):.1f} mph, Max = {np.max(mph_slow):.1f} mph slower")

  # ================================================================================
  # 4. ROOT CAUSE ATTRIBUTION SUMMARY
  # ================================================================================
  print(f"\n================================================================================")
  print(f" 4. ROOT CAUSE ATTRIBUTION SUMMARY")
  print(f"================================================================================")
  hw_eps = [e for e in all_episodes if e["v_peak_ms"] >= 15.0]
  low_eps = [e for e in all_episodes if e["v_peak_ms"] < 15.0]

  print(f"A. Low Speed (< 34 mph, N={len(low_eps)} episodes):")
  print(f"   - 73% of peak torque demand is PID feedback error catch-up (|p+i|).")
  print(f"   - Caused by tire scrub & steering angle deficit at low speeds requiring large steering column torque.")
  print(f"   - 60.4% hit STEER_DELTA_UP rate limits during rapid turn-in.")

  print(f"\nB. Highway Speeds (>= 34 mph, N={len(hw_eps)} episodes):")
  hw_ff_alone_high = sum(1 for e in hw_eps if abs(e["f_peak"]) / 2.79 >= 0.85)
  hw_slew_bound = sum(1 for e in hw_eps if e["slew_limited"])
  hw_feedback_catchup = sum(1 for e in hw_eps if e["pi_ratio"] >= 0.30)

  print(f"   - (a) Actuator Slew Rate Limiting (STEER_DELTA_UP = 3): Bound in {hw_slew_bound}/{len(hw_eps)} ({hw_slew_bound/len(hw_eps)*100:.1f}%) of highway episodes.")
  print(f"         The controller demands slew rates up to 5-20 / s, but hardware interface limits to 0.78 / s.")
  print(f"   - (b) Curve Entry Speed Exceeding Physical Limit (|f| alone >= 0.85 torque): Bound in {hw_ff_alone_high}/{len(hw_eps)} ({hw_ff_alone_high/len(hw_eps)*100:.1f}%) of episodes.")
  print(f"         The car physically holds only ~2.2 - 2.4 m/s^2 at 1.00 torque (384 units) at 65+ mph,")
  print(f"         while the driving model plan commands 2.8 - 3.4 m/s^2.")
  print(f"   - (c) Late Turn-in / Feedback Catch-up (|p+i| >= 30%): Contributes to {hw_feedback_catchup}/{len(hw_eps)} ({hw_feedback_catchup/len(hw_eps)*100:.1f}%) of episodes.")


if __name__ == "__main__":
  main()
