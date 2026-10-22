#!/usr/bin/env python3
"""Closed-loop longitudinal simulator for the Kia Stinger 2022 on StarPilot: approach to a stopped lead (baseline for the "stopped-lead" tune).

Reuses openpilot's own selfdrive/test/longitudinal_maneuvers/plant.py (Plant) from a built StarPilot checkout (default
$STINGER_DEV_HOME/x86, only imported, never checked out or written) and subclasses it from outside: the REAL LongitudinalPlanner +
acados MPC at DT_MDL (20 Hz), the REAL LongControl at 100 Hz, and (optionally) the real Hyundai car-side tune shaping, in a loop
with a simple car model. What differs from the stock Plant, and why (each is a known gap of Plant that changes the answer):
  CarParams       the Stinger's (opendbc hyundai get_non_essential_params + what the logged carParams has: openpilotLongitudinalControl,
                  radarUnavailable False, pcmCruise False), not HONDA_CIVIC. --delay sets CP.longitudinalActuatorDelay (the planner's
                  look-ahead: 0.5 on tune B/C, 0.15 on tune A in the logs).
  starpilotPlan   Plant's stub has accelerationJerk/dangerJerk/speedJerk 5/5/5, dangerFactor 1.0, tFollow 1.45. The logs have (qlogs and
                  rlogs, trackingLead, city speeds): Standard tFollow 1.45, jerk 250/100/5.5 (A_CHANGE_COST/DANGER_ZONE_COST/J_EGO_COST x
                  personality factor 1), minAcceleration -0.5; Aggressive tFollow 1.25, 125/100/2.75 (factors 0.5/1/0.5), minAcceleration -0.5
                  (routes 0d/0f) or -2.0 (routes 11/12); dangerFactor 0.75 (LEAD_DANGER_FACTOR); maxAcceleration ~1.9 at 5 mph .. 1.4 at
                  25 mph. trackingLead is StarPilot's own should_track_lead/hold logic, with the radar flag. Overridable (--t-follow,
                  --jerks, --min-accel).
  lead            radarState.leadOne.radar = True and modelProb 1.0 (--vision: radar False, modelProb 0.9), so the radar paths run and the
                  vision-only caps do not; leadTwo = leadOne as in the stock Plant and in the logs (radarState.leadTwo is identical to
                  leadOne there; with an empty leadTwo the MPC's shared lead filter is pumped with a fake lead and the stopped lead
                  looks like it moves: --single-lead). modelV2.leadsV3[0] carries the lead (prob 1) so the MPC's model-lead path exists.
                  --detect-dist: the lead is only reported (status True) once the gap is below it.
  clock           time.monotonic() inside the planner is replaced by the sim clock (the planner's hold timers otherwise run in wall time).
  LongControl     the real LongControl.update at 100 Hz on the plan (aTarget, shouldStop; state machine, stopping ramp to stopAccel -2.0 at
                  stoppingDecelRate 0.8, kp = ki = 0 on this car), toggles vEgoStopping 0.5 / vEgoStarting 0.5 / stopAccel -2.0.
  car side        --car-tune A: accel passes (clipped to +-3.5). B: shape_hyundai_canfd_scc_accel with the Stinger flag at 50 Hz, lead
                  distance for the lower jerk. C: Ioniq 6 smoothing (update_ioniq_6_longitudinal_tuning at 20 Hz, Ioniq defaults).
  car response    the sent accel -> pure delay 0.20 s -> first-order lag tau 0.30 s -> aEgo (analysis/accel_lag.py, measured on this car);
                  speed and position are integrated from that. Independent of --delay (the planner's belief), on purpose.
  model           synthetic like Plant (--model plain), or --model stop: the model path ends --model-gap m short of the lead while closing.
Not modelled: experimental/blended mode (Conditional Experimental Mode flips it on during real stops; the synthetic model cannot drive
it, so the sim always runs 'acc'), radar noise and tracking latency, Force Stop / red-light logic, grade, the driver.

Subcommands (all print markdown):
  scenarios   approach a stopped car from 15..35 mph: lead visible from far (120 m) and late detection at 60/40/30 m; per-run metrics table,
              --csv/--series write them to files. Options: --personality, --delay, --car-tune (comma lists give the full grid; 240 runs
              in about 40 s with --jobs 4)
  run         one scenario, with the time series printed (--v0-mph, --gap, --detect-dist, ...); --trace names the planner line that set
              output_a_target at each step (the binding rule)
  validate    sim vs the real approaches in the logs, started from the logged conditions at the moment the stopped lead is first tracked
              (ego speed and accel, gap, set speed, personality, delay, tune, min accel, tFollow) with the logged radar lead track (vLead,
              aLeadK; the lead's physical motion = logged gap + ego displacement). --list shows the named examples, --scan ROUTE... finds
              every clean stopped-lead approach (>= --min-mph at the acquisition, lead < 1.5 m/s, longActive, no pedal, no Force Stop) in
              the routes, --variants adds sensitivity runs, --ideal-lead feeds a stationary lead instead of the logged track
  openloop    planner-only check: the sim's planner on the LOGGED car state/lead/set speed vs the logged aTarget (separates wrongly built
              planner inputs from closed-loop divergence)
Metrics per run (same definitions for the real logs, there from carState.aEgo / radarState.leadOne):
  onset      first applied accel < -0.3: gap, speed, time since the lead became visible
  peak       most negative applied accel, its gap, and the fraction of the way from onset gap to final gap
  a@v        applied decel when speed first falls through 20/15/10/5/2 mph
  creep      time and distance from the first time below 3 mph until the stop; gap@6mph = gap at first below 6 mph; final gap; min gap
  jerk       max |d(applied accel)/dt| (20 Hz difference of the car's accel, until the last metre before the stop), and the hardest brake-in jerk
  contact    a run whose gap reaches 0.3 m is flagged CONTACT with the impact speed (no AEB or driver in the sim)

Usage: sim_stopped_lead.py scenarios [--personality standard,aggressive] [--delay 0.15,0.5] [--car-tune A,B,C] [--speeds-mph 15,20,25,30,35]
                           [--detect 0,60,40,30] [--vision] [--csv out.csv] [--series DIR] [--jobs 4]
       sim_stopped_lead.py run --v0-mph 25 --gap 120 [--detect-dist 40] [--personality aggressive] [--delay 0.5] [--car-tune B] [--print-series] [--trace]
       sim_stopped_lead.py validate [--list] [--only B-11-1219] [--scan ROUTE ...] [--min-mph 15] [--variants] [--series DIR]
       sim_stopped_lead.py openloop ROUTE T0 T1
Stopped-lead tune: --stopped-lead-tune A|B stubs Testing Ground slot 10 ("Stinger Stopped Lead", latched in LongitudinalPlanner.__init__; scenarios takes
A,B); --tune-param NAME=VAL,... overrides KIA_STINGER_2022_STOPPED_LEAD_<NAME>; --lead-filter primed|fast (MPC lead filter state: a fresh planner has rc 0.8 s
primed to v_ego+10, which delays the reaction to a late lead; after a drive above ~50 mph it is rc 0 = fast); --exp-no-close-cap is an experiment (see
analysis/stopped_lead_results.md). A/B tables: stopped_lead_ab.py on a --series directory.
Shared model options: --vision, --single-lead, --model plain|stop [--model-gap M], --model-lead const|oracle|off, --min-accel, --t-follow, --jerks.
Needs the venv with StarPilot's built x86 tree (see README); the logs for validate (comma_dl.py).
"""
import argparse
import itertools
import os
import sys
import time as _time
import types
from unittest import mock
from concurrent.futures import ProcessPoolExecutor
from types import SimpleNamespace

import numpy as np

sys.dont_write_bytecode = True

DEV = os.environ.get("STINGER_DEV_HOME", os.path.expanduser("~/stinger/starpilot-dev"))
BUILD = os.environ.get("STARPILOT_BUILD", os.path.join(DEV, "x86"))
HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (BUILD, os.path.join(BUILD, "opendbc_repo"), HERE):
  if _p not in sys.path:
    sys.path.insert(0, _p)

MPH = 2.23694
DT_PLAN = 0.05           # DT_MDL
DT_CTRL = 0.01
SUBSTEPS = 5

# ------------------------------------------------------------------ logged StarPilot configurations (see docstring)
PERSONALITIES = {
  # name: (personality id, tFollow at city speed, (accelerationJerk, dangerJerk, speedJerk), minAcceleration)
  "standard": (1, 1.45, (250.0, 100.0, 5.5), -0.5),
  "relaxed": (2, 1.6, (250.0, 100.0, 5.5), -0.5),
  "aggressive": (0, 1.25, (125.0, 100.0, 2.75), -2.0),     # routes 11/12 (min accel -2.0; routes 0d/0f logged -0.5)
  "aggressive-min05": (0, 1.25, (125.0, 100.0, 2.75), -0.5),
}
MAX_ACCEL_BP = np.array([0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 35.0]) / MPH       # mph -> m/s, logged maxAcceleration (route 11 seg 20)
MAX_ACCEL_V = np.array([1.95, 1.90, 1.78, 1.62, 1.55, 1.47, 1.30])
DANGER_FACTOR = 0.75
ACTUATOR_DELAY_S = 0.20   # accel_lag.py on this car
ACTUATOR_TAU_S = 0.30


def _import_stack():
  """Import the StarPilot modules (needs the built x86 tree on sys.path, done above)."""
  global Plant, LongitudinalPlanner, LongControl, LongCtrlState, CarInterface, CAR, lp_mod, ACCEL_MIN, ACCEL_MAX
  global should_track_lead, should_hold_tracked_vision_lead, THRESHOLD, STOP_DISTANCE, _LEAD_ACCEL_TAU
  global log, messaging, ModelConstants, tunes, ccm
  from cereal import log as _log
  import cereal.messaging as _messaging
  from opendbc.car.hyundai.interface import CarInterface as _CI
  from opendbc.car.hyundai.values import CAR as _CAR
  from opendbc.car.interfaces import ACCEL_MAX as _AMAX, ACCEL_MIN as _AMIN
  from openpilot.selfdrive.controls.lib.lead_behavior import should_hold_tracked_vision_lead as _hold, should_track_lead as _track
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl as _LC, LongCtrlState as _LCS
  from openpilot.selfdrive.controls.lib import longitudinal_planner as _lp
  from openpilot.selfdrive.controls.lib import longitudinal_vehicle_tunes as _tunes
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import STOP_DISTANCE as _SD
  from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU as _TAU
  from openpilot.selfdrive.modeld.constants import ModelConstants as _MC
  from openpilot.selfdrive.test.longitudinal_maneuvers.plant import Plant as _Plant
  from openpilot.starpilot.common.starpilot_variables import THRESHOLD as _TH
  import opendbc.car.hyundai.carcontroller as _ccm
  Plant, LongitudinalPlanner, LongControl, LongCtrlState = _Plant, _lp.LongitudinalPlanner, _LC, _LCS
  CarInterface, CAR, lp_mod, ACCEL_MIN, ACCEL_MAX = _CI, _CAR, _lp, _AMIN, _AMAX
  should_track_lead, should_hold_tracked_vision_lead, THRESHOLD = _track, _hold, _TH
  STOP_DISTANCE, _LEAD_ACCEL_TAU = _SD, _TAU
  log, messaging, ModelConstants, tunes, ccm = _log, _messaging, _MC, _tunes, _ccm


class SimClock:
  """time.monotonic() replacement for the planner module."""
  def __init__(self):
    self.t = 1000.0

  def monotonic(self):
    return self.t

  time = monotonic

  def sleep(self, s):
    pass


def stinger_cp(delay):
  CP = CarInterface.get_non_essential_params(CAR.KIA_STINGER_2022)
  CP.longitudinalActuatorDelay = float(delay)
  # what the logged carParams has and get_non_essential_params does not (checked against 00000011 seg 20)
  CP.openpilotLongitudinalControl = True
  CP.radarUnavailable = False
  CP.pcmCruise = False
  CP.enableBsm = True
  return CP


def stinger_toggles(delay):
  """The few toggles the planner and LongControl read (values from the logged starpilotToggles of 00000011 seg 20)."""
  return SimpleNamespace(
    taco_tune=False, classic_model=False, tinygrad_model=True, model_version="v11", longitudinalActuatorDelay=float(delay),
    vEgoStopping=0.5, vEgoStarting=0.5, stopAccel=-2.0, startAccel=0.0, stoppingDecelRate=0.8, max_desired_acceleration=4.0,
    custom_accel_profile=False, radar_takeoffs=True, force_stop_distance_offset=0, lane_change_close_gap=False,
    minimum_lane_change_speed=8.9408)


# ------------------------------------------------------------------ car side: tune shaping + measured actuator response
class CarSide:
  """Sent accel (tune A/B/C shaping) -> delay -> first-order lag -> a_ego. Ticks at 100 Hz."""

  def __init__(self, CP, tune, a0=0.0, delay=ACTUATOR_DELAY_S, tau=ACTUATOR_TAU_S):
    self.CP, self.tune = CP, tune
    self.n_delay = max(1, int(round(delay / DT_CTRL)))
    self.alpha = 1.0 - np.exp(-DT_CTRL / tau)
    self.hist = [a0] * self.n_delay          # sent accel history at 100 Hz
    self.a_act = a0
    self.sent = a0
    self.accel_last = a0
    self.tick_n = 0
    self.c_state = ccm.Ioniq6LongitudinalTuningState() if tune == "C" else None
    if self.c_state is not None:
      self.c_state.accel_last = a0
      self.c_state.actual_accel = a0

  def shape(self, accel_cmd, v_ego, a_ego, lcs, lead_visible, lead_dist):
    stopping = lcs == LongCtrlState.stopping
    accel_cmd = float(np.clip(accel_cmd, -3.5, 3.5))
    self.tick_n += 1
    if self.tune == "B":
      if self.tick_n % 2 == 0:       # 50 Hz
        kw = {"kia_stinger_2022_tune": True, "jerk_upper": 3.0, "lead_visible": lead_visible, "lead_distance": lead_dist}
        self.sent = tunes.shape_hyundai_canfd_scc_accel(self.CP, True, False, stopping, accel_cmd, self.accel_last, **kw)
        self.accel_last = self.sent
      return self.sent
    if self.tune == "C":
      st = self.c_state
      use = lcs in (LongCtrlState.starting, LongCtrlState.pid, LongCtrlState.stopping)
      if self.tick_n % 5 == 0 and use:
        self.c_state = st = ccm.update_ioniq_6_longitudinal_tuning(st, accel_cmd, v_ego, a_ego, lcs, True)
      smoothed = use and (accel_cmd >= st.actual_accel or st.launch_active or st.stopping)
      if self.tick_n % 2 == 0:
        if smoothed:
          self.sent = st.actual_accel
        else:
          step = ccm.get_canfd_scc_decel_step(self.CP)
          self.sent = float(np.clip(accel_cmd, self.accel_last - step, self.accel_last + ccm.IONIQ_6_CANFD_SCC_ACCEL_STEP))
          if use:
            st.desired_accel, st.actual_accel, st.accel_last = accel_cmd, self.sent, self.sent
            st.jerk_upper, st.jerk_lower, st.launch_active, st.stopping = 3.0, 5.0, False, stopping
            st.long_control_state_last = lcs
        self.accel_last = self.sent
      return self.sent
    self.sent = accel_cmd                  # tune A: straight through
    return self.sent

  def tick(self, sent):
    self.hist.append(sent)
    delayed = self.hist.pop(0)
    self.a_act += self.alpha * (delayed - self.a_act)
    return self.a_act


# ------------------------------------------------------------------ the plant
def make_plant_class():
  """Plant subclass with the Stinger CP, StarPilot-like starpilotPlan, real LongControl and the measured car response (built after the
  openpilot imports, which need the x86 tree)."""
  class _StingerPlant(Plant):
    def __init__(self, *, v0, gap0, set_speed, personality="standard", delay=0.5, car_tune="A", radar=True, detect_dist=None,
                 t_follow=None, jerks=None, min_accel=None, a0=0.0, model="plain", model_gap=None, two_leads=False, exp_mode=False,
                 trace=False, model_lead="const", model_lead_range=80.0, stopped_lead_tune="A", lead_filter="primed", tune_params=None, exp_no_close_cap=False, **_):
      pid, tf, jk, mn = PERSONALITIES[personality]
      # Plant.__init__ opens msgq sockets that nothing here reads (and that break parallel runs): mark them as initialised and
      # stub the SubMaster, so only the bookkeeping of Plant (clock, filters, ratekeeper) is reused.
      Plant.messaging_initialized, Plant.messaging_prefix = True, os.environ.get("OPENPILOT_PREFIX")
      with mock.patch.object(messaging, "SubMaster", lambda *a, **k: None):
        super().__init__(lead_relevancy=True, speed=v0, distance_lead=gap0, enabled=True, only_radar=radar,
                         track_lead_with_gate=True, e2e=exp_mode, personality=pid)
      self.clock = SimClock()
      lp_mod.time = types.SimpleNamespace(monotonic=self.clock.monotonic, time=self.clock.time, sleep=_time.sleep)
      self.CP = stinger_cp(delay)
      self.toggles = stinger_toggles(delay)
      # Testing Ground slot 10 ("Stinger Stopped Lead") is latched in LongitudinalPlanner.__init__ via testing_ground.use("10", "B"):
      # stub the module-level name for the construction only (other slots fall through to the real, inactive, state).
      for k, val in (tune_params or {}).items():     # e.g. TARGET_DECEL=1.3: overrides KIA_STINGER_2022_STOPPED_LEAD_<k> for this process
        assert hasattr(tunes, "KIA_STINGER_2022_STOPPED_LEAD_" + k), k
        setattr(tunes, "KIA_STINGER_2022_STOPPED_LEAD_" + k, float(val))
      real_tg = lp_mod.testing_ground
      lp_mod.testing_ground = SimpleNamespace(use=lambda slot, variant="B": (str(slot) == "10" and str(variant) == "B" and stopped_lead_tune == "B")
                                              or (str(slot) != "10" and real_tg.use(slot, variant)))
      try:
        self.planner = LongitudinalPlanner(self.CP, init_v=v0, init_a=a0)
      finally:
        lp_mod.testing_ground = real_tg
      if exp_no_close_cap:    # EXPERIMENT (proposal only, not in the planner): drop the generic close-lead brake push for stopped leads
        orig_cap = self.planner.get_close_lead_brake_cap
        self.planner.get_close_lead_brake_cap = lambda lead, v, am: None if float(lead.vLead) < 1.0 else orig_cap(lead, v, am)
      assert self.planner.stinger_stopped_lead_tune == (stopped_lead_tune == "B")
      self.LoC = LongControl(self.CP)
      self.LoC.experimental_mode = exp_mode
      self.LoC.last_output_accel = a0
      self.car = CarSide(self.CP, car_tune, a0)
      self.pers = SimpleNamespace(t_follow=tf if t_follow is None else t_follow, jerks=jk if jerks is None else jerks,
                                  min_accel=mn if min_accel is None else min_accel)
      self.radar, self.detect_dist, self.set_speed = radar, detect_dist, set_speed
      self.acceleration = a0
      self.a_ego = a0
      self.model, self.model_gap, self.two_leads, self.trace = model, model_gap, two_leads, trace
      self.model_lead, self.model_lead_range, self.lead_track = model_lead, model_lead_range, None
      # Without an active lead the MPC feeds a fake fast lead (v_ego + 10) through its lead_v filter (rc 0.8 s), so a lead that appears
      # later is seen decaying from that value; the real planner has been in this state before every acquisition.
      self.planner.mpc.lead_v_filter.x = v0 + 10.0
      if lead_filter == "fast":
        # long_mpc recreates lead_a/v_filter with rc = 0 once the car has been below 47 mph after having been above ~51 mph
        # (current_filter_time), i.e. in any drive that has seen a highway; the fresh planner keeps rc 0.8 s from __init__.
        self.planner.mpc.lead_v_filter.update_alpha(0.0)
        self.planner.mpc.lead_a_filter.update_alpha(0.0)

    def update_planner(self, sm):
      """planner.update; with trace=True also returns the source line of the last change and of the biggest drop of output_a_target."""
      if not self.trace:
        self.planner.update(sm, self.toggles)
        return ""
      code = LongitudinalPlanner.update.__code__
      state, changes = {}, []

      def local_trace(frame, event, arg):
        v = frame.f_locals.get("output_a_target")
        if event == "line":
          if state.get("prev") is not None and v is not None and v != state.get("last"):
            changes.append((state["prev"], float(v)))
            state["last"] = v
          state["prev"] = frame.f_lineno
        elif event == "return" and v is not None and v != state.get("last"):
          changes.append((state["prev"], float(v)))
        return local_trace

      sys.settrace(lambda frame, event, arg: local_trace if frame.f_code is code else None)
      try:
        self.planner.update(sm, self.toggles)
      finally:
        sys.settrace(None)
      if not hasattr(self, "_src"):
        self._src = open(lp_mod.__file__).read().splitlines()
      txt = lambda ln: self._src[ln - 1].strip()[:60] if ln else "-"
      drops = [(changes[k][1] - (changes[k - 1][1] if k else changes[k][1]), changes[k][0]) for k in range(len(changes))]
      big = min(drops)[1] if drops and min(drops)[0] < -1e-3 else 0
      return f"final L{changes[-1][0]}: {txt(changes[-1][0])} | drop L{big}: {txt(big)}" if changes else ""

    def step(self, v_lead=0.0, prob_lead=1.0, v_cruise=None, pitch=0.0, prob_throttle=1.0, lead_obs=None):
      """lead_obs = (vLead, aLeadK) as the radar reports them when they differ from the lead's true motion (logged tracks)."""
      v_cruise = self.set_speed if v_cruise is None else v_cruise
      d_rel = max(0.0, self.distance_lead - self.distance)
      visible = self.detect_dist is None or d_rel < self.detect_dist
      a_lead = (v_lead - self.v_lead_prev) / DT_PLAN
      self.v_lead_prev = v_lead
      v_obs, a_obs = (v_lead, a_lead) if lead_obs is None else lead_obs

      radar = messaging.new_message("radarState")
      car_state = messaging.new_message("carState")
      lp_ = messaging.new_message("liveParameters")
      car_control = messaging.new_message("carControl")
      model = messaging.new_message("modelV2")
      control = messaging.new_message("controlsState")
      ss = messaging.new_message("selfdriveState")

      lead = log.RadarState.LeadData.new_message()
      if visible:
        lead.dRel = float(d_rel)
        lead.yRel = 0.0
        lead.vRel = float(v_obs - self.speed)
        lead.aRel = float(a_obs - self.a_ego)
        lead.vLead = float(v_obs)
        lead.vLeadK = float(v_obs)
        lead.aLeadK = float(a_obs)
        lead.aLeadTau = float(_LEAD_ACCEL_TAU)
        lead.status = True
        lead.radar = bool(self.radar)
        lead.modelProb = 1.0 if self.radar else 0.9
      else:
        lead.dRel = 200.0
      radar.radarState.leadOne = lead
      if self.two_leads and visible:
        radar.radarState.leadTwo = lead

      # synthetic model: straight ahead; --model stop ends its path short of the lead while closing
      t_idx = np.array(ModelConstants.T_IDXS)
      xs = (self.speed + 0.5) * t_idx
      if self.model == "stop" and visible and self.model_gap is not None and v_obs < self.speed:
        xs = np.minimum(xs, max(0.0, d_rel - self.model_gap))
      position = log.XYZTData.new_message()
      position.x = [float(x) for x in xs]
      model.modelV2.position = position
      model.modelV2.action.desiredAcceleration = float(self.a_ego + 0.1)
      velocity = log.XYZTData.new_message()
      velocity.x = [float(x) for x in (self.speed + 0.5) * np.ones_like(t_idx)]
      velocity.x[0] = float(self.speed)
      model.modelV2.velocity = velocity
      acceleration = log.XYZTData.new_message()
      acceleration.x = [float(x) for x in np.zeros_like(t_idx)]
      model.modelV2.acceleration = acceleration
      model.modelV2.meta.disengagePredictions.gasPressProbs = [float(prob_throttle) for _ in range(6)]
      if self.model_lead != "off" and visible and d_rel <= self.model_lead_range:
        # the vision model's lead prediction (leadsV3[0]): the MPC builds the lead path from it when the lead is not braking hard and
        # the closing TTC is long; otherwise it falls back to the radar-only path (see long_mpc.build_model_lead_trajectory)
        lt = np.array(ModelConstants.LEAD_T_IDXS)
        if self.model_lead == "oracle" and self.lead_track is not None:
          # a perfect model: the lead's logged future (position relative to the ego now, speed)
          tt = self.current_time + lt
          lx = np.interp(tt, self.lead_track[0], self.lead_track[1]) - self.distance
          lv = np.gradient(np.interp(tt, self.lead_track[0], self.lead_track[1]), tt)
          lv[0] = v_obs
        else:      # constant lead speed
          lx = d_rel + v_obs * lt
          lv = np.full(len(lt), max(v_obs, 0.0))
        ml = model.modelV2.init("leadsV3", 3)[0]
        ml.prob = 1.0
        ml.probTime = 0.0
        ml.t = [float(x) for x in lt]
        ml.x = [float(x) for x in lx]
        ml.v = [float(max(x, 0.0)) for x in lv]
        ml.a = [0.0 for _ in lt]
        ml.y = [0.0 for _ in lt]

      control.controlsState.longControlState = LongCtrlState.pid
      ss.selfdriveState.experimentalMode = self.e2e
      ss.selfdriveState.personality = self.personality
      car_state.carState.vEgo = float(self.speed)
      car_state.carState.aEgo = float(self.a_ego)
      car_state.carState.standstill = bool(self.speed < 0.01)
      car_state.carState.vCruise = float(v_cruise * 3.6)
      car_control.carControl.orientationNED = [0.0, float(pitch), 0.0]

      # StarPilot's tracking_lead (starpilot_planner.update_lead_status), with the radar flag
      model_len = float(position.x[-1])
      cand = should_track_lead(visible, float(d_rel), model_len, STOP_DISTANCE, float(self.speed), v_lead=float(v_obs),
                               radar=bool(self.radar))
      cont = self.tracking_lead_filter.x >= THRESHOLD * 0.6
      if not cand and cont:
        cand = should_hold_tracked_vision_lead(visible, float(d_rel), model_len, STOP_DISTANCE, float(self.speed),
                                              model_prob=float(lead.modelProb), y_rel=0.0, radar=bool(self.radar))
      self.tracking_lead_filter.update(cand)
      tracking_lead = self.tracking_lead_filter.x >= THRESHOLD

      max_accel = float(np.interp(self.speed, MAX_ACCEL_BP, MAX_ACCEL_V))
      starpilot_plan = SimpleNamespace(
        vCruise=float(v_cruise), minAcceleration=float(self.pers.min_accel), maxAcceleration=max_accel,
        cscControllingSpeed=False, disableThrottle=False,
        accelerationJerk=float(self.pers.jerks[0]), dangerJerk=float(self.pers.jerks[1]), speedJerk=float(self.pers.jerks[2]),
        trackingLead=bool(tracking_lead), desiredFollowDistance=float(d_rel) if visible else 0.0, dangerFactor=DANGER_FACTOR,
        tFollow=float(self.pers.t_follow), forcingStop=False, forcingStopLength=2, redLight=False, stopSignConfirmed=False)
      sm = {"radarState": radar.radarState, "carState": car_state.carState, "carControl": car_control.carControl,
            "controlsState": control.controlsState, "selfdriveState": ss.selfdriveState, "liveParameters": lp_.liveParameters,
            "modelV2": model.modelV2, "starpilotPlan": starpilot_plan}
      bind = self.update_planner(sm)
      a_target = float(self.planner.output_a_target)
      should_stop = bool(self.planner.output_should_stop)

      # ---- 100 Hz: LongControl, car-side tune, actuator response, kinematics
      cs = SimpleNamespace(vEgo=0.0, aEgo=0.0, brakePressed=False, cruiseState=SimpleNamespace(standstill=False))
      leads = (radar.radarState.leadOne, radar.radarState.leadTwo)
      for _ in range(SUBSTEPS):
        cs.vEgo, cs.aEgo = float(self.speed), float(self.a_ego)
        cmd = self.LoC.update(True, cs, a_target, should_stop, (ACCEL_MIN, ACCEL_MAX), self.toggles, has_lead=bool(visible),
                              profile_max_accel=max_accel, leads=leads)
        cmd = float(min(cmd, self.toggles.max_desired_acceleration))
        sent = self.car.shape(cmd, self.speed, self.a_ego, self.LoC.long_control_state, bool(visible), float(d_rel))
        a = self.car.tick(sent)
        v_new = self.speed + a * DT_CTRL
        if v_new <= 0.0:
          v_new, a = 0.0, 0.0
          self.car.a_act = 0.0
        self.speed = v_new
        self.a_ego = a
        self.distance += self.speed * DT_CTRL
        self.distance_lead += v_lead * DT_CTRL
        self.clock.t += DT_CTRL
      self.rk.monitor_time()
      return {"t": self.current_time, "gap": max(0.0, self.distance_lead - self.distance), "v": self.speed, "a_target": a_target,
              "cmd": cmd, "sent": sent, "a": self.a_ego, "should_stop": should_stop, "visible": visible,
              "tracking": bool(tracking_lead), "src": self.planner.mpc.source, "state": str(self.LoC.long_control_state),
              "bind": bind}

  return _StingerPlant


# ------------------------------------------------------------------ metrics
PROFILE_MPH = (20, 15, 10, 5, 2)


def metrics(t, gap, v, a, jerk_a=None, v_stop=0.05):
  """Approach metrics on arrays that start when the lead becomes visible. a = applied (car) accel, positive = accelerating."""
  t, gap, v, a = (np.asarray(x, float) for x in (t, gap, v, a))
  m = {}
  stop_idx = np.where(v < v_stop)[0]
  end = int(stop_idx[0]) if len(stop_idx) else len(t) - 1
  m["stopped"] = bool(len(stop_idx))
  m["final_gap"] = float(gap[end]) if len(stop_idx) else np.nan
  m["min_gap"] = float(np.min(gap[:end + 1]))
  m["t_stop"] = float(t[end] - t[0]) if len(stop_idx) else np.nan
  idx = np.where(a[:end + 1] < -0.3)[0]
  i = int(idx[0]) if len(idx) else 0
  if len(idx):
    m["onset_gap"], m["onset_v_mph"], m["onset_t"] = float(gap[i]), float(v[i] * MPH), float(t[i] - t[0])
  else:
    m["onset_gap"] = m["onset_v_mph"] = m["onset_t"] = np.nan
  k = int(np.argmin(a[:end + 1]))
  m["peak_decel"], m["peak_gap"] = float(-a[k]), float(gap[k])
  denom = gap[i] - gap[end]
  m["peak_frac"] = float((gap[i] - gap[k]) / denom) if denom > 1.0 and len(idx) and len(stop_idx) else np.nan
  for mph in PROFILE_MPH:
    below = np.where((v[:end + 1] <= mph / MPH) & (np.arange(end + 1) >= i))[0]
    m[f"a@{mph}"] = float(-a[below[0]]) if len(below) and v[0] > mph / MPH else np.nan
  b6 = np.where(v[:end + 1] <= 6 / MPH)[0]
  m["v6_gap"] = float(gap[b6[0]]) if len(b6) and v[0] > 6 / MPH else np.nan
  b3 = np.where(v[:end + 1] <= 3 / MPH)[0]
  if len(b3) and len(stop_idx):
    j = int(b3[0])
    m["creep_t"], m["creep_d"] = float(t[end] - t[j]), float(gap[j] - gap[end])
  else:
    m["creep_t"] = m["creep_d"] = np.nan
  ja = a if jerk_a is None else np.asarray(jerk_a, float)
  moving = np.where(v[:end + 1] > 0.5)[0]            # the last metre before the standstill (a is forced to 0 there) is not jerk
  last = int(moving[-1]) if len(moving) else end
  dt = np.diff(t[:last + 1])
  if len(dt) > 2 and np.all(dt > 0):
    j = np.diff(ja[:last + 1]) / dt
    m["jerk_max"], m["jerk_brake"] = float(np.max(np.abs(j))), float(-np.min(j))
  else:
    m["jerk_max"] = m["jerk_brake"] = np.nan
  return m


# ------------------------------------------------------------------ scenario runner
def run_sim(v0_mph=25.0, gap0=120.0, detect_dist=None, personality="standard", delay=0.5, car_tune="A", radar=True, set_speed_mph=None,
            a0=0.0, v0=None, set_speed=None, max_t=120.0, lead_v=0.0, stop_hold=3.0, lead_track=None, **kw):
  v0 = v0_mph / MPH if v0 is None else v0
  if set_speed is None:
    set_speed = v0 if set_speed_mph is None else set_speed_mph / MPH
  if detect_dist is not None:
    gap0 = max(gap0, detect_dist + 4.0 * v0)
  plant = make_plant_class()(v0=v0, gap0=gap0, set_speed=set_speed, personality=personality, delay=delay, car_tune=car_tune, radar=radar,
                             detect_dist=detect_dist, a0=a0, **kw)
  plant.lead_track = lead_track
  rows, still = [], 0
  while plant.current_time < max_t:
    if lead_track is None:
      r = plant.step(lead_v)
    else:      # (t, lead position relative to the ego start, radar vLead, radar aLeadK) from a log
      tt = plant.current_time
      plant.distance_lead = float(np.interp(tt, lead_track[0], lead_track[1]))
      v_phys = float((np.interp(tt + DT_PLAN, lead_track[0], lead_track[1]) - plant.distance_lead) / DT_PLAN)
      r = plant.step(v_phys, lead_obs=(float(np.interp(tt, lead_track[0], lead_track[2])), float(np.interp(tt, lead_track[0], lead_track[3]))))
    rows.append(r)
    still = still + 1 if r["v"] < 0.02 else 0
    if still * DT_PLAN > stop_hold or r["gap"] < 0.3:
      break
  tab = {k: np.array([r[k] for r in rows]) for k in rows[0]}
  vis = np.where(tab["visible"])[0]
  i0 = int(vis[0]) if len(vis) else 0
  sl = {k: val[i0:] for k, val in tab.items()}
  m = metrics(sl["t"], sl["gap"], sl["v"], sl["a"])
  m["collision"] = bool(tab["gap"][-1] < 0.3)
  m["impact_mph"] = float(tab["v"][-1] * MPH) if m["collision"] else np.nan
  return tab, i0, m


def _job(kw):
  if "Plant" not in globals():
    _import_stack()
  tab, i0, m = run_sim(**kw)
  return kw, m, tab, i0


def fmt(x, p=1):
  return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{p}f}"


METRIC_COLS = [("onset_gap", "onset gap m", 0), ("onset_v_mph", "onset mph", 0), ("peak_decel", "peak decel", 2), ("peak_gap", "peak gap m", 0),
               ("peak_frac", "peak at frac", 2), ("a@20", "a@20", 2), ("a@15", "a@15", 2), ("a@10", "a@10", 2), ("a@5", "a@5", 2),
               ("a@2", "a@2", 2), ("v6_gap", "gap@6mph", 0), ("creep_t", "creep s", 1), ("creep_d", "creep m", 1), ("final_gap", "final gap", 1),
               ("min_gap", "min gap", 1), ("jerk_max", "jerk max", 1), ("jerk_brake", "jerk brake", 1)]


def metrics_table(rows, key_heads):
  head = list(key_heads) + [h for _, h, _ in METRIC_COLS]
  out = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
  for key, m in rows:
    cells = [str(k) for k in key] + [fmt(m.get(c), p) for c, _, p in METRIC_COLS]
    if m.get("collision"):
      cells[0] += f" CONTACT {m.get('impact_mph', 0):.0f} mph"
    out.append("| " + " | ".join(cells) + " |")
  return "\n".join(out)


def write_series(path, tab, i0):
  keys = ["t", "gap", "v", "a_target", "cmd", "sent", "a", "should_stop", "visible", "tracking", "src", "state"]
  with open(path, "w") as f:
    f.write(",".join(keys) + "\n")
    t0 = tab["t"][i0]
    for i in range(i0, len(tab["t"])):
      f.write(",".join([f"{tab['t'][i] - t0:.2f}", f"{tab['gap'][i]:.2f}", f"{tab['v'][i]:.3f}", f"{tab['a_target'][i]:.3f}",
                        f"{tab['cmd'][i]:.3f}", f"{tab['sent'][i]:.3f}", f"{tab['a'][i]:.3f}", str(int(tab["should_stop"][i])),
                        str(int(tab["visible"][i])), str(int(tab["tracking"][i])), str(tab["src"][i]), str(tab["state"][i])]) + "\n")


def _opts(a):
  return dict(radar=not a.vision, model_lead=a.model_lead or ("off" if a.no_model_lead else "const"), model_lead_range=a.model_lead_range, model=a.model, model_gap=a.model_gap, min_accel=a.min_accel, t_follow=a.t_follow, trace=a.trace,
              jerks=tuple(float(x) for x in a.jerks.split(",")) if a.jerks else None, two_leads=not a.single_lead,
              stopped_lead_tune=getattr(a, "stopped_lead_tune", "A"), lead_filter=a.lead_filter,
              exp_no_close_cap=a.exp_no_close_cap,
              tune_params=dict(kv.split("=") for kv in a.tune_param.split(",")) if a.tune_param else None)


def cmd_scenarios(a):
  speeds = [float(x) for x in a.speeds_mph.split(",")]
  detects = [None if float(x) == 0 else float(x) for x in a.detect.split(",")]
  jobs = []
  for pers, delay, tune, slt, v, dd in itertools.product(a.personality.split(","), [float(x) for x in a.delay.split(",")],
                                                         a.car_tune.split(","), a.stopped_lead_tune.split(","), speeds, detects):
    jobs.append(dict(v0_mph=v, gap0=a.gap, detect_dist=dd, personality=pers, delay=delay, car_tune=tune, **{**_opts(a), "stopped_lead_tune": slt}))
  t0 = _time.time()
  if a.jobs > 1:
    with ProcessPoolExecutor(a.jobs) as ex:
      res = list(ex.map(_job, jobs))
  else:
    _import_stack()
    res = [_job(j) for j in jobs]
  print(f"<!-- {len(jobs)} runs in {_time.time() - t0:.0f} s -->", file=sys.stderr)
  if a.csv:
    import csv
    cols = ["personality", "delay", "car_tune", "slt", "v0_mph", "detect_dist"] + [c for c, _, _ in METRIC_COLS] + ["stopped", "collision", "impact_mph"]
    with open(a.csv, "w", newline="") as f:
      w = csv.writer(f)
      w.writerow(cols)
      for kw, m, _, _ in res:
        w.writerow([kw["personality"], kw["delay"], kw["car_tune"], kw["stopped_lead_tune"], kw["v0_mph"], kw["detect_dist"] or 0] +
                   [m.get(c, "") for c, _, _ in METRIC_COLS] + [m["stopped"], m["collision"], m.get("impact_mph", "")])
  if a.series:
    os.makedirs(a.series, exist_ok=True)
    for kw, m, tab, i0 in res:
      name = f"{kw['personality']}_d{kw['delay']}_{kw['car_tune']}_slt{kw['stopped_lead_tune']}_v{int(kw['v0_mph'])}_det{int(kw['detect_dist'] or 0)}.csv"
      write_series(os.path.join(a.series, name), tab, i0)
  groups = {}
  for kw, m, _, _ in res:
    groups.setdefault((kw["personality"], kw["delay"], kw["car_tune"], kw["stopped_lead_tune"]), []).append(((kw["v0_mph"], kw["detect_dist"] or "far"), m))
  for (pers, delay, tune, slt), rows in groups.items():
    print(f"\n### {pers}, planner delay {delay}, car tune {tune}, stopped-lead tune {slt}, {'vision' if a.vision else 'radar'} lead\n")
    print(metrics_table(rows, ["mph", "detect m"]))


def cmd_run(a):
  _import_stack()
  kw = dict(v0_mph=a.v0_mph, gap0=a.gap, detect_dist=a.detect_dist, personality=a.personality, delay=a.delay, car_tune=a.car_tune,
            set_speed_mph=a.set_speed_mph, **_opts(a))
  tab, i0, m = run_sim(**kw)
  print(metrics_table([((a.v0_mph, a.detect_dist or "far"), m)], ["mph", "detect m"]))
  if a.series:
    write_series(a.series, tab, i0)
  if a.print_series:
    print("\n t    gap    mph   a_tgt    cmd   sent  a_car  trk stop src      state")
    step = max(1, int(round(a.print_dt / DT_PLAN)))
    t0 = tab["t"][i0]
    for i in range(i0, len(tab["t"]), step):
      print(f"{tab['t'][i] - t0:5.1f} {tab['gap'][i]:6.1f} {tab['v'][i] * MPH:5.1f} {tab['a_target'][i]:7.2f} {tab['cmd'][i]:6.2f} "
            f"{tab['sent'][i]:6.2f} {tab['a'][i]:6.2f}  {int(tab['tracking'][i])}   {int(tab['should_stop'][i])}  {tab['src'][i]:8s} {tab['state'][i]}"
            + (f"  {tab['bind'][i]}" if a.trace else ""))


# ------------------------------------------------------------------ real drives: loader, scanner, validation
COLS = ["t", "v", "a", "ped", "vcruise", "cmd", "lact", "st", "d", "vl", "al", "rad", "prob", "pers", "exp", "tf", "track", "force",
        "minacc", "maxacc", "atgt", "stop"]
CI = {c: i for i, c in enumerate(COLS)}
GRID_DT = 0.1

# tune the route was driven on (analysis/stopped_lead_approaches.py OP table) and what the car-side model uses
ROUTE_TUNE = {"0000000d--5e0d1d244b": "A", "0000000e--5ebf4636a1": "A", "0000000f--79cf51e7fa": "A", "00000011--c7d29db317": "B",
              "00000012--2aada6ce32": "C"}
# name -> (route, acquisition s, window end s, lead speed from the log?, note). Route seconds as in logs.route_time (60 s per segment
# plus the offset from the segment's first carState). lead_log: the lead is still slowing down at the acquisition, so its logged speed
# is replayed instead of a stationary lead.
EXAMPLES = {
  "B-11-1219": ("00000011--c7d29db317", 1230.0, 1244, False,
                "tune B, Aggressive, delay 0.5; stopped car picked up by the radar at ~42 m, 10 m/s closing, set speed 21 mph; a red-light/Force Stop "
                "flag blips for 0.5 s at 1231.6 and Conditional Experimental Mode flips to blended during the approach"),
  "A-0f-1883": ("0000000f--79cf51e7fa", 1887.5, 1905, True,
                "tune A, Aggressive, delay 0.15; follows a car at 25 mph that brakes to a stop (lead speed replayed from the log)"),
  "A-0e-712": ("0000000e--5ebf4636a1", 712.0, 756, True,
               "tune A, Standard, delay 0.15; jam brake from 62 mph, lead slows from 26 to 3 m/s (lead speed replayed); not a clean stop"),
}


def _real_seg(arg):
  """Rows (COLS) of one segment file: a row per carState >= GRID_DT apart, everything else sample-and-hold."""
  path, = arg
  import logs
  evs = logs.sorted_events(path)
  seg = logs.seg_num(path)
  s0 = next((e.logMonoTime for e in evs if e.which() == "carState"), None)
  if s0 is None:
    return np.zeros((0, len(COLS))), {}
  S = dict.fromkeys(COLS, 0.0)
  S["pers"] = -1.0
  cp = {}
  rows, last_t = [], -9.0
  for e in evs:
    w = e.which()
    t = seg * 60 + (e.logMonoTime - s0) / 1e9
    if w == "carState":
      c = e.carState
      S.update(v=c.vEgo, a=c.aEgo, ped=float(c.gasPressed or c.brakePressed), vcruise=c.vCruise / 3.6 if c.vCruise < 250 else 0.0)
      if t - last_t >= GRID_DT - 1e-6:
        S["t"] = t
        rows.append([S[k] for k in COLS])
        last_t = t
    elif w == "carControl":
      S.update(cmd=e.carControl.actuators.accel, lact=float(e.carControl.longActive))
    elif w == "radarState":
      l = e.radarState.leadOne
      S.update(st=float(l.status), d=l.dRel, vl=l.vLead, al=l.aLeadK, rad=float(l.radar), prob=l.modelProb)
    elif w == "selfdriveState":
      S.update(pers=float(int(e.selfdriveState.personality.raw)), exp=float(e.selfdriveState.experimentalMode))
    elif w == "starpilotPlan":
      p = e.starpilotPlan
      S.update(tf=p.tFollow, track=float(p.trackingLead), force=float(p.forcingStop or p.redLight), minacc=p.minAcceleration,
               maxacc=p.maxAcceleration)
    elif w == "longitudinalPlan":
      S.update(atgt=e.longitudinalPlan.aTarget, stop=float(e.longitudinalPlan.shouldStop))
    elif w == "carParams" and not cp:
      p = e.carParams
      cp = {"delay": float(p.longitudinalActuatorDelay), "fingerprint": str(p.carFingerprint)}
  return np.array(rows, float).reshape(-1, len(COLS)), cp


def load_real(route, segs=None, kind=None):
  """All rows of the route's segments (rlog where it exists, else qlog) in time order, plus the logged CarParams bits."""
  import logs
  by_seg = {}
  for k in ([kind] if kind else ["qlog", "rlog"]):          # rlog last so it wins
    for f in logs.segment_files(route, k, segs):
      by_seg[logs.seg_num(f)] = f
  files = [by_seg[s] for s in sorted(by_seg)]
  res = logs.pmap(_real_seg, [(f,) for f in files]) if len(files) > 2 else [_real_seg((f,)) for f in files]
  rows = [r for r, _ in res if len(r)]
  cp = next((c for _, c in res if c), {})
  return (np.vstack(rows) if rows else np.zeros((0, len(COLS)))), cp, sorted(by_seg)


def smooth(x, n):
  if n <= 1:
    return np.asarray(x, float)
  k = np.ones(n) / n
  pad = np.r_[np.full(n // 2, x[0]), x, np.full(n - 1 - n // 2, x[-1])]
  return np.convolve(pad, k, mode="valid")


def find_approaches(R, v_min=15 / MPH, max_lead_v=1.5, stop_v=0.12, max_gap=100.0, min_gap=8.0, window=60.0, vl_stopped=1.0):
  """Clean stopped-lead approaches in rows R: [(i_acq, i_stop)]. The car ends below stop_v behind a lead that is stopped (<vl_stopped);
  from the acquisition (first sample of the last unbroken run with a visible lead below max_lead_v, within window s of the stop) to the
  stop longActive, no pedal, no Force Stop / red light, and the car is above v_min at the acquisition."""
  t, v = R[:, CI["t"]], R[:, CI["v"]]
  good = (R[:, CI["lact"]] > 0) & (R[:, CI["ped"]] == 0) & (R[:, CI["force"]] == 0)
  lead = (R[:, CI["st"]] > 0) & (R[:, CI["vl"]] < max_lead_v) & (R[:, CI["d"]] < max_gap) & (R[:, CI["d"]] > 1.0)
  out, i, n = [], 1, len(t)
  while i < n:
    if v[i] < stop_v <= v[i - 1]:
      j = i
      lo = np.searchsorted(t, t[i] - window)
      # walk back over the unbroken run of "stopped lead visible and engaged"
      bad_since = None
      k = i
      while k > lo:
        ok = lead[k] and good[k]
        if ok:
          bad_since = None
        else:
          bad_since = t[k] if bad_since is None else bad_since
          if bad_since - t[k] > 1.0:
            break
        k -= 1
      k += 1
      while k < i and not (lead[k] and good[k]):
        k += 1
      acq = k
      seg_ok = bool(np.all(good[acq:i + 1])) and R[i, CI["vl"]] < vl_stopped and lead[i]
      if acq < i and seg_ok and v[acq] >= v_min and min_gap <= R[acq, CI["d"]] <= max_gap and np.all(np.diff(t[acq:i + 1]) < 1.0):
        out.append((acq, i))
      while i < n and v[i] < 1.0:
        i += 1
    i += 1
  return out


def real_conditions(R, cp, acq, route):
  r = R[acq]
  pers = int(r[CI["pers"]])
  tune = ROUTE_TUNE.get(route, "A")
  # a few samples around the acquisition for the logged config
  sl = slice(max(0, acq - 5), acq + 20)
  return dict(
    v0=float(r[CI["v"]]), gap0=float(r[CI["d"]]), a0=float(np.mean(R[max(0, acq - 2):acq + 3, CI["a"]])),
    set_speed=float(r[CI["vcruise"]]) if r[CI["vcruise"]] > 1 else float(r[CI["v"]]), personality={0: "aggressive", 1: "standard", 2: "relaxed"}.get(pers, "standard"),
    pers=pers, delay=float(cp.get("delay", 0.5)), car_tune=tune, radar=bool(np.median(R[sl, CI["rad"]]) > 0.5),
    min_accel=float(np.median(R[sl, CI["minacc"]])), t_follow=float(np.median(R[sl, CI["tf"]])) or None)


def real_metrics(R, acq, end):
  seg = R[acq:end + 1]
  t, gap, v = seg[:, CI["t"]], seg[:, CI["d"]], seg[:, CI["v"]]
  a = smooth(seg[:, CI["a"]], 3)
  m = metrics(t, gap, v, a, jerk_a=smooth(seg[:, CI["cmd"]], 3), v_stop=0.12)
  m["peak_cmd"] = float(-np.min(seg[:, CI["cmd"]]))
  m["collision"] = False
  return m, dict(t=t - t[0], gap=gap, v=v, a=a, cmd=seg[:, CI["cmd"]], atgt=seg[:, CI["atgt"]], track=seg[:, CI["track"]], exp=seg[:, CI["exp"]])


def sim_for(cond, lead_track=None, extra=None):
  kw = dict(v0=cond["v0"], gap0=cond["gap0"], a0=cond["a0"], set_speed=cond["set_speed"], personality=cond["personality"],
            delay=cond["delay"], car_tune=cond["car_tune"], radar=cond["radar"], min_accel=cond["min_accel"], stop_hold=2.0)
  if cond.get("t_follow"):
    kw["t_follow"] = cond["t_follow"]
  if lead_track is not None:
    kw["lead_track"] = lead_track
  kw.update({k: v for k, v in (extra or {}).items() if v is not None})
  tab, i0, m = run_sim(**kw)
  m["peak_cmd"] = float(-np.min(tab["cmd"][i0:]))
  return tab, i0, m


def curves(real, tab, i0, gaps=(40, 30, 25, 20, 15, 12, 10, 8, 6)):
  """speed (mph) and applied accel at fixed gaps, real vs sim (first time the gap falls through)."""
  rows = []
  sg, sv, sa = tab["gap"][i0:], tab["v"][i0:], tab["a"][i0:]
  for g in gaps:
    cells = []
    for gg, vv, aa in ((real["gap"], real["v"], real["a"]), (sg, sv, sa)):
      idx = np.where(gg <= g)[0]
      cells += [f"{vv[idx[0]] * MPH:.1f}", f"{aa[idx[0]]:.2f}"] if len(idx) and gg[0] > g else ["-", "-"]
    rows.append((g, *cells))
  return rows


VAL_COLS = [("v0", "v0 mph", 1), ("gap0", "gap0 m", 0), ("peak_decel", "peak decel", 2), ("peak_gap", "peak gap m", 0), ("peak_cmd", "peak cmd", 2),
            ("a@10", "a@10", 2), ("a@5", "a@5", 2), ("v6_gap", "gap@6mph", 1), ("creep_t", "creep s", 1), ("creep_d", "creep m", 1),
            ("final_gap", "final gap", 1), ("t_stop", "t stop s", 1)]


def val_table(rows):
  head = ["case"] + [h for _, h, _ in VAL_COLS]
  out = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
  for name, m in rows:
    out.append("| " + " | ".join([name] + [fmt(m.get(c), p) for c, _, p in VAL_COLS]) + " |")
  return "\n".join(out)


def lead_track_from(R, acq, end):
  """The logged lead as (t, position relative to the ego start, radar vLead, radar aLeadK): position = logged gap + the ego's logged
  displacement, so the sim's ego meets the same physical lead; the planner is fed the radar's own speed/accel estimates."""
  seg = R[acq:end + 1]
  t = seg[:, CI["t"]] - seg[0, CI["t"]]
  ego = np.r_[0.0, np.cumsum(0.5 * (seg[1:, CI["v"]] + seg[:-1, CI["v"]]) * np.diff(t))]
  pos = seg[:, CI["d"]] + ego
  return (t, pos, np.maximum(seg[:, CI["vl"]], 0.0), seg[:, CI["al"]])


def compare(name, R, cp, acq, end, route, a, verbose=True, lead_log=False):
  """Real approach R[acq:end] vs the sim started from its logged conditions. Returns (real metrics, {variant: sim metrics}, cond)."""
  cond = real_conditions(R, cp, acq, route)
  real_m, real_c = real_metrics(R, acq, end)
  real_m["v0"], real_m["gap0"] = cond["v0"] * MPH, cond["gap0"]
  lp = None if a.ideal_lead else lead_track_from(R, acq, end)
  variants = [("base", {})]
  if a.variants:
    variants += [("model-gap 5", dict(model="stop", model_gap=5.0)), ("model-gap 7", dict(model="stop", model_gap=7.0)),
                 ("vision lead", dict(radar=False)), ("single lead", dict(two_leads=False)), ("min-accel -0.5", dict(min_accel=-0.5)),
                 ("min-accel -2.0", dict(min_accel=-2.0))]
  sims = {}
  for vname, extra in variants:
    opts = {k: v for k, v in _opts(a).items() if k not in extra}
    tab, ii, m = sim_for(cond, lp, {**opts, **extra})
    m["v0"], m["gap0"] = cond["v0"] * MPH, cond["gap0"]
    sims[vname] = (tab, ii, m)
  if verbose:
    print(f"\n## {name}: {route} t {R[acq, 0]:.1f}-{R[end, 0]:.1f} s")
    print(f"start: v {cond['v0'] * MPH:.1f} mph, a {cond['a0']:+.2f}, gap {cond['gap0']:.1f} m, set speed {cond['set_speed'] * MPH:.0f} mph, "
          f"personality {cond['pers']} ({cond['personality']}), delay {cond['delay']}, tune {cond['car_tune']}, radar lead {cond['radar']}, "
          f"min accel {cond['min_accel']:.1f}, tFollow {cond['t_follow']}, logged tracking {np.mean(real_c['track']) * 100:.0f}% of samples, "
          f"experimental {np.mean(real_c['exp']) * 100:.0f}%")
    print(val_table([("real", real_m)] + [(f"sim {k}", v[2]) for k, v in sims.items()]))
    bm = sims["base"][2]
    err = {k: bm.get(k, np.nan) - real_m.get(k, np.nan) for k in ERR_KEYS}
    print("sim - real (base): " + ", ".join(f"{k} {v:+.2f}" for k, v in err.items() if not np.isnan(v)))
    print("\nspeed (mph) / applied accel (m/s^2) when the gap first falls through:\n")
    print("| gap m | real mph | real a | sim mph | sim a |\n|---|---|---|---|---|")
    for g, rv, ra, sv, sa in curves(real_c, *sims["base"][:2]):
      print(f"| {g} | {rv} | {ra} | {sv} | {sa} |")
  if a.series:
    os.makedirs(a.series, exist_ok=True)
    write_series(os.path.join(a.series, f"val_{name}_sim.csv"), sims["base"][0], sims["base"][1])
    with open(os.path.join(a.series, f"val_{name}_real.csv"), "w") as f:
      f.write("t,gap,v,a,cmd,atgt,track,exp\n")
      for k in range(len(real_c["t"])):
        f.write(",".join(f"{real_c[c][k]:.3f}" for c in ("t", "gap", "v", "a", "cmd", "atgt", "track", "exp")) + "\n")
  return real_m, sims, cond


ERR_KEYS = ["peak_decel", "peak_gap", "a@10", "a@5", "v6_gap", "creep_t", "final_gap", "t_stop"]


def cmd_validate(a):
  if a.model_lead is None and not a.ideal_lead:
    a.model_lead = "oracle"
  import logs
  _import_stack()
  if a.list:
    for n, (route, t0, t1, ll, note) in EXAMPLES.items():
      print(f"{n:10s} {route} {t0}-{t1} s  {note}")
    return
  summary = []
  if a.scan:
    for route in a.scan:
      R, cp, _ = load_real(route)
      for acq, end in find_approaches(R, v_min=a.min_mph / MPH, max_lead_v=a.max_lead_v):
        real_m, sims, cond = compare(f"{logs.short(route)}@{R[acq, 0]:.0f}", R, cp, acq, end, route, a, verbose=a.verbose)
        summary.append((f"{logs.short(route)}@{R[acq, 0]:.0f} {cond['car_tune']} p{cond['pers']} d{cond['delay']}", real_m, sims["base"][2]))
  else:
    for name in ([a.only] if a.only else list(EXAMPLES)):
      route, t_acq, t1, lead_log, note = EXAMPLES[name]
      R, cp, _ = load_real(route, list(range(int(t_acq // 60) - 1, int(t1 // 60) + 1)))
      acq = int(np.searchsorted(R[:, CI["t"]], t_acq))
      end = next((k for k in range(acq + 20, len(R)) if R[k, CI["v"]] < 0.12 or R[k, CI["t"]] > t1), len(R) - 1)
      print(f"\n(example {name}: {note})")
      real_m, sims, cond = compare(name, R, cp, acq, end, route, a, lead_log=lead_log)
      summary.append((name, real_m, sims["base"][2]))
  if len(summary) > 1:
    print("\n## sim - real, base variant (per approach)\n")
    print("| case | real peak | sim peak | " + " | ".join(f"d {k}" for k in ERR_KEYS) + " |\n|" + "---|" * (len(ERR_KEYS) + 3))
    errs = {k: [] for k in ERR_KEYS}
    for n, rm, sm in summary:
      e = {k: sm.get(k, np.nan) - rm.get(k, np.nan) for k in ERR_KEYS}
      for k in ERR_KEYS:
        errs[k].append(e[k])
      print(f"| {n} | {fmt(rm['peak_decel'], 2)} | {fmt(sm['peak_decel'], 2)} | " + " | ".join(f"{e[k]:+.2f}" if not np.isnan(e[k]) else "-" for k in ERR_KEYS) + " |")
    print("| **mean** | | | " + " | ".join(f"{np.nanmean(errs[k]):+.2f}" for k in ERR_KEYS) + " |")
    print("| **mean abs** | | | " + " | ".join(f"{np.nanmean(np.abs(errs[k])):.2f}" for k in ERR_KEYS) + " |")


def cmd_openloop(a):
  """Planner-only check: feed the sim's planner the LOGGED car state, lead and set speed, and compare its aTarget with the logged one.
  Separates 'the planner inputs are built wrongly' from 'the closed loop (car model, history) diverges'."""
  _import_stack()
  R, cp, _ = load_real(a.route, list(range(int(a.t0 // 60) - 1, int(a.t1 // 60) + 1)))
  R = R[(R[:, CI["t"]] >= a.t0 - 5) & (R[:, CI["t"]] <= a.t1)]
  t = R[:, CI["t"]]
  i0 = int(np.searchsorted(t, a.t0))
  cond = real_conditions(R, cp, i0, a.route)
  opts = _opts(a)
  opts.update({k: v for k, v in dict(min_accel=cond["min_accel"], t_follow=cond["t_follow"]).items() if opts.get(k) is None})
  P = make_plant_class()(v0=R[0, CI["v"]], gap0=R[0, CI["d"]], set_speed=R[0, CI["vcruise"]], personality=cond["personality"],
                         delay=cond["delay"], car_tune=cond["car_tune"], **opts)
  tt = np.arange(t[0], t[-1], DT_PLAN)
  f = {c: np.interp(tt, t, R[:, CI[c]]) for c in COLS if c != "t"}
  out = []
  for i, ti in enumerate(tt):
    P.speed, P.a_ego, P.distance, P.distance_lead = float(f["v"][i]), float(f["a"][i]), 0.0, float(f["d"][i])
    P.set_speed = float(f["vcruise"][i]) or P.speed
    P.detect_dist = None if f["st"][i] > 0.5 else 0.0          # no lead in the log -> hidden
    r = P.step(0.0, lead_obs=(float(max(f["vl"][i], 0.0)), float(f["al"][i])))
    out.append(r["a_target"])
  out = np.array(out)
  m = tt >= a.t0
  print(f"open-loop planner check {a.route} {a.t0}-{a.t1} s: {cond['personality']}, delay {cond['delay']}, min accel {cond['min_accel']:.1f}, "
        f"tFollow {cond['t_follow']}, model lead {opts.get('model_lead')}")
  print("| t | mph | gap m | lead v | logged aTarget | sim aTarget |\n|---|---|---|---|---|---|")
  for k in range(0, len(tt), int(round(a.dt / DT_PLAN))):
    if m[k]:
      print(f"| {tt[k]:.1f} | {f['v'][k] * MPH:.1f} | {f['d'][k]:.1f} | {f['vl'][k]:.1f} | {f['atgt'][k]:.2f} | {out[k]:.2f} |")
  d = out[m] - f["atgt"][m]
  print(f"\nmean |sim - logged| {np.mean(np.abs(d)):.2f}, rms {np.sqrt(np.mean(d ** 2)):.2f}, bias {np.mean(d):+.2f} m/s^2 over {m.sum() * DT_PLAN:.0f} s "
        f"(sim min {out[m].min():.2f}, logged min {f['atgt'][m].min():.2f})")


def main():
  ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
  sub = ap.add_subparsers(dest="cmd", required=True)

  def common(p):
    p.add_argument("--vision", action="store_true", help="vision-only lead (radar False, modelProb 0.9)")
    p.add_argument("--model", choices=("plain", "stop"), default="plain")
    p.add_argument("--model-gap", type=float, default=None, help="--model stop: the model path ends this many m short of the lead")
    p.add_argument("--min-accel", type=float, default=None, help="starpilotPlan.minAcceleration (default by personality)")
    p.add_argument("--t-follow", type=float, default=None)
    p.add_argument("--jerks", default=None, help="accelerationJerk,dangerJerk,speedJerk")
    p.add_argument("--single-lead", action="store_true",
                   help="leave leadTwo empty (the real radarState has leadTwo == leadOne here, like the stock Plant; with an empty "
                        "leadTwo the MPC's shared lead filter is pumped with a fake lead and the stopped lead looks like it moves)")
    p.add_argument("--no-model-lead", action="store_true", help="no modelV2.leadsV3 (the MPC then always uses the radar-only lead path)")
    p.add_argument("--model-lead", choices=("const", "oracle", "off"), default=None,
                   help="modelV2.leadsV3 content: const = lead keeps its speed (default; exact for a stopped lead), oracle = the lead's logged "
                        "future (validate only: what a perfect model would predict for a lead that is braking), off")
    p.add_argument("--model-lead-range", type=float, default=80.0, help="the vision model lead exists below this gap (m)")
    p.add_argument("--stopped-lead-tune", default="A",
                   help="Testing Ground slot 10 (Stinger Stopped Lead) A = installed, B = tune; scenarios takes a comma list")
    p.add_argument("--tune-param", default=None,
                   help="override KIA_STINGER_2022_STOPPED_LEAD_<NAME> constants for the run, e.g. TARGET_DECEL=1.3,FINAL_GAP_M=4.0")
    p.add_argument("--exp-no-close-cap", action="store_true",
                   help="experiment: planner.get_close_lead_brake_cap returns None for stopped leads (what binds the approach/creep)")
    p.add_argument("--lead-filter", choices=("primed", "fast"), default="primed",
                   help="MPC lead_v/a filter: primed = fresh planner (rc 0.8 s, primed to v_ego+10), fast = rc 0 as after a drive above ~50 mph")
    p.add_argument("--trace", action="store_true", help="record which planner line set output_a_target (slow; shown with --print-series)")

  s = sub.add_parser("scenarios")
  s.add_argument("--personality", default="standard,aggressive")
  s.add_argument("--delay", default="0.15,0.5", help="planner CP.longitudinalActuatorDelay list")
  s.add_argument("--car-tune", default="A,B")
  s.add_argument("--speeds-mph", default="15,20,25,30,35")
  s.add_argument("--detect", default="0,60,40,30", help="detection distances in m, 0 = visible from --gap")
  s.add_argument("--gap", type=float, default=120.0)
  s.add_argument("--csv")
  s.add_argument("--series", help="directory for one time-series csv per run")
  s.add_argument("--jobs", type=int, default=4)
  common(s)
  s.set_defaults(fn=cmd_scenarios)

  r = sub.add_parser("run")
  r.add_argument("--v0-mph", type=float, default=25.0)
  r.add_argument("--gap", type=float, default=120.0)
  r.add_argument("--detect-dist", type=float, default=None)
  r.add_argument("--set-speed-mph", type=float, default=None)
  r.add_argument("--personality", default="standard")
  r.add_argument("--delay", type=float, default=0.5)
  r.add_argument("--car-tune", default="A")
  r.add_argument("--series", help="write the time series csv here")
  r.add_argument("--print-series", action="store_true")
  r.add_argument("--print-dt", type=float, default=0.5)
  common(r)
  r.set_defaults(fn=cmd_run)

  v = sub.add_parser("validate", help="sim vs the real approaches in the logs")
  v.add_argument("--list", action="store_true", help="list the named examples")
  v.add_argument("--only", help="one named example")
  v.add_argument("--scan", nargs="+", metavar="ROUTE", help="find every clean stopped-lead approach in these routes and compare them all")
  v.add_argument("--min-mph", type=float, default=15.0, help="--scan: minimum speed at the acquisition")
  v.add_argument("--max-lead-v", type=float, default=1.5, help="--scan: the lead counts as stopped below this radar speed (m/s)")
  v.add_argument("--variants", action="store_true", help="also run the sensitivity variants (model gap, vision lead, min accel ...)")
  v.add_argument("--verbose", action="store_true", help="--scan: print every approach in full")
  v.add_argument("--ideal-lead", action="store_true", help="a stationary lead with vLead = aLeadK = 0 instead of the logged radar track")
  v.add_argument("--series", help="directory for the real and sim time series csv files")
  common(v)
  v.set_defaults(fn=cmd_validate)

  o = sub.add_parser("openloop", help="planner-only check on the logged car state/lead")
  o.add_argument("route")
  o.add_argument("t0", type=float)
  o.add_argument("t1", type=float)
  o.add_argument("--dt", type=float, default=1.0)
  common(o)
  o.set_defaults(fn=cmd_openloop)

  a = ap.parse_args()
  a.fn(a)


if __name__ == "__main__":
  main()
