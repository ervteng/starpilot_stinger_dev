#!/usr/bin/env python3
"""Replay StarPilot's real LongitudinalPlanner on one drive's logged inputs, twice, changing only CP.longitudinalActuatorDelay.

Answers: does the 0.5 s delay that Stinger tune B sets (vs the 0.15 s default) make the longitudinal planner brake earlier or
harder? The planner (selfdrive/controls/lib/longitudinal_planner.py, acados MPC included) is imported unmodified from a
built StarPilot checkout (default $STINGER_DEV_HOME/x86, which has params_pyx and the acados solver compiled; it is only
read, never checked out or written). Per modelV2 cycle (one logged longitudinalPlan) it is fed the latest logged carState,
carControl, controlsState, liveParameters, radarState, modelV2, selfdriveState, starpilotCarState and starpilotPlan; the
starpilot toggles are the JSON the planner itself received (starpilotPlan.starpilotToggles) and CP is the logged carParams.
The StarPilot planner (starpilotPlan) is NOT re-run: it never reads longitudinalActuatorDelay, so the logged starpilotPlan
is what it would produce for either delay. time.monotonic() inside the planner is replaced by the log clock.

OPEN LOOP: the car's motion (vEgo, aEgo, radar, lead) is the logged one in both runs; a planner that brakes earlier here
would in reality have changed the gap and closing speed. Only the planner's *request* is compared.

Binding path: update() is line-traced and every change of the local output_a_target is recorded (source line + new value).
"final" = the last line that changed it (what produced the published value), "min" = the line that lowered it the most.
Run 1 (--delay 0.5) is checked against the logged longitudinalPlan.aTarget / shouldStop / source.

Usage: planner_delay_replay.py [ROUTE] [--segments 16-21] [--delays 0.5 0.15] [--events 1019 1158 ...] [--sp-lag 0]
"""
import argparse
import logging
import os
import pickle
import sys
import time as _time
import types
from collections import Counter, defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
  sys.path.insert(0, HERE)
import toolchain  # noqa: E402  (STINGER_TOOLCHAIN=old|sg; old: $STINGER_DEV_HOME/x86, sg: x86-sg)

DEV = toolchain.DEV
BUILD = toolchain.BUILD
toolchain.setup_paths(extra=(HERE,))

if toolchain.SG:
  # the logs are read with the old schema (logs.py); the planner and its inputs are the SG ones (sg_compat.convert)
  import sg_compat  # noqa: E402
else:
  from cereal import log as _cereal_log  # noqa: E402,F401  (load the capnp schema once, before logs.py would)
import logs  # noqa: E402

DEFAULT_ROUTE = "00000011--c7d29db317"
SERVICES = ("carState", "carControl", "controlsState", "liveParameters", "radarState", "modelV2", "selfdriveState",
            "starpilotCarState", "starpilotPlan")
EVENTS = (921, 1019, 1158, 1226, 1276)  # user-reported city hard-braking moments, route seconds
THRESHOLDS = (-0.3, -0.5, -1.0)


class FakeSM:
  """Just enough SubMaster for LongitudinalPlanner.update: indexing, logMonoTime, all_checks, frame/updated."""
  def __init__(self, d, mono, frame):
    self.d = d
    self.logMonoTime = defaultdict(lambda: mono)
    self.frame = frame
    self.updated = defaultdict(lambda: True)
    self.valid = defaultdict(lambda: True)
    self.alive = defaultdict(lambda: True)

  def __getitem__(self, k):
    return self.d[k]

  def all_checks(self, *a, **k):
    return True


def load_frames(route, segments, sp_lag=0):
  """One frame per logged longitudinalPlan (= per modelV2 cycle) with the latest logged inputs at that moment."""
  files = logs.segment_files(route, "rlog", segments)
  assert [logs.seg_num(f) for f in files] == list(segments), f"missing rlogs: {[logs.seg_num(f) for f in files]}"
  latest, frames, cp, toggles_text = {}, [], None, ""
  sp_hist = []
  for f in files:
    evs = logs.sorted_events(f)
    seg = logs.seg_num(f)
    seg0 = next(e.logMonoTime for e in evs if e.which() == "carState")
    for e in evs:
      w = e.which()
      if w in SERVICES:
        latest[w] = getattr(e, w)
        latest[w + "_t"] = e.logMonoTime
        if w == "starpilotPlan":
          sp_hist.append(latest[w])
          if latest[w].starpilotToggles:
            toggles_text = latest[w].starpilotToggles
      elif w == "carParams" and cp is None:
        cp = e.carParams
      elif w == "longitudinalPlan" and all(s in latest for s in SERVICES):
        d = {s: latest[s] for s in SERVICES}
        if sp_lag:
          d["starpilotPlan"] = sp_hist[max(0, len(sp_hist) - 1 - sp_lag)]
        frames.append(dict(t=e.logMonoTime, route_t=logs.route_time(f, e.logMonoTime, seg0), sm=d, toggles=toggles_text,
                           log=e.longitudinalPlan, seg=seg, mono={s: latest[s + "_t"] for s in SERVICES}))
  return frames, cp


class CPProxy:
  """CarParams whose longitudinalActuatorDelay is `action` when read directly in LongitudinalPlanner.update (the action_t
  that picks the output accel off the plan) and `caps` when read anywhere else (reaction_t in the lead caps, dynamic t_follow)."""
  def __init__(self, cp, action, caps, update_code):
    self._cp, self._action, self._caps, self._code = cp, action, caps, update_code

  @property
  def longitudinalActuatorDelay(self):
    return self._action if sys._getframe(1).f_code is self._code else self._caps

  def __getattr__(self, k):
    return getattr(self._cp, k)


def parse_delay(spec):
  a, _, c = str(spec).partition(":")
  return (float(a), float(c)) if c else (float(a), float(a))


# sg: the logged services the SG planner reads, as (SG service name, logged service name); everything else it needs is a default
SG_SERVICES = (("carState", "carState"), ("carControl", "carControl"), ("controlsState", "controlsState"),
               ("vehicleParameters", "liveParameters"), ("radarState", "radarState"), ("modelV2", "modelV2"),
               ("selfdriveState", "selfdriveState"))


def replay(frames, cp, delay, tracer=True, dump=None, sync_filters=False, testing_ground=None):
  if toolchain.SG:
    return replay_sg(frames, cp, delay, tracer, dump, sync_filters, testing_ground)
  from cereal import car
  from openpilot.starpilot.common.starpilot_variables import process_starpilot_toggles
  import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
  logging.disable(logging.CRITICAL)

  clock = [0.0]
  lp.time = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0], sleep=_time.sleep)
  action_d, caps_d = parse_delay(delay)
  CP = cp.as_builder()
  CP.longitudinalActuatorDelay = caps_d
  code = lp.LongitudinalPlanner.update.__code__
  if action_d != caps_d:
    CP = CPProxy(CP, action_d, caps_d, code)
  forced = {tuple(x.split(":")) for x in (testing_ground or "").split(",") if x}
  if forced:  # opt-in --testing-ground SLOT:VARIANT: those read as active (latched at construction) in the planner and in longitudinal_vehicle_tunes
    import openpilot.selfdrive.controls.lib.longitudinal_vehicle_tunes as vt
    real = lp.testing_ground
    fake = types.SimpleNamespace(use=lambda slot, variant="B": (str(slot), str(variant)) in forced or
                                 (str(slot) not in {x for x, _ in forced} and real.use(slot, variant)))
    lp.testing_ground = fake
    vt.testing_ground = fake
    if hasattr(vt, "_kia_stinger_2022_tune_latch"):
      vt._kia_stinger_2022_tune_latch.clear()
  planner = lp.LongitudinalPlanner(CP)
  if forced:
    print(f"testing-ground {testing_ground}: far_follow_brake/release_slew_rate = "
          f"{planner.far_follow_brake_slew_rate}/{planner.far_follow_release_slew_rate}", flush=True)
  if sync_filters:  # what-if (monkeypatch only, tree untouched): keep lead_a/v filters tracking the radar lead on the model-lead path
    planner.mpc.sync_model_lead_filters = True

  src = open(lp.__file__).read().splitlines()
  state = {}
  changes = []
  dbg = _install_dump_hooks(planner, lp) if dump else None

  def local_trace(frame, event, arg):
    if event == "line":
      if state.get("prev") is not None:
        v = frame.f_locals.get("output_a_target")
        if v is not None and v != state.get("last"):
          changes.append((state["prev"], float(v)))
          state["last"] = v
      state["prev"] = frame.f_lineno
    elif event == "return":
      if dbg is not None and dbg.get("rec") is not None:
        dbg["planner_locals"] = {k: frame.f_locals.get(k) for k in PLANNER_LOCALS}
      v = frame.f_locals.get("output_a_target")
      if v is not None and v != state.get("last"):
        changes.append((state["prev"], float(v)))
    return local_trace

  def global_trace(frame, event, arg):
    if frame.f_code is code:
      return local_trace
    if dbg is not None and frame.f_code is dbg["mpc_update_code"]:
      return dbg["mpc_local_trace"]
    return None

  out = []
  for i, fr in enumerate(frames):
    clock[0] = fr["t"] / 1e9
    sm = FakeSM(fr["sm"], fr["t"], i)
    toggles = process_starpilot_toggles(fr["toggles"])
    state.clear(); changes.clear()
    in_dump = bool(dump) and dump[0] <= fr["route_t"] <= dump[1]
    if dbg is not None:
      dbg["rec"] = {} if in_dump else None
      dbg["planner_locals"] = {}
    if tracer:
      sys.settrace(global_trace)
    try:
      planner.update(sm, toggles)
    finally:
      sys.settrace(None)
    cs, lead = fr["sm"]["carState"], fr["sm"]["radarState"].leadOne
    final = changes[-1][0] if changes else 0
    drops = [(changes[k][1] - (changes[k - 1][1] if k else changes[k][1]), changes[k][0]) for k in range(len(changes))]
    minline = min(drops)[1] if drops and min(drops)[0] < -1e-3 else 0
    out.append(dict(
      t=fr["route_t"], a=float(planner.output_a_target), stop=bool(planner.output_should_stop), src=planner.mpc.source,
      a0=float(planner.a_desired_trajectory[0]), plan_a=changes[0][1] if changes else np.nan,
      final=final, minline=minline, v=float(cs.vEgo), vcruise=float(fr["sm"]["starpilotPlan"].vCruise),
      lead=bool(lead.status), d=float(lead.dRel), vl=float(lead.vLead), al=float(lead.aLeadK), radar=bool(lead.radar),
      log_a=float(fr["log"].aTarget), log_stop=bool(fr["log"].shouldStop), log_src=str(fr["log"].longitudinalPlanSource),
      log_a0=float(fr["log"].accels[0]), seg=fr["seg"], aego=float(cs.aEgo), chg=list(changes),
      e2e=float(fr["sm"]["modelV2"].action.desiredAcceleration), exp=bool(fr["sm"]["selfdriveState"].experimentalMode),
      ffa=bool(planner.far_follow_output_slew_active), ffs=float(planner.far_follow_brake_slew_rate),
      tf=float(fr["sm"]["starpilotPlan"].tFollow), prob=float(lead.modelProb), minacc=float(fr["sm"]["starpilotPlan"].minAcceleration),
      model_stop=bool(fr["sm"]["modelV2"].action.shouldStop), forcing=bool(fr["sm"]["starpilotPlan"].forcingStop)))
    if in_dump:
      out[-1]["dump"] = _collect_dump(planner, fr, dbg, changes)
  labels = {ln: src[ln - 1].strip() for r in out for ln in (r["final"], r["minline"]) if ln}
  return out, labels


def replay_sg(frames, cp, delay, tracer=True, dump=None, sync_filters=False, testing_ground=None):
  """replay() for the SecretGoodStarPilot planner (STINGER_TOOLCHAIN=sg).

  The logs are from the old build: its inputs are converted to the SG schema (sg_compat.convert), and the SG planner is called with
  update(sm) and no StarPilot-side options (no profile / cruise ceiling / force-stop / lane-change providers, which in the real car
  come from the starpilot planner and its toggles, not from anything in these logs). So the replay follows the SG planner's own
  defaults and its match with the logged (old-build) longitudinalPlan is expected to be poor. The vision-lead path of the MPC is fed
  from the logged modelV2 whose time radarState.mdMonoTime names (sg_compat.ModelLeadFeed), standing in for the real-clock-gated
  qualified_model_leads.
  """
  if testing_ground:
    toolchain.require_old("--testing-ground")
  if sync_filters:
    toolchain.require_old("--sync-filters (mpc.sync_model_lead_filters)")
  if dump:
    toolchain.require_sg_port("--dump", "it wraps the old MPC's set_weights / update / process_lead signatures; the SG MPC has set_cost_weights / "
                              "update(radarstate, personality, *, follow_seconds, acceleration_max, stop_line_m, modelV2)")
  import openpilot.cereal.messaging as messaging
  from opendbc.car.hyundai.interface import CarInterface  # noqa: F401  (loads the SG opendbc before the planner)
  import openpilot.selfdrive.controls.lib.longitudinal_planner as lp
  import openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc as lm
  logging.disable(logging.CRITICAL)

  clock = [0.0]
  clock_ns = sg_compat.patch_planner_clock(lp, lm, lambda: clock[0])
  action_d, caps_d = parse_delay(delay)
  from opendbc.car.structs import CarParams
  CP = CarParams.new_message()
  sg_compat.convert(cp, CP)
  CP.longitudinalActuatorDelay = caps_d
  code = lp.LongitudinalPlanner.update.__code__
  if action_d != caps_d:
    CP = CPProxy(CP, action_d, caps_d, code)
  src_names = {int(v): k for k, v in lm.LongitudinalPlanSource.schema.enumerants.items()}   # enum ints -> the names the log has
  feed = sg_compat.ModelLeadFeed()
  lp.qualified_model_leads = feed
  planner = lp.LongitudinalPlanner(CP, clock_ns=clock_ns)

  src = open(lp.__file__).read().splitlines()
  state = {}
  changes = []

  def local_trace(frame, event, arg):
    if event == "line":
      if state.get("prev") is not None:
        v = frame.f_locals.get("output_a_target")
        if v is not None and v != state.get("last"):
          changes.append((state["prev"], float(v)))
          state["last"] = v
      state["prev"] = frame.f_lineno
    elif event == "return":
      v = frame.f_locals.get("output_a_target")
      if v is not None and v != state.get("last"):
        changes.append((state["prev"], float(v)))
    return local_trace

  def global_trace(frame, event, arg):
    return local_trace if frame.f_code is code else None

  conv_cache = {}

  def sg_service(name, old_name, fr):
    key = (name, fr["mono"][old_name])
    if key not in conv_cache:
      if len(conv_cache) > 64:
        conv_cache.clear()
      _, sb = sg_compat.new_service(messaging, name, fr["sm"][old_name])
      conv_cache[key] = sb.as_reader()
    return conv_cache[key]

  out = []
  for i, fr in enumerate(frames):
    clock[0] = fr["t"] / 1e9
    d = {name: sg_service(name, old_name, fr) for name, old_name in SG_SERVICES}
    # the logged radarState's modelV2 reference (mdMonoTime) must find that model frame: keep the converted model under its log time
    feed.observe(fr["mono"]["modelV2"], d["modelV2"])
    sm = sg_compat.SgSM(d, {**fr["mono"], "vehicleParameters": fr["mono"]["liveParameters"]}, i)
    state.clear(); changes.clear()
    if tracer:
      sys.settrace(global_trace)
    try:
      planner.update(sm, now_ns=int(fr["t"]))
    finally:
      sys.settrace(None)
    cs, lead = fr["sm"]["carState"], fr["sm"]["radarState"].leadOne
    final = changes[-1][0] if changes else 0
    drops = [(changes[k][1] - (changes[k - 1][1] if k else changes[k][1]), changes[k][0]) for k in range(len(changes))]
    minline = min(drops)[1] if drops and min(drops)[0] < -1e-3 else 0
    out.append(dict(
      t=fr["route_t"], a=float(planner.output_a_target), stop=bool(planner.output_should_stop), src=src_names.get(int(planner.mpc.source), str(planner.mpc.source)),
      a0=float(planner.a_desired_trajectory[0]), plan_a=changes[0][1] if changes else np.nan,
      final=final, minline=minline, v=float(cs.vEgo), vcruise=float(fr["sm"]["starpilotPlan"].vCruise),
      lead=bool(lead.status), d=float(lead.dRel), vl=float(lead.vLead), al=float(lead.aLeadK), radar=bool(lead.radar),
      log_a=float(fr["log"].aTarget), log_stop=bool(fr["log"].shouldStop), log_src=str(fr["log"].longitudinalPlanSource),
      log_a0=float(fr["log"].accels[0]), seg=fr["seg"], aego=float(cs.aEgo), chg=list(changes),
      e2e=float(fr["sm"]["modelV2"].action.desiredAcceleration), exp=bool(fr["sm"]["selfdriveState"].experimentalMode),
      ffa=False, ffs=float("nan"),
      tf=float(fr["sm"]["starpilotPlan"].tFollow), prob=float(lead.modelProb), minacc=float(fr["sm"]["starpilotPlan"].minAcceleration),
      model_stop=bool(fr["sm"]["modelV2"].action.shouldStop), forcing=bool(fr["sm"]["starpilotPlan"].forcingStop)))
  labels = {ln: src[ln - 1].strip() for r in out for ln in (r["final"], r["minline"]) if ln}
  if any("DEPRECATED" not in k for _, k in sg_compat.NOT_CONVERTED):
    print("note: old-log fields with no SG counterpart were dropped: " +
          ", ".join(f"{n}.{k} x{c}" for (n, k), c in sorted(sg_compat.NOT_CONVERTED.items()) if "DEPRECATED" not in k)[:600], flush=True)
  return out, labels


PLANNER_LOCALS = ("v_cruise", "effective_t_follow", "lead_control_active", "lead_one_active", "tracking_lead", "experimental_mode",
                  "uncertainty", "panic_bypass", "stopped_lead_obstacle_bias", "force_stop_x", "prev_accel_constraint",
                  "accel_limits_turns", "scene_v_ego", "early_truck_follow", "nonurgent_duplicate_vision_follow",
                  "steady_follow_filter_floor", "output_a_target_mpc", "output_a_target")
MPC_LOCALS = ("lead_0_obstacle", "lead_1_obstacle", "cruise_obstacle", "x_obstacles", "prev_source", "candidate_source",
              "sticky_source", "v_cruise_clipped", "lead_0_bias", "lead_1_bias")
OBS_IDX = (0, 1, 2, 3, 4, 6, 8, 10, 12)


def _install_dump_hooks(planner, lp):
  """Opt-in (--dump): record MPC inputs/outputs by wrapping the planner's MPC methods. Does not change any value."""
  import openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc as lm
  mpc = planner.mpc
  dbg = dict(rec=None, planner_locals={}, mpc_update_code=type(mpc).update.__code__, resets=0)

  def mpc_local_trace(frame, event, arg):
    if event == "return" and dbg["rec"] is not None:
      dbg["rec"]["mpc_locals"] = {k: (np.array(v, copy=True) if isinstance(v, np.ndarray) else v)
                                  for k, v in ((k, frame.f_locals.get(k)) for k in MPC_LOCALS)}
    return mpc_local_trace
  dbg["mpc_local_trace"] = mpc_local_trace

  def wrap(obj, name, grab):
    orig = getattr(obj, name)
    def w(*a, **k):
      if dbg["rec"] is not None:
        grab(a, k)
      return orig(*a, **k)
    setattr(obj, name, w)

  wrap(mpc, "set_weights", lambda a, k: dbg["rec"].__setitem__("set_weights", dict(k)))
  wrap(mpc, "set_accel_limits", lambda a, k: dbg["rec"].__setitem__("accel_limits", tuple(a)))
  wrap(mpc, "update", lambda a, k: dbg["rec"].__setitem__("update_args", dict(
    v_cruise=a[1], danger_factor=a[5], t_follow=a[6], **{kk: k[kk] for kk in ("personality", "tracking_lead", "stop_x",
    "lead_obstacle_bias", "smooth_duplicate_vision", "silverado_early_follow", "optional_far_lead_comfort") if kk in k})))

  orig_pl = mpc.process_lead
  def pl(lead, *a, **k):
    li = k.get("lead_index", 0)
    used = {"m": False}
    orig_b = lm.build_model_lead_trajectory
    def b(*aa, **kk):
      r = orig_b(*aa, **kk)
      used["m"] = r is not None
      return r
    lm.build_model_lead_trajectory = b
    try:
      r = orig_pl(lead, *a, **k)
    finally:
      lm.build_model_lead_trajectory = orig_b
    if dbg["rec"] is not None:
      ml = k.get("model_lead")
      dbg["rec"][f"lead{li}"] = dict(
        from_model_path=used["m"], active=bool(lead.status) and bool(a[0] if a else k.get("tracking_lead", True)),
        radar=dict(dRel=float(lead.dRel), vLead=float(lead.vLead), aLeadK=float(lead.aLeadK), aLeadTau=float(lead.aLeadTau),
                   radar=bool(lead.radar), status=bool(lead.status), yRel=float(lead.yRel), modelProb=float(lead.modelProb)),
        model_lead=None if ml is None else dict(prob=float(ml.prob), x0=float(ml.x[0]), v0=float(ml.v[0]), a0=float(ml.a[0]) if len(ml.a) else None),
        xv=np.array(r, copy=True),
        filt=dict(a=float(mpc.lead_a_filter.x), v=float(mpc.lead_v_filter.x), tau=float(mpc.lead_a_filter.alpha) if hasattr(mpc.lead_a_filter, "alpha") else None))
    return r
  mpc.process_lead = pl

  orig_reset = mpc.reset
  def rs(*a, **k):
    dbg["resets"] += 1
    if dbg["rec"] is not None:
      dbg["rec"]["reset_called"] = True
    return orig_reset(*a, **k)
  mpc.reset = rs
  orig_run = mpc.run
  def run(*a, **k):
    if dbg["rec"] is not None:
      dbg["rec"]["x0"] = np.array(mpc.x0, copy=True)
      dbg["rec"]["params"] = np.array(mpc.params, copy=True)
      dbg["rec"]["mode"] = mpc.mode
      dbg["rec"]["cruise_min_a"], dbg["rec"]["max_a"] = float(mpc.cruise_min_a), float(mpc.max_a)
    r = orig_run(*a, **k)
    if dbg["rec"] is not None:
      dbg["rec"]["solution_status"] = int(mpc.solution_status)
      dbg["rec"]["solve_time"] = float(mpc.solve_time)
      dbg["rec"]["a_sol"] = np.array(mpc.a_solution, copy=True) if mpc.solution_status == 0 else None
    return r
  mpc.run = run
  return dbg


def _plain(v):
  """Make a value picklable (capnp enums / readers -> str)."""
  if v is None or isinstance(v, (bool, int, float, str)):
    return v
  if isinstance(v, np.ndarray):
    return v.copy()
  if isinstance(v, (tuple, list)):
    return type(v)(_plain(x) for x in v)
  if isinstance(v, np.generic):
    return v.item()
  return str(v)


def _collect_dump(planner, fr, dbg, changes):
  rec = dbg["rec"]
  mpc = planner.mpc
  for key in ("update_args", "set_weights", "planner_locals_tmp"):
    if key in rec:
      rec[key] = {k: _plain(v) for k, v in rec[key].items()}
  # a_solution/v_solution as of the solve (reset() inside run() zeroes them on a failed solve)
  rec["a_solution_final"] = np.array(mpc.a_solution, copy=True)
  rec["v_solution_final"] = np.array(mpc.v_solution, copy=True)
  rec["x_sol"] = np.array(mpc.x_sol, copy=True)
  rec["crash_cnt"] = float(mpc.crash_cnt)
  rec["source"] = mpc.source
  rec["planner_locals"] = {k: _plain(v) for k, v in dbg["planner_locals"].items()}
  rec["chg"] = list(changes)
  rec["resets_total"] = dbg["resets"]
  rec["a_desired"] = float(planner.a_desired)
  rec["plan_flags"] = dict(redLight=bool(fr["sm"]["starpilotPlan"].redLight), forcingStop=bool(fr["sm"]["starpilotPlan"].forcingStop),
                           dangerFactor=float(fr["sm"]["starpilotPlan"].dangerFactor), tFollow=float(fr["sm"]["starpilotPlan"].tFollow),
                           vCruise=float(fr["sm"]["starpilotPlan"].vCruise), accelJerk=float(fr["sm"]["starpilotPlan"].accelerationJerk),
                           dangerJerk=float(fr["sm"]["starpilotPlan"].dangerJerk), speedJerk=float(fr["sm"]["starpilotPlan"].speedJerk),
                           minAccel=float(fr["sm"]["starpilotPlan"].minAcceleration))
  return rec


def _fmt(a, idx=OBS_IDX):
  return "[" + " ".join(f"{float(a[i]):.1f}" for i in idx) + "]"


def dump_report(results, args, labels):
  """Per-cycle table (and with --dump-full a full block per cycle) for the first replay (the logged delay)."""
  T0, T1 = args.dump
  d0, r0, _ = results[0]
  rows = [r for r in r0 if "dump" in r]
  print(f"\n== dump {T0}-{T1} s, delay {d0}: {len(rows)} cycles; obstacle/xv indices {OBS_IDX} = t "
        f"{[round(float(i), 1) for i in __import__('numpy').array(_tidx())[list(OBS_IDX)]]} s ==")
  print("   t       aOut  logA  src   stat solve_ms crash a_sol[0..5]                      L0.dRel  vRel  aLead  mdl0 | obs0(lead0) cruise(0,4,8)    | final L  mpc-> out")
  for r in rows:
    d = r["dump"]
    l0 = d.get("lead0", {})
    ra = l0.get("radar", {})
    a_sol = d["a_solution_final"]
    mloc = d.get("mpc_locals", {})
    obs0 = mloc.get("lead_0_obstacle"); cr = mloc.get("cruise_obstacle")
    print(f"  {r['t']:7.2f} {r['a']:6.2f} {r['log_a']:6.2f} {r['src']:5s} {d.get('solution_status', -1):3d} {d.get('solve_time', 0) * 1e3:7.2f} "
          f"{d['crash_cnt']:4.0f}  [{' '.join(f'{x:5.2f}' for x in a_sol[:6])}] "
          f"{ra.get('dRel', float('nan')):7.2f} {ra.get('vLead', 0) - r['v']:6.2f} {ra.get('aLeadK', 0):6.2f}  {'M' if l0.get('from_model_path') else 'r'}   | "
          f"{obs0[0] if obs0 is not None else float('nan'):6.1f} "
          f"{(cr[0] if cr is not None else float('nan')):6.1f} {(cr[4] if cr is not None else float('nan')):6.1f} {(cr[8] if cr is not None else float('nan')):6.1f} "
          f"| L{r['final']}{' reset' if d.get('reset_called') else ''}")
  if getattr(args, "dump_full", False):
    for r in rows:
      _print_block(r, labels)


def _tidx():
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
  return T_IDXS


def _print_block(r, labels):
  d = r["dump"]
  print(f"\n-- cycle t={r['t']:.2f} seg{r['seg']} aOut={r['a']:.3f} (log {r['log_a']:.3f}) mode={d.get('mode')} src={d['source']} "
        f"status={d.get('solution_status')} solve={d.get('solve_time', 0) * 1e3:.2f}ms crash_cnt={d['crash_cnt']} reset={d.get('reset_called', False)} "
        f"resets_total={d['resets_total']}")
  print(f"   x0={np.round(d['x0'], 3).tolist()} (x, v, a)  cruise_min_a={d['cruise_min_a']:.3f} max_a={d['max_a']:.3f} params ACCEL_MIN/max_a={d['params'][0, 0]:.2f}/{d['params'][0, 1]:.2f}")
  print(f"   update_args={ {k: (np.round(v, 3).tolist() if hasattr(v, 'tolist') else v) for k, v in d.get('update_args', {}).items()} }")
  print(f"   set_weights={ {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d.get('set_weights', {}).items()} } accel_limits={d.get('accel_limits')}")
  print(f"   starpilotPlan={d['plan_flags']}")
  pl = d["planner_locals"]
  print("   planner locals: " + ", ".join(f"{k}={(round(v, 3) if isinstance(v, float) else v)}" for k, v in pl.items() if k not in ('output_a_target', 'output_a_target_mpc') and v is not None))
  for li in (0, 1):
    L = d.get(f"lead{li}")
    if L:
      print(f"   lead{li}: active={L['active']} model-trajectory-path={L['from_model_path']} radar={ {k: round(v, 3) if isinstance(v, float) else v for k, v in L['radar'].items()} } "
            f"model_lead={L['model_lead']} filt={L['filt']}")
      print(f"           xv x={_fmt(L['xv'][:, 0])} v={_fmt(L['xv'][:, 1])}")
  m = d.get("mpc_locals", {})
  for k in ("lead_0_obstacle", "lead_1_obstacle", "cruise_obstacle"):
    if m.get(k) is not None:
      print(f"   {k} {_fmt(m[k])}")
  print(f"   prev_source={m.get('prev_source')} candidate={m.get('candidate_source')} sticky={m.get('sticky_source')} -> source {d['source']}; "
        f"params[:,2]=min obstacle {_fmt(d['params'][:, 2])}")
  print(f"   a_solution[0..5]={np.round(d['a_solution_final'][:6], 3).tolist()} a_sol idx {_fmt(d['a_solution_final'])}")
  print(f"   v_solution idx {_fmt(d['v_solution_final'])}  x_sol[:,0] idx {_fmt(d['x_sol'][:, 0])}")
  chg = " -> ".join(f"L{ln}:{v:.3f}" for ln, v in d["chg"])
  print(f"   post-processing (line: output_a_target): {chg}")
  for ln in {ln for ln, _ in d["chg"]}:
    print(f"      L{ln}: {labels.get(ln, '')}")



def onset(t, a, t0, t1, thr):
  m = (t >= t0) & (t <= t1) & (a < thr)
  return float(t[m][0]) if m.any() else None


def describe(labels, ln):
  return f"L{ln}: {labels.get(ln, '-')[:70]}" if ln else "mpc"


def match_report(res):
  a, la = np.array([r["a"] for r in res]), np.array([r["log_a"] for r in res])
  t = np.array([r["t"] for r in res])
  for name, m in (("all frames", np.ones_like(t, bool)), ("after 30 s warmup", t > t[0] + 30)):
    d = np.abs(a - la)[m]
    print(f"  {name}: n={m.sum()} |aTarget diff| median {np.median(d):.4f} p95 {np.percentile(d, 95):.4f} p99 {np.percentile(d, 99):.3f} "
          f"max {d.max():.3f}; within 0.05: {(d < 0.05).mean() * 100:.1f}%  within 0.2: {(d < 0.2).mean() * 100:.1f}%")
  m = t > t[0] + 30
  stop, lstop = np.array([r["stop"] for r in res])[m], np.array([r["log_stop"] for r in res])[m]
  print(f"  shouldStop agreement {np.mean(stop == lstop) * 100:.2f}%  (log true {lstop.sum()}, replay true {stop.sum()})")
  s, ls = np.array([r["src"] for r in res])[m], np.array([r["log_src"] for r in res])[m]
  print(f"  MPC source agreement {np.mean(s == ls) * 100:.2f}%")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route", nargs="?", default=DEFAULT_ROUTE)
  ap.add_argument("--segments", default="16-21")
  ap.add_argument("--delays", nargs="+", default=["0.5", "0.15"],
                  help="logged delay first, then the alternative, then optional split variants ACTION:CAPS (e.g. 0.5:0.15 = "
                       "0.5 s for action_t only, 0.15 s for every reaction_t cap)")
  ap.add_argument("--approach", type=float, nargs="*", default=[], help="pairs T0 T1 (route s): print a stopped/slow-lead approach profile")
  ap.add_argument("--events", type=float, nargs="*", default=list(EVENTS), help="route seconds of hard-braking events")
  ap.add_argument("--window", type=float, default=8.0, help="seconds around each event (+/-)")
  ap.add_argument("--sp-lag", type=int, default=0, help="use starpilotPlan this many cycles old (race between processes)")
  ap.add_argument("--dump", type=float, nargs=2, metavar=("T0", "T1"), help="opt-in debug: per-cycle MPC input/output table for route seconds T0..T1")
  ap.add_argument("--dump-full", action="store_true", help="with --dump: also print a full multi-line block per cycle")
  ap.add_argument("--load", help="with --dump: re-print from a pickle written by --save instead of replaying")
  ap.add_argument("--sync-filters", action="store_true", help="what-if: set mpc.sync_model_lead_filters=True (in memory) to test the stale-filter fix")
  ap.add_argument("--save", help="pickle the per-frame results here")
  ap.add_argument("--testing-ground", default=None, help="opt-in: SLOT:VARIANT[,..] read as active in the planner (e.g. 9:B for the Stinger far-follow slew)")
  ap.add_argument("--worker", help=argparse.SUPPRESS)
  ap.add_argument("--check-only", action="store_true", help="only replay the first delay and print the log match")
  args = ap.parse_args()
  if toolchain.SG:    # fail fast, before any worker starts (replay_sg checks again)
    if args.testing_ground:
      toolchain.require_old("--testing-ground")
    if args.sync_filters:
      toolchain.require_old("--sync-filters (mpc.sync_model_lead_filters)")
    if args.dump or args.dump_full:
      toolchain.require_sg_port("--dump", "it wraps the old MPC's set_weights / update / process_lead; the SG MPC has set_cost_weights / "
                                "update(radarstate, personality, *, follow_seconds, acceleration_max, stop_line_m, modelV2)")
    if args.approach:
      toolchain.require_sg_port("--approach", "long_mpc.desired_follow_distance no longer exists in the SG MPC")
  if args.load:
    results = pickle.load(open(args.load, "rb"))
    dump_report(results, args, {**results[0][2], **(results[1][2] if len(results) > 1 else {})})
    return
  lo, _, hi = args.segments.partition("-")
  segs = list(range(int(lo), int(hi or lo) + 1))

  delays = args.delays[:1] if args.check_only else args.delays
  if args.worker is not None:  # one replay in this process
    frames, cp = load_frames(args.route, segs, args.sp_lag)
    print(f"{len(frames)} planner cycles, route t {frames[0]['route_t']:.1f}..{frames[-1]['route_t']:.1f}; "
          f"logged CP delay {cp.longitudinalActuatorDelay}, fingerprint {cp.carFingerprint}", flush=True)
    out, labels = replay(frames, cp, args.worker, dump=tuple(args.dump) if args.dump else None, sync_filters=args.sync_filters, testing_ground=args.testing_ground)
    pickle.dump((args.worker, out, labels), open(args.save, "wb"))
    return
  # fork-after-import hangs (acados/capnp threads), so each delay runs in its own process
  import subprocess
  import tempfile
  tmp = tempfile.mkdtemp()
  procs = []
  for d in delays:
    f = os.path.join(tmp, f"r{d.replace(':', '_')}.pkl")
    cmd = [sys.executable, "-I", os.path.abspath(__file__), args.route, "--segments", args.segments, "--sp-lag", str(args.sp_lag),
           "--worker", d, "--save", f]
    if args.dump:
      cmd += ["--dump", str(args.dump[0]), str(args.dump[1])]
    if args.sync_filters:
      cmd += ["--sync-filters"]
    if args.testing_ground:
      cmd += ["--testing-ground", args.testing_ground]
    procs.append((f, subprocess.Popen(cmd)))
  for f, p in procs:
    assert p.wait() == 0, "worker failed"
  results = [pickle.load(open(f, "rb")) for f, _ in procs]
  if args.save:
    pickle.dump(results, open(args.save, "wb"))
  report(results, args)
  if args.dump:
    dump_report(results, args, {**results[0][2], **(results[1][2] if len(results) > 1 else {})})


def report(results, args):
  (d0, r0, lab0) = results[0]
  print(f"\n== sanity: replay with delay {d0} vs logged longitudinalPlan ==")
  match_report(r0)
  if len(results) < 2:
    return
  (d1, r1, lab1) = results[1]
  labels = {**lab0, **lab1}
  t = np.array([r["t"] for r in r0])
  A = {d: np.array([r["a"] for r in r]) for d, r, _ in results}
  print(f"\n== per event: delay {d0} vs {d1} (open loop; same logged car motion) ==")
  for ev in args.events:
    t0, t1 = ev - args.window, ev + args.window
    m = (t >= t0) & (t <= t1)
    if not m.any():
      print(f"event {ev}: not in the replayed segments"); continue
    for d, r, _ in results:
      a = A[d]
      i = np.where(m)[0][np.argmin(a[m])]
      row = r[i]
      ons = "/".join(f"{onset(t, a, t0, t1, th) - ev:+.2f}" if onset(t, a, t0, t1, th) is not None else "  - " for th in THRESHOLDS)
      print(f"event {ev:5.0f} delay {d:>9}: onset(-0.3/-0.5/-1.0) rel event {ons} s; peak {a[i]:6.2f} at {t[i] - ev:+.2f}s "
            f"v={row['v']:.1f} lead={row['lead']} d={row['d']:.1f} vl={row['vl']:.1f} al={row['al']:.2f} src={row['src']} "
            f"final[{describe(labels, row['final'])}] min[{describe(labels, row['minline'])}]")
    i = np.where(m)[0][np.argmin(A[d0][m])]
    print(f"    logged aTarget peak in window: {min(r['log_a'] for r, mm in zip(r0, m) if mm):.2f}")
  print_overall(results, labels)
  for k in range(0, len(args.approach) - 1, 2):
    approach_report(results, labels, args.approach[k], args.approach[k + 1])


def binder(r, labels):
  """Which code path set the published accel: the line with the largest drop, else the plan read; e2e vs MPC told apart."""
  chg = r["chg"]
  if not chg:
    return "none"
  drops = [(chg[k][1] - chg[k - 1][1], chg[k][0]) for k in range(1, len(chg))]
  line = min(drops)[1] if drops and min(drops)[0] < -0.02 else chg[0][0]
  if line == 2459:
    return "e2e desiredAccel (experimental min)" if r["e2e"] <= chg[0][1] + 1e-6 else "MPC (min with e2e)"
  if line == 2456:
    return "MPC (src %s)" % r["src"]
  txt = labels.get(line) or ""
  name = txt.split("=")[1].strip()[:40] if "=" in txt else txt
  return {2998: "clip to accel limits"}.get(line, f"L{line} {name}")


def approach_report(results, labels, T0, T1):
  try:
    from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance
  except ImportError:
    toolchain.require_sg_port("--approach", "long_mpc.desired_follow_distance no longer exists in the SG MPC")
  print(f"\n== approach {T0:.0f}-{T1:.0f} s (logged car motion; lead gap = radarState.leadOne.dRel) ==")
  r0 = results[0][1]
  t = np.array([r["t"] for r in r0])
  m = (t >= T0) & (t <= T1)
  idx = np.where(m)[0]
  if not len(idx):
    print("  not in the replayed segments"); return
  v = np.array([r["v"] for r in r0])
  names = [d for d, _, _ in results]
  print("  t      v   gap  vl  desGap  exp tf  e2e   | " + " | ".join(f"a({d}) binder" for d in names))
  last = -9
  for i in idx:
    if t[i] - last < 1.0:
      continue
    last = t[i]
    r = r0[i]
    des = desired_follow_distance(r["v"], r["vl"], r["tf"]) if r["lead"] else float("nan")
    cols = " | ".join(f"{res[1][i]['a']:5.2f} {binder(res[1][i], labels)[:34]}" for res in results)
    print(f"  {t[i]:6.1f} {r['v']:4.1f} {r['d']:5.1f} {r['vl']:4.1f} {des:5.1f}  {int(r['exp'])}  {r['tf']:.2f} {r['e2e']:5.2f} | {cols}")
  below = idx[v[idx] < 2.5]
  stopped = idx[v[idx] < 0.1]
  if len(below):
    t_end = t[stopped[0]] if len(stopped) and stopped[0] > below[0] else t[idx[-1]]
    print(f"  logged motion: first v<2.5 m/s at {t[below[0]]:.1f} s (gap {r0[below[0]]['d']:.1f} m); "
          f"{'stopped' if len(stopped) and stopped[0] > below[0] else 'window end'} at {t_end:.1f} s -> time below 2.5 m/s {t_end - t[below[0]]:.1f} s; "
          f"final gap {r0[idx[-1]]['d']:.1f} m")
  for d, rr, _ in results:
    a = np.array([x["a"] for x in rr])
    on = "/".join(f"{onset(t, a, T0, T1, th) - T0:+.1f}" if onset(t, a, T0, T1, th) is not None else "-" for th in THRESHOLDS)
    hard = idx[a[idx] < -1.0]
    pre = hard[v[hard] >= 2.5] if len(hard) else hard
    line = f"  delay {d:>9}: onset(-0.3/-0.5/-1.0) {on} s after {T0:.0f}; peak {a[idx].min():.2f}"
    if len(hard):
      e = (pre if len(pre) else hard)[-1]
      line += (f"; hard phase (a<-1.0) {t[hard[0]]:.1f}-{t[e]:.1f} s ({len(hard) * 0.05:.1f} s total), last at v={v[e]:.1f} m/s gap {rr[e]['d']:.1f} m")
    print(line)
    for nm, sel in (("hard (a<-1.0)", hard), ("creep (v<2.5, a<0)", below[a[below] < 0] if len(below) else below)):
      if len(sel):
        c = Counter(binder(rr[i], labels) for i in sel)
        print(f"     binding during {nm}, {len(sel)} frames: " + "; ".join(f"{k} x{n}" for k, n in c.most_common(4)))


def print_overall(results, labels):
  (d0, r0, _), (d1, r1, _) = results[0], results[1]
  t = np.array([r["t"] for r in r0])
  a0, a1 = (np.array([r["a"] for r in r]) for r in (r0, r1))
  v = np.array([r["v"] for r in r0])
  lead = np.array([r["lead"] for r in r0])
  m = lead & (v < 20) & (v > 3) & (t > t[0] + 30) & ((a0 < -0.1) | (a1 < -0.1))
  diff = a1 - a0
  print(f"\n== overall, frames with a lead, 3<v<20 m/s and either run braking (< -0.1): n={m.sum()} of {lead.sum()} lead frames ==")
  for nm, d in ((f"a({d1}) - a({d0})", diff[m]),):
    print(f"  {nm}: mean {d.mean():+.3f} median {np.median(d):+.3f} p5 {np.percentile(d, 5):+.3f} p95 {np.percentile(d, 95):+.3f}")
  print(f"  fraction where delay {d0} brakes harder by >0.1: {(diff[m] > 0.1).mean() * 100:.1f}%  by >0.3: {(diff[m] > 0.3).mean() * 100:.1f}%; "
        f"the {d1} run harder by >0.1: {(diff[m] < -0.1).mean() * 100:.1f}%")
  allm = (t > t[0] + 30)
  print(f"  all frames (any lead): |diff| median {np.median(np.abs(diff[allm])):.4f}, frames differing >0.05: {(np.abs(diff[allm]) > 0.05).mean() * 100:.1f}%")
  for d, r, _ in results:
    a = np.array([x["a"] for x in r])
    br = m & (a < -0.1)
    c = Counter((x["final"]) for x, b in zip(r, br) if b)
    print(f"  final-binding line when braking, delay {d}: " + "; ".join(f"{describe(labels, k)} x{n}" for k, n in c.most_common(6)))
  # episodes: lead-approach braking = contiguous runs with a < -0.5 and lead
  print(f"\n  braking episodes (a < -0.5 for >= 0.5 s with lead, v < 20): onset/peak by delay")
  for d, r, _ in results:
    a = np.array([x["a"] for x in r])
    ep = episodes(t, a, lead & (v < 20), -0.5)
    print(f"   delay {d}: {len(ep)} episodes, median peak {np.median([e[2] for e in ep]) if ep else float('nan'):.2f}, "
          f"min peak {min([e[2] for e in ep]) if ep else float('nan'):.2f}")
  # matched: for each episode in run 0 find the first time run 1 crosses
  print("  matched onsets (episode in run %s): onset(-0.5) in run %s minus in run %s, peak diff" % (d0, d1, d0))
  ep0 = episodes(t, a0, lead & (v < 20), -0.5)
  rows = []
  for (s, e, pk) in ep0:
    lo = max(0, s - 100)
    i1 = next((i for i in range(lo, e + 1) if a1[i] < -0.5), None)
    rows.append(((t[i1] - t[s]) if i1 is not None else np.nan, a1[s:e + 1].min() - pk))
  if rows:
    on = np.array([r[0] for r in rows]); pd = np.array([r[1] for r in rows])
    print(f"   {len(rows)} episodes; onset shift median {np.nanmedian(on):+.2f}s (neg = {d1} earlier), p10/p90 {np.nanpercentile(on, 10):+.2f}/{np.nanpercentile(on, 90):+.2f}; "
          f"never crosses in {d1}: {np.isnan(on).sum()}; peak diff median {np.median(pd):+.2f}")


def episodes(t, a, ok, thr, min_len=0.5):
  out, i, n = [], 0, len(a)
  while i < n:
    if ok[i] and a[i] < thr:
      j = i
      while j < n and a[j] < thr and ok[j]:
        j += 1
      if t[j - 1] - t[i] >= min_len:
        out.append((i, j - 1, float(a[i:j].min())))
      i = j
    else:
      i += 1
  return out


if __name__ == "__main__":
  main()
