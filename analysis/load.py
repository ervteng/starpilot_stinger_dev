"""Pull the longitudinal signals the stop analyses need out of one log file."""
from logs import Series, events


def load_long(path: str) -> dict:
  s = {k: Series() for k in ("cs", "cc", "co", "rs", "lp", "dm", "md")}
  info = {}
  for e in events(path):
    w = e.which()
    t = e.logMonoTime
    if w == "carState":
      c = e.carState
      s["cs"].add(t, dict(v=c.vEgo, a=c.aEgo, standstill=c.standstill, brake=c.brakePressed, gas=c.gasPressed))
    elif w == "carControl":
      c = e.carControl
      s["cc"].add(t, dict(long_active=c.longActive, state=str(c.actuators.longControlState), accel=c.actuators.accel))
    elif w == "carOutput":
      s["co"].add(t, e.carOutput.actuatorsOutput.accel)
    elif w == "radarState":
      l = e.radarState.leadOne
      s["rs"].add(t, dict(status=l.status, d=l.dRel, v=l.vLead, a=l.aLeadK, y=l.yRel, radar=l.radar))
    elif w == "longitudinalPlan":
      p = e.longitudinalPlan
      s["lp"].add(t, dict(a_target=p.aTarget, should_stop=p.shouldStop, v_end=p.speeds[-1] if len(p.speeds) else 0.0))
    elif w == "drivingModelData":
      s["dm"].add(t, e.drivingModelData.action.shouldStop)
    elif w == "modelV2":
      l = e.modelV2.leadsV3
      s["md"].add(t, (l[0].prob, l[0].x[0]) if len(l) else (0.0, 0.0))
    elif w == "initData" and "commit" not in info:
      info.update(commit=e.initData.gitCommit[:10], branch=e.initData.gitBranch, dirty=e.initData.dirty)
    elif w == "carParams" and "fingerprint" not in info:
      p = e.carParams
      info.update(fingerprint=str(p.carFingerprint), op_long=p.openpilotLongitudinalControl,
                  radar_unavailable=p.radarUnavailable, v_ego_stopping=round(p.vEgoStopping, 2),
                  stopping_decel_rate=round(p.stoppingDecelRate, 2))
  s["info"] = info
  s["t0"] = s["cs"].t[0] if s["cs"].t else 0
  return s


def find_stops(s: dict, moving_speed=1.0, stopped_speed=0.05):
  """Times (s from segment start) where the car comes to a standstill after moving, with who was driving."""
  stops, moving = [], False
  for t, c in zip(s["cs"].t, s["cs"].v):
    if c["v"] > moving_speed:
      moving = True
    if moving and c["v"] < stopped_speed:
      cc = s["cc"].at(t)
      stops.append(((t - s["t0"]) / 1e9, bool(cc and cc["long_active"]), c["brake"]))
      moving = False
  return stops
