# 20 Hz per-segment dump for low-speed gap analysis. Usage: stinger_gap_dump.py RLOG TAG  (writes /tmp/gap_TAG_SEG.pkl)
# columns: mono_s, vEgo, aEgo, gas, brake, standstill, longActive, override, dRel, vLead, vRel, present, radar, expMode, planSource(str), aTarget, accelCmd
import sys, pickle
from openpilot.tools.lib.logreader import LogReader
f = sys.argv[1]; tag = sys.argv[2]; seg = int(f.split('--')[2]); out = []
cc = rs = ss = lp = None; n = 0
for m in LogReader(f):
  w = m.which()
  if w == 'carControl': cc = m.carControl
  elif w == 'radarState': rs = m.radarState.leadOne
  elif w == 'selfdriveState': ss = m.selfdriveState
  elif w == 'longitudinalPlan': lp = m.longitudinalPlan
  elif w == 'carState' and cc is not None:
    n += 1
    if n % 5: continue
    s = m.carState
    out.append((m.logMonoTime * 1e-9, s.vEgo, s.aEgo, int(s.gasPressed), int(s.brakePressed), int(s.standstill),
                int(cc.longActive), int(cc.cruiseControl.override),
                rs.dRel if rs else -1, rs.vLead if rs else 0, rs.vRel if rs else 0,
                int(rs.present) if rs else 0, int(rs.radar) if rs else 0,
                int(ss.experimentalMode) if ss else -1,
                str(lp.longitudinalPlanSource) if lp else 'none',
                lp.aTarget if lp else float('nan'), cc.actuators.accel))
pickle.dump(out, open(f'/tmp/gap_{tag}_{seg}.pkl', 'wb'))
