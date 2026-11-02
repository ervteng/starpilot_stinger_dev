# 20 Hz per-segment dump for the Stinger brake-delivery work. Usage: stinger_brake_delivery_dump.py RLOG TAG  (writes /tmp/bd_TAG_SEG.pkl)
# columns: mono_s, vEgo, aEgo, gas, brake, standstill, req(carControl.actuators.accel), out(carOutput.actuatorsOutput.accel), longActive, enabled, longControlState, lead dRel, lead vLead, lead present, override
import sys, pickle
from openpilot.tools.lib.logreader import LogReader
f=sys.argv[1]; tag=sys.argv[2]; seg=int(f.split('--')[2]); out=[]
cc=co=rs=None; n=0
for m in LogReader(f):
  w=m.which()
  if w=='carControl': cc=m.carControl
  elif w=='carOutput': co=m.carOutput
  elif w=='radarState': rs=m.radarState.leadOne
  elif w=='carState' and cc is not None:
    n+=1
    if n%5: continue
    s=m.carState
    out.append((m.logMonoTime*1e-9,s.vEgo,s.aEgo,int(s.gasPressed),int(s.brakePressed),int(s.standstill),cc.actuators.accel,co.actuatorsOutput.accel if co else float('nan'),int(cc.longActive),int(cc.enabled),int(cc.actuators.longControlState.raw) if hasattr(cc.actuators.longControlState,'raw') else int(cc.actuators.longControlState),rs.dRel if rs else -1,rs.vLead if rs else 0,int(rs.present) if rs else 0,int(cc.cruiseControl.override)))
pickle.dump(out,open(f'/tmp/bd_{tag}_{seg}.pkl','wb'))
