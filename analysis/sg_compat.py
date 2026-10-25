"""Helpers for running the planner tools against the SecretGoodStarPilot (STINGER_TOOLCHAIN=sg) planner / MPC.

Only imported on the sg path of the planner tools; nothing here is used by the old toolchain.

* `convert()` copies a message read with the OLD schema (drive logs) into a new message of the SG schema, by field name (SG renamed
  a few: RadarState.LeadData.status -> present, liveParameters -> vehicleParameters), dropping what SG no longer has.
* `SgSM` is the minimal SubMaster the SG planner needs (valid / alive / freq_ok / recv_time / logMonoTime are plain dicts, as
  `qualified_model_leads` checks them with .get()).
* `ModelLeadFeed` replaces the real-time-clock-gated `qualified_model_leads` with an offline equivalent: the model frame the radar
  state was computed from (radarState.mdMonoTime) is looked up in the frames seen so far.
"""
import types

RENAMES = {
  # (struct short name, old field) -> sg field
  ("LeadData", "status"): "present",
}
NOT_CONVERTED = {}   # (struct short name) -> number of fields dropped, filled while converting (for the run summary)


def _short(builder):
  return builder.schema.node.displayName.rsplit(":", 1)[-1].rsplit(".", 1)[-1]


def _fill(b, d):
  fields = b.schema.fields
  name = _short(b)
  for k, v in d.items():
    k = RENAMES.get((name, k), k)
    if k not in fields:
      NOT_CONVERTED[(name, k)] = NOT_CONVERTED.get((name, k), 0) + 1
      continue
    try:
      proto = fields[k].proto
      if proto.which() == "group":
        _fill(getattr(b, k), v)
        continue
      t = proto.slot.type
      kind = t.which()
      if kind == "struct":
        _fill(b.init(k), v)
      elif kind == "list" and t.list.elementType.which() == "struct":
        lst = b.init(k, len(v))
        for i, e in enumerate(v):
          _fill(lst[i], e)
      else:
        setattr(b, k, v)
    except Exception:    # a type changed between the schemas: leave the SG default
      NOT_CONVERTED[(name, k)] = NOT_CONVERTED.get((name, k), 0) + 1


def convert(old_reader, sg_builder):
  """Fill the SG struct builder `sg_builder` from an old-schema reader (same-named fields; see RENAMES)."""
  _fill(sg_builder, old_reader.to_dict())
  return sg_builder


def new_service(messaging, service, old_reader=None):
  """messaging.new_message(service) with the service struct filled from old_reader; returns (event builder, struct builder)."""
  ev = messaging.new_message(service)
  sb = getattr(ev, service)
  if old_reader is not None:
    convert(old_reader, sb)
  return ev, sb


class SgSM:
  """SubMaster stand-in for the SG LongitudinalPlanner.update (indexing + the dicts it .get()s)."""
  def __init__(self, d, mono, frame, recv=None):
    self.d = d
    names = list(d)
    self.logMonoTime = {k: mono.get(k, 0) if isinstance(mono, dict) else mono for k in names}
    self.recv_time = {k: (self.logMonoTime[k] / 1e9 if recv is None else recv) for k in names}
    self.valid = dict.fromkeys(names, True)
    self.alive = dict.fromkeys(names, True)
    self.freq_ok = dict.fromkeys(names, True)
    self.frame = frame
    self.updated = dict.fromkeys(names, True)

  def __getitem__(self, k):
    return self.d[k]

  def all_checks(self, *a, **k):
    return True


class ModelLeadFeed:
  """Offline stand-in for long_mpc.qualified_model_leads: returns the modelV2 whose logMonoTime equals radarState.mdMonoTime."""
  def __init__(self, keep=8):
    self.frames = {}
    self.keep = keep

  def observe(self, mono, model_reader):
    if mono not in self.frames:
      self.frames[mono] = model_reader
      while len(self.frames) > self.keep:
        del self.frames[min(self.frames)]

  def __call__(self, sm, now_ns, history=None):
    return self.frames.get(int(sm["radarState"].mdMonoTime))


def patch_planner_clock(planner_module, mpc_module, clock):
  """Make the planner + MPC modules read `clock()` (seconds) instead of the wall clock; returns the clock_ns callable for the planner."""
  ns = types.SimpleNamespace(monotonic=clock, time=clock, monotonic_ns=lambda: int(clock() * 1e9), sleep=lambda s: None)
  for mod in (planner_module, mpc_module):
    mod.time = ns
  return ns.monotonic_ns
