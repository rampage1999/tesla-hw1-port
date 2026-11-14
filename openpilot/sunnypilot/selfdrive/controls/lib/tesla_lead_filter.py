"""Legacy Tesla (AP1) one-frame radar lead glitch filter.

Ported from BogPilot (`selfdrive/car/tesla/lead_filter.py`, MIT), adapted to this
tree's radard lead dict (`present` instead of `status`; keys dRel/yRel/vRel/vLead/
modelProb/radar/present).

Why: a single radar frame can swap the lead for a roadside stationary track, or a
close radar-only point can win radard's low-speed override under a confident
camera lead. Either one steps the plan by 1-3 m/s^2 in 50 ms — a friction-brake
thunk. This filter delays such a change by at most one radar frame (50 ms) and
never delays a lead appearing or disappearing.

Rules, per lead slot, at the radard rate (20 Hz):
- Camera-confirmed leads are never filtered (model prob above threshold and the
  lead's dRel / vLead agree with the model lead).
- A radar-only lead closer than CLOSE_RADAR_ONLY_M that is new this frame is
  dropped for that frame. Still there next frame -> accepted.
- A jump vs the previous output holds the previous lead (dRel extrapolated by
  vRel*dt) for one frame. Agreement next frame -> accepted.
- A close radar-only point well short of a confident camera lead while the
  previous output was a lead is held for up to CONTRADICT_HOLD_FRAMES frames.
Pure Python: no CAN, no panda change.
"""
from __future__ import annotations

CLOSE_RADAR_ONLY_M = 3.0
JUMP_D_M = 5.0
JUMP_V_MS = 3.0
CONFIRM_D_M = 4.0
CONFIRM_D_FRAC = 0.15
CONFIRM_V_MS = 3.0
CONTRADICT_PROB = 0.9
CONTRADICT_GAP_M = 4.0
CONTRADICT_HOLD_FRAMES = 4


def _present(lead) -> bool:
  return bool(lead.get("present", False))


def camera_confirms(lead, model_prob, model_x, model_v, prob_threshold) -> bool:
  if not _present(lead) or model_prob <= prob_threshold:
    return False
  if not lead.get("radar", False):
    return True  # vision lead
  d = lead["dRel"]
  return (abs(d - model_x) <= max(CONFIRM_D_M, CONFIRM_D_FRAC * d) and
          abs(lead["vLead"] - model_v) <= CONFIRM_V_MS)


def camera_contradicts(lead, model_prob, model_x) -> bool:
  """A close radar-only lead well short of a confident camera lead."""
  return (_present(lead) and bool(lead.get("radar", False)) and lead["dRel"] < CLOSE_RADAR_ONLY_M and
          model_prob >= CONTRADICT_PROB and model_x - lead["dRel"] > CONTRADICT_GAP_M)


def _jumped(a, b, dt) -> bool:
  pred = a["dRel"] + a.get("vRel", 0.0) * dt
  return abs(b["dRel"] - pred) > JUMP_D_M or abs(b["vLead"] - a["vLead"]) > JUMP_V_MS


class LegacyLeadGlitchFilter:
  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.prev = None
    self.pending = None
    self.last_held = False
    self.held = 0
    self.contradict = 0

  def update(self, lead, model_prob, model_x, model_v, prob_threshold):
    if camera_contradicts(lead, model_prob, model_x):
      self.contradict += 1
      if self.prev is not None and self.contradict <= CONTRADICT_HOLD_FRAMES:
        out = self._hold()
        self.pending = None
        self.last_held = True
        self.prev = dict(out) if _present(out) else None
        return out
      if self.contradict > CONTRADICT_HOLD_FRAMES:
        # Held long enough and still there: accept it as a real close object.
        self.pending = None
        self.last_held = False
        self.prev = dict(lead)
        return lead
    else:
      self.contradict = 0

    out = lead
    suspect = False
    if _present(lead) and not camera_confirms(lead, model_prob, model_x, model_v, prob_threshold):
      jump = self.prev is not None and _jumped(self.prev, lead, self.dt)
      new_close = lead["dRel"] < CLOSE_RADAR_ONLY_M and self.prev is None
      suspect = new_close or jump
    if suspect:
      if self.last_held or (self.pending is not None and not _jumped(self.pending, lead, self.dt)):
        # Seen two frames in a row, or held last frame already: accept.
        self.pending = None
        self.last_held = False
      else:
        self.pending = dict(lead)
        out = self._hold()
        self.last_held = True
    else:
      self.pending = None
      self.last_held = False
    self.prev = dict(out) if _present(out) else None
    return out

  def _hold(self):
    self.held += 1
    if self.prev is None:
      return {"present": False}
    h = dict(self.prev)
    h["dRel"] = h["dRel"] + h.get("vRel", 0.0) * self.dt
    return h
