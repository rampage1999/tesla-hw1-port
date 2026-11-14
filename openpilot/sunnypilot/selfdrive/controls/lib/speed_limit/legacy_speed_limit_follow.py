"""Legacy Tesla (Model S/X 2012-2016) speed-limit follow.

sunnypilot's Speed Limit Assist only ever *lowers* the planner target: the planner
picks `min(cruise, sccVision, sccMap, speedLimitAssist)`, and SLA's state machine
confirms set-speed changes through the PCM/cluster path, which an AP1 car does not
have. So on these cars a higher posted limit can never raise the cruise target.

This module raises or lowers the planner's cruise target directly when the posted
limit changes, in the spirit of BogPilot's `slc_raise` (MIT), with two guards:

- the new limit must persist CONFIRM_S seconds before it is adopted, so a single
  flapping Mobileye frame does not move the set speed;
- the change must be at least MIN_STEP (1 mph) away from the current target;
- a manual set-speed change from the driver always wins: if the incoming cruise
  target moved, the follow base is reset to it.

Input `limit_final` already includes the user's SLC offset (resolver.speed_limit_final_last).
"""
from __future__ import annotations

CONFIRM_S = 2.0
MIN_STEP_MS = 1.0 * 0.44704  # 1 mph in m/s
# Never follow a limit below this (parking-lot signs / map noise).
MIN_LIMIT_MS = 5.0 * 0.44704  # 5 mph


class LegacySpeedLimitFollow:
  def __init__(self, confirm_s: float = CONFIRM_S, min_step: float = MIN_STEP_MS):
    self.confirm_s = confirm_s
    self.min_step = min_step
    self.base = None          # cruise target we last produced / adopted
    self.prev_in = None       # last v_cruise seen from controlsd
    self.pending = None       # candidate limit
    self.timer = 0.0
    self.applied = False      # True on the step a new limit was adopted

  def reset(self):
    self.base = None
    self.prev_in = None
    self.pending = None
    self.timer = 0.0
    self.applied = False

  def update(self, v_cruise_in: float, limit_final: float, enabled: bool, dt: float) -> tuple[float, bool]:
    """Returns (cruise target to use this step, adopted_new_limit)."""
    self.applied = False
    v_cruise_in = float(v_cruise_in)

    # Driver changed the set speed (stalk / UI): adopt it and forget any pending limit.
    if self.prev_in is not None and abs(v_cruise_in - self.prev_in) > 1e-3:
      self.base = v_cruise_in
      self.pending = None
      self.timer = 0.0
    self.prev_in = v_cruise_in

    if self.base is None:
      self.base = v_cruise_in

    if not enabled:
      self.pending = None
      self.timer = 0.0
      self.base = v_cruise_in
      return v_cruise_in, False

    limit_final = float(limit_final)
    if limit_final < MIN_LIMIT_MS:
      # No usable limit: hold the current target, forget the candidate.
      self.pending = None
      self.timer = 0.0
      return self.base, False

    if abs(limit_final - self.base) < self.min_step:
      self.pending = None
      self.timer = 0.0
      return self.base, False

    if self.pending is not None and abs(limit_final - self.pending) <= self.min_step:
      self.timer += dt
    else:
      self.pending = limit_final
      self.timer = 0.0

    if self.timer >= self.confirm_s:
      self.base = limit_final
      self.pending = None
      self.timer = 0.0
      self.applied = True

    return self.base, self.applied
