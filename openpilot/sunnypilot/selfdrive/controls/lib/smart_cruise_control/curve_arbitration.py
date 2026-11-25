"""Comfort arbitration for the Tesla curve-brake candidate.

Ported from BogPilot (`selfdrive/controls/lib/ap1_curve_arbitration.py`, MIT).

The planner picks min() over its acceleration candidates. Each candidate is
smooth on its own, but a switch between the cruise request (+0.7 m/s^2) and a
curve brake that starts at -0.07 m/s^2 is a 0.8 m/s^2 step in one frame, and the
release is the same step back up.

CurveArbiter keeps the min() (it can only ever ADD braking relative to each
candidate) and slews only the comfort class (cruise + curve brake):

  comfort = min(cruise, curve brake), then at most COMFORT_JERK_DOWN m/s^3 more
            braking per second and COMFORT_JERK_UP m/s^3 less braking
  output  = min(comfort, every other candidate)  -- lead MPC, e2e, stopping are never slewed

A comfort request at or below BYPASS_ACCEL (near the AP1 regen floor) passes
straight through, so strong braking is never held back.
"""
from __future__ import annotations

COMFORT_JERK_DOWN = 5.0   # m/s^3, more braking
COMFORT_JERK_UP = 1.2     # m/s^3, less braking (same as the curve brake release jerk)
BYPASS_ACCEL = -1.0       # m/s^2, comfort requests at or below this are never slewed
COMFORT_NAMES = ("cruise", "curve")
DT_MIN, DT_MAX = 0.01, 0.2


class CurveArbiter:
  def __init__(self, dt: float):
    self.dt = dt
    self.comfort = 0.0
    self.initialized = False
    self.bypassing = False
    self.last = {}

  def reset(self, a: float = 0.0):
    self.comfort = float(a)
    self.initialized = True
    self.bypassing = False

  def arbitrate(self, candidates, dt: float, reset: bool):
    """candidates: [(name, accel)] -> (accel, winner name, record)."""
    dt = float(min(max(dt, DT_MIN), DT_MAX))
    comfort_raw = min((a for n, a in candidates if n in COMFORT_NAMES), default=None)
    comfort = comfort_raw
    bypass = False
    if comfort_raw is not None:
      if reset or not self.initialized:
        self.comfort = comfort_raw
        self.initialized = True
      elif comfort_raw <= BYPASS_ACCEL:
        self.comfort = comfort_raw
        bypass = True
      else:
        self.comfort = float(min(max(comfort_raw, self.comfort - COMFORT_JERK_DOWN * dt),
                                 self.comfort + COMFORT_JERK_UP * dt))
      comfort = self.comfort
    self.bypassing = bypass
    others = [(n, a) for n, a in candidates if n not in COMFORT_NAMES]
    pool = others + ([("comfort", comfort)] if comfort is not None else [])
    winner, out = min(pool, key=lambda c: c[1])
    if winner == "comfort":
      winner = "curve" if any(n == "curve" and a <= comfort_raw + 1e-9 for n, a in candidates) else "cruise"
    self.last = {"winner": winner, "out": float(out),
                 "candidates": {n: float(a) for n, a in candidates},
                 "comfort_raw": comfort_raw, "comfort": comfort, "bypass": bypass, "dt": dt}
    return float(out), winner, {"bypass": bypass, "comfort_raw": comfort_raw, "comfort": comfort}
