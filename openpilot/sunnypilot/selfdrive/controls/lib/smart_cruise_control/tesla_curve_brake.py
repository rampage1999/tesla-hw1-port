"""Legacy Tesla look-ahead curve brake for sunnypilot's Smart Cruise Control.

Ported from BogPilot (`opendbc/car/tesla_ap1/curve_speed.py`, MIT). The pure
math is unchanged; the CSC target and the engagement gate live in the caller,
because sunnypilot already computes a curve target in
`SmartCruiseControlVision`.

The reactive CSC candidate only brakes gently toward its target, so the car can
arrive at the tightest part of a curve still above the CSC speed. This module
asks for the steady decel needed to reach sqrt(limit / curvature) LEAD_S before
every node of the model path, capped so the car lands on the curve speed
instead of braking below it, clipped to the AP1 regen range, and jerk limited.

It can only ADD braking: the planner appends the request as one more min()
candidate. Panda safety is untouched.
"""
from __future__ import annotations

import numpy as np

TESLA_CSC_MAX_LAT_ACCEL = 2.0  # m/s^2, same cap as sunnypilot's _A_LAT_REG_MAX
TESLA_CURVE_LEAD_S = 0.5       # s, reach the curve speed this long before the node

DEADBAND = 0.15                # m/s^2, a request must be this strong to start braking
DEADBAND_OFF = 0.08            # m/s^2, once braking, keep going until weaker than this
ON_DEBOUNCE_S = 0.3            # s, a weak request must stay past DEADBAND this long to start
STRONG_REQUEST = 0.3           # m/s^2, a request this strong starts at once
MIN_ON_S = 0.5                 # s, once started, the deadband cannot switch it off sooner
A_MIN = -1.2                   # m/s^2, stays in the AP1 regen range (no friction thunk)
ONSET_JERK = 1.5               # m/s^3
RELEASE_JERK = 1.2             # m/s^3
TARGET_MARGIN = 0.0            # m/s, land this far above the curve speed
ACTUATOR_LAG_S = 0.3           # s, delay + lag between the request and the car's decel
MIN_MODEL_SPEED = 5.0          # m/s, skip nodes the model plans to drive slower than this
MIN_CURVATURE = 1e-4           # 1/m, straighter than this never limits speed
MIN_DISTANCE = 5.0             # m, floor on the braking distance


def _path_curve_speeds(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel):
  """(t, x, v_curve, valid) for the model path nodes; v_curve = sqrt(limit / curvature)."""
  t = np.asarray(t_idxs, dtype=float)
  x = np.asarray(x_path, dtype=float)
  v = np.asarray(v_path, dtype=float)
  yaw = np.asarray(yaw_rate_path, dtype=float)
  n = min(len(t), len(x), len(v), len(yaw))
  t, x, v, yaw = t[:n], x[:n], v[:n], yaw[:n]
  valid = v >= MIN_MODEL_SPEED
  curvature = np.abs(yaw) / np.clip(v, MIN_MODEL_SPEED, None)
  valid &= (curvature > MIN_CURVATURE) & (t > 0.0)
  v_curve = np.sqrt(max_lat_accel / np.where(valid, curvature, 1.0))
  return t, x, v_curve, valid


def curve_target_speed(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL) -> float:
  """Curve speed (m/s) for the node with the most lateral acceleration; inf when nothing limits."""
  v = np.asarray(v_path, dtype=float)
  yaw = np.asarray(yaw_rate_path, dtype=float)
  n = min(len(v), len(yaw))
  if n == 0 or max_lat_accel <= 0.0:
    return float('inf')
  v, yaw = v[:n], yaw[:n]
  i = int(np.argmax(np.abs(yaw * v)))
  curvature = abs(yaw[i] * v[i]) / max(v[i], 1.0) ** 2
  if curvature <= MIN_CURVATURE:
    return float('inf')
  return float(np.sqrt(max_lat_accel / curvature))


def curve_brake_request(v_ego, t_idxs, x_path, v_path, yaw_rate_path,
                        max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL, lead_s=TESLA_CURVE_LEAD_S) -> float:
  """Steady decel (<= 0, m/s^2) needed to reach every node's curve speed LEAD_S early.

  Returns 0.0 when no node needs braking or the input is unusable.
  """
  try:
    v_ego = float(v_ego)
  except (TypeError, ValueError):
    return 0.0
  if not np.isfinite(v_ego) or v_ego <= 0.0 or max_lat_accel <= 0.0:
    return 0.0
  _, x, v_curve, valid = _path_curve_speeds(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
  if not np.any(valid):
    return 0.0
  # Distance left to slow down: up to the node, minus the lead time at current speed.
  dist = x - v_ego * lead_s
  valid &= v_curve < v_ego
  if not np.any(valid):
    return 0.0
  dist = np.clip(dist, MIN_DISTANCE, None)
  decel = (v_ego ** 2 - v_curve ** 2) / (2.0 * dist)
  return -float(np.max(np.where(valid, decel, 0.0)))


def landing_decel(speed_gap: float) -> float:
  """Strongest decel (>= 0, m/s^2) that still lands the car TARGET_MARGIN above the curve speed."""
  g = float(speed_gap) - TARGET_MARGIN
  if not np.isfinite(g):
    return float('inf')
  if g <= 0.0:
    return 0.0
  return RELEASE_JERK * (np.sqrt(ACTUATOR_LAG_S ** 2 + 2.0 * g / RELEASE_JERK) - ACTUATOR_LAG_S)


class TeslaCurveBrake:
  """Curve brake request (<= 0, m/s^2), deadbanded, regen-clipped and jerk limited."""

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.reset()

  def reset(self):
    self.a_request = 0.0
    self.a_raw = 0.0
    self.braking = False     # past the deadband (with hysteresis)
    self.on_timer = 0.0      # time a weak request has been past DEADBAND while not braking
    self.on_time = 0.0       # time since braking started
    self.v_target = float('inf')
    self.landing_cap = float('inf')

  @property
  def at_target(self) -> bool:
    return self.landing_cap == 0.0

  def update(self, csc_active, v_ego, t_idxs, x_path, v_path, yaw_rate_path,
             max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL) -> float:
    if csc_active:
      raw = curve_brake_request(v_ego, t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
      self.v_target = curve_target_speed(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
    else:
      raw = 0.0
      self.v_target = float('inf')

    # Release at the curve speed: never ask for more than lands the car on it (0 once there).
    self.landing_cap = landing_decel(float(v_ego) - self.v_target)
    a = max(raw, A_MIN, -self.landing_cap)

    # Deadband with hysteresis: start past DEADBAND (held ON_DEBOUNCE_S, or at once past
    # STRONG_REQUEST), keep going until weaker than DEADBAND_OFF, but at least MIN_ON_S.
    if self.braking:
      self.on_time += self.dt
      if a > -DEADBAND_OFF and (self.on_time >= MIN_ON_S - 1e-6 or not csc_active):
        self.braking = False
    else:
      self.on_timer = self.on_timer + self.dt if a <= -DEADBAND else 0.0
      if self.on_timer >= ON_DEBOUNCE_S - 1e-6 or a <= -STRONG_REQUEST:
        self.braking, self.on_time, self.on_timer = True, 0.0, 0.0

    a = min(a, 0.0) if self.braking else 0.0
    self.a_raw = a
    if a < self.a_request:
      self.a_request = max(a, self.a_request - ONSET_JERK * self.dt)
    else:
      self.a_request = min(a, self.a_request + RELEASE_JERK * self.dt)
    return self.a_request
