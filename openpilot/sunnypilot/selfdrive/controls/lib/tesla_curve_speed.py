"""Tesla Curve Speed Controller for the sunnypilot-based tesla-hw1-port tree.

This is a port of BogPilot's AP1 curve speed stack (MIT):
  * ``bogpilot_csc.py`` -- FrogsGoMoo's Curve Speed Controller learning and target
  * ``ap1_curve_brake.py`` -- Tesla look-ahead curve brake
  * ``ap1_curve_arbitration.py`` -- comfort arbitration is intentionally not ported here;
    the sunnypilot planner already takes the minimum over its candidates.

Adaptations for this tree:
  * Params replaces BogPilot's file-backed ParamStore.
  * The curve brake is exposed as one extra acceleration candidate, never as a replacement
    for the lead/e2e/cruise candidates, so it can only add braking.
  * Everything fails closed: non-Tesla, no openpilot longitudinal, toggle off, stale or
    malformed model/carState inputs, or a non-finite speed leave the cruise speed alone
    and release the brake request.

Safety properties:
  * The CSC target only ever lowers ``v_cruise``.
  * The brake request is <= 0, clipped to the AP1 regen floor (-1.2 m/s^2), and jerk limited.
  * No panda safety code is touched.
"""
from __future__ import annotations

import json
import math

import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.modeld.constants import ModelConstants

# 20 Hz planner tick; kept local so this module stays importable without the
# full openpilot runtime (setproctitle/zstandard) for unit tests.
DT_MDL = 0.05

# --- CurveSpeedController constants (BogPilot / FrogPilot) ---
CRUISING_SPEED = 5.0                       # m/s
DEFAULT_LATERAL_ACCELERATION = 2.0         # m/s^2
MINIMUM_LATERAL_ACCELERATION = 1.3         # m/s^2
PLANNER_TIME = ModelConstants.T_IDXS[-1]   # s
THRESHOLD = 0.63
CALIBRATION_PROGRESS_THRESHOLD = 10 / DT_MDL
MAX_CURVATURE = 0.1
TARGET_RELEASE_RATE = 1.2                  # m/s^2
MIN_CURVATURE = 0.001
PERCENTILE = 90
ROUNDING_PRECISION = 5
STEP = 0.001

# --- Tesla curve brake constants (BogPilot ap1_curve_brake.py) ---
TESLA_CSC_MAX_LAT_ACCEL = 2.0              # m/s^2, cap on the learned cornering limit
TESLA_CURVE_LEAD_S = 0.5                   # s, reach the curve speed this long before the node
DEADBAND = 0.15                            # m/s^2
DEADBAND_OFF = 0.08                        # m/s^2
ON_DEBOUNCE_S = 0.3                        # s
STRONG_REQUEST = 0.3                       # m/s^2
MIN_ON_S = 0.5                             # s
A_MIN = -1.2                               # m/s^2, AP1 regen range
ONSET_JERK = 1.5                           # m/s^3
RELEASE_JERK = 1.2                         # m/s^3
TARGET_MARGIN = 0.0                        # m/s
ACTUATOR_LAG_S = 0.3                       # s
MIN_MODEL_SPEED = 5.0                      # m/s
BRAKE_MIN_CURVATURE = 1e-4                 # 1/m
MIN_DISTANCE = 5.0                         # m

TOGGLE = "TeslaCurveSpeedController"
TOGGLE_CACHE_S = 1.0
CSC_HOLD_S = 2.0


def calculate_road_curvature(orientation_rate_z, velocity_x, timebase):
  """(signed road curvature 1/m, time to the curve s) at the path node with the most lateral accel."""
  orientation_rate = np.asarray(orientation_rate_z, dtype=float)
  velocity = np.asarray(velocity_x, dtype=float)
  t = np.asarray(timebase, dtype=float)

  lateral_acceleration = orientation_rate * velocity
  index = int(np.argmax(np.abs(lateral_acceleration)))
  predicted_lateral_acc = float(lateral_acceleration[index])
  time_to_curve = float(t[index])

  return float(predicted_lateral_acc / max(velocity[index], 1) ** 2), max(time_to_curve, 1)


def _median3(a):
  """3-point running median along the path; end nodes keep their own value."""
  p = np.concatenate([a[:1], a, a[-1:]])
  return np.median(np.stack([p[:-2], p[1:-1], p[2:]]), axis=0)


class RoadCurvatureFilter:
  """calculate_road_curvature() with node-spike and argmax-hop filtering."""

  NODE_SWITCH_RATIO = 1.10
  MIN_NODE_SPEED = CRUISING_SPEED
  RELEASE_TAU = 0.5

  def __init__(self):
    self.reset()

  def reset(self):
    self.index = None
    self.magnitude = 0.0

  def update(self, orientation_rate_z, velocity_x, timebase, dt: float = DT_MDL):
    orientation_rate = np.asarray(orientation_rate_z, dtype=float)
    velocity = np.asarray(velocity_x, dtype=float)
    t = np.asarray(timebase, dtype=float)

    lateral_acceleration = orientation_rate * velocity
    usable = (velocity >= self.MIN_NODE_SPEED) & (t > 0.0) & np.isfinite(lateral_acceleration)
    magnitude = _median3(np.abs(np.where(usable, lateral_acceleration, 0.0)))
    magnitude = np.where(usable, magnitude, 0.0)
    best = int(np.argmax(magnitude))
    if self.index is not None and self.index < len(magnitude) and magnitude[best] <= magnitude[self.index] * self.NODE_SWITCH_RATIO:
      best = self.index
    self.index = best

    if magnitude[best] <= 0.0:
      curvature, time_to_curve = 0.0, 1.0
      self.magnitude = 0.0
    else:
      raw = magnitude[best] / max(velocity[best], 1) ** 2
      if raw >= self.magnitude:
        self.magnitude = float(raw)
      else:
        self.magnitude += (float(raw) - self.magnitude) * (1.0 - math.exp(-max(dt, 0.0) / self.RELEASE_TAU))
      curvature = math.copysign(self.magnitude, float(lateral_acceleration[best]))
      time_to_curve = max(float(t[best]), 1)
    return float(curvature), time_to_curve


def road_curvature_detected(road_curvature: float, v_ego: float, blinker: bool) -> bool:
  if not math.isfinite(road_curvature) or road_curvature == 0.0:
    return False
  return (1 / abs(road_curvature)) ** 0.5 < v_ego > CRUISING_SPEED and not blinker


class CurveDetector:
  """road_curvature_detected() with hysteresis and an entry debounce."""

  EXIT_RATIO = 0.8
  EXIT_HOLD_S = 0.5
  ENTRY_DEBOUNCE_S = 0.25

  def __init__(self):
    self.reset()

  def reset(self):
    self.active = False
    self.entry_time = 0.0
    self.exit_time = 0.0

  def update(self, road_curvature: float, v_ego: float, blinker: bool, limiting: bool, dt: float) -> bool:
    raw = road_curvature_detected(road_curvature, v_ego, blinker)
    ratio = v_ego * v_ego * abs(road_curvature) if math.isfinite(road_curvature) else 0.0
    if blinker or not math.isfinite(road_curvature) or road_curvature == 0.0 or v_ego <= CRUISING_SPEED:
      self.reset()
      return False
    if self.active:
      self.exit_time = self.exit_time + dt if ratio < self.EXIT_RATIO else 0.0
      if self.exit_time >= self.EXIT_HOLD_S - 1e-9:
        self.reset()
      return self.active
    if not raw:
      self.entry_time = 0.0
      return False
    self.entry_time += dt
    if limiting or self.entry_time >= self.ENTRY_DEBOUNCE_S - 1e-9:
      self.active, self.exit_time = True, 0.0
    return self.active


def load_curvature_data(text) -> dict:
  try:
    raw = json.loads(text) if text else {}
  except (TypeError, ValueError):
    return {}
  if not isinstance(raw, dict):
    return {}
  out = {}
  for key, data in raw.items():
    try:
      average, count = float(data["average"]), int(data["count"])
      float(key)
    except (KeyError, TypeError, ValueError):
      continue
    if math.isfinite(average) and average >= 0.0 and count > 0:
      out[str(key)] = {"average": average, "count": count}
  return out


class TrackingLead:
  """Closing in on, or following, a lead inside the model's view."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.filter = FirstOrderFilter(0, 0.5, dt)
    self.tracking = False

  def update(self, present: bool, d_rel: float, v_rel: float, model_length: float, standstill: bool) -> bool:
    if standstill:
      return self.tracking
    closing_lead = present and v_rel < 0 and d_rel + (v_rel * PLANNER_TIME) < model_length + 5.0
    following_lead = present and d_rel < model_length + 5.0
    self.filter.update(following_lead)
    self.tracking = bool(closing_lead or self.filter.x >= THRESHOLD)
    return self.tracking

  def reset(self):
    self.filter.x = 0.0
    self.tracking = False


class CurveSpeedController:
  def __init__(self, store, max_lateral_acceleration: float | None = None):
    self.store = store
    self.enable_training = False
    self.target_set = False
    self.target = 0.0
    self.training_timer = 0.0
    self.max_lateral_acceleration = max_lateral_acceleration

    self.curvature_data = load_curvature_data(self._read("CurvatureData"))
    self.required_curvatures = [str(round(float(k), ROUNDING_PRECISION))
                                for k in np.arange(MIN_CURVATURE, MAX_CURVATURE + STEP, STEP)]
    self.update_lateral_acceleration()

  def _read(self, key):
    try:
      value = self.store.get(key, "") if self.store is not None else ""
    except Exception:
      return ""
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    return value

  def _put(self, key, value):
    if self.store is None:
      return
    try:
      self.store.put(key, value)
    except Exception:
      pass

  def log_data(self, v_ego, tracking_lead, long_active, driving_in_curve, blinker, lateral_acceleration, road_curvature):
    self.enable_training = v_ego > CRUISING_SPEED
    self.enable_training &= not tracking_lead
    self.enable_training &= not long_active

    if self.enable_training:
      self.training_timer += DT_MDL
      sample_ok = math.isfinite(lateral_acceleration) and math.isfinite(road_curvature)
      if self.training_timer >= PLANNER_TIME and driving_in_curve and not blinker and sample_ok:
        lateral_acceleration = abs(lateral_acceleration)
        road_curvature = abs(round(road_curvature, ROUNDING_PRECISION))
        key = str(road_curvature)
        if key in self.curvature_data:
          data = self.curvature_data[key]
          self.curvature_data[key] = {
            "average": ((data["average"] * data["count"]) + lateral_acceleration) / (data["count"] + 1),
            "count": data["count"] + 1,
          }
        else:
          self.curvature_data[key] = {"average": lateral_acceleration, "count": 1}
        self.update_lateral_acceleration()
      else:
        self.enable_training = False
    elif self.training_timer >= PLANNER_TIME:
      self.persist()
      self.enable_training = False
      self.training_timer = 0.0
    else:
      self.enable_training = False
      self.training_timer = 0.0

  def calibration_progress(self) -> float:
    progress = 0.0
    for key in self.required_curvatures:
      if key in self.curvature_data:
        progress += min(self.curvature_data[key]["count"] / CALIBRATION_PROGRESS_THRESHOLD, 1.0)
    return (progress / len(self.required_curvatures)) * 100

  def persist(self):
    self._put("CalibrationProgress", float(self.calibration_progress()))
    self._put("CurvatureData", json.dumps(self.curvature_data))
    self._put("CalibratedLateralAcceleration", float(self.lateral_acceleration))

  def update_lateral_acceleration(self):
    if self.curvature_data:
      self.lateral_acceleration = float(np.percentile([d["average"] for d in self.curvature_data.values()], PERCENTILE))
    else:
      self.lateral_acceleration = DEFAULT_LATERAL_ACCELERATION

  def effective_lateral_acceleration(self) -> float:
    lateral_acceleration = self.lateral_acceleration
    if self.max_lateral_acceleration is not None:
      lateral_acceleration = min(lateral_acceleration, self.max_lateral_acceleration)
    return lateral_acceleration

  def update_target(self, v_ego, road_curvature, time_to_curve, v_cap=None, dt=DT_MDL):
    lateral_acceleration = self.effective_lateral_acceleration()

    if self.target_set:
      csc_speed = (lateral_acceleration / abs(road_curvature)) ** 0.5
      decel_rate = (v_ego - csc_speed) / time_to_curve
      target = max(self.target - decel_rate * dt, csc_speed)
      target = float(np.clip(target, CRUISING_SPEED, max(v_ego, csc_speed)))
      self.target = min(target, self.target + TARGET_RELEASE_RATE * dt)
    else:
      self.target_set = True
      self.target = v_ego
      if v_cap is not None and math.isfinite(v_cap) and v_cap >= CRUISING_SPEED and road_curvature != 0.0:
        csc_speed = (lateral_acceleration / abs(road_curvature)) ** 0.5
        self.target = float(np.clip(csc_speed, v_ego, max(v_ego, v_cap)))
    return self.target


def _path_curve_speeds(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel):
  t = np.asarray(t_idxs, dtype=float)
  x = np.asarray(x_path, dtype=float)
  v = np.asarray(v_path, dtype=float)
  yaw = np.asarray(yaw_rate_path, dtype=float)
  n = min(len(t), len(x), len(v), len(yaw))
  t, x, v, yaw = t[:n], x[:n], v[:n], yaw[:n]
  valid = v >= MIN_MODEL_SPEED
  curvature = np.abs(yaw) / np.clip(v, MIN_MODEL_SPEED, None)
  valid &= (curvature > BRAKE_MIN_CURVATURE) & (t > 0.0)
  v_curve = np.sqrt(max_lat_accel / np.where(valid, curvature, 1.0))
  return t, x, v_curve, valid


def curve_target_speed(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL) -> float:
  v = np.asarray(v_path, dtype=float)
  yaw = np.asarray(yaw_rate_path, dtype=float)
  n = min(len(v), len(yaw))
  if n == 0 or max_lat_accel <= 0.0:
    return float("inf")
  v, yaw = v[:n], yaw[:n]
  i = int(np.argmax(np.abs(yaw * v)))
  curvature = abs(yaw[i] * v[i]) / max(v[i], 1.0) ** 2
  if curvature <= BRAKE_MIN_CURVATURE:
    return float("inf")
  return float(np.sqrt(max_lat_accel / curvature))


def curve_brake_request(v_ego, t_idxs, x_path, v_path, yaw_rate_path,
                        max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL, lead_s=TESLA_CURVE_LEAD_S) -> float:
  v_ego = float(v_ego)
  if v_ego <= 0.0 or max_lat_accel <= 0.0:
    return 0.0
  _, x, v_curve, valid = _path_curve_speeds(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
  if not np.any(valid):
    return 0.0
  dist = x - v_ego * lead_s
  valid &= v_curve < v_ego
  if not np.any(valid):
    return 0.0
  dist = np.clip(dist, MIN_DISTANCE, None)
  decel = (v_ego ** 2 - v_curve ** 2) / (2.0 * dist)
  return -float(np.max(np.where(valid, decel, 0.0)))


def landing_decel(speed_gap: float) -> float:
  g = float(speed_gap) - TARGET_MARGIN
  if not np.isfinite(g):
    return float("inf")
  if g <= 0.0:
    return 0.0
  return RELEASE_JERK * (np.sqrt(ACTUATOR_LAG_S ** 2 + 2.0 * g / RELEASE_JERK) - ACTUATOR_LAG_S)


class TeslaCurveBrake:
  """Curve brake request (<= 0, m/s^2), capped to land on the curve speed."""

  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.reset()

  def reset(self):
    self.a_request = 0.0
    self.a_raw = 0.0
    self.braking = False
    self.on_timer = 0.0
    self.on_time = 0.0
    self.v_target = float("inf")
    self.landing_cap = float("inf")

  def update(self, csc_active, v_ego, t_idxs, x_path, v_path, yaw_rate_path,
             max_lat_accel=TESLA_CSC_MAX_LAT_ACCEL) -> float:
    if csc_active:
      raw = curve_brake_request(v_ego, t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
      self.v_target = curve_target_speed(t_idxs, x_path, v_path, yaw_rate_path, max_lat_accel)
    else:
      raw = 0.0
      self.v_target = float("inf")

    self.landing_cap = landing_decel(float(v_ego) - self.v_target)
    a = max(raw, A_MIN, -self.landing_cap)

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


class _ParamsStore:
  def __init__(self, params=None):
    if params is None:
      from openpilot.common.params import Params
      params = Params()
    self.params = params

  def get(self, key, default=""):
    try:
      value = self.params.get(key)
    except Exception:
      return default
    if value is None:
      return default
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    return value

  def get_bool(self, key, default=False):
    try:
      if hasattr(self.params, "get_bool"):
        return bool(self.params.get_bool(key))
    except Exception:
      pass
    return self.get(key, "0") in ("1", "true", "True")

  def put(self, key, value):
    try:
      self.params.put(key, str(value))
    except Exception:
      pass


class ModelClock:
  def __init__(self, fallback: float):
    self.fallback = fallback
    self.last = None

  def step(self, sm) -> float:
    try:
      mono = int(sm.logMonoTime["modelV2"])
    except (AttributeError, KeyError, TypeError, ValueError):
      return self.fallback
    dt = self.fallback if self.last is None or mono <= self.last else (mono - self.last) * 1e-9
    self.last = mono
    return float(np.clip(dt, 0.01, 0.2))


def _get(obj, name, default=None):
  try:
    return getattr(obj, name)
  except AttributeError:
    try:
      return obj[name]
    except (KeyError, TypeError):
      return default


def _service_ok(sm, name) -> bool:
  try:
    return bool(sm.alive[name]) and bool(sm.valid[name])
  except (AttributeError, KeyError, TypeError):
    return False


def _finite_list(values, n):
  try:
    out = [float(v) for v in values]
  except (TypeError, ValueError):
    return None
  if len(out) != n or not all(math.isfinite(v) for v in out):
    return None
  return out


def model_path(model):
  n = ModelConstants.IDX_N
  try:
    x = _finite_list(model.position.x, n)
    v = _finite_list(model.velocity.x, n)
    yaw = _finite_list(model.orientationRate.z, n)
  except AttributeError:
    return None
  if x is None or v is None or yaw is None:
    return None
  return list(ModelConstants.T_IDXS[:n]), x, v, yaw


def _legacy_tesla_cars():
  try:
    from opendbc.car.tesla.values import LEGACY_CARS
    return LEGACY_CARS
  except Exception:
    return ()


class TeslaCurveSpeedController:
  """Facade: caps v_cruise and exposes a curve-brake acceleration candidate."""

  def __init__(self, CP, store=None, dt: float = DT_MDL, supported: bool | None = None):
    if supported is None:
      brand = _get(CP, "brand")
      fingerprint = _get(CP, "carFingerprint")
      long_control = bool(_get(CP, "openpilotLongitudinalControl", False))
      supported = brand == "tesla" and fingerprint in _legacy_tesla_cars() and long_control

    self.available = bool(supported)
    self.dt = dt
    self.store = store if store is not None else (_ParamsStore() if self.available else None)
    self.csc = CurveSpeedController(self.store, TESLA_CSC_MAX_LAT_ACCEL) if self.available else None
    self.brake = TeslaCurveBrake(dt) if self.available else None
    self.tracking_lead = TrackingLead(dt)
    self.curvature_filter = RoadCurvatureFilter()
    self.detector = CurveDetector()
    self.clock = ModelClock(dt)

    self.frame = 0
    self.enabled_toggle = False
    self.active = False
    self.controlling = False
    self.training = False
    self.valid = False
    self.road_curvature = 0.0
    self.time_to_curve = 0.0
    self.target = 0.0
    self.v_cruise_out = 0.0
    self.a_brake = 0.0
    self._hold_s = 0.0
    self._last_target = 0.0

  def _toggle_on(self) -> bool:
    try:
      if hasattr(self.store, "get_bool"):
        return bool(self.store.get_bool(TOGGLE))
      return self.store.get(TOGGLE, "0") in ("1", "true", "True")
    except Exception:
      return False

  def _inactive(self, v_cruise, reset_state):
    self.curvature_filter.reset()
    self.detector.reset()
    if self.csc is not None:
      self.csc.target_set = False
      if self.csc.training_timer > 0:
        self.csc.log_data(0.0, False, False, False, False, 0.0, 0.0)
      self.csc.enable_training = False
    self.valid = False
    self.active = False
    self.controlling = False
    self.training = False
    self._hold_s = 0.0
    self._last_target = 0.0
    self.target = v_cruise
    self.road_curvature = 0.0
    self.v_cruise_out = v_cruise
    if self.brake is not None:
      if reset_state:
        self.brake.reset()
        self.a_brake = 0.0
      else:
        self.a_brake = self.brake.update(False, 0.0, [], [], [], [])
    else:
      self.a_brake = 0.0
    return v_cruise

  def update(self, sm, v_ego, v_cruise, reset_state=False) -> float:
    if not self.available:
      return self._inactive(v_cruise, reset_state)

    dt_s = self.clock.step(sm)
    if self.brake is not None:
      self.brake.dt = dt_s
    self.frame += 1
    if self.frame == 1 or self.frame % max(1, int(round(TOGGLE_CACHE_S / self.dt))) == 0:
      self.enabled_toggle = self._toggle_on()

    path = model_path(sm["modelV2"]) if _service_ok(sm, "modelV2") else None
    fresh = path is not None and _service_ok(sm, "carState") and math.isfinite(v_ego) and math.isfinite(v_cruise)
    if reset_state or not self.enabled_toggle or not fresh or v_cruise <= 0.0:
      return self._inactive(v_cruise, reset_state)

    t, x, v, yaw = path
    CS = sm["carState"]
    try:
      lane_change = int(sm["modelV2"].meta.laneChangeState) != 0
    except (AttributeError, KeyError, TypeError):
      return self._inactive(v_cruise, reset_state)
    blinker = bool(_get(CS, "leftBlinker", False) or _get(CS, "rightBlinker", False) or lane_change)
    v_ego = max(float(v_ego), 0.0)

    self.road_curvature, self.time_to_curve = self.curvature_filter.update(yaw, v, t, dt_s)
    limiting = (math.isfinite(self.road_curvature) and self.road_curvature != 0.0 and
                (self.csc.effective_lateral_acceleration() / abs(self.road_curvature)) ** 0.5 < v_ego)
    detected = self.detector.update(self.road_curvature, v_ego, blinker, limiting, dt_s)

    lead = sm["radarState"].leadOne
    tracking_lead = self.tracking_lead.update(bool(_get(lead, "present", False)), float(_get(lead, "dRel", 0.0)),
                                              float(_get(lead, "vRel", 0.0)), x[-1], bool(_get(CS, "standstill", False)))

    long_enabled = bool(_get(sm["carControl"], "enabled", False))
    cruise_control = _get(sm["carControl"], "cruiseControl", None)
    long_override = bool(_get(cruise_control, "override", False)) if cruise_control is not None else False
    engaged = bool(_get(sm["selfdriveState"], "enabled", False))

    try:
      meas_curv = float(sm["controlsState"].curvature)
    except (KeyError, AttributeError, TypeError):
      meas_curv = float("nan")
    lateral_acceleration = v_ego ** 2 * meas_curv if math.isfinite(meas_curv) else float("nan")

    self.csc.log_data(v_ego, tracking_lead, engaged, detected, blinker, lateral_acceleration, self.road_curvature)
    self.training = self.csc.enable_training

    # Learning and curvature tracking continue while the driver is manual; only the
    # speed cap and brake are gated on openpilot longitudinal being active.
    if not long_enabled or long_override:
      self.active = False
      self.controlling = False
      self.valid = True
      self.target = v_cruise
      self.v_cruise_out = v_cruise
      self._hold_s = 0.0
      self._last_target = 0.0
      self.csc.target_set = False
      self.a_brake = self.brake.update(False, v_ego, t, x, v, yaw)
      return v_cruise

    if not detected:
      if self._hold_s < CSC_HOLD_S and self._last_target > 0.0:
        self._hold_s += dt_s
        self.v_cruise_out = min(v_cruise, self._last_target)
        self.controlling = True
        self.active = False
        self.valid = True
        self.a_brake = self.brake.update(False, v_ego, t, x, v, yaw)
        return self.v_cruise_out
      return self._inactive(v_cruise, reset_state)

    self._hold_s = 0.0
    self.csc.update_target(v_ego, self.road_curvature, self.time_to_curve, v_cap=v_cruise, dt=dt_s)
    self._last_target = self.csc.target
    self.target = self.csc.target
    self.v_cruise_out = min(v_cruise, self.target)
    self.active = True
    self.controlling = self.v_cruise_out < v_cruise - 1e-3
    self.valid = True
    self.a_brake = self.brake.update(True, v_ego, t, x, v, yaw, self.csc.effective_lateral_acceleration())
    return self.v_cruise_out

  @property
  def braking(self) -> bool:
    return self.a_brake < -DEADBAND_OFF
