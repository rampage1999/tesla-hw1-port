import math
import types

import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib import tesla_curve_speed as csc


class FakeStore:
  def __init__(self, data=None):
    self.data = dict(data or {})

  def get(self, key, default=""):
    return self.data.get(key, default)

  def get_bool(self, key, default=False):
    return bool(self.data.get(key, default))

  def put(self, key, value):
    self.data[key] = value


def _path(v=20.0, curvature=0.01, n=ModelConstants.IDX_N):
  t = np.array(ModelConstants.T_IDXS[:n])
  x = v * t
  yaw = curvature * v * np.ones(n)
  vel = v * np.ones(n)
  return t.tolist(), x.tolist(), vel.tolist(), yaw.tolist()


def test_curve_target_speed_straight_is_infinite():
  t, x, v, yaw = _path(curvature=0.0)
  assert math.isinf(csc.curve_target_speed(t, x, v, yaw))


def test_curve_target_speed_uses_lateral_limit():
  t, x, v, yaw = _path(v=20.0, curvature=0.01)
  # sqrt(2.0 / 0.01) = 14.142
  assert csc.curve_target_speed(t, x, v, yaw) == pytest.approx(math.sqrt(2.0 / 0.01))


def test_curve_brake_request_positive_when_approaching_a_slow_curve():
  t, x, v, yaw = _path(v=20.0, curvature=0.01)
  request = csc.curve_brake_request(20.0, t, x, v, yaw)
  assert request < 0.0


def test_curve_brake_request_zero_when_curve_allows_current_speed():
  t, x, v, yaw = _path(v=20.0, curvature=0.001)
  # sqrt(2 / 0.001) = 44.7 m/s, far above v_ego
  assert csc.curve_brake_request(20.0, t, x, v, yaw) == 0.0


def test_landing_decel_never_overshoots_the_gap():
  assert csc.landing_decel(0.0) == 0.0
  assert csc.landing_decel(-1.0) == 0.0
  for gap in (0.5, 1.0, 3.0, 8.0):
    a = csc.landing_decel(gap)
    shed = a * csc.ACTUATOR_LAG_S + a * a / (2.0 * csc.RELEASE_JERK)
    assert shed <= gap + 1e-6


def test_brake_deadband_and_jerk_limit():
  brake = csc.TeslaCurveBrake(dt=0.05)
  t, x, v, yaw = _path(v=20.0, curvature=0.01)
  a = 0.0
  for _ in range(20):
    a = brake.update(True, 20.0, t, x, v, yaw)
  assert a < -csc.DEADBAND_OFF
  assert a >= csc.A_MIN
  # release must be jerk limited, not a step
  prev = a
  a = brake.update(False, 20.0, t, x, v, yaw)
  assert a - prev <= csc.RELEASE_JERK * brake.dt + 1e-9


def test_curve_detector_debounces_and_exits():
  detector = csc.CurveDetector()
  k = 0.02
  assert not detector.update(k, 20.0, False, False, 0.05)
  assert detector.update(k, 20.0, False, False, 0.05) is False  # 0.10 s
  for _ in range(3):
    detector.update(k, 20.0, False, False, 0.05)
  assert detector.active
  # blinker releases immediately
  assert not detector.update(k, 20.0, True, False, 0.05)


def test_curve_speed_controller_learns_from_manual_curve():
  store = FakeStore()
  controller = csc.CurveSpeedController(store, max_lateral_acceleration=2.0)
  assert controller.lateral_acceleration == csc.DEFAULT_LATERAL_ACCELERATION
  # Fill the training timer with a valid curve sample.
  for _ in range(int(csc.PLANNER_TIME / csc.DT_MDL) + 1):
    controller.log_data(15.0, False, False, True, False, 1.8, 0.02)
  controller.persist()
  assert store.data.get("CurvatureData")
  assert "CalibrationProgress" in store.data
  assert controller.effective_lateral_acceleration() <= 2.0


def test_controller_target_lowers_to_curve_speed():
  store = FakeStore()
  controller = csc.CurveSpeedController(store, max_lateral_acceleration=2.0)
  target = controller.update_target(20.0, 0.01, 2.0, v_cap=25.0)
  assert target == 20.0  # first activation seeds at v_ego
  for _ in range(40):
    target = controller.update_target(20.0, 0.01, 2.0, v_cap=25.0, dt=0.05)
  assert target == pytest.approx(math.sqrt(2.0 / 0.01), abs=0.2)


class FakeSM(dict):
  def __getattr__(self, name):
    try:
      return self[name]
    except KeyError as e:
      raise AttributeError(name) from e


def _fake_sm(v_ego=20.0, curvature=0.01, enabled=True):
  t, x, v, yaw = _path(v=v_ego, curvature=curvature)
  model = types.SimpleNamespace(
    position=types.SimpleNamespace(x=x),
    velocity=types.SimpleNamespace(x=v),
    orientationRate=types.SimpleNamespace(z=yaw),
    meta=types.SimpleNamespace(laneChangeState="off"),
  )
  cs = types.SimpleNamespace(leftBlinker=False, rightBlinker=False, standstill=False)
  lead = types.SimpleNamespace(present=False, dRel=0.0, vRel=0.0)
  sm = FakeSM()
  services = ("modelV2", "carState", "radarState", "carControl", "selfdriveState", "controlsState")
  sm.update({
    "alive": dict.fromkeys(services, True),
    "valid": dict.fromkeys(services, True),
    "logMonoTime": {"modelV2": 1_000_000_000},
    "modelV2": model,
    "carState": cs,
    "radarState": types.SimpleNamespace(leadOne=lead),
    "carControl": types.SimpleNamespace(enabled=True, cruiseControl=types.SimpleNamespace(override=False)),
    "selfdriveState": types.SimpleNamespace(enabled=enabled),
    "controlsState": types.SimpleNamespace(curvature=curvature),
  })
  return sm


def test_facade_non_tesla_is_inactive():
  controller = csc.TeslaCurveSpeedController(types.SimpleNamespace(brand="toyota", carFingerprint="X",
                                                                   openpilotLongitudinalControl=True),
                                             store=FakeStore(), supported=False)
  assert not controller.available
  assert controller.update(_fake_sm(), 20.0, 25.0) == 25.0


def test_facade_caps_cruise_and_requests_brake():
  store = FakeStore({csc.TOGGLE: True})
  controller = csc.TeslaCurveSpeedController(types.SimpleNamespace(brand="tesla", carFingerprint="TESLA_MODEL_S_HW1",
                                                                   openpilotLongitudinalControl=True),
                                             store=store, supported=True)
  controller.enabled_toggle = True
  v_out = 20.0
  for _ in range(40):
    v_out = controller.update(_fake_sm(), 20.0, 25.0)
  assert v_out < 20.0
  assert v_out == pytest.approx(math.sqrt(2.0 / 0.01), abs=0.5)
  assert controller.active
  assert controller.braking
  assert controller.a_brake < 0.0


def test_facade_toggle_off_leaves_cruise_alone():
  store = FakeStore({csc.TOGGLE: False})
  controller = csc.TeslaCurveSpeedController(types.SimpleNamespace(brand="tesla", carFingerprint="TESLA_MODEL_S_HW1",
                                                                   openpilotLongitudinalControl=True),
                                             store=store, supported=True)
  controller.enabled_toggle = False
  assert controller.update(_fake_sm(), 20.0, 25.0) == 25.0
  assert controller.a_brake == 0.0
