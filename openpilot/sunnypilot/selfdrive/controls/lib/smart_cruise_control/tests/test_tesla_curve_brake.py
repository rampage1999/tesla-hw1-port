"""Tests for the Tesla look-ahead curve brake and its comfort arbitration.

Math and filter tests ported from BogPilot's `test_ap1_curve_brake.py` (MIT);
the import path and the Pre-AP factory cases (not in this tree) were dropped.
"""
import numpy as np
import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.curve_arbitration import (
  BYPASS_ACCEL, COMFORT_JERK_DOWN, COMFORT_JERK_UP, CurveArbiter)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.tesla_curve_brake import (
  A_MIN, ACTUATOR_LAG_S, DEADBAND, DEADBAND_OFF, MIN_DISTANCE, MIN_ON_S, ON_DEBOUNCE_S, ONSET_JERK,
  RELEASE_JERK, STRONG_REQUEST, TARGET_MARGIN, TESLA_CSC_MAX_LAT_ACCEL, TESLA_CURVE_LEAD_S,
  TeslaCurveBrake, curve_brake_request, curve_target_speed, landing_decel)

T = np.array(ModelConstants.T_IDXS)
DT = 0.05


def path(v_ego, curvature_at, v_model=None):
  """Constant-speed model path; curvature_at(x) -> 1/m along the path."""
  v = np.full(len(T), v_ego if v_model is None else v_model, dtype=float)
  x = v * T
  k = np.array([curvature_at(xi) for xi in x])
  return T, x, v, k * v


def curve_ahead(start_m, k):
  return lambda x: k if x >= start_m else 0.0


def path_for_raw(v_ego, raw, start=80.0):
  """Path with a curve at `start` whose raw request is `raw` (bisection on curvature)."""
  lo, hi = 1e-4, 0.2
  for _ in range(80):
    k = 0.5 * (lo + hi)
    if curve_brake_request(v_ego, *path(v_ego, curve_ahead(start, k))) < raw:
      hi = k
    else:
      lo = k
  return path(v_ego, curve_ahead(start, 0.5 * (lo + hi)))


def run_brake(v_ego, p, steps, active=True):
  b = TeslaCurveBrake(DT)
  return b, [b.update(active, v_ego, *p) for _ in range(steps)]


# --- constants ---

def test_cap_value_and_lead_time():
  assert TESLA_CSC_MAX_LAT_ACCEL == 2.0 and TESLA_CURVE_LEAD_S == 0.5


def test_hysteresis_thresholds():
  assert 0.0 < DEADBAND_OFF < DEADBAND < STRONG_REQUEST


def test_braking_distance_floor():
  assert MIN_DISTANCE == 5.0


def test_release_jerk_is_gentle():
  assert RELEASE_JERK <= 1.2


# --- request math ---

def test_straight_road_no_request():
  assert curve_brake_request(13.4, *path(13.4, lambda x: 0.0)) == 0.0


def test_gentle_highway_bend_no_request():
  # 1/1500 m at 29 m/s is ~0.56 m/s^2, well under the limit.
  assert curve_brake_request(29.0, *path(29.0, lambda x: 1 / 1500)) == 0.0


def test_request_matches_kinematics():
  v_ego, start, k = 13.4, 60.0, 0.0161
  a = curve_brake_request(v_ego, *path(v_ego, curve_ahead(start, k)))
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  x_entry = min(xi for xi in v_ego * T if xi >= start)
  expected = -(v_ego ** 2 - v_curve ** 2) / (2 * (x_entry - v_ego * TESLA_CURVE_LEAD_S))
  assert a == pytest.approx(expected, rel=1e-6)
  assert a < -DEADBAND


def test_reaching_curve_speed_by_the_curve():
  v_ego, start, k = 13.4, 60.0, 0.0161
  a = curve_brake_request(v_ego, *path(v_ego, curve_ahead(start, k)))
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  x_entry = min(xi for xi in v_ego * T if xi >= start)
  d = x_entry - v_ego * TESLA_CURVE_LEAD_S
  v_at_entry = (v_ego ** 2 + 2 * a * d) ** 0.5
  assert v_at_entry == pytest.approx(v_curve, abs=0.05)


def test_already_slow_enough_no_request():
  k = 0.0161
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  assert curve_brake_request(v_curve - 0.5, *path(v_curve - 0.5, curve_ahead(40.0, k))) == 0.0


def test_model_planned_stop_ignored():
  # Model plans to be nearly stopped: yaw rate / speed looks like a sharp curve, must be skipped.
  t, x, v, _ = path(13.4, lambda x: 0.0, v_model=3.0)
  yaw = np.full(len(t), 0.3)
  assert curve_brake_request(13.4, t, x, v, yaw) == 0.0


def test_bad_input_fails_closed():
  assert curve_brake_request(float('nan'), *path(13.4, curve_ahead(40.0, 0.0161))) == 0.0
  assert curve_brake_request(13.4, [], [], [], []) == 0.0
  assert curve_brake_request(13.4, T, [1.0], [1.0], [1.0]) == 0.0


# --- TeslaCurveBrake filter ---

def test_deadband_ignores_weak_request():
  v_ego = 13.4
  k = TESLA_CSC_MAX_LAT_ACCEL / (v_ego - 0.15) ** 2
  p = path(v_ego, curve_ahead(100.0, k))
  raw = curve_brake_request(v_ego, *p)
  assert -DEADBAND < raw < 0.0
  _, out = run_brake(v_ego, p, 20)
  assert all(a == 0.0 for a in out)


def test_request_clipped_to_regen_range():
  _, out = run_brake(20.0, path(20.0, curve_ahead(25.0, 0.05)), 100)
  assert min(out) == pytest.approx(A_MIN)
  assert all(a >= A_MIN - 1e-9 for a in out)


def test_onset_jerk_limited():
  _, out = run_brake(20.0, path(20.0, curve_ahead(25.0, 0.05)), 40)
  steps = np.diff([0.0] + out)
  assert np.min(steps) >= -ONSET_JERK * DT - 1e-9
  first = next(i for i, a in enumerate(out) if a < 0.0)
  assert out[first] == pytest.approx(-ONSET_JERK * DT)


def test_release_jerk_limited_when_csc_stops():
  b, _ = run_brake(20.0, path(20.0, curve_ahead(25.0, 0.05)), 40)
  out = [b.update(False, 20.0, *path(20.0, lambda x: 0.0)) for _ in range(30)]
  steps = np.diff([A_MIN] + out)
  assert np.max(steps) <= RELEASE_JERK * DT + 1e-9
  assert out[-1] == 0.0


def test_inactive_never_requests_and_reset_clears():
  _, out = run_brake(20.0, path(20.0, curve_ahead(25.0, 0.05)), 20, active=False)
  assert all(a == 0.0 for a in out)
  b, _ = run_brake(20.0, path(20.0, curve_ahead(25.0, 0.05)), 20)
  b.reset()
  assert b.a_request == 0.0 and b.a_raw == 0.0


def test_single_frame_spike_is_not_a_tap():
  v = 13.4
  b = TeslaCurveBrake(DT)
  weak, none = path_for_raw(v, -0.2), path(v, lambda x: 0.0)
  out = [b.update(True, v, *weak)] + [b.update(True, v, *none) for _ in range(20)]
  assert all(a == 0.0 for a in out)


def test_weak_request_debounced():
  v = 13.4
  b = TeslaCurveBrake(DT)
  p = path_for_raw(v, -0.2)
  out = [b.update(True, v, *p) for _ in range(20)]
  n_hold = int(round(ON_DEBOUNCE_S / DT))
  assert all(a == 0.0 for a in out[:n_hold - 1])
  assert out[n_hold - 1] < 0.0


def test_strong_request_starts_at_once():
  v = 13.4
  b = TeslaCurveBrake(DT)
  assert b.update(True, v, *path_for_raw(v, -2 * STRONG_REQUEST)) == pytest.approx(-ONSET_JERK * DT)


def test_hysteresis_holds_between_thresholds():
  v = 13.4
  b = TeslaCurveBrake(DT)
  on, mid = path_for_raw(v, -0.4), path_for_raw(v, -0.5 * (DEADBAND + DEADBAND_OFF))
  assert -DEADBAND < curve_brake_request(v, *mid) < -DEADBAND_OFF
  for _ in range(20):
    b.update(True, v, *on)
  assert b.braking
  segs, prev = 0, True
  for i in range(40):
    b.update(True, v, *(mid if i % 2 else on))
    segs += int(b.braking and not prev)
    prev = b.braking
  assert b.braking and segs == 0


def test_min_on_time_then_release_below_off_threshold():
  v = 13.4
  b = TeslaCurveBrake(DT)
  on, weak = path_for_raw(v, -0.4), path_for_raw(v, -0.5 * DEADBAND_OFF)
  b.update(True, v, *on)
  assert b.braking
  n_min = int(round(MIN_ON_S / DT))
  for _ in range(n_min - 2):
    b.update(True, v, *weak)
    assert b.braking
  for _ in range(3):
    b.update(True, v, *weak)
  assert not b.braking and b.a_raw == 0.0


def test_csc_inactive_releases_even_inside_min_on_time():
  v = 13.4
  b = TeslaCurveBrake(DT)
  b.update(True, v, *path_for_raw(v, -0.4))
  b.update(False, v, *path(v, lambda x: 0.0))
  assert not b.braking and b.a_raw == 0.0


# --- release at the curve speed ---

def test_curve_target_speed_matches_csc_curvature():
  k = 0.0161
  assert curve_target_speed(*path(13.4, curve_ahead(40.0, k))) == pytest.approx((TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5)
  assert curve_target_speed(*path(13.4, lambda x: 0.0)) == float('inf')


def test_landing_decel_lands_on_target():
  assert landing_decel(TARGET_MARGIN) == 0.0 and landing_decel(-1.0) == 0.0
  assert landing_decel(float('inf')) == float('inf')
  prev = 0.0
  for gap in np.linspace(TARGET_MARGIN + 0.05, 3.0, 30):
    a = landing_decel(gap)
    assert a > prev
    assert a * ACTUATOR_LAG_S + a * a / (2 * RELEASE_JERK) == pytest.approx(gap - TARGET_MARGIN, rel=1e-9)
    prev = a


def test_request_capped_near_curve_speed_even_if_node_is_close():
  k = 0.005
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  b = TeslaCurveBrake(DT)
  for _ in range(40):
    b.update(True, v_curve + 3.0, *path(v_curve + 3.0, curve_ahead(30.0, k)))
  assert b.a_request == pytest.approx(A_MIN)
  gap = 0.1
  v = v_curve + gap
  p = path(v, curve_ahead(3.0, k))
  assert curve_brake_request(v, *p) < -landing_decel(gap) - 0.05
  b.update(True, v, *p)
  assert b.a_raw == pytest.approx(-landing_decel(gap))


def test_released_at_curve_speed():
  k = 0.05
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  b = TeslaCurveBrake(DT)
  for _ in range(40):
    b.update(True, v_curve + 3.0, *path(v_curve + 3.0, curve_ahead(20.0, k)))
  v = v_curve - 0.01
  out = [b.update(True, v, *path(v, curve_ahead(3.0, k))) for _ in range(40)]
  assert b.at_target and b.a_raw == 0.0 and out[-1] == 0.0
  assert np.max(np.diff([A_MIN] + out)) <= RELEASE_JERK * DT + 1e-9


def test_closed_loop_lands_on_curve_speed():
  # Point mass with the fitted actuator lag (0.1 s delay + 0.25 s first-order lag).
  k, x_curve = 0.0322, 70.0
  v_curve = (TESLA_CSC_MAX_LAT_ACCEL / k) ** 0.5
  v, s, a = 13.4, 0.0, 0.0
  hist = [0.0, 0.0, 0.0]
  b = TeslaCurveBrake(DT)
  v_min, v_entry = v, None
  while s < x_curve + 10.0:
    d = x_curve - s
    req = b.update(True, v, *path(v, curve_ahead(d, k)))
    hist.append(req)
    a += (1 - np.exp(-DT / 0.25)) * (hist[-3] - a)
    v = max(0.0, v + a * DT)
    s += v * DT
    v_min = min(v_min, v)
    if v_entry is None and s >= x_curve:
      v_entry = v
  assert v_entry <= v_curve + 0.5
  assert v_min >= v_curve - 0.3


# --- arbitration ---

def test_arbiter_absorbs_the_winner_switch_step():
  # cruise jumps from +0.7 to a curve brake at -0.07: without arbitration the output steps 0.77.
  arb = CurveArbiter(DT)
  arb.reset(0.7)
  out, winner, _ = arb.arbitrate([("cruise", 0.7), ("curve", -0.07), ("mpc", 0.6), ("e2e", 0.6)], DT, False)
  assert winner == "curve"
  assert out == pytest.approx(0.7 - COMFORT_JERK_DOWN * DT)
  assert 0.7 - out < 0.77  # the 0.77 step is absorbed into a 0.25 ramp


def test_arbiter_release_is_jerk_limited():
  arb = CurveArbiter(DT)
  arb.reset(-0.2)
  out, _, _ = arb.arbitrate([("cruise", 0.7), ("curve", 0.0)], DT, False)
  assert out == pytest.approx(-0.2 + COMFORT_JERK_UP * DT)


def test_arbiter_bypasses_strong_braking():
  arb = CurveArbiter(DT)
  arb.reset(0.7)
  out, _, rec = arb.arbitrate([("cruise", 0.7), ("curve", BYPASS_ACCEL - 0.2)], DT, False)
  assert rec["bypass"] and out == pytest.approx(BYPASS_ACCEL - 0.2)


def test_arbiter_never_slews_lead_or_e2e():
  arb = CurveArbiter(DT)
  arb.reset(0.0)
  out, winner, _ = arb.arbitrate([("cruise", 0.5), ("curve", 0.0), ("mpc", -2.0)], DT, False)
  assert winner == "mpc" and out == -2.0


def test_arbiter_reset_seeds_comfort():
  arb = CurveArbiter(DT)
  arb.reset(0.3)
  out, winner, _ = arb.arbitrate([("cruise", 0.9), ("curve", 0.4)], DT, True)
  assert out == pytest.approx(0.4) and winner == "curve"
