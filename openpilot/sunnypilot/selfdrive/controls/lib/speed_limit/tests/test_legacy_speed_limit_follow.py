import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.legacy_speed_limit_follow import (CONFIRM_S,
                                                                                              LegacySpeedLimitFollow)

DT = 0.05
LIMIT_30 = 13.4112   # 30 mph
LIMIT_45 = 20.1168   # 45 mph


def run(follow, v_in, limit, seconds, enabled=True):
  steps = int(round(seconds / DT))
  out = v_in
  adopted = False
  for _ in range(steps):
    out, applied = follow.update(out, limit, enabled, DT)
    adopted = adopted or applied   # `applied` is only true on the adoption step
  return out, adopted


class TestLegacySpeedLimitFollow(unittest.TestCase):
  def test_raises_after_confirmation(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_30, LIMIT_30, True, DT)
    out, adopted = run(f, LIMIT_30, LIMIT_45, CONFIRM_S + 0.1)
    self.assertAlmostEqual(out, LIMIT_45)
    self.assertTrue(adopted)

  def test_lowers_after_confirmation(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_45, LIMIT_45, True, DT)
    out, adopted = run(f, LIMIT_45, LIMIT_30, CONFIRM_S + 0.1)
    self.assertAlmostEqual(out, LIMIT_30)

  def test_short_flap_does_not_move(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_30, LIMIT_30, True, DT)
    out, _ = run(f, LIMIT_30, LIMIT_45, CONFIRM_S - 0.2)   # not long enough
    self.assertAlmostEqual(out, LIMIT_30)
    out, _ = run(f, LIMIT_30, LIMIT_30, 0.1)               # limit returns
    self.assertAlmostEqual(out, LIMIT_30)

  def test_disabled_holds_cruise(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_30, LIMIT_30, True, DT)
    out, _ = run(f, LIMIT_30, LIMIT_45, 5.0, enabled=False)
    self.assertAlmostEqual(out, LIMIT_30)

  def test_no_limit_holds(self):
    f = LegacySpeedLimitFollow()
    out, _ = run(f, LIMIT_30, 0.0, 5.0)
    self.assertAlmostEqual(out, LIMIT_30)

  def test_tiny_limit_ignored(self):
    f = LegacySpeedLimitFollow()
    out, _ = run(f, LIMIT_30, 1.0, 5.0)
    self.assertAlmostEqual(out, LIMIT_30)

  def test_user_change_wins(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_30, LIMIT_45, True, DT)   # pending raise
    out, _ = f.update(26.0, LIMIT_45, True, DT)  # driver lowered the set speed
    self.assertAlmostEqual(out, 26.0)
    out, _ = run(f, 26.0, LIMIT_45, CONFIRM_S + 0.1)
    self.assertAlmostEqual(out, LIMIT_45)

  def test_no_change_within_one_mph(self):
    f = LegacySpeedLimitFollow()
    f.update(LIMIT_30, LIMIT_30, True, DT)
    out, adopted = run(f, LIMIT_30, LIMIT_30 + 0.3, 5.0)  # < 1 mph away
    self.assertAlmostEqual(out, LIMIT_30)
    self.assertFalse(adopted)

  def test_offset_already_applied(self):
    f = LegacySpeedLimitFollow()
    limit_plus_offset = LIMIT_30 + 2.2352  # +5 mph offset
    out, _ = run(f, LIMIT_30, limit_plus_offset, CONFIRM_S + 0.1)
    self.assertAlmostEqual(out, limit_plus_offset)


if __name__ == "__main__":
  unittest.main()
