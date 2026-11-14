import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.tesla_lead_filter import (LegacyLeadGlitchFilter,
                                                                          camera_contradicts,
                                                                          camera_confirms)


def lead(dRel=10.0, vLead=10.0, vRel=0.0, radar=True, present=True):
  return {"present": present, "dRel": dRel, "yRel": 0.0, "vRel": vRel, "vLead": vLead,
          "modelProb": 0.0, "radar": radar}


class TestLeadFilter(unittest.TestCase):
  def test_vision_lead_never_filtered(self):
    f = LegacyLeadGlitchFilter()
    out = f.update(lead(radar=False, dRel=30.0), 0.9, 30.0, 10.0, 0.5)
    self.assertTrue(out["present"])
    self.assertEqual(f.held, 0)

  def test_close_radar_only_new_point_dropped_once(self):
    f = LegacyLeadGlitchFilter()
    first = f.update(lead(dRel=1.5), 0.0, 50.0, 10.0, 0.5)
    self.assertFalse(first["present"])
    # still there next frame -> accepted
    second = f.update(lead(dRel=1.5), 0.0, 50.0, 10.0, 0.5)
    self.assertTrue(second["present"])

  def test_jump_holds_previous(self):
    f = LegacyLeadGlitchFilter()
    f.update(lead(dRel=46.0, vLead=10.6, vRel=-0.5), 0.9, 46.0, 10.6, 0.5)
    out = f.update(lead(dRel=42.0, vLead=0.3, vRel=-9.0), 0.0, 46.0, 10.6, 0.5)
    # previous lead held, dRel extrapolated by vRel*dt
    self.assertAlmostEqual(out["dRel"], 46.0 - 0.5 * 0.05)
    self.assertEqual(f.held, 1)
    # agreeing next frame -> accepted
    out2 = f.update(lead(dRel=41.9, vLead=0.3, vRel=-9.0), 0.0, 46.0, 10.6, 0.5)
    self.assertAlmostEqual(out2["dRel"], 41.9)

  def test_lead_appearing_never_delayed(self):
    f = LegacyLeadGlitchFilter()
    out = f.update(lead(dRel=40.0, vLead=9.0), 0.0, 50.0, 10.0, 0.5)
    self.assertTrue(out["present"])

  def test_lead_disappearing_never_delayed(self):
    f = LegacyLeadGlitchFilter()
    f.update(lead(dRel=40.0), 0.0, 50.0, 10.0, 0.5)
    out = f.update({"present": False}, 0.0, 50.0, 10.0, 0.5)
    self.assertFalse(out["present"])

  def test_contradicted_close_point_held_then_accepted(self):
    f = LegacyLeadGlitchFilter()
    f.update(lead(dRel=9.5, vLead=2.0, radar=True), 0.95, 9.5, 2.0, 0.5)
    close = lead(dRel=2.06, vLead=0.0, radar=True)
    for _ in range(4):
      out = f.update(close, 0.95, 9.5, 2.0, 0.5)
      self.assertAlmostEqual(out["dRel"], 9.5, places=3)
    out = f.update(close, 0.95, 9.5, 2.0, 0.5)
    self.assertAlmostEqual(out["dRel"], 2.06, places=3)

  def test_camera_confirms(self):
    self.assertTrue(camera_confirms(lead(dRel=30.0, radar=False), 0.9, 30.0, 10.0, 0.5))
    self.assertFalse(camera_confirms(lead(dRel=30.0, radar=False), 0.1, 30.0, 10.0, 0.5))
    self.assertTrue(camera_confirms(lead(dRel=30.0, vLead=10.0), 0.9, 30.0, 10.0, 0.5))
    self.assertFalse(camera_confirms(lead(dRel=60.0, vLead=10.0), 0.9, 30.0, 10.0, 0.5))

  def test_camera_contradicts(self):
    self.assertTrue(camera_contradicts(lead(dRel=2.0), 0.95, 9.5))
    self.assertFalse(camera_contradicts(lead(dRel=2.0), 0.5, 9.5))
    self.assertFalse(camera_contradicts(lead(dRel=2.0, radar=False), 0.95, 9.5))


if __name__ == "__main__":
  unittest.main()
