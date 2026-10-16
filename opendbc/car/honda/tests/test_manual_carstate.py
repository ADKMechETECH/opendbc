import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR, DBC, HondaFlags

GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType

GEAR_MT_CLUTCH = 0
GEAR_MT_REVERSE = 14


def build_car_state(candidate, fingerprint):
  CP = CarInterface.get_params(candidate, fingerprint, [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], False, False, False)
  CS = CarInterface.CarState(CP, CP_SP)
  return CP, CS, CS.get_can_parsers(CP, CP_SP)


class TestHondaManualCarState(unittest.TestCase):
  """Synthetic-frame tests for the manual-transmission branch of honda/carstate.py (Bosch radarless)."""

  def setUp(self):
    # an empty fingerprint has neither GEARBOX_AUTO (0x1A3) nor GEARBOX_CVT (0x191), so the car is detected as manual
    self.CP, self.CS, self.parsers = build_car_state(CAR.HONDA_CIVIC_2022, gen_empty_fingerprint())
    self.packer = CANPacker(DBC[self.CP.carFingerprint][Bus.pt])
    self.bus = CanBus(self.CP).pt
    self.t = 0
    # CANParser.vl subscribes messages lazily on first read, so run one update before feeding frames
    self.CS.update(self.parsers)

  def _step(self, gear_mt, rpm=0., reverse_light=False):
    msgs = [
      self.packer.make_can_msg("GEARBOX_ALT_2", self.bus, {"GEAR_MT": gear_mt}),
      self.packer.make_can_msg("ENGINE_DATA", self.bus, {"ENGINE_RPM": rpm}),
      self.packer.make_can_msg("SCM_FEEDBACK", self.bus, {"REVERSE_LIGHT": int(reverse_light)}),
    ]
    self.t += int(0.01 * 1e9)
    self.parsers[Bus.pt].update([self.t, msgs])
    ret, _ = self.CS.update(self.parsers)
    return ret

  def test_detected_as_manual_radarless(self):
    self.assertEqual(self.CP.transmissionType, TransmissionType.manual)
    self.assertTrue(self.CP.flags & HondaFlags.BOSCH_RADARLESS)

  def test_clutch_in(self):
    ret = self._step(GEAR_MT_CLUTCH, rpm=1800.)
    self.assertTrue(ret.deprecated.clutchPressed)
    self.assertEqual(ret.gearShifter, GearShifter.drive)

  def test_each_forward_gear(self):
    for gear in range(1, 7):
      with self.subTest(gear=gear):
        ret = self._step(gear, rpm=1500. + 300. * gear)
        self.assertFalse(ret.deprecated.clutchPressed)
        self.assertEqual(ret.gearShifter, GearShifter.drive)
        self.assertAlmostEqual(ret.deprecated.engineRpm, 1500. + 300. * gear)

  def test_reverse(self):
    ret = self._step(GEAR_MT_REVERSE, rpm=900., reverse_light=True)
    self.assertFalse(ret.deprecated.clutchPressed)
    self.assertEqual(ret.gearShifter, GearShifter.reverse)

  def test_shift_sequence(self):
    # 3rd -> clutch in -> 4th: clutchPressed should follow GEAR_MT frame by frame
    seq = [(3, False), (3, False), (GEAR_MT_CLUTCH, True), (GEAR_MT_CLUTCH, True), (4, False)]
    for gear_mt, expected in seq:
      self.assertEqual(self._step(gear_mt, rpm=2500.).deprecated.clutchPressed, expected)

  def test_engine_rpm_passthrough(self):
    for rpm in (0., 750., 3000., 6800.):
      with self.subTest(rpm=rpm):
        self.assertAlmostEqual(self._step(3, rpm=rpm).deprecated.engineRpm, rpm)


class TestHondaManualSignalsScope(unittest.TestCase):
  """The new signals must not change behavior for automatics/CVTs or for non-radarless manuals."""

  def test_cvt_unaffected(self):
    fingerprint = gen_empty_fingerprint()
    fingerprint[0][0x191] = 8  # GEARBOX_CVT present -> CVT
    CP, CS, parsers = build_car_state(CAR.HONDA_CIVIC_2022, fingerprint)
    self.assertEqual(CP.transmissionType, TransmissionType.cvt)

    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    bus = CanBus(CP).pt
    CS.update(parsers)
    _ = parsers[Bus.pt].vl["GEARBOX_ALT_2"]  # force-subscribe so the clutch frame really is decoded
    msgs = [packer.make_can_msg("GEARBOX_ALT_2", bus, {"GEAR_MT": GEAR_MT_CLUTCH}),
            packer.make_can_msg("ENGINE_DATA", bus, {"ENGINE_RPM": 2000.})]
    parsers[Bus.pt].update([int(1e7), msgs])
    self.assertEqual(parsers[Bus.pt].vl["GEARBOX_ALT_2"]["GEAR_MT"], GEAR_MT_CLUTCH)
    self.assertEqual(parsers[Bus.pt].vl["ENGINE_DATA"]["ENGINE_RPM"], 2000.)
    ret, _ = CS.update(parsers)
    self.assertFalse(ret.deprecated.clutchPressed)
    self.assertEqual(ret.deprecated.engineRpm, 0.)

  def test_non_radarless_manual_unaffected(self):
    # Nidec Civic has no GEARBOX_ALT_2 in its DBC; the manual branch must not try to read it
    CP, CS, parsers = build_car_state(CAR.HONDA_CIVIC, gen_empty_fingerprint())
    self.assertEqual(CP.transmissionType, TransmissionType.manual)
    ret, _ = CS.update(parsers)
    self.assertFalse(ret.deprecated.clutchPressed)
    self.assertEqual(ret.deprecated.engineRpm, 0.)


if __name__ == "__main__":
  unittest.main()
