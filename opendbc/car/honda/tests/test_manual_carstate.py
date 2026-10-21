import math
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.honda.carstate import PhantomGearGuard, PHANTOM_RPM_MAX, PHANTOM_RPM_RELEASE, PHANTOM_MAX_SPEED
from opendbc.car.honda.hondacan import CanBus
from opendbc.car.structs import CLUTCH_PEDAL_UNKNOWN, CLUTCH_PEDAL_UP, CLUTCH_PEDAL_TRAVEL, CLUTCH_PEDAL_FLOOR
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

  def _step(self, gear_mt, rpm=0., reverse_light=False, pedal=None):
    return self._step_full(gear_mt, rpm, reverse_light, pedal)[0]

  def _step_full(self, gear_mt, rpm=0., reverse_light=False, pedal=None, engine_data_rpm=0., torque=0.):
    """pedal = (CLUTCH_PEDAL_UP, CLUTCH_PEDAL_FLOOR) switch bits in GAS_PEDAL_2, or None = GAS_PEDAL_2 not sent."""
    msgs = [
      self.packer.make_can_msg("GEARBOX_ALT_2", self.bus, {"GEAR_MT": gear_mt}),
      self.packer.make_can_msg("POWERTRAIN_DATA", self.bus, {"ENGINE_RPM": rpm}),
      self.packer.make_can_msg("ENGINE_DATA", self.bus, {"ENGINE_RPM": engine_data_rpm}),
      self.packer.make_can_msg("SCM_FEEDBACK", self.bus, {"REVERSE_LIGHT": int(reverse_light)}),
    ]
    if pedal is not None:
      msgs.append(self.packer.make_can_msg("GAS_PEDAL_2", self.bus, {"CLUTCH_PEDAL_UP": pedal[0], "CLUTCH_PEDAL_FLOOR": pedal[1],
                                                                        "ENGINE_TORQUE_ESTIMATE": torque}))
    self.t += int(0.01 * 1e9)
    self.parsers[Bus.pt].update([self.t, msgs])
    return self.CS.update(self.parsers)

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

  def _step_sp(self, gear_mt=None, rpm=0.):
    """Like _step, but GEARBOX_ALT_2 is optional (None = not on the bus this frame); returns (ret, ret_sp)."""
    msgs = [self.packer.make_can_msg("POWERTRAIN_DATA", self.bus, {"ENGINE_RPM": rpm}),
            self.packer.make_can_msg("SCM_FEEDBACK", self.bus, {"REVERSE_LIGHT": 0})]
    if gear_mt is not None:
      msgs.append(self.packer.make_can_msg("GEARBOX_ALT_2", self.bus, {"GEAR_MT": gear_mt}))
    self.t += int(0.01 * 1e9)
    self.parsers[Bus.pt].update([self.t, msgs])
    return self.CS.update(self.parsers)

  def test_gearbox_alt_2_never_affects_can_valid(self):
    # Task 12 B5 [A-F-M6]: registered with NaN frequency -> ignore_alive, so its absence can't invalidate CAN
    state = self.parsers[Bus.pt].message_states[0x1DD]
    self.assertTrue(state.ignore_alive)

  def test_clutch_signal_liveness(self):
    # Task 12 B5 [A-F-M6]: absent -> clutchSignalValid False (and gearRaw -1); a missing frame reads as GEAR_MT 0
    _, sp = self._step_sp(None)
    self.assertFalse(sp.clutchSignalValid)
    self.assertEqual(sp.gearRaw, -1)
    _, sp = self._step_sp(4, rpm=2000.)
    self.assertTrue(sp.clutchSignalValid)
    self.assertEqual(sp.gearRaw, 4)
    for _ in range(30):   # 0.3 s without GEARBOX_ALT_2 while the bus keeps going
      ret, sp = self._step_sp(None, rpm=2000.)
    self.assertFalse(sp.clutchSignalValid)
    self.assertEqual(sp.gearRaw, -1)

  def test_unknown_gear_mt_values_are_decoupled(self):
    # Task 12 A2.7 [B-L1]: only 1-6 and 14 have a gear meshed
    for gear_mt, pressed in ((7, True), (13, True), (15, True), (0, True), (6, False), (14, False)):
      with self.subTest(gear_mt=gear_mt):
        self.assertEqual(self._step(gear_mt, rpm=2000.).deprecated.clutchPressed, pressed)

  # --- Task 15: rpm source, clutch pedal switches ----------------------------------------------------------------------

  def test_rpm_from_powertrain_data_not_engine_data(self):
    # Stage 1 (2026-10-20): ENGINE_DATA's ENGINE_RPM reads 0 on the car; POWERTRAIN_DATA carries the real rpm
    ret, _ = self._step_full(3, rpm=2100., engine_data_rpm=0.)
    self.assertAlmostEqual(ret.deprecated.engineRpm, 2100.)
    ret, _ = self._step_full(3, rpm=2100., engine_data_rpm=4000.)
    self.assertAlmostEqual(ret.deprecated.engineRpm, 2100., msg="ENGINE_DATA must not be read")

  def test_clutch_pedal_decode(self):
    for bits, want in (((1, 0), CLUTCH_PEDAL_UP), ((0, 0), CLUTCH_PEDAL_TRAVEL), ((0, 1), CLUTCH_PEDAL_FLOOR),
                       ((1, 1), CLUTCH_PEDAL_UNKNOWN)):
      with self.subTest(bits=bits):
        _, sp = self._step_full(4, rpm=2500., pedal=bits)
        self.assertEqual(sp.clutchPedal, want)

  def test_clutch_pedal_unknown_when_absent_or_stale(self):
    _, sp = self._step_full(4, rpm=2500.)
    self.assertEqual(sp.clutchPedal, CLUTCH_PEDAL_UNKNOWN, "never received")
    _, sp = self._step_full(4, rpm=2500., pedal=(1, 0))
    self.assertEqual(sp.clutchPedal, CLUTCH_PEDAL_UP)
    for _ in range(30):   # 0.3 s without GAS_PEDAL_2 while the bus keeps going
      _, sp = self._step_full(4, rpm=2500.)
    self.assertEqual(sp.clutchPedal, CLUTCH_PEDAL_UNKNOWN, "stale")

  def test_engine_torque_axle(self):
    # Task 17: the engine's own axle-torque figure, NaN until / unless GAS_PEDAL_2 is alive
    _, sp = self._step_full(4, rpm=2500.)
    self.assertTrue(math.isnan(sp.engineTorqueAxle), "never received")
    _, sp = self._step_full(4, rpm=2500., pedal=(1, 0), torque=-82.)
    self.assertAlmostEqual(sp.engineTorqueAxle, -82.)
    for _ in range(30):
      _, sp = self._step_full(4, rpm=2500.)
    self.assertTrue(math.isnan(sp.engineTorqueAxle), "stale")

  def test_pedal_switch_leads_gear_mt(self):
    # the pedal leaving its top switch decouples at once, while GEAR_MT still reports the gear
    self.assertFalse(self._step(4, rpm=2500., pedal=(1, 0)).deprecated.clutchPressed)
    self.assertTrue(self._step(4, rpm=2500., pedal=(0, 0)).deprecated.clutchPressed)
    self.assertTrue(self._step(4, rpm=2500., pedal=(0, 1)).deprecated.clutchPressed)
    self.assertFalse(self._step(4, rpm=2500., pedal=(1, 0)).deprecated.clutchPressed)

  def test_neutral_clutch_out_is_decoupled_and_distinguishable(self):
    ret, sp = self._step_full(GEAR_MT_CLUTCH, rpm=850., pedal=(1, 0))
    self.assertTrue(ret.deprecated.clutchPressed)
    self.assertEqual(sp.clutchPedal, CLUTCH_PEDAL_UP)        # neutral: pedal up, no gear
    ret, sp = self._step_full(GEAR_MT_CLUTCH, rpm=850., pedal=(0, 1))
    self.assertTrue(ret.deprecated.clutchPressed)
    self.assertEqual(sp.clutchPedal, CLUTCH_PEDAL_FLOOR)     # clutch in

  def test_no_pedal_switch_falls_back_to_gear_mt(self):
    self.assertFalse(self._step(4, rpm=2500.).deprecated.clutchPressed)
    self.assertTrue(self._step(GEAR_MT_CLUTCH, rpm=2500.).deprecated.clutchPressed)

  def test_phantom_gear_at_standstill_idle(self):
    # Stage 1: parked / stopped in neutral at idle, GEAR_MT read 1 for 19 s with the pedal up. vEgo is 0 here.
    for _ in range(20):
      self.assertTrue(self._step(1, rpm=830., pedal=(1, 0)).deprecated.clutchPressed)
    # the engine driven clearly above idle at the same reported gear: trusted
    self.assertFalse(self._step(1, rpm=PHANTOM_RPM_RELEASE + 50., pedal=(1, 0)).deprecated.clutchPressed)


def run_bus(CP, CS, parsers, skip=(), seconds=1.0):
  """Feed every message the parsers subscribed (at 100 Hz, valid counters/checksums) except `skip`; return can_valid."""
  CS.update(parsers)
  packers = {b: CANPacker(cp.dbc_name) for b, cp in parsers.items()}
  for i in range(int(seconds * 100)):
    t = int((i + 1) * 1e7)
    for b, cp in parsers.items():
      msgs = [packers[b].make_can_msg(addr, cp.bus, {}) for addr in cp.message_states if addr not in skip]
      cp.update([t, msgs])
    CS.update(parsers)
  return all(cp.can_valid for cp in parsers.values())


class TestCanValid(unittest.TestCase):
  def test_civic_manual_can_valid_without_gearbox_alt_2(self):
    # Task 12 B5 [A-F-M6]: 0x1DD absent -> CAN stays valid, the clutch signal is reported dead
    CP, CS, parsers = build_car_state(CAR.HONDA_CIVIC_2022, gen_empty_fingerprint())
    self.assertTrue(run_bus(CP, CS, parsers, skip=(0x1DD,)))
    _, sp = CS.update(parsers)
    self.assertFalse(sp.clutchSignalValid)

  def test_integra_manual_can_valid(self):
    # Task 12 B5 [C-13]
    CP, CS, parsers = build_car_state(CAR.ACURA_INTEGRA, gen_empty_fingerprint())
    self.assertEqual(CP.transmissionType, TransmissionType.manual)
    self.assertTrue(run_bus(CP, CS, parsers))


class TestGearboxAlt2PresentButBad(unittest.TestCase):
  """Task 13 A3 [review2 N2]: 0x1DD arriving with a bad counter or checksum reads not-live (the clutch features go
  inert), and never invalidates CAN for the car — the counterpart of the absent-0x1DD canValid test above."""

  def setUp(self):
    self.CP, self.CS, self.parsers = build_car_state(CAR.HONDA_CIVIC_2022, gen_empty_fingerprint())
    self.CS.update(self.parsers)
    self.packers = {b: CANPacker(cp.dbc_name) for b, cp in self.parsers.items()}
    self.i = 0

  def frame(self, counter=None, bad_checksum=False, gear_mt=4):
    """One 100 Hz frame of every subscribed message (valid) plus 0x1DD as given; returns (canValid, CarStateSP)."""
    self.i += 1
    t = int(self.i * 1e7)
    for b, cp in self.parsers.items():
      msgs = [self.packers[b].make_can_msg(addr, cp.bus, {}) for addr in cp.message_states if addr != 0x1DD]
      if b == Bus.pt:
        vals = {"GEAR_MT": gear_mt} if counter is None else {"GEAR_MT": gear_mt, "COUNTER": counter}
        addr, dat, bus = self.packers[b].make_can_msg("GEARBOX_ALT_2", cp.bus, vals)
        if bad_checksum:
          dat = bytearray(dat)
          dat[7] ^= 0x0F   # CHECKSUM is bits 59..56 (byte 7, low nibble)
          dat = bytes(dat)
        msgs.append((addr, dat, bus))
      cp.update([t, msgs])
    _, sp = self.CS.update(self.parsers)
    return all(cp.can_valid for cp in self.parsers.values()), sp

  def test_clean_is_live(self):
    for _ in range(50):
      valid, sp = self.frame()
    self.assertTrue(valid)
    self.assertTrue(sp.clutchSignalValid)
    self.assertEqual(sp.gearRaw, 4)

  def test_corrupt_counter_not_live_can_still_valid(self):
    for _ in range(50):
      self.frame()
    for i in range(100):   # 1 s of a stuck counter (the parser alone would invalidate CAN after 5 frames)
      valid, sp = self.frame(counter=0)
      self.assertTrue(valid, f"canValid dropped at frame {i}")
      if i >= 1:
        self.assertFalse(sp.clutchSignalValid, f"live with a bad counter at frame {i}")
        self.assertEqual(sp.gearRaw, -1)
    # counters clean again: live after the bad-counter score decays (a few frames), never invalid CAN
    for _ in range(20):
      valid, sp = self.frame()
      self.assertTrue(valid)
    self.assertTrue(sp.clutchSignalValid)

  def test_single_counter_skip_recovers(self):
    for _ in range(20):
      self.frame()
    _, sp = self.frame(counter=2)   # the packer's sequence is at 20 % 4 == 0: a skip
    self.assertFalse(sp.clutchSignalValid)
    valid, sp = self.frame(counter=3)
    self.assertTrue(valid)
    self.assertTrue(sp.clutchSignalValid, "one clean frame clears a single skip")

  def test_bad_checksum_not_live_at_once_can_still_valid(self):
    for _ in range(50):
      self.frame()
    valid, sp = self.frame(bad_checksum=True, gear_mt=0)
    self.assertTrue(valid)
    self.assertFalse(sp.clutchSignalValid, "not live from the first bad frame (not after the 250 ms timeout)")
    for _ in range(100):
      valid, sp = self.frame(bad_checksum=True, gear_mt=0)
      self.assertTrue(valid)
      self.assertFalse(sp.clutchSignalValid)
    for _ in range(15):    # clean frames for > the 100 ms reject hold
      valid, sp = self.frame()
    self.assertTrue(valid)
    self.assertTrue(sp.clutchSignalValid)

  def test_counter_ignored_only_for_gearbox_alt_2(self):
    pt = self.parsers[Bus.pt]
    self.assertTrue(pt.message_states[0x1DD].ignore_counter)
    self.assertFalse(any(st.ignore_counter for addr, st in pt.message_states.items() if addr != 0x1DD))


class TestIntegraManual(unittest.TestCase):
  def test_integra_manual_reads_rpm_from_powertrain_data(self):
    # Task 12 B5 [C-13]: the Integra has no ENGINE_DATA; reading it would subscribe a message that never arrives
    CP, CS, parsers = build_car_state(CAR.ACURA_INTEGRA, gen_empty_fingerprint())
    self.assertEqual(CP.transmissionType, TransmissionType.manual)
    self.assertTrue(CP.flags & HondaFlags.BOSCH_RADARLESS)
    CS.update(parsers)
    packer, bus = CANPacker(DBC[CP.carFingerprint][Bus.pt]), CanBus(CP).pt
    parsers[Bus.pt].update([int(1e7), [packer.make_can_msg("POWERTRAIN_DATA", bus, {"ENGINE_RPM": 2500.}),
                                       packer.make_can_msg("GEARBOX_ALT_2", bus, {"GEAR_MT": 3})]])
    ret, sp = CS.update(parsers)
    self.assertAlmostEqual(ret.deprecated.engineRpm, 2500.)
    self.assertNotIn(0x158, parsers[Bus.pt].message_states, "ENGINE_DATA must not be subscribed on the Integra")


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


class TestPhantomGearGuard(unittest.TestCase):
  """Task 15: GEAR_MT is ratio-derived; in neutral at idle it reports whichever gear idle rpm matches. Cases from the
  Stage 1 route (00000002--6c1a3bb7d4)."""

  @staticmethod
  def feed(guard, frames):
    return [guard.update(g, rpm, v) for g, rpm, v in frames]

  def test_neutral_sweep_stays_untrusted(self):
    # t=825.7 s: 6th reported at 844 rpm / 10.9 m/s while braking in neutral; rpm stays at idle as the speed falls
    g = PhantomGearGuard()
    frames = [(6, 844. + (i % 3), 10.9 - 0.012 * i) for i in range(75)]
    self.assertTrue(all(self.feed(g, frames)))
    # then 5th as idle matches 5th's ratio (a new value is judged afresh)
    self.assertTrue(all(self.feed(g, [(5, 849., 8.4 - 0.012 * i) for i in range(30)])))

  def test_direct_sweep_two_to_one(self):
    g = PhantomGearGuard()
    self.assertTrue(all(self.feed(g, [(2, 840., 2.3 - 0.01 * i) for i in range(20)])))
    self.assertTrue(all(self.feed(g, [(1, 840., 1.9 - 0.01 * i) for i in range(20)])))

  def test_real_low_rpm_gear_trusted_once_rpm_follows_speed(self):
    # t=312.1 s style: 4th at 1011 rpm / 8.6 m/s, then the car slows in gear and the rpm follows at the ratio
    g = PhantomGearGuard()
    r0, v0 = 1011., 8.6
    out = self.feed(g, [(4, r0 * (v0 - 0.02 * i) / v0, v0 - 0.02 * i) for i in range(60)])
    self.assertTrue(out[0], "untrusted when first reported")
    self.assertFalse(out[-1], "trusted once the rpm tracked the speed")
    self.assertLess(out.index(False), 40, "within ~0.4 s at 2 m/s^2")

  def test_rpm_above_idle_trusts(self):
    g = PhantomGearGuard()
    self.assertTrue(g.update(2, 1000., 4.0))
    self.assertFalse(g.update(2, PHANTOM_RPM_RELEASE, 4.0))

  def test_outside_the_phantom_region_trusted_at_once(self):
    for rpm, v in ((PHANTOM_RPM_MAX, 5.), (900., PHANTOM_MAX_SPEED), (2400., 15.)):
      with self.subTest(rpm=rpm, v=v):
        self.assertFalse(PhantomGearGuard().update(3, rpm, v))

  def test_reverse_and_zero_never_flagged(self):
    for gear in (0, 14):
      self.assertFalse(PhantomGearGuard().update(gear, 800., 1.0))


if __name__ == "__main__":
  unittest.main()
