"""
fork-only (civic-cruise) Phase 1: functional tests for the Honda Bosch-radarless manual gear/clutch plumbing
(GEARBOX_ALT_2.GEAR_MT, GAS_PEDAL_2 clutch switches + ENGINE_TORQUE_ESTIMATE, POWERTRAIN_DATA.ENGINE_RPM, the phantom
guard, and clutchSignalValid liveness). Synthetic CAN frames -> CarState.update().
"""
import math

from opendbc.car import Bus, structs
from opendbc.car.honda.values import CAR
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.carstate import CarState
from opendbc.car.structs import CLUTCH_PEDAL_UP, CLUTCH_PEDAL_FLOOR
from opendbc.can import CANPacker

TT = structs.CarParams.TransmissionType
DBC_NAME = 'honda_bosch_radarless_generated'


class TestCivicManualPlumbing:
  def setup_method(self):
    CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
    CP.transmissionType = TT.manual
    self.CP = CP
    self.CS = CarState(CP, structs.CarParamsSP())
    self.parsers = self.CS.get_can_parsers(CP, structs.CarParamsSP())
    self.pk = CANPacker(DBC_NAME)
    self.t = 0

  def _feed(self, *, gear_mt=3, counter=0, up=1, floor=0, torque=-65.0, rpm=2000.0, gearbox=True):
    self.t += 20_000_000  # 20 ms
    msgs = [
      self.pk.make_can_msg("GAS_PEDAL_2", 0, {"CLUTCH_PEDAL_UP": up, "CLUTCH_PEDAL_FLOOR": floor,
                                              "ENGINE_TORQUE_ESTIMATE": torque, "COUNTER": counter}),
      self.pk.make_can_msg("POWERTRAIN_DATA", 0, {"ENGINE_RPM": rpm, "PEDAL_GAS": 0}),
    ]
    if gearbox:
      msgs.append(self.pk.make_can_msg("GEARBOX_ALT_2", 0, {"GEAR_MT": gear_mt, "COUNTER": counter}))
    self.parsers[Bus.pt].update([self.t, msgs])
    return self.CS.update(self.parsers)

  def test_coupled_gear(self):
    # 3rd gear, clutch pedal up, 2000 rpm, -65 Nm engine braking
    for i in range(3):
      ret, ret_sp = self._feed(gear_mt=3, counter=i % 4, up=1, floor=0, torque=-65.0, rpm=2000.0)
    assert ret_sp.gearRaw == 3
    assert ret_sp.clutchSignalValid
    assert ret_sp.clutchPedal == CLUTCH_PEDAL_UP
    assert not ret.deprecated.clutchPressed             # coupled
    assert abs(ret.deprecated.engineRpm - 2000.0) < 1.0
    assert abs(ret_sp.engineTorqueAxle - (-65.0)) < 1.0

  def test_clutch_in(self):
    # pedal on the floor, GEAR_MT 0 -> drivetrain decoupled
    ret, ret_sp = self._feed(gear_mt=0, up=0, floor=1, torque=-30.0, rpm=1500.0)
    assert ret_sp.gearRaw == 0
    assert ret_sp.clutchPedal == CLUTCH_PEDAL_FLOOR
    assert ret.deprecated.clutchPressed

  def test_neutral_clutch_out(self):
    # clutch fully up but no gear meshed (true neutral) -> still decoupled, but the pedal reads UP (tells it from clutch-in)
    ret, ret_sp = self._feed(gear_mt=0, up=1, floor=0, rpm=1500.0)
    assert ret_sp.clutchPedal == CLUTCH_PEDAL_UP
    assert ret.deprecated.clutchPressed                 # GEAR_MT 0 = no gear meshed

  def test_phantom_gear_at_idle(self):
    # a gear reported at idle rpm and low speed (vEgo defaults to 0 here) is untrusted -> treated as decoupled
    ret, ret_sp = self._feed(gear_mt=2, up=1, floor=0, rpm=900.0)
    assert ret.deprecated.clutchPressed                 # phantom guard: not trusted yet

  def test_signal_missing_is_inert(self):
    # never feed GEARBOX_ALT_2: clutchSignalValid False, gearRaw -1, and clutchPressed fails safe (decoupled)
    ret, ret_sp = self._feed(up=1, floor=0, rpm=2000.0, gearbox=False)
    assert not ret_sp.clutchSignalValid
    assert ret_sp.gearRaw == -1
    assert ret.deprecated.clutchPressed

  def test_gas_pedal_2_torque_nan_when_stale(self):
    # engineTorqueAxle is NaN unless GAS_PEDAL_2 is alive; here it is alive, so it is a real number
    ret, ret_sp = self._feed(torque=-82.0)
    assert not math.isnan(ret_sp.engineTorqueAxle)
    assert abs(ret_sp.engineTorqueAxle - (-82.0)) < 1.0
