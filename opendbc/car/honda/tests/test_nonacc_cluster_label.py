"""
fork-only (civic-cruise): the gauge cluster's conventional-cruise icon. openpilot draws the cluster from the ACC_HUD it
sends, so the icon only follows the car's ACC <-> Cruise switch if openpilot sets CRUISE_CONTROL_LABEL. That is gated on
the NonAccClusterLabel param (default ON, owner decision 2026-11-05), on the car being in conventional mode, and on a radarless Bosch manual with
openpilot longitudinal. Everywhere else ACC_HUD is byte-for-byte unchanged.
"""
from opendbc.car import structs
from opendbc.car.honda import hondacan
from opendbc.car.honda.carcontroller import NonAccClusterLabel
from opendbc.car.honda.interface import CarInterface
from opendbc.car.honda.values import CAR
from opendbc.can import CANPacker
from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value

TT = structs.CarParams.TransmissionType
DBC_NAME = 'honda_bosch_radarless_generated'


class FakeParams:
  def __init__(self, value):
    self.value = value
    self.gets = 0

  def get(self, key, return_default=False):
    assert key == "NonAccClusterLabel" and return_default   # must ask for the registered default (ON)
    self.gets += 1
    if isinstance(self.value, Exception):
      raise self.value
    return self.value


def _cp(manual=True, op_long=True):
  CP = CarInterface.get_non_essential_params(CAR.HONDA_CIVIC_2022)
  CP.transmissionType = TT.manual if manual else TT.automatic
  CP.openpilotLongitudinalControl = op_long
  return CP


def _acc_hud(conventional, enabled=True, lead=True):
  hud = structs.CarControl.HUDControl(leadVisible=lead, leadDistanceBars=2)
  kw = {} if conventional is None else {"conventional_cruise": conventional}
  addr, dat, bus = hondacan.create_acc_hud(CANPacker(DBC_NAME), 0, _cp(), enabled, 0, 0.0, hud, 100, False, {}, 0, True, **kw)
  sigs = DBC(DBC_NAME).msgs[addr].sigs
  return dat, {n: get_raw_value(dat, s) for n, s in sigs.items()}


class TestNonAccClusterLabel:
  def test_label_only_when_toggle_on_and_car_in_conventional_mode(self):
    assert NonAccClusterLabel(_cp(), FakeParams(True)).update(True) is True
    assert NonAccClusterLabel(_cp(), FakeParams(True)).update(False) is False    # car in ACC mode
    assert NonAccClusterLabel(_cp(), FakeParams(False)).update(True) is False    # switched off over SSH
    assert NonAccClusterLabel(_cp(), FakeParams(None)).update(True) is True      # unset: default ON

  def test_inert_on_other_cars(self):
    for CP in (_cp(manual=False), _cp(op_long=False), CarInterface.get_non_essential_params(CAR.HONDA_CIVIC)):
      p = FakeParams(True)
      assert NonAccClusterLabel(CP, p).update(True) is False
      assert p.gets == 0                                                       # not even a param read

  def test_param_read_at_1hz_and_live(self):
    p = FakeParams(False)
    lab = NonAccClusterLabel(_cp(), p)
    assert [lab.update(True) for _ in range(10)] == [False] * 10
    assert p.gets == 1                    # one read per 10 calls (ACC_HUD is 10 Hz)
    p.value = True
    assert lab.update(True) is True       # picked up on the next read, no restart
    assert p.gets == 2

  def test_params_failure_never_shows_the_label_or_raises(self):
    assert NonAccClusterLabel(_cp(), FakeParams(KeyError("unknown key"))).update(True) is False

  def test_acc_hud_unchanged_without_the_label(self):
    base, _ = _acc_hud(None)
    off, sig = _acc_hud(False)
    assert off == base
    assert sig["CRUISE_CONTROL_LABEL"] == 0 and sig["HUD_LEAD"] == 2

  def test_acc_hud_with_the_label_matches_the_stock_camera(self):
    _, sig = _acc_hud(True)
    assert sig["CRUISE_CONTROL_LABEL"] == 1
    assert sig["HUD_LEAD"] == 0           # stock shows no lead-car graphic in conventional mode
    _, base = _acc_hud(False)
    changed = {n for n in sig if sig[n] != base[n]} - {"CHECKSUM"}
    assert changed == {"CRUISE_CONTROL_LABEL", "HUD_LEAD"}   # nothing else in the message moves
