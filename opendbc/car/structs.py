from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


# CarStateSP.clutchPedal values (fork-only, Task 15). 0 = unknown, so the default never reads as 'pedal up'.
CLUTCH_PEDAL_UNKNOWN, CLUTCH_PEDAL_UP, CLUTCH_PEDAL_TRAVEL, CLUTCH_PEDAL_FLOOR = 0, 1, 2, 3


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()
  # fork-only (civic-cruise): the manual gearbox's own gear signal (Honda radarless: GEARBOX_ALT_2.GEAR_MT) and whether
  # that message is alive. A missing message would read as GEAR_MT 0 (= clutch pressed), so consumers stay inert while
  # clutchSignalValid is False.
  gearRaw: int = auto_field()
  clutchSignalValid: bool = auto_field()
  # fork-only (Task 15): the clutch pedal switches (Honda radarless: GAS_PEDAL_2) - 0 unknown / not alive (the default), 1 fully
  # up, 2 in travel, 3 on the floor. Tells clutch-in from neutral-with-the-clutch-out (both decouple: clutchPressed).
  clutchPedal: int = auto_field()
  # fork-only (Task 17): the engine's own torque estimate at the wheels (Honda radarless: GAS_PEDAL_2.ENGINE_TORQUE_ESTIMATE),
  # N*m, negative = engine braking; NaN while not alive. Engine side only (never includes the service brakes): the
  # engine-brake learner's brake-free measurement.
  engineTorqueAxle: float = auto_field()
