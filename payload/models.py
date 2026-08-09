"""
Payload sistemine özgü veri modelleri ve enum'lar.

NOT: Mission-level state'ler models.py içinde (MissionState).
     Bu dosya yalnızca payload mekanizmasının iç durumunu temsil eder.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class PayloadMechanismState(str, Enum):
    """Fiziksel payload mekanizmasının anlık durumu."""
    UNINITIALIZED  = "UNINITIALIZED"   # Başlatılmamış
    READY          = "READY"           # Hazır, hareket bekliyor
    MOVING_DOWN    = "MOVING_DOWN"     # Stepper aşağı hareket ediyor
    MOVING_UP      = "MOVING_UP"       # Stepper yukarı hareket ediyor
    PICKING_UP     = "PICKING_UP"      # Pickup sekansı süruyor
    RELEASING      = "RELEASING"       # Release sekansı sürüyor
    EMERGENCY_STOP = "EMERGENCY_STOP"  # Acil durdurma aktif
    ERROR          = "ERROR"           # Kurtarılamaz hata


class OperationResult(str, Enum):
    SUCCESS          = "SUCCESS"
    FAILED           = "FAILED"
    TIMEOUT          = "TIMEOUT"
    HARDWARE_ERROR   = "HARDWARE_ERROR"
    INVALID_STATE    = "INVALID_STATE"
    ALREADY_DONE     = "ALREADY_DONE"
    EMERGENCY_ACTIVE = "EMERGENCY_ACTIVE"


class MagnetId(str, Enum):
    MAGNET_1 = "MAGNET_1"   # Mavi kablo → GPIO 5 (varsayılan)
    MAGNET_2 = "MAGNET_2"   # Kırmızı kablo → GPIO 6 (varsayılan)


@dataclass
class PayloadOperationResult:
    result:  OperationResult
    message: str = ""
    details: dict = field(default_factory=dict)

    @property
    def success(self) -> bool:
        return self.result == OperationResult.SUCCESS

    @classmethod
    def ok(cls, message: str = "") -> "PayloadOperationResult":
        return cls(result=OperationResult.SUCCESS, message=message)

    @classmethod
    def fail(cls, reason: OperationResult, message: str = "") -> "PayloadOperationResult":
        return cls(result=reason, message=message)


@dataclass
class MagnetState:
    magnet_1_on: bool = True
    magnet_2_on: bool = True


@dataclass
class PayloadSystemStatus:
    """Backend'in anlık snapshot'ı."""
    mechanism_state: PayloadMechanismState = PayloadMechanismState.UNINITIALIZED
    magnets: MagnetState = field(default_factory=MagnetState)
    carried_count: int = 2      # Başlangıçta 2 payload taşınıyor
    backend_type:  str = "unknown"
