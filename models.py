from dataclasses import dataclass
from enum import Enum
from typing import Optional


class MissionState(str, Enum):
    # ArduPilot pre-arm kontrolleri geçmeden görev başlatılmaz.
    # Bu state'te kamera/vision/HUD/GStreamer CANLI çalışır; yalnızca
    # uçuş görevleri (arm/takeoff/nav) kilitlidir. Pre-arm GOOD → INIT.
    WAITING_FOR_PREARM = "WAITING_FOR_PREARM"
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    SEARCHING_FIRST_TARGET = "SEARCHING_FIRST_TARGET"
    TARGET_DETECTED = "TARGET_DETECTED"
    TARGET_CLASSIFIED = "TARGET_CLASSIFIED"
    APPROACHING_TARGET = "APPROACHING_TARGET"
    ALIGNING_TARGET = "ALIGNING_TARGET"
    TARGET_ALIGNED = "TARGET_ALIGNED"
    RELEASING_PAYLOAD = "RELEASING_PAYLOAD"
    ASCENDING_AFTER_FIRST_RELEASE = "ASCENDING_AFTER_FIRST_RELEASE"
    SEARCHING_SECOND_TARGET = "SEARCHING_SECOND_TARGET"
    NAVIGATING_TO_FIRST_TARGET = "NAVIGATING_TO_FIRST_TARGET"
    SEARCHING_PAYLOAD = "SEARCHING_PAYLOAD"
    ALIGNING_PAYLOAD = "ALIGNING_PAYLOAD"
    OFFSET_CORRECTION = "OFFSET_CORRECTION"
    ASCENDING_FOR_PAYLOAD_RETRY = "ASCENDING_FOR_PAYLOAD_RETRY"
    PICKING_UP_PAYLOAD = "PICKING_UP_PAYLOAD"
    ASCENDING_FOR_VERIFICATION = "ASCENDING_FOR_VERIFICATION"
    VERIFYING_PAYLOAD = "VERIFYING_PAYLOAD"
    TASK_2_COMPLETE = "TASK_2_COMPLETE"
    TASK_2_FAILED = "TASK_2_FAILED"
    ALIGNING_ON_TARGET_FOR_PICKUP = "ALIGNING_ON_TARGET_FOR_PICKUP"   # Yük aramadan önce hedef merkezine hizala
    RETURNING_PAYLOAD = "RETURNING_PAYLOAD"
    RELEASING_RETURNED_PAYLOAD = "RELEASING_RETURNED_PAYLOAD"
    # Pilot RC'den GUIDED dışı bir mod seçti (LAND/LOITER/RTL/STABILIZE/...) —
    # mission DURDU. Bu state'te Python hiçbir uçuş komutu göndermez ve state
    # otomatik ilerlemez (terminal durdurma — güvenlik). Pilot geri GUIDED'a
    # geçse bile mission otomatik devam ETMEZ.
    PILOT_CONTROL = "PILOT_CONTROL"
    RTL = "RTL"
    COMPLETE = "COMPLETE"


class TargetType(str, Enum):
    HEXAGON = "HEXAGON"
    TRIANGLE = "TRIANGLE"


class TargetColor(str, Enum):
    BLUE = "BLUE"
    RED = "RED"


@dataclass
class Detection:
    color: TargetColor
    shape: str
    center_x: float
    center_y: float
    area_px: float
    bbox_width_px: float
    bbox_height_px: float
    confidence: float
    color_purity: float = 0.0  # kontur içi baskın renk oranı (0..1)


@dataclass
class Target:
    target_id: int
    shape: TargetType
    color: TargetColor
    latitude: float
    longitude: float
    confidence: float
    detected: bool = True
    payload_released: bool = False


@dataclass
class VehiclePosition:
    latitude: float = 0.0
    longitude: float = 0.0
    relative_altitude: float = 0.0
    heading_deg: float = 0.0
    battery_voltage_v: float = 0.0   # BATTERY_STATUS hücre toplamı (0 = henüz bilinmiyor)
    battery_percent: int = -1        # SYS_STATUS battery_remaining (-1 = bilinmiyor)


@dataclass
class PrePickupSignature:
    """
    Pickup başlamadan hemen önce kaydedilen payload'ın görsel referansı.
    Birkaç stabil frame'den elde edilen medyan değerler kullanılır.
    """
    area_px:      float   # medyan piksel alanı
    width_px:     float   # medyan bbox genişliği
    height_px:    float   # medyan bbox yüksekliği
    aspect_ratio: float   # medyan en-boy oranı (max/min)
    confidence:   float   # medyan detector confidence
    # Kaç frame'den oluşturuldu (güvenilirlik göstergesi)
    sample_count: int = 0


@dataclass
class MissionData:
    state: MissionState = MissionState.INIT
    first_target: Optional[Target] = None
    second_target: Optional[Target] = None
    pending_detection: Optional[Detection] = None
    pickup_attempts: int = 0
    release_count: int = 0
    mission_failed: bool = False
