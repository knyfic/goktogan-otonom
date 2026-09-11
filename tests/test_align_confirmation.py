"""
Hizalama — ardışık frame onayı (temporal confirmation) testleri.

Sorun: tek bir frame'de hedefin kadraj merkezine milisaniyelik denk gelmesi
(jitter / tesadüfi geçiş) İHA'yı yanlış "ortaladım" sonucuna götürüyordu.
Çözüm: "hizalı" kararı için ALIGNED_FRAMES_REQUIRED ardışık onay şartı;
araya hizasız frame girerse sayaç sıfırlanır.
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from config import (
    ALIGN_ON_TARGET_TIMEOUT_S,
    ALIGNED_FRAMES_REQUIRED,
    TARGET_ALIGNED_FRAMES_REQUIRED,
    PAYLOAD_DESCENT_STEP_M,
    PAYLOAD_PICKUP_VISION_MIN_S,
    PAYLOAD_RELEASE_ALTITUDE,
    RED_PAYLOAD_VERIFY_ALTITUDE,
)
from models import TargetColor, TargetType


def _real_sm(altitude: float | None = None):
    """Gerçek __init__ ile kurulan örnek (tüm öznitelikler hazır)."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude if altitude is not None
            else PAYLOAD_RELEASE_ALTITUDE,
        latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.nav = nav
    sm.camera = camera
    return sm, nav


def _detection(cx: float = 32.0, cy: float = 32.0, bbox_w: float = 10.0):
    """Kadraj merkezindeki (ve küçük bbox'lı) sahte tespit."""
    d = MagicMock()
    d.center_x = cx
    d.center_y = cy
    d.bbox_width_px = bbox_w
    d.confidence = 0.9
    return d


# ── 1) Hedef hizalama (_do_align_target) ──────────────────────────────────── #
def test_align_target_single_centered_frame_is_not_aligned():
    """Tek frame'de merkezde görünmek 'hizalı' sayılmaz — iniş başlamaz."""
    sm, nav = _real_sm()
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm.data.pending_detection = _detection(32.0, 32.0)  # frame 64x64 → merkez

    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._last_align_cmd_t = 0.0
    sm._aligned_frame_count = 0

    sm._do_align_target()

    assert sm._aligned_frame_count == 1
    assert sm._align_phase == "HORIZONTAL"
    nav.set_relative_altitude.assert_not_called()
    nav.goto_body_offset.assert_not_called()


def test_align_target_requires_consecutive_frames():
    """TARGET_ALIGNED_FRAMES_REQUIRED ardışık onay dolmadan OFFSET/inişe geçilmez."""
    sm, nav = _real_sm(altitude=RED_PAYLOAD_VERIFY_ALTITUDE)
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm.data.pending_detection = _detection(32.0, 32.0)

    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0

    # Kırmızı üçgen hedefinde verify irtifasında (RED_PAYLOAD_VERIFY_ALTITUDE =
    # 3m) onay dolunca → OFFSET (kademeli inişe geçmeden).
    for _ in range(TARGET_ALIGNED_FRAMES_REQUIRED - 1):
        sm._last_align_cmd_t = 0.0
        sm._do_align_target()
        assert sm._align_phase == "HORIZONTAL", "onay dolmadan faz değişmemeli"

    sm._last_align_cmd_t = 0.0
    sm._do_align_target()
    assert sm._align_phase == "OFFSET", "ardışık onay dolunca OFFSET'e geçilmeli"


def test_align_target_resets_counter_on_unaligned_frame():
    """Araya hizasız frame girerse sayaç sıfırlanır, yeniden onay gerekir."""
    sm, nav = _real_sm()
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm.data.pending_detection = _detection(32.0, 32.0)

    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0

    # 1 hizalı → sayaç 1
    sm._last_align_cmd_t = 0.0
    sm._do_align_target()
    assert sm._aligned_frame_count == 1

    # 1 hizasız → sayaç 0
    sm.position_controller.align_detection.return_value = False
    sm._last_align_cmd_t = 0.0
    sm._do_align_target()
    assert sm._aligned_frame_count == 0
    assert sm._align_phase == "HORIZONTAL"

    # Yeniden 1 hizalı → sayaç yalnızca 1 (sıfırdan başladı)
    sm.position_controller.align_detection.return_value = True
    sm._last_align_cmd_t = 0.0
    sm._do_align_target()
    assert sm._aligned_frame_count == 1
    assert sm._align_phase == "HORIZONTAL"


# ── 2) Payload hizalama (_do_align_payload) ───────────────────────────────── #
def _payload_sm():
    sm, nav = _real_sm()
    # Detector/tracker/filtre mock'ları: kadraj merkezinde sahte payload
    # (frame 704x1920 → merkez (960, 352))
    pl = SimpleNamespace(
        center_x=960.0, center_y=352.0,
        width_px=10.0, height_px=10.0, area_px=100.0,
        confidence=0.9, color=TargetColor.RED,
    )
    sm.payload_detector = MagicMock()
    sm.payload_detector.detect.return_value = [pl]
    sm.payload_tracker = MagicMock()
    sm.payload_tracker.update.return_value = pl
    sm._filter_payload_results = MagicMock(side_effect=lambda results, alt: results)
    sm.position_controller = MagicMock()
    sm.data.first_target = SimpleNamespace(
        color=TargetColor.BLUE, shape=TargetType.HEXAGON,
    )
    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0
    return sm, nav


def test_align_payload_single_centered_frame_does_not_descend():
    """Tek frame'de payload merkezde görünse bile basamak inişi başlamaz."""
    sm, nav = _payload_sm()

    sm._last_align_cmd_t = 0.0
    sm._do_align_payload()

    assert sm._aligned_frame_count == 1
    assert sm._align_phase == "HORIZONTAL"
    nav.set_relative_altitude.assert_not_called()


def test_align_payload_requires_consecutive_frames():
    """ALIGNED_FRAMES_REQUIRED ardışık onay dolunca basamak inişi başlar."""
    sm, nav = _payload_sm()

    for _ in range(ALIGNED_FRAMES_REQUIRED - 1):
        sm._last_align_cmd_t = 0.0
        sm._do_align_payload()
        assert sm._align_phase == "HORIZONTAL", "onay dolmadan iniş başlamamalı"

    sm._last_align_cmd_t = 0.0
    sm._do_align_payload()

    assert sm._align_phase == "DESCENDING"
    expected_alt = max(
        0.0, PAYLOAD_RELEASE_ALTITUDE - PAYLOAD_DESCENT_STEP_M
    )
    nav.set_relative_altitude.assert_called_once_with(expected_alt)


def test_align_payload_resets_counter_on_unaligned_frame():
    """Payload tarafında da araya hizasız frame girerse sayaç sıfırlanır."""
    sm, nav = _payload_sm()
    # Merkez dışı payload → hizasız (kadraj merkezinden uzak)
    sm.payload_detector.detect.return_value = [
        SimpleNamespace(
            center_x=2000.0, center_y=2000.0,   # 704x1920 frame merkezinden uzak
            width_px=10.0, height_px=10.0, area_px=100.0,
            confidence=0.9, color=TargetColor.RED,
        )
    ]
    sm.payload_tracker.update.return_value = sm.payload_detector.detect.return_value[0]

    sm._last_align_cmd_t = 0.0
    sm._do_align_payload()
    assert sm._aligned_frame_count == 0
    assert sm._align_phase == "HORIZONTAL"
    nav.set_relative_altitude.assert_not_called()


# ── 3) Pickup öncesi hedef merkezleme (_do_align_on_target_for_pickup) ─────── #
def _pickup_center_sm():
    """ALIGNING_ON_TARGET_FOR_PICKUP fazı için örnek (hedef kadraj merkezinde)."""
    from models import MissionState

    sm, nav = _real_sm()
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm.data.state = MissionState.ALIGNING_ON_TARGET_FOR_PICKUP
    sm.data.first_target = SimpleNamespace(
        color=TargetColor.BLUE, shape=TargetType.HEXAGON,
    )
    # Kadraj merkezinde hedef şekli (frame 704x1920 → merkez (960,352))
    det = SimpleNamespace(
        color=TargetColor.BLUE, shape="HEXAGON",
        center_x=960.0, center_y=352.0,
        bbox_width_px=100.0, bbox_height_px=100.0,
        confidence=0.9,
    )
    sm.target_detector = MagicMock()
    sm.target_detector.detect.return_value = [det]
    sm._align_for_pickup_deadline = time.monotonic() + ALIGN_ON_TARGET_TIMEOUT_S
    sm._align_for_pickup_start = time.monotonic()
    sm._last_align_cmd_t = 0.0
    sm._aligned_frame_count = 0
    return sm, nav


def test_pickup_center_single_centered_frame_does_not_advance():
    """Tek frame'de hedef merkezde görünse bile SEARCHING'e geçilmez."""
    from models import MissionState

    sm, nav = _pickup_center_sm()

    sm._do_align_on_target_for_pickup()

    assert sm._aligned_frame_count == 1
    assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP


def test_pickup_center_waits_min_vision_time():
    """Hizalı sayılsa bile faz PAYLOAD_PICKUP_VISION_MIN_S dolmadan SEARCHING'e
    geçilmez — İHA hedefi gerçekten ortalamalı."""
    from models import MissionState

    sm, nav = _pickup_center_sm()
    # Faz yeni başladı: min süre henüz dolmadı
    sm._align_for_pickup_start = time.monotonic()

    for _ in range(ALIGNED_FRAMES_REQUIRED):
        sm._last_align_cmd_t = 0.0
        sm._do_align_on_target_for_pickup()
        assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP, \
            "min süre dolmadan SEARCHING'e geçilmemeli"


def test_pickup_center_advances_after_min_vision_time():
    """Ardışık onay + min süre dolunca SEARCHING_PAYLOAD'e geçilir."""
    from models import MissionState

    sm, nav = _pickup_center_sm()
    # Faz yeterince uzun sürdü (min süre doldu)
    sm._align_for_pickup_start = time.monotonic() - PAYLOAD_PICKUP_VISION_MIN_S - 1.0

    for _ in range(ALIGNED_FRAMES_REQUIRED - 1):
        sm._last_align_cmd_t = 0.0
        sm._do_align_on_target_for_pickup()
        assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP

    sm._last_align_cmd_t = 0.0
    sm._do_align_on_target_for_pickup()
    assert sm.data.state == MissionState.SEARCHING_PAYLOAD


def test_pickup_center_resets_counter_when_target_lost():
    """Hedef kadrajda görünmezse onay sayacı sıfırlanır."""
    from models import MissionState

    sm, nav = _pickup_center_sm()
    sm._align_for_pickup_start = time.monotonic() - PAYLOAD_PICKUP_VISION_MIN_S - 1.0

    # 1 hizalı onay
    sm._last_align_cmd_t = 0.0
    sm._do_align_on_target_for_pickup()
    assert sm._aligned_frame_count == 1

    # Hedef kayboldu → sayaç sıfırlanır
    sm.target_detector.detect.return_value = []
    sm._do_align_on_target_for_pickup()
    assert sm._aligned_frame_count == 0
    assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP


# ── 4) Payload görünmezken minimum görüntü işleme süresi ───────────────────── #
def test_align_payload_no_detection_waits_min_vision_time():
    """Payload görünmese bile min süre dolmadan SEARCHING'e dönülmez."""
    from models import MissionState

    sm, nav = _payload_sm()
    sm.data.state = MissionState.ALIGNING_PAYLOAD
    # Payload hiç görünmüyor
    sm.payload_detector.detect.return_value = []
    sm.payload_tracker.update.return_value = None
    sm._align_phase_start_t = None   # HORIZONTAL girişinde set edilecek

    sm._last_align_cmd_t = 0.0
    sm._do_align_payload()

    assert sm.data.state == MissionState.ALIGNING_PAYLOAD, \
        "min süre dolmadan SEARCHING'e dönülmemeli"


def test_align_payload_no_detection_returns_after_min_vision_time():
    """Payload min süre boyunca görünmezse SEARCHING_PAYLOAD'e dönülür."""
    from models import MissionState

    sm, nav = _payload_sm()
    sm.data.state = MissionState.ALIGNING_PAYLOAD
    sm.payload_detector.detect.return_value = []
    sm.payload_tracker.update.return_value = None
    # Faz zaten PAYLOAD_PICKUP_VISION_MIN_S kadar sürdü ve eşik aşıldı
    sm._align_phase_start_t = time.monotonic() - PAYLOAD_PICKUP_VISION_MIN_S - 1.0
    sm._no_detection_count = sm._NO_DETECTION_LIMIT + 1   # > eşik (config'ten)

    sm._last_align_cmd_t = 0.0
    sm._do_align_payload()

    assert sm.data.state == MissionState.SEARCHING_PAYLOAD
