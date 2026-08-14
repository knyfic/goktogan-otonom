"""
Hizalama — ardışık frame onayı (temporal confirmation) testleri.

Sorun: tek bir frame'de hedefin kadraj merkezine milisaniyelik denk gelmesi
(jitter / tesadüfi geçiş) İHA'yı yanlış "ortaladım" sonucuna götürüyordu.
Çözüm: "hizalı" kararı için ALIGNED_FRAMES_REQUIRED ardışık onay şartı;
araya hizasız frame girerse sayaç sıfırlanır.
"""
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np

from config import (
    ALIGNED_FRAMES_REQUIRED,
    PAYLOAD_DESCENT_STEP_M,
    PAYLOAD_RELEASE_ALTITUDE,
)
from models import TargetColor, TargetType


def _real_sm():
    """Gerçek __init__ ile kurulan örnek (tüm öznitelikler hazır)."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=PAYLOAD_RELEASE_ALTITUDE,
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
    """ALIGNED_FRAMES_REQUIRED ardışık onay dolmadan OFFSET/inişe geçilmez."""
    sm, nav = _real_sm()
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm.data.pending_detection = _detection(32.0, 32.0)

    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0

    # 2.5m'de (PAYLOAD_RELEASE_ALTITUDE) olduğu için onay dolunca → OFFSET
    for _ in range(ALIGNED_FRAMES_REQUIRED - 1):
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
