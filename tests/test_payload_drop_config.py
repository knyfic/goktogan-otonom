"""
Hedefe özel payload bırakma konfigürasyonu testleri.

Görev kuralı:
  Payload 1 (MAVİ payload) → KIRMIZI ÜÇGEN   → verify 3m  + RED offset
  Payload 2 (KIRMIZI payload) → MAVİ ALTIGEN → verify 8m  + BLUE offset
  Bırakma her zaman PAYLOAD_DROP_ALTITUDE (1m)'de — verify irtifası bırakma
  irtifası DEĞİLDİR; verify → OFFSET → görüntü işlemesiz dikey iniş → 1m → bırak.

Kapsanan gereksinimler:
  Test 1 — kırmızı payload: RED_TRIANGLE → 3m verify + RED offset + dikey iniş 1m
  Test 2 — mavi payload:   BLUE_HEXAGON → 8m verify + BLUE offset + dikey iniş 1m
  Test 3 — returning payload: 2. hedef MAVİ ALTIGEN ise BLUE offset seçilir
  Test 4 — offset ayrılığı: RED/BLUE offsetler farklı ve doğru hedefe bağlı
  Test 5 — vision cutoff: verify sonrası görüntü tabanlı yatay düzeltme YOK
  Test 6 — final descent: verify → OFFSET → görüntüsüz dikey iniş → 1m → bırakma
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import (
    BLUE_PAYLOAD_OFFSET_M,
    BLUE_PAYLOAD_VERIFY_ALTITUDE,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_DROP_ALTITUDE,
    RED_PAYLOAD_OFFSET_M,
    RED_PAYLOAD_VERIFY_ALTITUDE,
    TARGET_ALIGNED_FRAMES_REQUIRED,
)
from models import MissionState, Target, TargetColor, TargetType


def _align_sm(target_color, target_shape, altitude):
    """ALIGNING_TARGET durumunda, hedef tipi + irtifa ile gerçek SM kur."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._alt_waiting = False
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    sm._align_phase = "HORIZONTAL"
    sm._no_detection_count = 0
    sm._aligned_frame_count = TARGET_ALIGNED_FRAMES_REQUIRED - 1  # bir onay kaldı
    sm._last_align_cmd_t = 0.0
    det = SimpleNamespace(
        color=target_color, shape=target_shape,
        center_x=960.0, center_y=352.0,
        bbox_width_px=100.0, bbox_height_px=100.0,
        area_px=5000.0, confidence=0.9,
    )
    sm.data.pending_detection = det
    sm.target_detector = MagicMock()
    sm.target_detector.detect.return_value = [det]
    return sm, nav


def _run_horizontal_decision(sm, nav):
    """HORIZONTAL'da onayı doldurur; dönüş fazını döndürür (DESCENDING/OFFSET)."""
    sm._last_align_cmd_t = 0.0
    sm._do_align_target()
    return sm._align_phase


def _enter_offset(sm):
    """OFFSET fazına al ve ilk offset komutunu çalıştır (goto_body_offset)."""
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None
    sm._do_align_target()
    return sm._align_phase


# ── Test 1 — KIRMIZI üçgen (payload 1) ──────────────────────────────────── #
def test_red_triangle_verify_at_3m_then_red_offset():
    """RED_TRIANGLE: 3m'de (verify) onay dolunca OFFSET; OFFSET adımı RED offset."""
    sm, nav = _align_sm(TargetColor.RED, "TRIANGLE", RED_PAYLOAD_VERIFY_ALTITUDE)
    assert _run_horizontal_decision(sm, nav) == "OFFSET", \
        "verify irtifasında (3m) onay dolunca OFFSET'e geçilmeli"
    _enter_offset(sm)
    nav.goto_body_offset.assert_called_once_with(
        forward_m=RED_PAYLOAD_OFFSET_M, right_m=0.0
    )


# ── Test 2 — MAVİ altıgen (payload 2) ────────────────────────────────────── #
def test_blue_hexagon_verify_at_8m_then_blue_offset():
    """BLUE_HEXAGON: 8m'de (verify) onay dolunca OFFSET; OFFSET adımı BLUE offset."""
    sm, nav = _align_sm(TargetColor.BLUE, "HEXAGON", BLUE_PAYLOAD_VERIFY_ALTITUDE)
    assert _run_horizontal_decision(sm, nav) == "OFFSET", \
        "verify irtifasında (8m) onay dolunca OFFSET'e geçilmeli"
    _enter_offset(sm)
    nav.goto_body_offset.assert_called_once_with(
        forward_m=BLUE_PAYLOAD_OFFSET_M, right_m=0.0
    )


# ── Test 3 — Returning payload (2. hedef) ────────────────────────────────── #
def _return_sm(second_color, second_shape):
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=8.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.second_target = Target(
        target_id=2, shape=TargetType(second_shape), color=second_color,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    return sm, nav


def test_returning_payload_blue_hexagon_uses_blue_offset():
    """Geri getirilen yük akışı: 2. hedef MAVİ ALTIGEN → BLUE_PAYLOAD_OFFSET_M.

    (İlk payload'ın RED offseti asla ikinci payload akışına sızmamalı.)
    """
    sm, nav = _return_sm(TargetColor.BLUE, "HEXAGON")
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None
    sm._do_release_returned_payload()
    nav.goto_body_offset.assert_called_once_with(
        forward_m=BLUE_PAYLOAD_OFFSET_M, right_m=0.0
    )


def test_returning_payload_red_triangle_uses_red_offset():
    """Geri getirilen yük akışı: 2. hedef KIRMIZI ÜÇGEN → RED_PAYLOAD_OFFSET_M."""
    sm, nav = _return_sm(TargetColor.RED, "TRIANGLE")
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None
    sm._do_release_returned_payload()
    nav.goto_body_offset.assert_called_once_with(
        forward_m=RED_PAYLOAD_OFFSET_M, right_m=0.0
    )


# ── Test 4 — Offset ayrılığı ─────────────────────────────────────────────── #
def test_offsets_are_separate_and_mapped_per_target():
    """Kırmızı/mavi offsetler FARKLI ve doğru hedefe bağlıdır."""
    assert RED_PAYLOAD_OFFSET_M != BLUE_PAYLOAD_OFFSET_M

    sm, nav = _align_sm(TargetColor.RED, "TRIANGLE", RED_PAYLOAD_VERIFY_ALTITUDE)
    _enter_offset(sm)
    nav.goto_body_offset.assert_called_once_with(
        forward_m=RED_PAYLOAD_OFFSET_M, right_m=0.0
    )

    nav.reset_mock()
    sm2, nav2 = _align_sm(TargetColor.BLUE, "HEXAGON", BLUE_PAYLOAD_VERIFY_ALTITUDE)
    _enter_offset(sm2)
    nav2.goto_body_offset.assert_called_once_with(
        forward_m=BLUE_PAYLOAD_OFFSET_M, right_m=0.0
    )


# ── Test 5 — Vision cutoff (verify sonrası görüntü işleme YOK) ───────────── #
def test_above_verify_altitude_descends_not_offset():
    """Verify irtifasının ÜSTÜNDE onay dolunca OFFSET'e DEĞİL kademeli inişe geçilir
    (görüntü işleme verify irtifasına KADAR sürer)."""
    sm, nav = _align_sm(TargetColor.RED, "TRIANGLE", RED_PAYLOAD_VERIFY_ALTITUDE + 1.0)
    assert _run_horizontal_decision(sm, nav) == "DESCENDING"
    assert sm._alt_target == pytest.approx(RED_PAYLOAD_VERIFY_ALTITUDE)

    sm2, nav2 = _align_sm(TargetColor.BLUE, "HEXAGON", BLUE_PAYLOAD_VERIFY_ALTITUDE + 1.0)
    assert _run_horizontal_decision(sm2, nav2) == "DESCENDING"
    assert sm2._alt_target == pytest.approx(BLUE_PAYLOAD_VERIFY_ALTITUDE)


def test_no_vision_correction_after_verify_altitude():
    """Verify sonrası (OFFSET + DROPPING fazları) görüntü tabanlı yatay düzeltme
    ASLA yapılmaz — yalnızca dikey iniş."""
    sm, nav = _align_sm(TargetColor.RED, "TRIANGLE", RED_PAYLOAD_VERIFY_ALTITUDE)
    sm.position_controller.align_detection.reset_mock()

    # OFFSET girişi: offset komutu gönderilir — görüntü işleme yok
    assert _enter_offset(sm) == "OFFSET"
    sm.position_controller.align_detection.assert_not_called()

    # OFFSET süresi doldu → DROPPING (1m'ye dikey iniş) — görüntü işleme yok
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._do_align_target()
    assert sm._align_phase == "DROPPING"
    sm.position_controller.align_detection.assert_not_called()

    # İniş sürerken (bekleme tiki) — görüntü işleme yok
    sm._alt_waiting = True
    sm._do_align_target()
    sm.position_controller.align_detection.assert_not_called()


# ── Test 6 — Final descent sırası ────────────────────────────────────────── #
def test_final_descent_sequence_verify_offset_vertical_drop():
    """Sıra: verify → OFFSET → görüntüsüz dikey iniş → 1m → bırakma fazına geçiş."""
    sm, nav = _align_sm(TargetColor.BLUE, "HEXAGON", BLUE_PAYLOAD_VERIFY_ALTITUDE)

    # 1) verify irtifasında (8m) onay → OFFSET
    assert _run_horizontal_decision(sm, nav) == "OFFSET"
    # 2) OFFSET: ileri hareket komutu
    _enter_offset(sm)
    nav.goto_body_offset.assert_called_once()
    # 3) OFFSET süresi dolunca → DROPPING + 1m'ye dikey iniş komutu
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._do_align_target()
    assert sm._align_phase == "DROPPING"
    assert sm._alt_target == pytest.approx(PAYLOAD_DROP_ALTITUDE), \
        "dikey iniş hedefi bırakma irtifası (1m) olmalı"
    # 4) İniş tamamlanınca bırakma fazına (TARGET_ALIGNED → RELEASING_PAYLOAD) geçiş
    sm._alt_waiting = False
    sm._do_align_target()
    assert sm.data.state == MissionState.TARGET_ALIGNED


# ── Config seviyesi ──────────────────────────────────────────────────────── #
def test_verify_altitudes_config_values():
    """Verify irtifaları hedefe özel; bırakma her zaman 1m'de."""
    assert RED_PAYLOAD_VERIFY_ALTITUDE == pytest.approx(3.0)
    assert BLUE_PAYLOAD_VERIFY_ALTITUDE == pytest.approx(8.0)
    assert PAYLOAD_DROP_ALTITUDE == pytest.approx(1.0)
    assert RED_PAYLOAD_VERIFY_ALTITUDE > PAYLOAD_DROP_ALTITUDE
    assert BLUE_PAYLOAD_VERIFY_ALTITUDE > PAYLOAD_DROP_ALTITUDE
