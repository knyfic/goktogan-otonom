"""
Alignment KABUL / CONFIRMATION — gerçek uçuş jitter'ına dayanıklılık testleri.

Gereksinim: İHA'nın hareket hızları DEĞİŞMEZ; "yeterince ortalandım" KARARI
gerçek uçuş koşullarına (rüzgâr, küçük pitch/yaw, titreşim → centroid'in
birkaç/onlarca piksel oynaması) karşı daha toleranslı olmalı — ama gerçek
alignment hataları yine de reddedilmeli.

Değişiklikler (yalnızca ACCEPTANCE/CONFIRMATION):
  1. PositionController.aligned_tolerance_px(alt) — "ortalandım" eşiği irtifaya
     göre uyarlanır: yüksek irtifada geniş (jitter'a toleranslı), alçakta hassas.
  2. _update_aligned_counter — hysteresis: araya EXIT toleransı içinde tek bir
     jitter frame'i girerse onay sayacı SIFIRLANMAZ (yalnızca gerçek sapma sıfırlar).

Hareket/gain/adım mantığı (ALIGN_GAIN, ALIGN_MAX_STEP_*, adaptive gain) VE
payload alımı toleransı (PAYLOAD_PIXEL_TOLERANCE_PX) bu testte DOKUNULMAZ.
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import (
    ALIGN_TOL_ALT_HI_M,
    ALIGN_TOL_ALT_LO_M,
    ALIGN_TOL_EXIT_FACTOR,
    ALIGN_TOL_HI_PX,
    ALIGN_TOL_LO_PX,
    ALIGNED_FRAMES_REQUIRED,
    PAYLOAD_PICKUP_VISION_MIN_S,
    RED_PAYLOAD_VERIFY_ALTITUDE,
    VISION_HEIGHT_PX,
    VISION_VERTICAL_FOV_DEG,
    VISION_WIDTH_PX,
)
from flight.position_controller import PositionController
from models import MissionState, Target, TargetColor, TargetType
from vision.geometry import CameraModel

_CAM = CameraModel(VISION_WIDTH_PX, VISION_HEIGHT_PX, 66.0, VISION_VERTICAL_FOV_DEG)
_FW, _FH = VISION_WIDTH_PX, VISION_HEIGHT_PX
_CX, _CY = _FW / 2.0, _FH / 2.0


def _pc() -> PositionController:
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=41.0, longitude=28.9, heading_deg=0.0,
    )
    return PositionController(nav, _CAM)


# ── 1) Adaptive tolerans: yüksek irtifa geniş / alçak irtifa hassas ────────── #
def test_aligned_tolerance_is_altitude_adaptive():
    """Eşik: ≤3m → 30px (hassas), ≥8m → 50px (jitter'a toleranslı), arada lerp."""
    pc = _pc()
    assert pc.aligned_tolerance_px(2.0) == pytest.approx(ALIGN_TOL_LO_PX)
    assert pc.aligned_tolerance_px(ALIGN_TOL_ALT_HI_M) == pytest.approx(ALIGN_TOL_HI_PX)
    mid = (ALIGN_TOL_ALT_LO_M + ALIGN_TOL_ALT_HI_M) / 2.0
    expected = (ALIGN_TOL_LO_PX + ALIGN_TOL_HI_PX) / 2.0
    assert pc.aligned_tolerance_px(mid) == pytest.approx(expected)


# ── 2) Yüksek irtifa: küçük/orta jitter kabul → komut YOK ──────────────────── #
@pytest.mark.parametrize("err_px", [5.0, 8.0, 15.0, 30.0, 45.0])
def test_high_altitude_jitter_accepted(err_px):
    """8m'de 5-45px centroid oynaması (rüzgâr/pitch/yaw) 'hizalı' sayılır —
    sürekli düzeltme komutu üretmez (descent'a geçilebilir)."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(latitude=41.0, longitude=28.9, heading_deg=0.0)
    pc = PositionController(nav, _CAM)

    ok = pc.align_detection(_CX + err_px, _CY, _FW, _FH, ALIGN_TOL_ALT_HI_M)

    assert ok is True, f"{err_px:.0f}px hata {ALIGN_TOL_ALT_HI_M:.0f}m'de kabul edilmeli"
    nav.goto_global.assert_not_called()


# ── 3) Gerçek büyük hata her irtifada reddedilir ───────────────────────────── #
def test_high_altitude_big_error_rejected():
    """8m'de 60px (> yüksek irtifa eşiği 50px) → RED: komut gönderilir."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(latitude=41.0, longitude=28.9, heading_deg=0.0)
    pc = PositionController(nav, _CAM)

    ok = pc.align_detection(_CX + 60.0, _CY, _FW, _FH, ALIGN_TOL_ALT_HI_M)

    assert ok is False
    nav.goto_global.assert_called_once()


def test_low_altitude_stays_precise():
    """Aynı 40px hata: 8m'de kabul edilir ama 3m'de (bırakma öncesi) RED —
    alçak irtifada hassasiyet korunur."""
    pc = _pc()
    assert pc.align_detection(_CX + 40.0, _CY, _FW, _FH, 8.0) is True, \
        "8m'de 40px kabul (jitter toleransı)"
    assert pc.align_detection(_CX + 40.0, _CY, _FW, _FH, RED_PAYLOAD_VERIFY_ALTITUDE) is False, \
        "3m'de 40px red (bırakma hassasiyeti)"


# ── 4) Confirmation hysteresis: jitter sayacı sıfırlamaz ───────────────────── #
def _pickup_center_sm(detections_per_frame, align_results):
    """ALIGNING_ON_TARGET_FOR_PICKUP — frame başına detection + align sonucu."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=3.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((_FH, _FW, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_ON_TARGET_FOR_PICKUP
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    sm._align_for_pickup_deadline = time.monotonic() + 60.0
    sm._align_for_pickup_start = time.monotonic() - PAYLOAD_PICKUP_VISION_MIN_S - 1.0
    sm._last_align_cmd_t = 0.0
    sm._aligned_frame_count = 0
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.side_effect = list(align_results)
    sm.position_controller.aligned_tolerance_px.return_value = ALIGN_TOL_LO_PX
    sm.target_detector = MagicMock()
    sm.target_detector.detect.side_effect = list(detections_per_frame)
    return sm


def _det(cx: float):
    return [SimpleNamespace(
        color=TargetColor.RED, shape="TRIANGLE",
        center_x=cx, center_y=_CY,
        bbox_width_px=50.0, bbox_height_px=50.0,
        confidence=0.9,
    )]


def test_jitter_pattern_does_not_lose_confirmation():
    """Kullanıcı pattern'i: ALIGNED, ALIGNED, NOT(EXIT içi jitter), ALIGNED,
    ALIGNED — aradaki tek jitter frame sayacı SIFIRLAMAZ → onay tamamlanır
    (eski davranış: sayaç 1,2,0,1,2 → asla tamamlanamazdı)."""
    # Frames:  hizalı, hizalı, jitter(dx=40px — exit tol 48px içinde), hizalı, hizalı
    frames = [_det(_CX), _det(_CX), _det(_CX + 40.0), _det(_CX), _det(_CX)]
    aligns = [True, True, False, True, True]
    sm = _pickup_center_sm(frames, aligns)

    reached_searching = False
    for _ in range(len(frames)):
        sm._last_align_cmd_t = 0.0
        sm._do_align_on_target_for_pickup()
        if sm.data.state == MissionState.SEARCHING_PAYLOAD:
            reached_searching = True

    assert reached_searching, \
        "jitter frame'i sayacı sıfırlamamalı — onay tamamlanıp SEARCHING'e geçilmeli " \
        "(eski davranış 1,2,0,1,2 → asla tamamlanamazdı)"
    assert sm.data.state == MissionState.SEARCHING_PAYLOAD


def test_real_offset_resets_confirmation():
    """Gerçek sapma (EXIT toleransı dışı, dx=200px) sayacı SIFIRLAR — onay
    tamamlanamaz, SEARCHING'e geçilmez (güvenlik: yanlış alignment kabul YOK)."""
    frames = [_det(_CX), _det(_CX + 200.0), _det(_CX), _det(_CX)]
    aligns = [True, False, True, True]
    sm = _pickup_center_sm(frames, aligns)

    for _ in range(len(frames)):
        sm._last_align_cmd_t = 0.0
        sm._do_align_on_target_for_pickup()

    assert sm._aligned_frame_count < ALIGNED_FRAMES_REQUIRED, \
        "gerçek sapma sayacı sıfırlamalı — yanlış onay birikmemeli"
    assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP


def test_persistently_far_target_never_confirmed():
    """Sürekli büyük hata (dx=150px) → sayaç 0 kalır, asla onaylanmaz."""
    frames = [_det(_CX + 150.0)] * 6
    aligns = [False] * 6
    sm = _pickup_center_sm(frames, aligns)

    for _ in range(len(frames)):
        sm._last_align_cmd_t = 0.0
        sm._do_align_on_target_for_pickup()

    assert sm._aligned_frame_count == 0
    assert sm.data.state == MissionState.ALIGNING_ON_TARGET_FOR_PICKUP


# ── 5) ALIGNED/confirmation eşiklerinin mevcut değerleri ───────────────────── #
def test_confirmation_thresholds_unchanged():
    """Ardışık onay eşikleri DEĞİŞMEDİ (yalnızca jitter dayanıklılığı eklendi)."""
    assert ALIGNED_FRAMES_REQUIRED == 3
    assert ALIGN_TOL_EXIT_FACTOR > 1.0, "exit toleransı enter'dan geniş olmalı"
