"""
Payload bırakımı — 2.5m hizalama + OFFSET + dik iniş testleri.

Yeni akış (kadraj kırpma sonrası):
  1. 2.5m'de (PAYLOAD_RELEASE_ALTITUDE) hassas görüntü hizalaması.
  2. Kamera-mekanizma OFFSET düzeltmesi (İHA ileri — mekanizma hedef üzerine).
  3. DİK iniş: 2.5m → 1m (PAYLOAD_DROP_ALTITUDE) — yalnızca irtifa.
  4. 1m'de bırakma (PAYLOAD_DROP_ALIGN_TIMEOUT_S=0 → 1m'de ek hizalama YOK).
"""
import logging
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import (
    BLUE_PAYLOAD_OFFSET_M,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_DROP_ALIGN_TIMEOUT_S,
    PAYLOAD_DROP_ALTITUDE,
    PAYLOAD_RELEASE_ALTITUDE,
    RED_PAYLOAD_OFFSET_M,
    RED_PAYLOAD_VERIFY_ALTITUDE,
)


def _make_sm():
    """__init__ (ağır kurulum) olmadan yalnızca DROP_ALIGN için örnek kur."""
    from state_machine import MissionStateMachine

    sm = object.__new__(MissionStateMachine)
    sm.log = logging.getLogger("test_drop_align")
    sm._drop_align_deadline = None
    sm._last_align_cmd_t = 0.0
    sm._no_detection_count = 0
    sm._align_phase = "HORIZONTAL"
    sm.position_controller = MagicMock()
    sm.nav = MagicMock()
    return sm


def _frame() -> np.ndarray:
    return np.zeros((64, 64, 3), dtype=np.uint8)


def _detection(cx: float = 32.0, cy: float = 32.0):
    d = MagicMock()
    d.center_x = cx
    d.center_y = cy
    return d


def _real_sm():
    """Gerçek __init__ ile kurulan örnek (tüm öznitelikler hazır)."""
    from models import MissionState
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
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._alt_waiting = False
    return sm, nav


# ── 1) Config ──────────────────────────────────────────────────────────────── #
def test_drop_align_timeout_config_disabled():
    """Yeni akış: 1m'de ek hizalama KAPALI (dik iniş + doğrudan bırakma)."""
    assert PAYLOAD_DROP_ALIGN_TIMEOUT_S == 0.0


def test_release_altitude_above_drop_altitude():
    """Son hassas hizalama irtifası, yükün bırakıldığı irtifadan yüksektir
    (kadraj daraldığı için alçak irtifada görüntü hizalaması yapılamaz)."""
    assert PAYLOAD_RELEASE_ALTITUDE > PAYLOAD_DROP_ALTITUDE


# ── 2) OFFSET fazı ────────────────────────────────────────────────────────── #
def test_align_target_offset_sends_forward_offset():
    """OFFSET fazı: İHA, hedefe özel bırakma offseti kadar İLERİ gider.

    Hedef tipi bilinmiyor (test) → güvenli varsayılan RED_PAYLOAD_OFFSET_M.
    """
    sm, nav = _real_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None

    sm._do_align_target()

    nav.goto_body_offset.assert_called_once_with(
        forward_m=RED_PAYLOAD_OFFSET_M, right_m=0.0
    )


def test_align_target_offset_shows_display_notification():
    """OFFSET fazında görüntü işleme penceresinde bildirim gösterilir."""
    sm, nav = _real_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None

    sm._do_align_target()

    assert sm._pending_display is not None


def test_drop_offset_is_configurable():
    """Bırakma offsetleri config'ten okunur (sıfırdan büyük, ileri hareket).

    Kırmızı ve mavi offsetler BAĞIMSIZ ve farklıdır — hedefe özel seçim testi
    aşağıda (test_payload_drop_config).
    """
    assert RED_PAYLOAD_OFFSET_M > 0
    assert BLUE_PAYLOAD_OFFSET_M > 0


def test_align_target_offset_waits_then_drops():
    """Offset bekleyince DROPPING'e geçer ve dik iniş (1m) başlatılır."""
    sm, nav = _real_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0

    sm._do_align_target()

    assert sm._align_phase == "DROPPING"
    assert sm._alt_target == pytest.approx(PAYLOAD_DROP_ALTITUDE)


# ── 3) DROPPING tamamlanınca → doğrudan bırakma ───────────────────────────── #
def test_do_align_target_dropping_goes_straight_to_release():
    """
    1m sabit iniş tamamlanınca (PAYLOAD_DROP_ALIGN_TIMEOUT_S=0) DROP_ALIGN'a
    geçilmez — doğrudan TARGET_ALIGNED'a (bırakma) gidilir.
    """
    from models import MissionState

    sm, nav = _real_sm()
    sm._align_phase = "DROPPING"
    sm._alt_waiting = False          # iniş tamamlandı

    sm._do_align_target()

    assert sm.data.state == MissionState.TARGET_ALIGNED
    assert sm._align_phase == "HORIZONTAL"


# ── 4) Geri getirilen yük bırakma: OFFSET düzeltmesi (ilk bırakma ile AYNI) ── #
def _return_drop_sm(nav=None, altitude: float | None = None):
    from models import Target, TargetColor, TargetType
    from state_machine import MissionStateMachine

    nav = nav or MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude if altitude is not None else PAYLOAD_RELEASE_ALTITUDE,
        latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.second_target = Target(
        target_id=2, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    sm._target_rough_lat = None
    sm._target_rough_lon = None
    sm._last_align_cmd_t = 0.0
    # Hizalama taklidi: detektör tek hedef döndürür, align her zaman True
    sm.target_detector = MagicMock()
    det = MagicMock()
    det.color = TargetColor.RED
    det.shape = "TRIANGLE"
    det.center_x = 960.0
    det.center_y = 352.0
    det.confidence = 0.9
    sm.target_detector.detect.return_value = [det]
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    return sm, nav


def test_release_returned_payload_enters_offset_after_final_align():
    """Son hizalama bitince (kırmızı üçgen verify irtifası = 3m) DROPPING'e değil
    OFFSET'e geçilir."""
    sm, nav = _return_drop_sm(altitude=RED_PAYLOAD_VERIFY_ALTITUDE)
    sm._align_phase = "HORIZONTAL"

    sm._do_release_returned_payload()

    assert sm._align_phase == "OFFSET"
    assert sm._offset_correction_start is None


def test_release_returned_payload_offset_sends_forward_offset():
    """OFFSET fazı: İHA, hedefe özel offset kadar İLERİ gider.

    Fixture'daki 2. hedef kırmızı üçgen → RED_PAYLOAD_OFFSET_M kullanılır.
    """
    sm, nav = _return_drop_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None

    sm._do_release_returned_payload()

    nav.goto_body_offset.assert_called_once_with(
        forward_m=RED_PAYLOAD_OFFSET_M, right_m=0.0
    )
    assert sm._pending_display is not None   # görüntü işleme penceresinde bildirim


def test_release_returned_payload_offset_then_drops():
    """Offset bekleyince DROPPING'e geçer ve dik iniş (1m) başlatılır."""
    sm, nav = _return_drop_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0

    sm._do_release_returned_payload()

    assert sm._align_phase == "DROPPING"
    nav.set_relative_altitude.assert_called_once_with(PAYLOAD_DROP_ALTITUDE)


# ── 5) Payload offset correction yalnızca tasarlanan irtifada (≤0.4m) ──────── #
def _payload_align_sm(altitude_m):
    from models import Target, TargetColor, TargetType
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude_m, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.HEXAGON, color=TargetColor.BLUE,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    sm._align_phase = "HORIZONTAL"
    sm._align_phase_start_t = time.monotonic()
    sm._last_align_cmd_t = 0.0
    sm._payload_descent_alt = None

    det = MagicMock()
    det.center_x = 960.0
    det.center_y = 352.0
    det.width_px = 100.0
    det.height_px = 100.0
    det.area_px = 10000.0
    det.color_purity = 0.9
    det.color = TargetColor.RED
    det.confidence = 0.9

    sm.payload_detector = MagicMock()
    sm.payload_detector.detect.return_value = [det]
    sm._filter_payload_results = MagicMock(return_value=[det])
    sm.payload_tracker = MagicMock()
    sm.payload_tracker.update.return_value = det
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = True
    from config import ALIGNED_FRAMES_REQUIRED
    sm._aligned_frame_count = ALIGNED_FRAMES_REQUIRED - 1   # bir çağrıda tamamlanır
    return sm, nav


def test_payload_offset_not_triggered_at_0_6m():
    """0.6m'de offset correction YAPILMAZ — bir basamak daha inilir.
    (Kullanıcı raporu: 0.6m'de yüksekteyken offset yapılıyordu.)"""
    from models import MissionState

    sm, nav = _payload_align_sm(0.6)
    sm._do_align_payload()

    assert sm.data.state != MissionState.OFFSET_CORRECTION
    assert sm._align_phase == "DESCENDING"
    nav.set_relative_altitude.assert_called_once_with(pytest.approx(0.2))   # 0.6 → 0.2 basamağı


def test_payload_offset_triggered_at_align_altitude():
    """Tasarlanan hizalama irtifasında (0.4m) offset correction başlar."""
    from models import MissionState

    sm, _nav = _payload_align_sm(0.4)
    sm._do_align_payload()

    assert sm.data.state == MissionState.OFFSET_CORRECTION
