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
    DROP_OFFSET_M,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_DROP_ALIGN_TIMEOUT_S,
    PAYLOAD_DROP_ALTITUDE,
    PAYLOAD_RELEASE_ALTITUDE,
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
    """OFFSET fazı: İHA, bırakma offseti (DROP_OFFSET_M) kadar İLERİ gider."""
    sm, nav = _real_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None

    sm._do_align_target()

    nav.goto_body_offset.assert_called_once_with(
        forward_m=DROP_OFFSET_M, right_m=0.0
    )


def test_align_target_offset_shows_display_notification():
    """OFFSET fazında görüntü işleme penceresinde bildirim gösterilir."""
    sm, nav = _real_sm()
    sm._align_phase = "OFFSET"
    sm._offset_correction_start = None

    sm._do_align_target()

    assert sm._pending_display is not None


def test_drop_offset_is_configurable():
    """Bırakma offseti config'ten okunur (sıfırdan büyük, ileri hareket)."""
    assert DROP_OFFSET_M > 0


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
