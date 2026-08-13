"""
Payload bırakımı — 1m'de opsiyonel ek hizalama (DROP_ALIGN) testleri.

Davranış:
  PAYLOAD_DROP_ALIGN_TIMEOUT_S > 0 → 1m (PAYLOAD_DROP_ALTITUDE) inişi sonrası
  görüntüye göre timeout'lu EK hizalama denenir; tespit edilemez veya süre
  dolarsa mevcut konumdan bırakılır (zorunlu değil — yalnızca iyileştirme).
  0 → eski davranış (sabit iniş sonrası doğrudan bırakma).
"""
import logging
import time
from unittest.mock import MagicMock

import numpy as np

from config import PAYLOAD_DROP_ALIGN_TIMEOUT_S


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


# ── 1) Config ─────────────────────────────────────────────────────────────── #
def test_drop_align_timeout_config_enabled():
    """Varsayılan: 5 sn ek hizalama süresi tanımlı."""
    assert PAYLOAD_DROP_ALIGN_TIMEOUT_S == 5.0


# ── 2) _enter_drop_align ──────────────────────────────────────────────────── #
def test_enter_drop_align_sets_phase_and_deadline():
    sm = _make_sm()
    sm._enter_drop_align()
    assert sm._align_phase == "DROP_ALIGN"
    assert sm._drop_align_deadline is not None


# ── 3) _drop_align_tick ───────────────────────────────────────────────────── #
def test_drop_align_timeout_returns_true():
    """Süre doldu → tespit olsa da olmasa da bırakılabilir (True)."""
    sm = _make_sm()
    sm._enter_drop_align()
    sm._drop_align_deadline = time.monotonic() - 1.0
    assert sm._drop_align_tick(_frame(), None) is True


def test_drop_align_no_detection_waits():
    """1m'de tespit yok → bekle (False) — süre dolana kadar bırakılmaz."""
    sm = _make_sm()
    sm._enter_drop_align()
    assert sm._drop_align_tick(_frame(), None) is False


def test_drop_align_already_aligned_returns_true():
    """Hedef zaten merkezde (align_detection=True) → hemen bırak (True)."""
    sm = _make_sm()
    sm._enter_drop_align()
    sm.position_controller.align_detection.return_value = True
    sm.nav.position.return_value = MagicMock(
        relative_altitude=1.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    assert sm._drop_align_tick(_frame(), _detection()) is True


def test_drop_align_not_aligned_sends_command_and_waits():
    """Merkezde değil → düzeltme komutu gönderilir, beklenir (False)."""
    sm = _make_sm()
    sm._enter_drop_align()
    sm.position_controller.align_detection.return_value = False
    sm.nav.position.return_value = MagicMock(
        relative_altitude=1.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    assert sm._drop_align_tick(_frame(), _detection(cx=50.0, cy=30.0)) is False
    sm.position_controller.align_detection.assert_called_once()


# ── 4) DROPPING → DROP_ALIGN geçişi (entegrasyon) ─────────────────────────── #
def test_do_align_target_dropping_enters_drop_align():
    """
    1m sabit iniş tamamlanınca DROP_ALIGN fazına geçilir (timeout>0),
    state TARGET_ALIGNED'a hemen GEÇMEZ.
    """
    from models import MissionState
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=1.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = _frame()

    sm = MissionStateMachine(
        navigation=nav,
        camera=camera,
        payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._align_phase = "DROPPING"
    sm._alt_waiting = False          # iniş tamamlandı
    sm._drop_align_deadline = None

    sm._do_align_target()

    assert sm._align_phase == "DROP_ALIGN"
    assert sm.data.state == MissionState.ALIGNING_TARGET  # henüz bırakmadı
    assert sm._drop_align_deadline is not None
