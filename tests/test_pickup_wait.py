"""
Payload pickup zeminde bekleme (ground wait) regression testleri.

Gereksinim (hata.md): "Yükü almak için yere inildiğinde zeminde ~3 saniye
beklenmeli (PAYLOAD_SERVO_WAIT_S = 3)" — REAL modda da geçerlidir.

Bekleme süresi nereden gelir (her 3 modda):
  REAL  → payload/config.yaml → pickup_wait_s (3.0) → LocalRPiBackend.stepper_duration
          → PayloadController.servo_wait_s (3.0) → _do_pickup 3 sn bekler
  SIM   → SimulationBackend'de stepper_duration YOK → servo_wait_s None
          → PAYLOAD_SERVO_WAIT_S (3) fallback → _do_pickup 3 sn bekler
  HIL   → RemoteRPiBackend'de stepper_duration YOK → aynı fallback

Servo/stepper kodu SİLİNMEZ (ileride kullanılabilir) — mekanizma sabit durur,
mıknatıslar yükü tutar, İHA yerde 3 sn bekler. LAND akışında pickup beklemesi
(PICKING_UP_PAYLOAD) LAND doğrulandıktan SONRA çalışır ve tamamlanınca
ASCENDING_AFTER_PICKUP_LAND'e geçilir (LAND sonrası GUIDED + 1m TAKEOFF).
"""
import logging
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import PAYLOAD_SERVO_WAIT_S
from models import MissionState, Target, TargetColor, TargetType
from payload.backends.simulation import SimulationBackend
from payload.payload_controller import PayloadController

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PAYLOAD_CONFIG = _REPO_ROOT / "payload" / "config.yaml"


def _real_backend():
    """REAL backend — LocalRPiBackend.from_config (main.py'nin birebir yolu)."""
    from payload.backends.local_rpi import LocalRPiBackend

    return LocalRPiBackend.from_config(
        str(_PAYLOAD_CONFIG), logger=logging.getLogger("t")
    )


def _sm(payload):
    """PICKING_UP_PAYLOAD state'inde gerçek SM; kamera/nav mock'lu."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=0.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
        battery_voltage_v=24.8, battery_percent=85,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=payload,
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.PICKING_UP_PAYLOAD
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    return sm


# ── 1) REAL backend: pickup_wait_s → servo_wait_s ──────────────────────────── #
def test_real_backend_provides_pickup_wait_from_config():
    """REAL: LocalRPiBackend.from_config → stepper_duration = pickup_wait_s (3.0s)
    → PayloadController.servo_wait_s 3.0 döner (bekleme REAL'de de çalışır)."""
    backend = _real_backend()
    assert backend.stepper_duration == pytest.approx(3.0), \
        "pickup_wait_s (config.yaml) backend'e stepper_duration olarak yazılmalı"

    ctrl = PayloadController(backend=backend, logger=logging.getLogger("t"))
    assert ctrl.servo_wait_s == pytest.approx(3.0), \
        "real backend'de servo_wait_s 3.0 olmalı (yere indikten sonra bekleme)"


def test_sim_backend_has_no_duration_uses_fallback():
    """SIM: SimulationBackend'de stepper_duration yok → servo_wait_s None
    → state machine PAYLOAD_SERVO_WAIT_S (3) fallback kullanır."""
    ctrl = PayloadController(backend=SimulationBackend(), logger=logging.getLogger("t"))
    assert ctrl.servo_wait_s is None
    assert PAYLOAD_SERVO_WAIT_S == 3, "fallback 3 sn olmalı (hata.md gereksinimi)"


# ── 2) State machine: _do_pickup bekleme süresini UYGULAR ──────────────────── #
def _mock_payload(wait):
    payload = MagicMock()
    payload.servo_wait_s = wait
    payload.finish_pickup.return_value = SimpleNamespace(success=True)
    return payload


def test_do_pickup_waits_full_duration_before_finish():
    """_do_pickup: pickup komutu gönderilir, zeminde TAM bekleme süresi (3 sn)
    dolmadan finish_pickup ÇAĞRILMAZ; süre dolunca çağrılır ve LAND sonrası
    GUIDED/TAKEOFF akışına (ASCENDING_AFTER_PICKUP_LAND) geçilir."""
    payload = _mock_payload(wait=3.0)
    sm = _sm(payload)
    sm.data.pending_detection = None

    # 1) İlk çağrı: pickup komutu + bekleme süresi kaydedilir (finish YOK)
    sm._do_pickup()
    assert sm._pickup_servo_wait == 3.0
    payload.send_pickup_command.assert_called_once()
    payload.finish_pickup.assert_not_called()

    # 2) Süre dolmadan (1 sn geçti): hâlâ bekleniyor — finish YOK
    sm._pickup_servo_start = time.monotonic() - 1.0
    sm._do_pickup()
    payload.finish_pickup.assert_not_called()

    # 3) Süre doldu (3.5 sn): finish_pickup çağrılır → LAND sonrası yükselme
    sm._pickup_servo_start = time.monotonic() - 3.5
    sm._do_pickup()
    payload.finish_pickup.assert_called_once()
    assert sm.data.state == MissionState.ASCENDING_AFTER_PICKUP_LAND, \
        "pickup beklemesi sonrası GUIDED/TAKEOFF aşamasına geçilmeli"


def test_do_pickup_wait_uses_backend_duration_when_provided():
    """Backend süre verirse (REAL gibi) o süre kullanılır — PAYLOAD_SERVO_WAIT_S
    değil. (Örn. pickup_wait_s=2.0 → 2 sn beklenir.)"""
    payload = _mock_payload(wait=2.0)
    sm = _sm(payload)

    sm._do_pickup()

    assert sm._pickup_servo_wait == 2.0


def test_do_pickup_wait_falls_back_to_config_when_none():
    """Backend süre vermezse (SIM/HIL gibi) PAYLOAD_SERVO_WAIT_S (3) kullanılır."""
    payload = _mock_payload(wait=None)   # servo_wait_s None → fallback
    sm = _sm(payload)

    sm._do_pickup()

    assert sm._pickup_servo_wait == PAYLOAD_SERVO_WAIT_S
    assert sm._pickup_servo_wait == 3.0
