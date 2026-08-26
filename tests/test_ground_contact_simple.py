"""
Basit yer-teması algılama testleri (Branch: looks-final-2).

Yaklaşım: aşağı iniş komutu aktifken telemetri izlenir —
  irtifa donmuş + dikey hız aşağı yönlü değil + yatay hareket durmuş
  → GROUND_CONTACT_HOLD_S kesintisiz → PICKING_UP_PAYLOAD (yerde ARMED pickup).

Kapsanan senaryolar (kullanıcı isteği #12):
  1. Aşağı komut + irtifa düşüyor        → NOT LANDED
  2. Aşağı komut + alt≈sabit + vz≈0 + vh≈0 (yeterince uzun) → GROUND CONTACT
  3. Aynı durum kısa süreli              → NOT LANDED
  4. Temas sonrası araç ARMED kalır      → disarm YOK
  5. Pickup sonrası TAKEOFF akışı        → ASCENDING_FOR_VERIFICATION

GÜVENLİK: LAND mode / set_mode("LAND") kullanılmaz; DISARM yok; yeni sensör yok.
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import (
    GROUND_CONTACT_HOLD_S,
    GROUND_CONTACT_TIMEOUT_S,
    OFFSET_CORRECTION_WAIT_S,
)
from flight.navigation import Navigation
from models import MissionState, Target, TargetColor, TargetType, VehiclePosition


def _sm(alt: float = 0.0, vz: float = 0.0, vh: float = 0.0, armed: bool = True):
    nav = MagicMock()
    nav.armed = armed
    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=alt, heading_deg=0.0,
        vertical_speed_mps=vz, horizontal_speed_mps=vh,
    )
    nav.current_flight_mode = "GUIDED"
    nav.flight_mode_fresh.return_value = True

    camera = MagicMock()
    camera.latest.return_value = np.zeros((352, 960, 3), dtype=np.uint8)
    payload = MagicMock()
    payload.servo_wait_s = None
    payload.finish_pickup.return_value = SimpleNamespace(success=True)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=payload,
        logger=logging.getLogger("t"),
    )
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    return sm, nav


def _start(sm, nav):
    """GROUND_CONTACT_CHECK'i başlatır ve bir koşul tick'i işler."""
    sm.data.state = MissionState.GROUND_CONTACT_CHECK
    sm._do_ground_contact_check()   # başlat (aşağı iniş komutu + sayaç temiz)
    sm._do_ground_contact_check()   # koşul değerlendir


from state_machine import MissionStateMachine  # noqa: E402


# ── 1) Aşağı komut aktif + irtifa DÜŞÜYOR → NOT LANDED ────────────────────── #
def test_descending_altitude_not_landed():
    """Araç hâlâ iniyor (irtifa düşüyor / dikey hız aşağı) → temas DEĞİL."""
    sm, nav = _sm(alt=0.25, vz=-0.30, vh=0.05)
    _start(sm, nav)

    assert sm._ground_contact_ok(nav.position()) is False
    assert sm._contact_since is None
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


# ── 2) Alt≈sabit + vz≈0 + vh≈0, yeterince uzun → GROUND CONTACT ───────────── #
def test_sustained_frozen_telemetry_triggers_contact():
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)

    assert sm._ground_contact_ok(nav.position()) is True
    assert sm._contact_since is not None          # adaylık başladı
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK

    sm._contact_since = time.monotonic() - GROUND_CONTACT_HOLD_S - 0.1
    sm._do_ground_contact_check()

    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD


# ── 3) Kısa süreli durum → sayaç sıfırlanır → NOT LANDED ──────────────────── #
def test_brief_condition_resets_counter():
    """Koşullar bir an sağlanıp bozulursa sayaç sıfırlanır — temas üretilmez."""
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)
    assert sm._contact_since is not None

    # Koşul bozuldu: araç yeniden inmeye başladı (dikey hız aşağı)
    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=0.03, heading_deg=0.0,
        vertical_speed_mps=-0.30, horizontal_speed_mps=0.0,
    )
    sm._do_ground_contact_check()

    assert sm._contact_since is None
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


def test_altitude_drift_resets_counter():
    """Adaylık boyunca irtifa adaylık referansına göre oynarsa (yavaş sızma)
    koşul bozulur — \"irtifa artık anlamlı şekilde değişmiyor\" şartı."""
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)
    assert sm._contact_since is not None

    # İrtifa adaylık referansına göre 10cm kaydı (yavaşça aşağı süzülme)
    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=-0.08, heading_deg=0.0,
        vertical_speed_mps=-0.05, horizontal_speed_mps=0.0,
    )
    sm._do_ground_contact_check()

    assert sm._contact_since is None


def test_horizontal_movement_resets_counter():
    """Yatay hareket devam ediyorsa (yerde değilken kayma) koşul bozulur."""
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)
    assert sm._contact_since is not None

    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=0.02, heading_deg=0.0,
        vertical_speed_mps=0.0, horizontal_speed_mps=0.4,
    )
    sm._do_ground_contact_check()

    assert sm._contact_since is None


# ── 4) Temas sonrası araç ARMED kalır — disarm YOK ────────────────────────── #
def test_contact_keeps_armed_no_disarm():
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)
    sm._contact_since = time.monotonic() - GROUND_CONTACT_HOLD_S - 0.1
    sm._do_ground_contact_check()

    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD
    assert nav.armed is True
    nav.set_mode.assert_not_called()   # LAND mode YOK
    nav.arm.assert_not_called()


# ── 5) Pickup sonrası TAKEOFF akışı bozulmaz ──────────────────────────────── #
def test_contact_then_pickup_then_takeoff():
    sm, nav = _sm(alt=0.02, vz=0.0, vh=0.0)
    _start(sm, nav)
    sm._contact_since = time.monotonic() - GROUND_CONTACT_HOLD_S - 0.1
    sm._do_ground_contact_check()
    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD

    # Pickup: komut → bekleme → finish → yükselme (mevcut akış korundu)
    sm._do_pickup()
    sm._pickup_servo_start = time.monotonic() - 4.0
    sm._do_pickup()
    assert sm.data.state == MissionState.ASCENDING_FOR_VERIFICATION


# ── Geçişler: OFFSET / ALIGN → GROUND_CONTACT_CHECK ───────────────────────── #
def test_offset_completes_into_ground_contact_check():
    sm, nav = _sm(alt=0.4, vz=0.0, vh=0.0)
    sm.data.state = MissionState.OFFSET_CORRECTION
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "WAITING_OFFSET"

    sm._do_offset_correction()

    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK
    nav.set_relative_altitude.assert_not_called()   # "0m'ye in" komutu burada YOK


def test_align_descending_reaches_contact_goes_to_contact_check():
    sm, nav = _sm(alt=0.0, vz=0.0, vh=0.0)
    sm.data.state = MissionState.ALIGNING_PAYLOAD
    sm._align_phase = "DESCENDING"
    sm._alt_waiting = False
    sm.payload_detector = MagicMock()
    sm.payload_detector.detect.return_value = []
    sm._filter_payload_results = MagicMock(return_value=[])
    sm._set_display_payload = MagicMock()

    sm._do_align_payload()

    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


# ── Zaman aşımı → güvenli RTL ─────────────────────────────────────────────── #
def test_contact_timeout_goes_rtl():
    """Temas hiç doğrulanamazsa (ör. araç yüksekte takıldı) güvenli RTL."""
    sm, nav = _sm(alt=0.5, vz=-0.3, vh=0.1)
    sm.data.state = MissionState.GROUND_CONTACT_CHECK
    sm._do_ground_contact_check()                     # başlat
    sm._contact_start = time.monotonic() - GROUND_CONTACT_TIMEOUT_S - 1.0
    sm._do_ground_contact_check()

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True


# ── Navigation seviyesi: dikey/yatay hız takibi ───────────────────────────── #
def test_vertical_and_horizontal_speed_tracked_from_global_position():
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav._position = VehiclePosition()
    nav.home_lat = None
    nav.home_lon = None
    nav._last_message_s = 0.0
    msg = MagicMock()
    msg.lat = 10000000
    msg.lon = 20000000
    msg.relative_alt = 500
    msg.hdg = 9000
    msg.vz = -30          # NED: -30 cm/s → yükseliyor (pozitif kuralı: +0.3 m/s)
    msg.vx = 20           # NED: 20 cm/s kuzey
    msg.vy = 0
    nav._update_position(msg)
    assert nav._position.vertical_speed_mps == pytest.approx(0.3)
    assert nav._position.horizontal_speed_mps == pytest.approx(0.2)


def test_horizontal_speed_magnitude_from_vx_vy():
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav._position = VehiclePosition()
    nav.home_lat = None
    nav.home_lon = None
    nav._last_message_s = 0.0
    msg = MagicMock()
    msg.lat = 0
    msg.lon = 0
    msg.relative_alt = 0
    msg.hdg = 0
    msg.vz = 0
    msg.vx = 30
    msg.vy = 40
    nav._update_position(msg)
    assert nav._position.horizontal_speed_mps == pytest.approx(0.5)   # 3-4-5 üçgeni
