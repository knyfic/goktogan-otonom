"""
GUIDED yer-teması doğrulaması testleri (ArduPilot LAND detektörü uyarlaması).

Kapsanan davranış:
  OFFSET_CORRECTION / ALIGNING_PAYLOAD(descend) → GROUND_CONTACT_CHECK
    → (koşullar GROUND_CONTACT_HOLD_S kesintisiz) → LANDED_HOLD
    → (LANDED_HOLD_WAIT_S) → PICKING_UP_PAYLOAD → ASCENDING_FOR_VERIFICATION

GÜVENLİK garantileri:
  • Yalnızca \"irtifa ≈ 0\" landed DEĞİLDİR — throttle/eğim/dwell birlikte gerekir.
  • Kısa süreli koşul sağlanması sayacı sıfırlar (false positive engellenir).
  • LANDED_HOLD sırasında DISARM olursa görev durur — TAKEOFF/re-arm YOK.
  • LAND mode / set_mode(\"LAND\") HİÇ KULLANILMAZ; iniş dikey hız komutudur.
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
    LANDED_HOLD_WAIT_S,
    OFFSET_CORRECTION_WAIT_S,
)
from flight.navigation import Navigation
from models import MissionState, Target, TargetColor, TargetType, VehiclePosition


def _sm(alt: float = 0.0, vz: float = 0.0, throttle=None, roll=None, pitch=None,
        armed: bool = True):
    nav = MagicMock()
    nav.armed = armed
    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=alt, heading_deg=0.0,
        vertical_speed_mps=vz,
    )
    nav.throttle_pct = throttle          # None → koşul atlanır
    nav.attitude = (roll, pitch)         # (None, None) → koşul atlanır
    nav.descent_velocity = MagicMock()
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


def _start_contact_check(sm):
    """GROUND_CONTACT_CHECK state'ini başlatır ve bir koşul tick'i işler."""
    sm.data.state = MissionState.GROUND_CONTACT_CHECK
    sm._do_ground_contact_check()   # başlat (iniş komutu + sayacı temizle)
    sm._do_ground_contact_check()   # koşul değerlendir


from state_machine import MissionStateMachine  # noqa: E402


# ── 1) Altitude ≈ 0 ama araç HAVADA (throttle yüksek) → NOT LANDED ────────── #
def test_altitude_zero_but_hover_throttle_not_contact():
    """İrtifa 0 + dikey hız 0, ama throttle hover seviyesinde → TEMAS DEĞİL.

    Kullanıcı senaryosu: estimator 0 gösteriyor, araç fiziksel olarak havada.
    Havada motor yükü kaldırdığı için throttle yüksektir — temas yok.
    """
    sm, nav = _sm(alt=0.0, vz=0.0, throttle=45.0, roll=1.0, pitch=1.0)
    _start_contact_check(sm)

    assert sm._ground_contact_ok(nav.position()) is False
    assert sm._contact_since is None
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


# ── 2) Gerçek temas benzeri durum → LANDED_HOLD ──────────────────────────── #
def test_sustained_ground_contact_triggers_landed_hold():
    """Alçak irtifa + dikey hız ≈ 0 + throttle düşük + stabil eğim,
    GROUND_CONTACT_HOLD_S boyunca kesintisiz → LANDED_HOLD."""
    sm, nav = _sm(alt=0.02, vz=0.0, throttle=12.0, roll=1.0, pitch=0.5)
    _start_contact_check(sm)

    assert sm._contact_since is not None          # koşul sağlandı — sayaç başladı
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK

    sm._contact_since = time.monotonic() - GROUND_CONTACT_HOLD_S - 0.1
    sm._do_ground_contact_check()

    assert sm.data.state == MissionState.LANDED_HOLD
    nav.descent_velocity.assert_called()          # iniş komutu gönderildi/durduruldu


def test_contact_ignores_missing_throttle_and_attitude():
    """VFR_HUD/ATTITUDE yoksa ilgili koşullar atlanır (alt+vz+dwell yeterli)."""
    sm, nav = _sm(alt=0.05, vz=0.0)               # throttle=None, attitude=(None,None)
    _start_contact_check(sm)

    assert sm._ground_contact_ok(nav.position()) is True
    assert sm._contact_since is not None


# ── 3) Kısa süreli false condition → sayaç sıfırlanır ────────────────────── #
def test_contact_counter_resets_on_condition_break():
    """Koşullar bir an sağlanıp bozulursa (iniş sırasında dikey hız sıçraması)
    sayaç sıfırlanır — LANDED üretilmez."""
    sm, nav = _sm(alt=0.02, vz=0.0, throttle=12.0, roll=1.0, pitch=0.5)
    _start_contact_check(sm)
    assert sm._contact_since is not None

    # Koşul bozuldu: araç yeniden inmeye başladı (vz = -0.4 m/s)
    nav.position.return_value = VehiclePosition(
        latitude=1.0, longitude=1.0, relative_altitude=0.05, heading_deg=0.0,
        vertical_speed_mps=-0.4,
    )
    sm._do_ground_contact_check()

    assert sm._contact_since is None
    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


# ── 4) LANDED_HOLD: yerde ARMED bekleme — disarm YOK ─────────────────────── #
def test_landed_hold_keeps_armed_no_disarm():
    sm, nav = _sm(alt=0.0, vz=0.0, throttle=10.0)
    sm.data.state = MissionState.LANDED_HOLD

    sm._do_landed_hold()                          # başlat
    assert sm.data.state == MissionState.LANDED_HOLD

    sm._landed_hold_t = time.monotonic() - LANDED_HOLD_WAIT_S - 0.1
    sm._do_landed_hold()

    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD
    nav.arm.assert_not_called()                   # re-arm YOK
    nav.set_mode.assert_not_called()              # LAND mode YOK


def test_landed_hold_disarm_stops_mission_no_takeoff():
    """Yerdeyken DISARM olursa görev güvenli durur — TAKEOFF/re-arm YOK."""
    sm, nav = _sm(alt=0.0, vz=0.0, armed=False)
    sm.data.state = MissionState.LANDED_HOLD

    sm._do_landed_hold()                          # başlat
    sm._do_landed_hold()                          # disarm kontrolü → durdur

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True
    nav.takeoff_only.assert_not_called()
    nav.set_mode.assert_not_called()


# ── 5) LANDED_HOLD → PICKUP → TAKEOFF akışı ──────────────────────────────── #
def test_landed_hold_to_takeoff_flow():
    sm, nav = _sm(alt=0.0, vz=0.0)
    sm.data.state = MissionState.LANDED_HOLD
    sm._do_landed_hold()
    sm._landed_hold_t = time.monotonic() - LANDED_HOLD_WAIT_S - 0.1
    sm._do_landed_hold()
    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD

    # Pickup: komut → bekleme → finish → yükselme (mevcut akış bozulmadı)
    sm._do_pickup()
    sm._pickup_servo_start = time.monotonic() - 4.0
    sm._do_pickup()
    assert sm.data.state == MissionState.ASCENDING_FOR_VERIFICATION


# ── 6) OFFSET tamamlanınca GROUND_CONTACT_CHECK'e geçilir ────────────────── #
def test_offset_completes_into_ground_contact_check():
    sm, nav = _sm(alt=0.4, vz=0.0)
    sm.data.state = MissionState.OFFSET_CORRECTION
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "WAITING_OFFSET"

    sm._do_offset_correction()

    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK
    nav.set_relative_altitude.assert_not_called()   # 0m altitude komutu YOK


def test_align_descending_reaches_contact_goes_to_contact_check():
    sm, nav = _sm(alt=0.0, vz=0.0)
    sm.data.state = MissionState.ALIGNING_PAYLOAD
    sm._align_phase = "DESCENDING"
    sm._alt_waiting = False
    sm.payload_detector = MagicMock()
    sm.payload_detector.detect.return_value = []
    sm._filter_payload_results = MagicMock(return_value=[])
    sm._set_display_payload = MagicMock()

    sm._do_align_payload()

    assert sm.data.state == MissionState.GROUND_CONTACT_CHECK


# ── 7) Timeout → güvenli RTL ──────────────────────────────────────────────── #
def test_contact_timeout_goes_rtl():
    """Temas hiç doğrulanamazsa (ör. araç havada takıldı) güvenli RTL."""
    sm, nav = _sm(alt=0.5, vz=-0.3, throttle=45.0)   # iniyor + hover throttle
    sm.data.state = MissionState.GROUND_CONTACT_CHECK
    sm._do_ground_contact_check()                     # başlat
    sm._contact_start = time.monotonic() - GROUND_CONTACT_TIMEOUT_S - 1.0
    sm._do_ground_contact_check()

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True


# ── 8) Navigation seviyesi: descent_velocity + sinyal takibi ─────────────── #
def test_descent_velocity_sends_velocity_only_local_ned():
    from pymavlink import mavutil

    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav.system_id = 1
    nav.component_id = 1

    nav.descent_velocity(0.3)

    args = nav.master.mav.set_position_target_local_ned_send.call_args[0]
    assert args[3] == mavutil.mavlink.MAV_FRAME_LOCAL_NED
    # type_mask: x/y/z + ax/ay/az + yaw + yaw_rate yoksay, vx/vy/vz AKTİF
    assert args[4] == 0b110111000111
    assert (args[8], args[9], args[10]) == (0.0, 0.0, 0.3)   # vx, vy, vz (NED: aşağı +z)

    nav.descent_velocity(0.0)                              # durdurma komutu
    assert nav.master.mav.set_position_target_local_ned_send.call_args[0][10] == 0.0


def test_vertical_speed_tracked_from_global_position():
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
    nav._update_position(msg)
    assert nav._position.vertical_speed_mps == pytest.approx(0.3)


def test_throttle_and_attitude_freshness_and_dispatch():
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav._position = VehiclePosition()
    nav._throttle_pct = None
    nav._throttle_last_t = 0.0
    nav._roll_deg = None
    nav._pitch_deg = None
    nav._attitude_last_t = 0.0

    assert nav.throttle_pct is None          # hiç gelmedi → None
    assert nav.attitude == (None, None)

    vfr = MagicMock()
    vfr.get_type.return_value = "VFR_HUD"
    vfr.throttle = 12
    att = MagicMock()
    att.get_type.return_value = "ATTITUDE"
    att.roll = 0.02
    att.pitch = -0.01
    nav._dispatch_telemetry(vfr)
    nav._dispatch_telemetry(att)

    assert nav.throttle_pct == 12.0
    roll, pitch = nav.attitude
    assert roll == pytest.approx(1.15, abs=0.01)
    assert pitch == pytest.approx(-0.57, abs=0.01)


def test_throttle_and_attitude_stale_returns_none():
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav._throttle_pct = 5.0
    nav._throttle_last_t = time.monotonic() - 5.0    # bayat
    nav._roll_deg = 1.0
    nav._pitch_deg = 1.0
    nav._attitude_last_t = time.monotonic() - 5.0    # bayat

    assert nav.throttle_pct is None
    assert nav.attitude == (None, None)


def test_armed_tracked_from_heartbeat_base_mode():
    """armed, HEARTBEAT base_mode SAFETY_ARMED bitinden izlenir (LANDED_HOLD
    güvenlik kontrolü için)."""
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav.master.mode_mapping.return_value = {"GUIDED": 4}
    nav._hb_seen = 0
    nav._mode_name = None
    nav._mode_last_t = 0.0
    hb = MagicMock()
    hb.custom_mode = 4
    hb.type = 2
    hb.autopilot = 3
    hb.base_mode = 129   # CUSTOM_MODE_ENABLED(1) + SAFETY_ARMED(128)
    hb.get_srcSystem.return_value = 1
    hb.get_srcComponent.return_value = 1
    nav._update_mode(hb)
    assert nav.armed is True

    hb.base_mode = 1     # disarm edildi
    nav._update_mode(hb)
    assert nav.armed is False
