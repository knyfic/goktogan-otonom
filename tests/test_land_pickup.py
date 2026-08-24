"""
Payload alma öncesi inişin ArduPilot native LAND'e taşınması testleri.

YENİ davranış (yalnızca pickup öncesi iniş):
  OFFSET COMPLETE → MAV_CMD_NAV_LAND → ArduPilot kendi LAND controller'ı iner
  → landing doğrulanır (irtifa≈0 + dikey hız≈0, HOLD süresi) → PICKING_UP_PAYLOAD
  → 3 sn bekleme (mevcut) → ASCENDING_AFTER_PICKUP_LAND: armed? → GUIDED →
  GUIDED AKTİF mi? → TAKEOFF 1m → ASCENDING_FOR_VERIFICATION

GÜVENLİK kuralları (bu testlerin garantilediği):
  • İHA HİÇBİR ŞEKİLDE disarm olmaz (LAND_DISARMDELAY=-1 doğrulanmadan LAND yok).
  • Disarm olursa ASLA TAKEOFF/re-arm — görev durur.
  • GUIDED gerçekten aktif doğrulanmadan TAKEOFF gönderilmez.
  • LAND sırasında Python normal navigation komutu göndermez.
  • Pilot/RC müdahalesi → mevcut PILOT_CONTROL politikası (mission pilotu EZMEZ).
  • Telemetry/safety (heartbeat watchdog) LAND sırasında RTL enjekte etmez.
"""
import logging
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from config import (
    GUIDED_RESTORE_TIMEOUT_S,
    LAND_CONFIRM_HOLD_S,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_VERIFY_ALTITUDE,
)
from flight.navigation import Navigation
from models import MissionState, Target, TargetColor, TargetType


def _sm(nav, heartbeat_timeout_s=None):
    from state_machine import MissionStateMachine

    camera = MagicMock()
    camera.latest.return_value = np.zeros((352, 960, 3), dtype=np.uint8)

    payload = MagicMock()
    payload.servo_wait_s = None
    payload.finish_pickup.return_value = SimpleNamespace(success=True)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=payload,
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=heartbeat_timeout_s,
    )
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=1.0, longitude=1.0, confidence=0.9,
    )
    return sm


def _landing_nav(**kw):
    """LANDING_FOR_PICKUP için nav mock'u."""
    nav = MagicMock()
    nav.armed = kw.get("armed", True)
    nav.prepare_pickup_landing.return_value = kw.get("prepare_ok", True)
    nav.current_flight_mode = kw.get("mode", "LAND")
    nav.flight_mode_fresh.return_value = kw.get("fresh", True)
    nav.position.return_value = MagicMock(
        relative_altitude=kw.get("alt", 0.0),
        vertical_speed_mps=kw.get("vz", 0.0),
        latitude=1.0, longitude=1.0, heading_deg=0.0,
        battery_voltage_v=24.8, battery_percent=85,
    )
    return nav


# ── 1) Normal akış: OFFSET → LAND → landed → pickup → GUIDED → TAKEOFF ─────── #
def test_land_pickup_normal_sequence():
    nav = _landing_nav()
    sm = _sm(nav)
    sm.data.state = MissionState.OFFSET_CORRECTION
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "WAITING_OFFSET"

    # OFFSET bitti → LANDING_FOR_PICKUP (0m Python inişi YOK)
    sm._do_offset_correction()
    assert sm.data.state == MissionState.LANDING_FOR_PICKUP

    # LAND başlat: disarm garantisi + MAV_CMD_NAV_LAND
    sm._do_land_for_pickup()
    nav.prepare_pickup_landing.assert_called_once()
    nav.land.assert_called_once()

    # Landing doğrula: alt≈0, vz≈0, HOLD süresi stabil
    sm._do_land_for_pickup()                       # landed_since başladı
    assert sm.data.state == MissionState.LANDING_FOR_PICKUP
    sm._landed_since = time.monotonic() - LAND_CONFIRM_HOLD_S - 0.1
    sm._do_land_for_pickup()
    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD

    # 3 sn pickup bekleme → finish → LAND sonrası yükselme aşaması
    sm._do_pickup()                                # komut gönderildi
    sm._pickup_servo_start = time.monotonic() - 4.0
    sm._do_pickup()
    assert sm.data.state == MissionState.ASCENDING_AFTER_PICKUP_LAND

    # GUIDED restore (LAND'den geliniyor → set_mode) → GUIDED doğrulanınca TAKEOFF
    sm._do_ascending_after_pickup_land()
    nav.set_mode.assert_called_once_with("GUIDED")
    assert sm.data.state == MissionState.ASCENDING_AFTER_PICKUP_LAND
    nav.current_flight_mode = "GUIDED"             # FC mod değiştirdi
    sm._do_ascending_after_pickup_land()
    nav.takeoff_only.assert_called_once_with(PAYLOAD_VERIFY_ALTITUDE)
    assert sm.data.state == MissionState.ASCENDING_FOR_VERIFICATION
    assert sm.data.mission_failed is False


# ── 2) LAND sırasında Python navigation komutu GÖNDERMEZ ───────────────────── #
def test_landing_for_pickup_sends_no_navigation_commands():
    nav = _landing_nav(alt=1.5, vz=-0.5)   # hâlâ iniyor
    sm = _sm(nav)
    sm.data.state = MissionState.LANDING_FOR_PICKUP

    sm._do_land_for_pickup()               # LAND komutu
    for _ in range(5):
        sm._do_land_for_pickup()

    nav.goto_global.assert_not_called()
    nav.set_relative_altitude.assert_not_called()
    nav.goto_body_offset.assert_not_called()
    nav.set_mode.assert_not_called()       # LAND sırasında mod değişikliği YOK


# ── 3) GUIDED gerçekten aktif olmadan TAKEOFF yok ──────────────────────────── #
def test_no_takeoff_until_guided_verified():
    nav = _landing_nav(mode="LAND")
    sm = _sm(nav)
    sm.data.state = MissionState.ASCENDING_AFTER_PICKUP_LAND

    sm._do_ascending_after_pickup_land()   # set_mode(GUIDED) gönderildi
    nav.set_mode.assert_called_once_with("GUIDED")
    # FC henüz modu değiştirmedi (hâlâ LAND) → TAKEOFF YOK
    sm._do_ascending_after_pickup_land()
    nav.takeoff_only.assert_not_called()

    # Timeout dolunca → güvenli RTL, TAKEOFF yine de YOK
    sm._land_phase_start = time.monotonic() - GUIDED_RESTORE_TIMEOUT_S - 1.0
    sm._do_ascending_after_pickup_land()
    assert sm.data.state == MissionState.RTL
    nav.takeoff_only.assert_not_called()
    assert sm.data.mission_failed is True


# ── 4) Disarm olursa ASLA TAKEOFF/re-arm ───────────────────────────────────── #
def test_land_disarm_aborts_no_takeoff():
    nav = _landing_nav()
    sm = _sm(nav)
    sm.data.state = MissionState.LANDING_FOR_PICKUP

    sm._do_land_for_pickup()               # LAND komutu gönderildi
    nav.armed = False                      # iniş sırasında disarm oldu
    sm._do_land_for_pickup()

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True
    nav.takeoff_only.assert_not_called()
    nav.set_mode.assert_not_called()


def test_restore_phase_disarm_aborts_no_takeoff():
    nav = _landing_nav(mode="LAND", armed=False)
    sm = _sm(nav)
    sm.data.state = MissionState.ASCENDING_AFTER_PICKUP_LAND

    sm._do_ascending_after_pickup_land()

    assert sm.data.state == MissionState.RTL
    nav.takeoff_only.assert_not_called()
    nav.set_mode.assert_not_called()


def test_disarmdelay_not_ensured_aborts_landing():
    """LAND_DISARMDELAY=-1 garanti edilemezse LAND HİÇ BAŞLATILMAZ (disarm riski)."""
    nav = _landing_nav(prepare_ok=False)
    sm = _sm(nav)
    sm.data.state = MissionState.LANDING_FOR_PICKUP

    sm._do_land_for_pickup()

    nav.land.assert_not_called()
    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True


# ── 5) Pilot takeover: mission pilot kararını EZMEZ ────────────────────────── #
def test_pilot_takeover_during_landing_pauses():
    nav = _landing_nav(mode="STABILIZE")
    sm = _sm(nav)
    sm.data.state = MissionState.LANDING_FOR_PICKUP

    sm._do_land_for_pickup()               # LAND komutu
    sm._do_land_for_pickup()               # pilot STABILIZE → durdur

    assert sm.data.state == MissionState.PILOT_CONTROL
    nav.land.assert_called_once()


def test_pilot_takeover_during_restore_pauses_before_set_mode():
    """Pickup sonrası pilot başka moda geçtiyse mission GUIDED set ETMEZ (ezmez)."""
    nav = _landing_nav(mode="LOITER")
    sm = _sm(nav)
    sm.data.state = MissionState.ASCENDING_AFTER_PICKUP_LAND

    sm._do_ascending_after_pickup_land()

    assert sm.data.state == MissionState.PILOT_CONTROL
    nav.set_mode.assert_not_called()       # pilot modu ezilmez
    nav.takeoff_only.assert_not_called()


# ── 6) Telemetry/safety LAND sırasında devam eder (RTL enjekte YOK) ────────── #
def test_heartbeat_watchdog_does_not_rtl_during_landing():
    nav = _landing_nav()
    nav.heartbeat_ok.return_value = False
    sm = _sm(nav, heartbeat_timeout_s=5.0)
    sm.data.state = MissionState.LANDING_FOR_PICKUP

    sm._tick()

    assert sm.data.state == MissionState.LANDING_FOR_PICKUP
    assert sm.data.mission_failed is False
    nav.rtl.assert_not_called()


def test_mission_timer_does_not_rtl_during_pickup_land_flow():
    nav = _landing_nav()
    sm = _sm(nav)
    sm.data.state = MissionState.ASCENDING_AFTER_PICKUP_LAND
    sm._mission_start_t = time.monotonic() - 99999.0   # süre çoktan doldu

    sm._tick()

    assert sm.data.state == MissionState.ASCENDING_AFTER_PICKUP_LAND
    nav.rtl.assert_not_called()


# ── 7) Navigation seviyesi: LAND / takeoff_only / disarmdelay ──────────────── #
def _raw_nav():
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav.system_id = 1
    nav.component_id = 1
    nav._land_disarmdelay_original = None
    nav._armed = False
    nav._mode_name = None
    nav._mode_custom = None
    nav._mode_last_t = 0.0
    nav._hb_seen = 0
    nav.master.mode_mapping.return_value = {"GUIDED": 4, "LAND": 9}
    return nav


def test_land_sends_mav_cmd_nav_land():
    from pymavlink import mavutil

    nav = _raw_nav()
    nav.land()
    args = nav.master.mav.command_long_send.call_args[0]
    assert args[2] == mavutil.mavlink.MAV_CMD_NAV_LAND


def test_takeoff_only_never_arms_or_changes_mode():
    from pymavlink import mavutil

    nav = _raw_nav()
    nav.takeoff_only(1.0)
    args = nav.master.mav.command_long_send.call_args[0]
    assert args[2] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    assert args[10] == 1.0   # param7 = irtifa
    nav.master.arducopter_arm.assert_not_called()
    # command_long_send yalnızca TAKEOFF için çağrıldı (mode set yok)
    assert nav.master.mav.command_long_send.call_count == 1


def test_prepare_pickup_landing_sets_neg1_and_verifies():
    nav = _raw_nav()
    nav._read_param = MagicMock(side_effect=[0.0, -1.0])  # okundu → set → doğrula

    assert nav.prepare_pickup_landing() is True

    nav.master.mav.param_set_send.assert_called_once()
    args = nav.master.mav.param_set_send.call_args[0]
    assert args[2] == b"LAND_DISARMDELAY"
    assert args[3] == -1.0
    assert nav._land_disarmdelay_original == 0.0   # restore için saklandı


def test_prepare_pickup_landing_skips_write_if_already_neg1():
    nav = _raw_nav()
    nav._read_param = MagicMock(return_value=-1.0)

    assert nav.prepare_pickup_landing() is True
    nav.master.mav.param_set_send.assert_not_called()
    assert nav._land_disarmdelay_original == -1.0


def test_prepare_pickup_landing_fails_when_param_unreadable():
    nav = _raw_nav()
    nav._read_param = MagicMock(return_value=None)

    assert nav.prepare_pickup_landing() is False
    nav.master.mav.param_set_send.assert_not_called()


def test_restore_land_disarmdelay_restores_original():
    nav = _raw_nav()
    nav._land_disarmdelay_original = 0.0

    assert nav.restore_land_disarmdelay() is True
    args = nav.master.mav.param_set_send.call_args[0]
    assert args[2] == b"LAND_DISARMDELAY"
    assert args[3] == 0.0
    assert nav._land_disarmdelay_original is None


def test_armed_tracked_from_heartbeat():
    from pymavlink import mavutil

    nav = _raw_nav()
    # base_mode: CUSTOM_MODE_ENABLED(1) + SAFETY_ARMED(128) = 129, custom=4 (GUIDED)
    nav._update_mode(_hb(custom_mode=4, base_mode=129))
    assert nav.armed is True
    assert nav.current_flight_mode == "GUIDED"

    nav._update_mode(_hb(custom_mode=4, base_mode=1))   # disarm edildi
    assert nav.armed is False


def _hb(custom_mode, base_mode=1):
    msg = MagicMock()
    msg.custom_mode = custom_mode
    msg.type = 2
    msg.autopilot = 3
    msg.base_mode = base_mode
    msg.get_srcSystem.return_value = 1
    msg.get_srcComponent.return_value = 1
    return msg
