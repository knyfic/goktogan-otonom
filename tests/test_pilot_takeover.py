"""
Pilot takeover güvenlik testleri — RC flight mode asla Python tarafından ezilmez.

Bu dosya, saha testinde bulunan kritik güvenlik sorununu korur:

  OLMAMASI GEREKEN (eski davranış):
      Pilot → LAND → ArduPilot LAND → Python GUIDED → ArduPilot GUIDED

  OLMASI GEREKEN (yeni davranış):
      Pilot → herhangi bir mod → ArduPilot o modda kalır
      Python → mod değişikliğini algılar → mission durur (PILOT_CONTROL)
      Python → hiçbir uçuş komutu / GUIDED göndermez

Kapsam:
  1. Navigation: HEARTBEAT → current_flight_mode (GUIDED → LAND güncellemesi)
  2. Navigation: goto_global / set_relative_altitude ARTIK mod komutu göndermez
     (per-command GUIDED re-assertion kaldırıldı — race condition yok)
  3. State machine: mode LAND/LOITER/STABILIZE/ALT_HOLD/RTL → PILOT_CONTROL
  4. State machine: mode GUIDED → mission normal devam eder
  5. State machine: mod bilgisi yoksa (mock) → yanlış pozitif takeover YOK
  6. PILOT_CONTROL state'i: hiçbir navigation/mode komutu göndermez
  7. Heartbeat + mission timer watchdog'ları PILOT_CONTROL'de RTL tetiklemez
  8. Kalkış sırasında pilot takeover → kalkış derhal durur
"""
import logging
import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from config import MISSION_TIMEOUT_S
from flight.navigation import Navigation
from models import MissionState


class _FakeMsg:
    def __init__(self, msg_type, **kw):
        self._type = msg_type
        for k, v in kw.items():
            setattr(self, k, v)

    def get_type(self):
        return self._type


class _FakeCam:
    def __init__(self):
        self._frame = np.zeros((352, 960, 3), dtype=np.uint8)
        self.shown = 0

    def latest(self):
        return self._frame

    def show(self, frame):
        self.shown += 1

    def pump(self):
        pass

    def close_preview(self):
        pass


def _nav_obj():
    """Ağır __init__ (gerçek MAVLink bağlantısı) olmadan Navigation örneği."""
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav.master.mode_mapping.return_value = {
        "STABILIZE": 0, "ALT_HOLD": 2, "GUIDED": 4,
        "LOITER": 5, "RTL": 6, "LAND": 9,
    }
    nav._mode_custom = None
    nav._mode_name = None
    nav._mode_last_t = 0.0
    return nav


def _make_sm(nav, heartbeat_timeout_s=None):
    from state_machine import MissionStateMachine

    return MissionStateMachine(
        navigation=nav,
        camera=_FakeCam(),
        payload=MagicMock(),
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=heartbeat_timeout_s,
    )


def _pos(**kw):
    p = MagicMock()
    p.relative_altitude = kw.get("relative_altitude", 11.5)
    p.heading_deg = kw.get("heading_deg", 0.0)
    p.latitude = kw.get("latitude", 41.0)
    p.longitude = kw.get("longitude", 28.9)
    p.battery_voltage_v = kw.get("battery_voltage_v", 24.8)
    p.battery_percent = kw.get("battery_percent", 85)
    return p


# ══════════════════════════════════════════════════════════════════════════ #
#  1) Navigation: HEARTBEAT → mod takibi
# ══════════════════════════════════════════════════════════════════════════ #
def test_navigation_tracks_mode_from_heartbeat():
    nav = _nav_obj()
    assert nav.current_flight_mode is None
    assert nav.flight_mode_fresh(5.0) is False   # henüz mod bilgisi yok

    nav._dispatch_telemetry(_FakeMsg("HEARTBEAT", custom_mode=4, type=2))
    assert nav.current_flight_mode == "GUIDED"
    assert nav.flight_mode_fresh(5.0) is True

    # Pilot RC → LAND: heartbeat custom_mode değişir
    nav._dispatch_telemetry(_FakeMsg("HEARTBEAT", custom_mode=9, type=2))
    assert nav.current_flight_mode == "LAND"


def test_navigation_ignores_gcs_heartbeat():
    """MAV_TYPE_GCS heartbeat'leri mod taşımaz — yoksayılır."""
    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg("HEARTBEAT", custom_mode=4, type=2))
    nav._dispatch_telemetry(_FakeMsg("HEARTBEAT", custom_mode=0, type=6))  # GCS
    assert nav.current_flight_mode == "GUIDED"   # GCS mesajı modu bozmadı


def test_navigation_mode_fresh_goes_stale():
    nav = _nav_obj()
    nav._mode_last_t = time.monotonic() - 30.0
    nav._mode_name = "LAND"
    assert nav.flight_mode_fresh(5.0) is False   # bayat — güvenilmez


# ══════════════════════════════════════════════════════════════════════════ #
#  2) Navigation: navigasyon komutları artık mod komutu GÖNDERMEZ
# ══════════════════════════════════════════════════════════════════════════ #
@patch("flight.navigation.mavutil")
def test_goto_global_never_sends_mode_command(mavutil_mock):
    """goto_global artık set_mode / DO_SET_MODE / SET_MODE göndermez —
    pilotun seçtiği mod asla ezilmez (race condition yok)."""
    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    master.mode_mapping.return_value = {"GUIDED": 4}
    master.recv_match.return_value = None
    mavutil_mock.mavlink_connection.return_value = master
    nav = Navigation("udp:127.0.0.1:14550")
    nav.position = MagicMock(
        return_value=MagicMock(latitude=1.0, longitude=1.0,
                               heading_deg=90.0, relative_altitude=2.0)
    )

    nav.goto_global(1.0001, 1.0001, 2.0)

    master.mav.set_position_target_global_int_send.assert_called_once()
    master.set_mode.assert_not_called()
    master.mav.set_mode_send.assert_not_called()
    master.mav.command_long_send.assert_not_called()   # DO_SET_MODE yok


@patch("flight.navigation.mavutil")
def test_set_relative_altitude_never_sends_mode_command(mavutil_mock):
    """set_relative_altitude artık GUIDED yeniden göndermez."""
    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    master.mode_mapping.return_value = {"GUIDED": 4}
    master.recv_match.return_value = None
    mavutil_mock.mavlink_connection.return_value = master
    nav = Navigation("udp:127.0.0.1:14550")
    nav.position = MagicMock(
        return_value=MagicMock(latitude=1.0, longitude=1.0,
                               heading_deg=90.0, relative_altitude=2.0)
    )

    nav.set_relative_altitude(3.0)

    master.mav.set_position_target_global_int_send.assert_called_once()
    master.set_mode.assert_not_called()
    master.mav.command_long_send.assert_not_called()


def test_takeoff_still_sets_guided_once():
    """Mission init hâlâ GUIDED'a geçebilir (TAKEOFF — kullanıcı onayı)."""
    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    master.mode_mapping.return_value = {"GUIDED": 4}
    master.recv_match.return_value = None
    master.motors_armed.return_value = True
    with patch("flight.navigation.mavutil") as mavutil_mock:
        mavutil_mock.mavlink_connection.return_value = master
        nav = Navigation("udp:127.0.0.1:14550")
        nav.takeoff(10.0)
    master.set_mode.assert_called_once()
    master.mav.command_long_send.assert_called_once()   # MAV_CMD_NAV_TAKEOFF


# ══════════════════════════════════════════════════════════════════════════ #
#  3) State machine: pilot takeover → PILOT_CONTROL, komut yok
# ══════════════════════════════════════════════════════════════════════════ #
def test_takeover_land_pauses_mission():
    nav = MagicMock()
    nav.current_flight_mode = "LAND"
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm._mission_start_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.PILOT_CONTROL
    assert sm._pilot_takeover_mode == "LAND"
    nav.goto_global.assert_not_called()
    nav.set_mode.assert_not_called()


@pytest.mark.parametrize("mode", ["LOITER", "STABILIZE", "ALT_HOLD", "RTL"])
def test_takeover_any_pilot_mode_pauses_mission(mode):
    nav = MagicMock()
    nav.current_flight_mode = mode
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.APPROACHING_TARGET
    sm._mission_start_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.PILOT_CONTROL
    assert sm._pilot_takeover_mode == mode
    nav.goto_global.assert_not_called()


# ══════════════════════════════════════════════════════════════════════════ #
#  4) GUIDED → mission normal devam
# ══════════════════════════════════════════════════════════════════════════ #
def test_guided_mode_mission_continues():
    nav = MagicMock()
    nav.current_flight_mode = "GUIDED"
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm._mission_start_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    nav.set_mode.assert_not_called()


# ══════════════════════════════════════════════════════════════════════════ #
#  5) Mod bilgisi yok / bayat → yanlış pozitif takeover YOK
# ══════════════════════════════════════════════════════════════════════════ #
def test_unknown_mode_no_false_takeover():
    """current_flight_mode bir str değilse (mock / heartbeat yok) pause edilmez."""
    nav = MagicMock()   # current_flight_mode → otomatik MagicMock (str değil)
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm._mission_start_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET


def test_stale_mode_no_false_takeover():
    """Bayat mod bilgisiyle (telemetri kesildi) takeover kararı verilmez."""
    nav = MagicMock()
    nav.current_flight_mode = "LAND"
    nav.flight_mode_fresh.return_value = False
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm._mission_start_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    nav.set_mode.assert_not_called()


# ══════════════════════════════════════════════════════════════════════════ #
#  6) PILOT_CONTROL: terminal durdurma — hiçbir komut yok
# ══════════════════════════════════════════════════════════════════════════ #
def test_pilot_control_state_sends_no_commands():
    nav = MagicMock()
    nav.current_flight_mode = "LAND"
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.PILOT_CONTROL
    sm._pilot_takeover_mode = "LAND"
    sm._pilot_takeover_t = time.monotonic()

    sm._do_pilot_control()

    assert sm.data.state == MissionState.PILOT_CONTROL   # asla ilerlemez
    nav.goto_global.assert_not_called()
    nav.set_mode.assert_not_called()
    nav.set_relative_altitude.assert_not_called()
    nav.takeoff.assert_not_called()
    nav.rtl.assert_not_called()


def test_pilot_control_no_auto_resume_when_guided_again():
    """Pilot tekrar GUIDED'a geçse bile mission OTOMATİK devam etmez."""
    nav = MagicMock()
    nav.current_flight_mode = "GUIDED"   # pilot geri GUIDED'a aldı
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.PILOT_CONTROL
    sm._pilot_takeover_mode = "LAND"
    sm._pilot_takeover_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.PILOT_CONTROL
    nav.goto_global.assert_not_called()


# ══════════════════════════════════════════════════════════════════════════ #
#  7) Watchdog'lar PILOT_CONTROL'de RTL tetiklemez
# ══════════════════════════════════════════════════════════════════════════ #
def test_heartbeat_watchdog_does_not_rtl_from_pilot_control():
    nav = MagicMock()
    nav.heartbeat_ok.return_value = False
    nav.current_flight_mode = "LAND"
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav, heartbeat_timeout_s=5.0)
    sm.data.state = MissionState.PILOT_CONTROL
    sm._pilot_takeover_mode = "LAND"
    sm._pilot_takeover_t = time.monotonic()

    sm._tick()

    assert sm.data.state == MissionState.PILOT_CONTROL   # RTL'ye gitmez
    assert sm.data.mission_failed is False
    nav.rtl.assert_not_called()


def test_mission_timer_does_not_rtl_from_pilot_control():
    nav = MagicMock()
    nav.current_flight_mode = "LAND"
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm.data.state = MissionState.PILOT_CONTROL
    sm._pilot_takeover_mode = "LAND"
    sm._pilot_takeover_t = time.monotonic()
    sm._mission_start_t = time.monotonic() - MISSION_TIMEOUT_S - 60.0

    sm._tick()

    assert sm.data.state == MissionState.PILOT_CONTROL   # timeout RTL tetiklemez
    nav.rtl.assert_not_called()


# ══════════════════════════════════════════════════════════════════════════ #
#  8) Kalkış sırasında pilot takeover → kalkış derhal durur
# ══════════════════════════════════════════════════════════════════════════ #
def test_takeoff_loop_pauses_on_pilot_takeover():
    """Kalkış sırasında pilot LAND'e geçerse takeoff timeout'una (45s)
    beklenmez — mission derhal PILOT_CONTROL'a alınır."""
    nav = MagicMock()
    nav.current_flight_mode = "LAND"
    nav.flight_mode_fresh.return_value = True
    nav.position.return_value = _pos(relative_altitude=0.0)  # irtifa asla 11.5 olmaz
    sm = _make_sm(nav)
    sm.data.state = MissionState.TAKEOFF

    sm._do_takeoff()

    assert sm.data.state == MissionState.PILOT_CONTROL
    assert sm._pilot_takeover_mode == "LAND"
