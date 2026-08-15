"""
Pre-arm gate testleri — görev, ArduPilot pre-arm GOOD olmadan başlamaz.

Kapsam:
  1. Navigation: SYS_STATUS sağlık bitleri → prearm_good() (True/False/None)
  2. Navigation: STATUSTEXT 'PreArm: ...' → prearm_reason()
  3. State machine: pre-arm bekleme (mission başlamaz, arm/takeoff yok)
  4. State machine: pre-arm GOOD → mevcut INIT/TAKEOFF akışı
  5. Beklerken kamera/vision/HUD/GStreamer canlı
  6. HUD'da neden + GOOD gösterimi
  7. Beklerken MAVLink kaybı → mevcut watchdog → RTL
"""
import logging
from unittest.mock import MagicMock

import numpy as np

from models import MissionState
from tests.test_video_streamer import _RecordingStreamer  # noqa: F401 — yeniden tanım yerine


class _FakeMsg:
    def __init__(self, msg_type, **kw):
        self._type = msg_type
        for k, v in kw.items():
            setattr(self, k, v)

    def get_type(self):
        return self._type


class _FakeCam:
    def __init__(self):
        self._frame = np.zeros((704, 1920, 3), dtype=np.uint8)
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
    """Ağır __init__ (MAVLink bağlantısı) olmadan Navigation örneği."""
    from flight.navigation import Navigation
    from models import VehiclePosition

    nav = object.__new__(Navigation)
    nav._position = VehiclePosition()
    nav._last_message_s = 0.0
    nav.home_lat = None
    nav.home_lon = None
    nav._sys_status_health = 0
    nav._sys_status_present = 0
    nav._sys_status_last_t = 0.0
    nav._status_text = ""
    nav._prearm_reason = None
    nav._prearm_reason_t = 0.0
    return nav


def _make_sm(nav, camera=None, heartbeat_timeout_s=None):
    from state_machine import MissionStateMachine

    return MissionStateMachine(
        navigation=nav,
        camera=camera or _FakeCam(),
        payload=MagicMock(),
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=heartbeat_timeout_s,
    )


def _pos(**kw):
    """HUD formatlamasına uygun sayısal alanlı pozisyon mock'u."""
    p = MagicMock()
    p.relative_altitude = kw.get("relative_altitude", 0.0)
    p.heading_deg = kw.get("heading_deg", 0.0)
    p.latitude = kw.get("latitude", 41.0)
    p.longitude = kw.get("longitude", 28.9)
    p.battery_voltage_v = kw.get("battery_voltage_v", 24.8)
    p.battery_percent = kw.get("battery_percent", 85)
    return p


# ── 1) Navigation: SYS_STATUS → prearm_good ───────────────────────────────── #
def test_prearm_good_true_when_health_bits_set():
    from flight.navigation import Navigation

    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg(
        "SYS_STATUS",
        onboard_control_sensors_present=0xFFFFFFFF,
        onboard_control_sensors_health=(
            Navigation._SYS_AHRS | Navigation._SYS_GPS | Navigation._SYS_PREARM_CHECK
        ),
        battery_remaining=90,
    ))
    assert nav.prearm_good() is True


def test_prearm_good_false_when_gps_missing():
    from flight.navigation import Navigation

    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg(
        "SYS_STATUS",
        onboard_control_sensors_present=0xFFFFFFFF,
        onboard_control_sensors_health=(
            Navigation._SYS_AHRS | Navigation._SYS_PREARM_CHECK  # GPS biti YOK
        ),
        battery_remaining=90,
    ))
    assert nav.prearm_good() is False


def test_prearm_good_none_before_sys_status():
    nav = _nav_obj()
    assert nav.prearm_good() is None


# ── 2) Navigation: STATUSTEXT → prearm_reason ─────────────────────────────── #
def test_prearm_reason_from_statustext():
    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg("STATUSTEXT", text=b"PreArm: GPS 1: Bad fix"))
    assert nav.prearm_reason() == "PreArm: GPS 1: Bad fix"


def test_prearm_reason_updates_with_new_message():
    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg("STATUSTEXT", text=b"PreArm: GPS 1: Bad fix"))
    nav._dispatch_telemetry(_FakeMsg("STATUSTEXT", text=b"PreArm: Compass not calibrated"))
    assert nav.prearm_reason() == "PreArm: Compass not calibrated"


def test_non_prearm_statustext_does_not_change_reason():
    nav = _nav_obj()
    nav._dispatch_telemetry(_FakeMsg("STATUSTEXT", text=b"PreArm: GPS 1: Bad fix"))
    nav._dispatch_telemetry(_FakeMsg("STATUSTEXT", text=b"EKF3 IMU0 is using GPS"))
    assert nav.prearm_reason() == "PreArm: GPS 1: Bad fix"


# ── 3) Bekleme: mission başlamaz, ARM/TAKEOFF yok ─────────────────────────── #
def test_prearm_not_good_keeps_waiting_no_arm():
    nav = MagicMock()
    nav.prearm_good.return_value = False
    nav.prearm_reason.return_value = "PreArm: GPS 1: Bad fix"
    nav.position.return_value = _pos()
    sm = _make_sm(nav)

    assert sm.data.state == MissionState.WAITING_FOR_PREARM
    sm._do_wait_prearm()

    assert sm.data.state == MissionState.WAITING_FOR_PREARM
    assert sm._prearm_reason == "PreArm: GPS 1: Bad fix"
    nav.arm.assert_not_called()
    nav.takeoff.assert_not_called()
    nav.goto_global.assert_not_called()


def test_no_sys_status_yet_keeps_waiting():
    nav = MagicMock()
    nav.prearm_good.return_value = None
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm._do_wait_prearm()
    assert sm.data.state == MissionState.WAITING_FOR_PREARM


# ── 4) Pre-arm GOOD → mevcut akış ─────────────────────────────────────────── #
def test_prearm_good_starts_existing_mission_flow():
    from config import COMPETITION_CORNERS

    a = COMPETITION_CORNERS["A"]
    nav = MagicMock()
    nav.prearm_good.return_value = True
    nav.prearm_reason.return_value = None
    nav.position.return_value = MagicMock(
        relative_altitude=8.0, latitude=float(a[0]), longitude=float(a[1]),
        heading_deg=0.0,
    )
    nav.fresh_position.return_value = MagicMock(
        latitude=float(a[0]), longitude=float(a[1]),
    )
    sm = _make_sm(nav)

    sm._do_wait_prearm()
    assert sm.data.state == MissionState.INIT

    sm._do_init()                       # wait_for_position → TAKEOFF
    assert sm.data.state == MissionState.TAKEOFF

    sm._do_takeoff()                    # takeoff + irtifa → SEARCHING_FIRST_TARGET
    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    nav.takeoff.assert_called_once()


# ── 5) Beklerken kamera/vision/HUD/GStreamer canlı ────────────────────────── #
def test_vision_runs_during_prearm_wait():
    nav = MagicMock()
    nav.prearm_good.return_value = False
    nav.prearm_reason.return_value = None
    nav.position.return_value = _pos()
    sm = _make_sm(nav)
    sm._pending_display = None
    sm._do_wait_prearm()
    assert sm._pending_display is not None, "beklerken vision/display frame üretilmeli"


def test_stream_receives_display_frame_during_prearm():
    """Beklerken GStreamer'a display frame gitmeye devam eder (np.array_equal)."""
    from config import PREVIEW_DISPLAY_WIDTH_PX
    from services.video_streamer import StreamCamera
    from state_machine import MissionStateMachine

    recorder = _RecordingStreamer()
    cam = StreamCamera(_FakeCam(), recorder)
    nav = MagicMock()
    nav.prearm_good.return_value = False
    nav.prearm_reason.return_value = "PreArm: GPS 1: Bad fix"
    nav.position.return_value = _pos()
    sm = _make_sm(nav, camera=cam)

    sm._pending_display = None
    sm._do_wait_prearm()          # vision → _pending_display
    sm._pump_preview()            # → camera.show(disp) → recorder.push(disp)

    assert recorder.last_frame is not None, "GStreamer'a frame gitmeli"
    expected = MissionStateMachine._scale_for_display(sm._pending_display)
    assert np.array_equal(recorder.last_frame, expected), \
        "yayınlanan frame, display frame'in aynısı olmalı"


# ── 6) HUD ────────────────────────────────────────────────────────────────── #
def _hud_sm(nav):
    sm = _make_sm(nav)
    sm._mission_start_t = None
    return sm


def test_hud_shows_prearm_waiting_and_reason():
    nav = MagicMock()
    nav.prearm_good.return_value = False
    nav.prearm_reason.return_value = "PreArm: GPS 1: Bad fix"
    nav.position.return_value = MagicMock(
        relative_altitude=0.0, heading_deg=0.0,
        latitude=41.0, longitude=28.9,
        battery_voltage_v=24.8, battery_percent=85,
    )
    sm = _hud_sm(nav)
    sm._do_wait_prearm()

    l1, l2 = sm._hud_lines()
    assert "WAITING_FOR_PREARM" in l1
    assert "PRE-ARM: WAITING" in l2
    assert "GPS 1: Bad fix" in l2


def test_hud_shows_prearm_good():
    nav = MagicMock()
    nav.prearm_good.return_value = True
    nav.position.return_value = MagicMock(
        relative_altitude=0.0, heading_deg=0.0,
        latitude=41.0, longitude=28.9,
        battery_voltage_v=24.8, battery_percent=85,
    )
    sm = _hud_sm(nav)
    _l1, l2 = sm._hud_lines()
    assert "PRE-ARM: GOOD" in l2


def test_hud_prearm_only_in_waiting_state():
    """Görev başlayınca pre-arm satırı HUD'dan kalkar."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=8.0, heading_deg=90.0,
        latitude=41.0, longitude=28.9,
        battery_voltage_v=24.8, battery_percent=85,
    )
    sm = _hud_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    _l1, l2 = sm._hud_lines()
    assert "PRE-ARM" not in l2


# ── 7) Beklerken MAVLink kaybı → mevcut watchdog → RTL ────────────────────── #
def test_connection_loss_during_prearm_goes_rtl():
    nav = MagicMock()
    nav.heartbeat_ok.return_value = False
    nav.prearm_good.return_value = False
    sm = _make_sm(nav, heartbeat_timeout_s=5.0)
    sm.data.state = MissionState.WAITING_FOR_PREARM

    sm._tick()

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True
