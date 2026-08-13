"""
REAL mode kompozisyon testleri (Aşama 3).

1. _build_navigation: real → config'ten serial device + baud; sim/hil →
   MAVLINK_CONNECTION (eski davranış korunur).
2. REAL kamera: RPiCam3, preview=False (headless — cv2.imshow asla).
3. Heartbeat watchdog: MissionStateMachine'de bağlantı kaybı → RTL.
"""
import logging
from pathlib import Path
from unittest.mock import MagicMock, patch

import yaml

_REPO_ROOT = Path(__file__).resolve().parent.parent
_PAYLOAD_CONFIG = _REPO_ROOT / "payload" / "config.yaml"


def _load_config() -> dict:
    return yaml.safe_load(_PAYLOAD_CONFIG.read_text())


# ── 1) Navigation seçimi ──────────────────────────────────────────────────── #
@patch("flight.navigation.mavutil")
def test_build_navigation_real_uses_config_serial(mavutil_mock):
    from main import _build_navigation

    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    mavutil_mock.mavlink_connection.return_value = master

    cfg = _load_config()
    nav = _build_navigation("real", cfg, logging.getLogger("t"))
    mavutil_mock.mavlink_connection.assert_called_once_with(
        "/dev/serial0", baud=921600, force_mavlink2=True
    )
    assert nav.system_id == 1


@patch("flight.navigation.mavutil")
def test_build_navigation_sim_hil_uses_udp(mavutil_mock):
    from main import _build_navigation

    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    mavutil_mock.mavlink_connection.return_value = master

    for mode in ("simulation", "hil"):
        _build_navigation(mode, {}, logging.getLogger(f"t_{mode}"))
        mavutil_mock.mavlink_connection.assert_called_with(
            "udp:127.0.0.1:14550"
        )


@patch("flight.navigation.mavutil")
def test_build_navigation_real_fallback_without_device(mavutil_mock):
    """flight_controller yoksa MAVLINK_CONNECTION fallback (eski config uyumlu)."""
    from main import _build_navigation

    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    mavutil_mock.mavlink_connection.return_value = master

    _build_navigation("real", {}, logging.getLogger("t"))
    mavutil_mock.mavlink_connection.assert_called_once_with(
        "udp:127.0.0.1:14550"
    )


# ── 2) REAL kamera headless ───────────────────────────────────────────────── #
def test_real_camera_is_headless():
    """backend=real → RPiCam3 preview=False (cv2.imshow/namedWindow asla)."""
    from main import _build_camera
    from vision.rpi_camera import RPiCam3

    cam = _build_camera("real", _load_config(), logging.getLogger("t"))
    assert isinstance(cam, RPiCam3)
    assert cam._preview is False, "REAL modda yerel GUI/pencere açılmamalı"
    cam.close_preview()


# ── 3) Heartbeat watchdog → RTL ───────────────────────────────────────────── #
class _FakeNav:
    """heartbeat_ok False dönen nav — bağlantı koptu senaryosu."""

    def __init__(self):
        self.rtl_called = False
        self.age = 30.0

    @property
    def last_message_age_s(self):
        return self.age

    def heartbeat_ok(self, timeout_s):
        return False

    def rtl(self):
        self.rtl_called = True


def test_heartbeat_watchdog_triggers_rtl():
    from models import MissionState
    from state_machine import MissionStateMachine

    nav = _FakeNav()
    camera = MagicMock()
    payload = MagicMock()

    sm = MissionStateMachine(
        navigation=nav,
        camera=camera,
        payload=payload,
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=10.0,
    )
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm._tick()

    assert sm.data.state == MissionState.RTL
    assert sm.data.mission_failed is True
    assert sm._heartbeat_triggered is True
    assert nav.rtl_called is True   # mevcut RTL abstraction'ı kullanıldı


def test_heartbeat_watchdog_disabled_by_default():
    """heartbeat_timeout_s=None → watchdog hiç çalışmaz (sim/hil davranışı)."""
    from models import MissionState
    from state_machine import MissionStateMachine

    nav = _FakeNav()
    sm = MissionStateMachine(
        navigation=nav,
        camera=MagicMock(),
        payload=MagicMock(),
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=None,
    )
    # Watchdog kapalıyken state makinesi NORMAL ilerler (TARGET_CLASSIFIED →
    # APPROACHING_TARGET) — watchdog araya girip RTL'ye çekmez.
    sm.data.state = MissionState.TARGET_CLASSIFIED
    sm._tick()

    assert sm.data.state == MissionState.APPROACHING_TARGET
    assert sm._heartbeat_triggered is False


def test_heartbeat_watchdog_skips_rtl_state():
    """RTL state'inde watchdog çalışmaz — FC kendi failsafe'ini yürütür."""
    from models import MissionState
    from state_machine import MissionStateMachine

    nav = _FakeNav()
    sm = MissionStateMachine(
        navigation=nav,
        camera=MagicMock(),
        payload=MagicMock(),
        logger=logging.getLogger("t"),
        heartbeat_timeout_s=10.0,
    )
    sm._do_wait_rtl = lambda: None   # 5 dk'lık gerçek bekleme döngüsünü devre dışı bırak
    sm.data.state = MissionState.RTL
    sm._tick()

    # RTL handler'ı devreye girdi, watchdog RTL'yi yarıda kesmedi
    assert sm._heartbeat_triggered is False
