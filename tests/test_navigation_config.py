"""
Navigation MAVLink bağlantı + heartbeat watchdog testleri (Aşama 3 — REAL mode).

1. baud parametresi: serial bağlantıda mavutil'a iletilir; None ise iletilmez
   (UDP/TCP simülasyon/HIL bağlantıları bozulmaz).
2. heartbeat_ok: son telemetri yaşına göre True/False döner — mesaj tüketmez.
"""
import time
from unittest.mock import MagicMock, patch

from flight.navigation import Navigation


def _mock_master():
    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    return master


# ── 1) Bağlantı kurulumu ───────────────────────────────────────────────────── #
@patch("flight.navigation.mavutil")
def test_serial_connection_passes_baud(mavutil_mock):
    """REAL: serial cihaz + baudrate + MAVLink2 + write iletilmeli."""
    mavutil_mock.mavlink_connection.return_value = _mock_master()
    Navigation("/dev/serial0", baud=921600)
    mavutil_mock.mavlink_connection.assert_called_once_with(
        "/dev/serial0", baud=921600, force_mavlink2=True, write=True
    )


@patch("flight.navigation.mavutil")
def test_serial_connection_flow_control(mavutil_mock):
    """flow_control=True → serial portta rtscts açılır (TELEM2 RTS/CTS)."""
    master = _mock_master()
    mavutil_mock.mavlink_connection.return_value = master
    Navigation("/dev/serial0", baud=921600, flow_control=True)
    assert master.port.rtscts is True


@patch("flight.navigation.mavutil")
def test_serial_connection_sends_gcs_heartbeats(mavutil_mock):
    """Serial bağlantıda GCS heartbeat thread'i çalışır — ArduPilot stream
    başlatsın diye (uçuş komutu değil, MAVLink el sıkışması)."""
    import time

    master = _mock_master()
    mavutil_mock.mavlink_connection.return_value = master
    nav = Navigation("/dev/serial0", baud=921600)

    assert nav._hb_thread is not None
    assert nav._hb_thread.daemon is True
    # ~1.5 sn içinde en az bir heartbeat gönderilmiş olmalı (1 sn periyot)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and master.mav.heartbeat_send.call_count == 0:
        time.sleep(0.1)
    assert master.mav.heartbeat_send.call_count >= 1
    nav._hb_stop.set()


@patch("flight.navigation.mavutil")
def test_udp_connection_without_baud(mavutil_mock):
    """SIM/HIL: baud yok → eski davranış korunur (tek argüman)."""
    mavutil_mock.mavlink_connection.return_value = _mock_master()
    Navigation("udp:127.0.0.1:14550")
    mavutil_mock.mavlink_connection.assert_called_once_with(
        "udp:127.0.0.1:14550"
    )


# ── 2) Heartbeat watchdog ──────────────────────────────────────────────────── #
def _make_nav_with_age(age_s: float) -> Navigation:
    """__init__ (gerçek MAVLink bağlantısı) olmadan Navigation örneği kur."""
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav._position = MagicMock()
    nav.home_lat = None
    nav.home_lon = None
    nav._last_message_s = time.monotonic() - age_s
    return nav


def test_heartbeat_ok_true_when_fresh():
    nav = _make_nav_with_age(age_s=0.5)
    assert nav.heartbeat_ok(10.0) is True


def test_heartbeat_ok_false_when_stale():
    nav = _make_nav_with_age(age_s=20.0)
    assert nav.heartbeat_ok(10.0) is False


def test_heartbeat_ok_updates_on_new_position():
    """_update_position telemetri tazeler → heartbeat_ok yeniden True olur."""
    nav = _make_nav_with_age(age_s=100.0)
    assert nav.heartbeat_ok(10.0) is False

    msg = MagicMock()
    msg.lat = 410000000
    msg.lon = 290000000
    msg.relative_alt = 5000
    msg.hdg = 9000
    nav._update_position(msg)
    assert nav.heartbeat_ok(10.0) is True
    assert nav.last_message_age_s < 1.0


def test_heartbeat_ok_does_not_consume_messages():
    """heartbeat_ok alma buffer'ını tüketmemeli (recv_match çağrılmamalı)."""
    nav = _make_nav_with_age(age_s=0.1)
    nav.heartbeat_ok(10.0)
    nav.master.recv_match.assert_not_called()
