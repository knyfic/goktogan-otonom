"""
Kamera → görev mekanizması offset düzeltmesi testleri.

goto_body_offset artık MAV_FRAME_BODY_OFFSET_NED yerine kanıtlanmış
GPS tabanlı yöntemi kullanır: body offset → heading ile NED → goto_global.

Bu test, gönderilen GPS hedefinin her heading'de tam olarak body-forward
yönünde istenen mesafe (CAMERA_TO_MECHANISM_OFFSET_M = 8cm) kadar
kaydırıldığını doğrular.
"""

import math
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest

from config import (
    CAMERA_TO_MECHANISM_OFFSET_M,
    OFFSET_CORRECTION_WAIT_S,
    PAYLOAD_DESCENT_HOLD_S,
    PAYLOAD_DESCENT_SPEED_MPS,
    PAYLOAD_SERVO_WAIT_S,
)
from flight.navigation import Navigation
from models import MissionState, TargetColor, TargetType, VehiclePosition


def _make_nav(lat: float, lon: float, heading: float) -> Navigation:
    """__init__ çağırmadan (MAVLink bağlantısı olmadan) Navigation örneği kur."""
    nav = object.__new__(Navigation)
    nav.master = MagicMock()
    nav.master.recv_match.return_value = None  # position() → son bilinen konum
    nav._position = VehiclePosition(
        latitude=lat, longitude=lon, relative_altitude=0.4, heading_deg=heading
    )
    nav.goto_global = MagicMock()
    return nav


def _latlon_to_m(nav: Navigation, lat: float, lon: float):
    """Hedef GPS'in mevcut konumdan metre cinsinden kuzey/doğu sapması."""
    lat_m = (lat - nav._position.latitude) * 111320.0
    lon_m = (
        (lon - nav._position.longitude)
        * 111320.0
        * math.cos(math.radians(nav._position.latitude))
    )
    return lat_m, lon_m


def test_offset_is_8cm_forward_in_body_frame():
    """Her heading'de hedef, gövde forward yönünde tam 8cm uzakta olmalı."""
    lat, lon = 40.0, 29.0
    for heading, exp_north, exp_east in (
        (0.0, +CAMERA_TO_MECHANISM_OFFSET_M, 0.0),    # kuzeye bakıyor → kuzey
        (90.0, 0.0, +CAMERA_TO_MECHANISM_OFFSET_M),   # doğuya bakıyor → doğu
        (180.0, -CAMERA_TO_MECHANISM_OFFSET_M, 0.0),  # güneye bakıyor → güney
        (270.0, 0.0, -CAMERA_TO_MECHANISM_OFFSET_M),  # batıya bakıyor → batı
    ):
        nav = _make_nav(lat, lon, heading)
        nav.goto_body_offset(CAMERA_TO_MECHANISM_OFFSET_M, 0.0)

        assert nav.goto_global.called, f"heading={heading}: goto_global çağrılmalı"
        target_lat, target_lon, alt = nav.goto_global.call_args.args
        north_m, east_m = _latlon_to_m(nav, target_lat, target_lon)

        assert north_m == pytest.approx(exp_north, abs=1e-3), (
            f"heading={heading}: kuzey sapması {north_m:.3f}m, beklenen {exp_north:.3f}m"
        )
        assert east_m == pytest.approx(exp_east, abs=1e-3), (
            f"heading={heading}: doğu sapması {east_m:.3f}m, beklenen {exp_east:.3f}m"
        )


def test_offset_keeps_current_altitude():
    """goto_body_offset irtifayı korumalı (mevcut relative_altitude hedef olarak)."""
    nav = _make_nav(40.0, 29.0, 45.0)
    nav.goto_body_offset(CAMERA_TO_MECHANISM_OFFSET_M, 0.0)
    target_lat, target_lon, alt = nav.goto_global.call_args.args
    assert alt == pytest.approx(0.4)


def test_offset_right_moves_right_in_body_frame():
    """right_m > 0 → gövde sağ yönünde hareket."""
    nav = _make_nav(40.0, 29.0, 0.0)  # kuzeye bakıyor → sağ = doğu
    nav.goto_body_offset(0.0, 0.05)
    target_lat, target_lon, _ = nav.goto_global.call_args.args
    north_m, east_m = _latlon_to_m(nav, target_lat, target_lon)
    assert north_m == pytest.approx(0.0, abs=1e-3)
    assert east_m == pytest.approx(0.05, abs=1e-3)


# ====================================================================== #
#  Offset sonrası: havada mıknatıs + dikey aşağı HIZ komutu (LiDAR hatası)
# ====================================================================== #


def _real_sm():
    """Gerçek __init__ ile kurulan örnek (tüm öznitelikler hazır)."""
    import logging

    import numpy as np

    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=0.4, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    payload = MagicMock()
    payload.servo_wait_s = None  # simülasyon davranışı

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=payload,
        logger=logging.getLogger("t"),
    )
    # Mavi altıgen hedef → payload_for_target → RED payload
    sm.data.first_target = SimpleNamespace(
        color=TargetColor.BLUE, shape=TargetType.HEXAGON,
    )
    return sm, nav


def test_offset_complete_activates_magnets_airborne_and_sends_velocity():
    """Offset bekleyince: mıknatıslar HAVADAYKEN aktive edilir ve irtifa hedefi
    yerine doğrudan dikey aşağı HIZ komutu gönderilir (LiDAR hatası törpüleme)."""
    sm, nav = _real_sm()
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "WAITING_OFFSET"

    sm._do_offset_correction()

    sm.payload.send_pickup_command.assert_called_once_with(TargetColor.RED)
    nav.descent_velocity.assert_called_once_with(PAYLOAD_DESCENT_SPEED_MPS)
    nav.set_relative_altitude.assert_not_called()   # irtifa düşürme komutu YOK
    assert sm._offset_phase == "DESCENDING_VELOCITY"
    assert sm._descent_velocity_start is not None


def test_offset_velocity_descent_holds_then_enters_pickup():
    """Dikey hız inişi PAYLOAD_DESCENT_HOLD_S sürdükten sonra PICKING_UP_PAYLOAD'a
    geçilir."""
    sm, nav = _real_sm()
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "DESCENDING_VELOCITY"
    sm._descent_velocity_start = time.monotonic() - PAYLOAD_DESCENT_HOLD_S - 0.5
    sm._descent_velocity_last_t = time.monotonic()   # komut yeni gönderildi

    sm._do_offset_correction()

    assert sm.data.state == MissionState.PICKING_UP_PAYLOAD


def test_offset_velocity_descent_waits_until_hold_elapsed():
    """Süre dolmadıysa pickup'a geçilmez; hız komutu periyodik yeniden gönderilir."""
    sm, nav = _real_sm()
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "DESCENDING_VELOCITY"
    sm._descent_velocity_start = time.monotonic()
    sm._descent_velocity_last_t = 0.0   # ilk komut yeniden gönderimini zorla

    sm._do_offset_correction()

    assert sm.data.state != MissionState.PICKING_UP_PAYLOAD
    nav.descent_velocity.assert_called_once_with(PAYLOAD_DESCENT_SPEED_MPS)


def test_pickup_state_does_not_resend_magnet_command():
    """PICKING_UP_PAYLOAD'da mıknatıs komutu TEKRAR GÖNDERİLMEZ (havada aktive
    edildi; tekrar göndermek backend'de INVALID_STATE üretir)."""
    sm, nav = _real_sm()
    sm.data.state = MissionState.PICKING_UP_PAYLOAD
    sm._pickup_servo_start = None

    sm._do_pickup()

    sm.payload.send_pickup_command.assert_not_called()
    assert sm._pickup_servo_start is not None


def test_pickup_finishes_after_wait_without_resending_magnets():
    """Bekleme süresi dolunca finish_pickup çağrılır ve ASCENDING_FOR_VERIFICATION'a
    geçilir — yine mıknatıs komutu tekrar gönderilmez."""
    sm, nav = _real_sm()
    sm.data.state = MissionState.PICKING_UP_PAYLOAD
    sm._pickup_servo_start = time.monotonic() - PAYLOAD_SERVO_WAIT_S - 1.0

    sm._do_pickup()

    sm.payload.send_pickup_command.assert_not_called()
    sm.payload.finish_pickup.assert_called_once()
    assert sm.data.state == MissionState.ASCENDING_FOR_VERIFICATION


@patch("flight.navigation.mavutil")
def test_descent_velocity_sends_velocity_only_command(mavutil_mock):
    """descent_velocity: LOCAL_NED frame, yalnızca hız (vx,vy,vz) AKTİF;
    pozisyon/ivme/yaw YOKSAYILIR; vz = PAYLOAD_DESCENT_SPEED_MPS (NED aşağı)."""
    from flight.navigation import Navigation

    master = MagicMock()
    master.target_system = 1
    master.target_component = 1
    master.wait_heartbeat.return_value = None
    mavutil_mock.mavlink_connection.return_value = master

    nav = Navigation("udp:127.0.0.1:14550")
    nav.descent_velocity(PAYLOAD_DESCENT_SPEED_MPS)

    args = master.mav.set_position_target_local_ned_send.call_args[0]
    frame = args[3]
    type_mask = args[4]
    vx, vy, vz = args[8], args[9], args[10]

    assert frame == mavutil_mock.mavlink.MAV_FRAME_LOCAL_NED
    # Hız bitleri (3,4,5) AKTİF → ignore bitleri 0
    assert (type_mask >> 3) & 1 == 0
    assert (type_mask >> 4) & 1 == 0
    assert (type_mask >> 5) & 1 == 0
    # Pozisyon (0,1,2) + ivme (6,7,8) + yaw (9) + yaw_rate (10) + force (11) yoksayılır
    assert (type_mask >> 0) & 1 == 1
    assert (type_mask >> 1) & 1 == 1
    assert (type_mask >> 2) & 1 == 1
    assert type_mask == 0b110111000111
    assert vx == 0 and vy == 0
    assert vz == pytest.approx(PAYLOAD_DESCENT_SPEED_MPS)


def test_descent_velocity_does_not_touch_altitude_state():
    """Hız inişi sırasında altitude bekleme mekanizması (_alt_waiting) devreye
    sokulmaz — irtifa hedefi yok."""
    sm, nav = _real_sm()
    sm._offset_correction_start = time.monotonic() - OFFSET_CORRECTION_WAIT_S - 1.0
    sm._offset_phase = "WAITING_OFFSET"

    sm._do_offset_correction()

    assert sm._alt_waiting is False
    nav.set_relative_altitude.assert_not_called()
