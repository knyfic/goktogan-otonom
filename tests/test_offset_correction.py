"""
Kamera → görev mekanizması offset düzeltmesi testleri.

goto_body_offset artık MAV_FRAME_BODY_OFFSET_NED yerine kanıtlanmış
GPS tabanlı yöntemi kullanır: body offset → heading ile NED → goto_global.

Bu test, gönderilen GPS hedefinin her heading'de tam olarak body-forward
yönünde istenen mesafe (CAMERA_TO_MECHANISM_OFFSET_M = 8cm) kadar
kaydırıldığını doğrular.
"""

import math
from unittest.mock import MagicMock

import pytest

from config import CAMERA_TO_MECHANISM_OFFSET_M
from flight.navigation import Navigation
from models import VehiclePosition


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
