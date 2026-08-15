"""
Hedefe yaklaşma hız kapısı (E1) testleri — APPROACH_RADIUS_M.

Tasarım:
  - Hedef kaba GPS'e UZAK (> APPROACH_RADIUS_M): APPROACH_SPEED_MPS ile
    APPROACHING_TARGET state'inde kalır (her tick yeni goto_global).
  - Yarıçap içinde / kaba GPS yok: ALIGN_SPEED_MPS'e düşer, ALIGNING_TARGET'a geçer.

Eski davranış (bug): _do_approach aynı çağrıda 2.0 → 0.8 yapıyordu → tüm hedef
transit'i 0.8 m/s'de sürünüyordu.
"""
import logging
import math
from unittest.mock import MagicMock

import pytest

from config import ALIGN_SPEED_MPS, APPROACH_RADIUS_M, APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS
from models import MissionState


def _gps_offset(lat, lon, north_m, east_m):
    """Kuzey/doğu metre ofseti → GPS (haversine ile doğrulanabilir)."""
    radius = 6378137.0
    d_lat = north_m / radius
    d_lon = east_m / (radius * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)


def _pos(lat=41.0, lon=28.9):
    p = MagicMock()
    p.relative_altitude = 8.0
    p.heading_deg = 90.0
    p.latitude = lat
    p.longitude = lon
    p.battery_voltage_v = 24.8
    p.battery_percent = 85
    return p


def _make_sm(nav, rough_lat, rough_lon):
    from state_machine import MissionStateMachine

    sm = MissionStateMachine(
        navigation=nav,
        camera=MagicMock(),
        payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.APPROACHING_TARGET
    sm._target_rough_lat = rough_lat
    sm._target_rough_lon = rough_lon
    return sm


def test_approach_far_uses_approach_speed_and_stays():
    """Uzak hedef (> APPROACH_RADIUS_M): 2.0 m/s, state'te kalır, goto gönderilir."""
    o_lat, o_lon = 41.0, 28.9
    rough_lat, rough_lon = _gps_offset(o_lat, o_lon, north_m=APPROACH_RADIUS_M + 15.0, east_m=0.0)

    nav = MagicMock()
    nav.position.return_value = _pos(o_lat, o_lon)
    sm = _make_sm(nav, rough_lat, rough_lon)

    sm._do_approach()

    assert sm.data.state == MissionState.APPROACHING_TARGET, \
        "uzaktayken hizalama state'ine geçilmemeli"
    args = nav.set_max_speed.call_args[0]
    assert args[0] == APPROACH_SPEED_MPS, "uzak transit APPROACH_SPEED_MPS kullanmalı"
    assert args[1] == VERTICAL_SPEED_MPS
    nav.goto_global.assert_called_once()
    g = nav.goto_global.call_args[0]
    assert g[0] == pytest.approx(rough_lat, abs=1e-6)
    assert g[1] == pytest.approx(rough_lon, abs=1e-6)
    assert g[2] == pytest.approx(8.0)  # mevcut irtifa korunur


def test_approach_near_transitions_to_align_speed():
    """Yakın hedef (yarıçap içinde): 0.8 m/s + ALIGNING_TARGET."""
    o_lat, o_lon = 41.0, 28.9
    rough_lat, rough_lon = _gps_offset(o_lat, o_lon, north_m=2.0, east_m=0.0)

    nav = MagicMock()
    nav.position.return_value = _pos(o_lat, o_lon)
    sm = _make_sm(nav, rough_lat, rough_lon)

    sm._do_approach()

    assert sm.data.state == MissionState.ALIGNING_TARGET
    args = nav.set_max_speed.call_args[0]
    assert args[0] == ALIGN_SPEED_MPS
    assert args[1] == VERTICAL_SPEED_MPS
    nav.goto_global.assert_not_called()  # yakında kaba goto gerekmez


def test_approach_just_inside_radius_transitions_to_align():
    """Yarıçapın hemen içinde (> değil) → hizalamaya geç."""
    o_lat, o_lon = 41.0, 28.9
    rough_lat, rough_lon = _gps_offset(
        o_lat, o_lon, north_m=APPROACH_RADIUS_M - 0.5, east_m=0.0,
    )

    nav = MagicMock()
    nav.position.return_value = _pos(o_lat, o_lon)
    sm = _make_sm(nav, rough_lat, rough_lon)

    sm._do_approach()

    assert sm.data.state == MissionState.ALIGNING_TARGET
    assert nav.set_max_speed.call_args[0][0] == ALIGN_SPEED_MPS


def test_approach_just_outside_radius_stays_approaching():
    """Yarıçapın hemen dışında → yaklaşma state'inde kal (2.0 m/s)."""
    o_lat, o_lon = 41.0, 28.9
    rough_lat, rough_lon = _gps_offset(
        o_lat, o_lon, north_m=APPROACH_RADIUS_M + 0.5, east_m=0.0,
    )

    nav = MagicMock()
    nav.position.return_value = _pos(o_lat, o_lon)
    sm = _make_sm(nav, rough_lat, rough_lon)

    sm._do_approach()

    assert sm.data.state == MissionState.APPROACHING_TARGET
    assert nav.set_max_speed.call_args[0][0] == APPROACH_SPEED_MPS


def test_approach_without_rough_transitions_to_align():
    """Kaba GPS yoksa (defansif) hizalama fazına geç — eski davranış korunur."""
    nav = MagicMock()
    nav.position.return_value = _pos()
    sm = _make_sm(nav, None, None)

    sm._do_approach()

    assert sm.data.state == MissionState.ALIGNING_TARGET
    assert nav.set_max_speed.call_args[0][0] == ALIGN_SPEED_MPS


def test_approach_transit_speed_value_from_config():
    """APPROACH_SPEED_MPS > ALIGN_SPEED_MPS — iki aşamalı tasarım geçerli."""
    assert APPROACH_SPEED_MPS > ALIGN_SPEED_MPS
    assert APPROACH_RADIUS_M > 0
