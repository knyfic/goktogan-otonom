"""
Batarya voltajı — MAVLink telemetrisi → HUD görüntüsü.

1. BATTERY_STATUS hücre voltajları toplamı → toplam voltaj (models.VehiclePosition).
2. SYS_STATUS battery_remaining → yüzde (fallback).
3. _hud_lines üzerinden canlı gösterim (görüntü işleme penceresi üst bandı).
"""
import logging
from unittest.mock import MagicMock

import pytest


class _FakeMsg:
    def __init__(self, msg_type, **kw):
        self._type = msg_type
        for k, v in kw.items():
            setattr(self, k, v)

    def get_type(self):
        return self._type


def _nav():
    from flight.navigation import Navigation
    from models import VehiclePosition

    nav = object.__new__(Navigation)
    nav._position = VehiclePosition()
    nav._last_message_s = 0.0
    nav.home_lat = None
    nav.home_lon = None
    return nav


# ── BATTERY_STATUS → toplam voltaj ────────────────────────────────────────── #
def test_battery_status_sums_cell_voltages():
    nav = _nav()
    msg = _FakeMsg(
        "BATTERY_STATUS",
        voltages=(4200, 4200, 4200, 4200, 4200, 4200,
                  65535, 65535, 65535, 65535),
        battery_remaining=85,
    )
    nav._dispatch_telemetry(msg)
    assert nav._position.battery_voltage_v == pytest.approx(25.2)
    assert nav._position.battery_percent == 85


def test_battery_status_unused_cells_ignored():
    nav = _nav()
    msg = _FakeMsg(
        "BATTERY_STATUS",
        voltages=(4100, 4100, 0, 65535, 65535, 65535,
                  65535, 65535, 65535, 65535),
        battery_remaining=-1,
    )
    nav._dispatch_telemetry(msg)
    assert nav._position.battery_voltage_v == pytest.approx(8.2)
    assert nav._position.battery_percent == -1


def test_battery_status_no_data_keeps_unknown():
    nav = _nav()
    msg = _FakeMsg(
        "BATTERY_STATUS",
        voltages=(65535, 65535, 65535, 65535, 65535, 65535,
                  65535, 65535, 65535, 65535),
        battery_remaining=-1,
    )
    nav._dispatch_telemetry(msg)
    assert nav._position.battery_voltage_v == 0.0


# ── SYS_STATUS → yüzde fallback ───────────────────────────────────────────── #
def test_sys_status_updates_percent():
    nav = _nav()
    msg = _FakeMsg("SYS_STATUS", battery_remaining=72)
    nav._dispatch_telemetry(msg)
    assert nav._position.battery_percent == 72
    assert nav._position.battery_voltage_v == 0.0


# ── position() telemetriyi canlı işler ────────────────────────────────────── #
def test_position_drains_battery_telemetry():
    nav = _nav()
    batt = _FakeMsg(
        "BATTERY_STATUS",
        voltages=(4000, 4000, 4000, 65535, 65535, 65535,
                  65535, 65535, 65535, 65535),
        battery_remaining=90,
    )
    nav.master = MagicMock()
    nav.master.recv_match.side_effect = [batt, None]
    nav.position()
    assert nav._position.battery_voltage_v == pytest.approx(12.0)
    assert nav._position.battery_percent == 90


# ── HUD gösterimi ─────────────────────────────────────────────────────────── #
def _hud_sm(battery_v, battery_pct):
    from state_machine import MissionStateMachine

    sm = object.__new__(MissionStateMachine)
    sm.log = logging.getLogger("t")
    sm.data = MagicMock()
    sm.data.state.value = "SEARCHING_FIRST_TARGET"
    sm.nav = MagicMock()
    sm.nav.position.return_value = MagicMock(
        relative_altitude=8.0, heading_deg=90.0,
        latitude=41.0, longitude=28.9,
        battery_voltage_v=battery_v, battery_percent=battery_pct,
    )
    sm._mission_start_t = None
    sm._mission_timeout_triggered = False
    return sm


def test_hud_line_shows_battery_voltage():
    sm = _hud_sm(16.8, 85)
    _l1, l2 = sm._hud_lines()
    assert "Bat 16.8V" in l2
    assert "85%" in l2


def test_hud_line_unknown_battery():
    sm = _hud_sm(0.0, -1)
    _l1, l2 = sm._hud_lines()
    assert "Bat --.-V" in l2
