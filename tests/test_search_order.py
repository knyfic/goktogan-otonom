"""Search sırası testleri.

Yeni sıra: ÖNCE ORTA line → home'a UZAK kenar line → home'a YAKIN kenar line.
(coverage planner geometrisi DEĞİŞMEZ; yalnızca line traversal sırası.)
Yönler yeni sıraya göre ZİG-ZAG (↔) — İHA yaw yapmadan ilerler/geri döner.
"""
import logging
from unittest.mock import MagicMock

from config import COMPETITION_CORNERS, SEARCH_FOV_WIDTH_M
from search.coverage_planner import CompetitionField
from state_machine import MissionStateMachine


def _sm(home_lat, home_lon):
    nav = MagicMock()
    nav.home_lat = home_lat
    nav.home_lon = home_lon
    nav.position.return_value = MagicMock(
        latitude=home_lat, longitude=home_lon, relative_altitude=9.0, heading_deg=0.0,
    )
    nav.fresh_position.return_value = MagicMock(latitude=home_lat, longitude=home_lon)
    sm = MissionStateMachine(
        navigation=nav, camera=MagicMock(), payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    return sm, nav


def _eqp(px, py, q) -> bool:
    return abs(px - q[0]) < 0.5 and abs(py - q[1]) < 0.5


def _line_order(sm):
    """_search_points'i line'lara eşleyip transversal sıra indekslerini döndür."""
    field = CompetitionField(COMPETITION_CORNERS)
    lines = field.search_lines_local(SEARCH_FOV_WIDTH_M, 3)
    order = []
    pts = sm._search_points
    for i in range(0, len(pts), 2):
        p0, p1 = pts[i], pts[i + 1]
        for li, (s, e) in enumerate(lines):
            if (_eqp(p0.x_m, p0.y_m, s) and _eqp(p1.x_m, p1.y_m, e)) or \
               (_eqp(p0.x_m, p0.y_m, e) and _eqp(p1.x_m, p1.y_m, s)):
                order.append(li)
                break
    return order


def test_order_middle_first_home_west():
    """Home batıda → kenar line0 (batı) YAKIN, line2 (doğu) UZAK."""
    field = CompetitionField(COMPETITION_CORNERS)
    hl, ho = field.local_to_gps(-30.0, field.length_m / 2.0)   # alanın batısı
    sm, _nav = _sm(hl, ho)
    sm._init_search()

    assert len(sm._search_points) == 6
    assert _line_order(sm) == [1, 2, 0]          # orta → uzak → yakın
    assert sm._search_idx == 0


def test_order_middle_first_home_east():
    """Home doğuda → kenar line2 YAKIN, line0 UZAK."""
    field = CompetitionField(COMPETITION_CORNERS)
    hl, ho = field.local_to_gps(field.width_m + 30.0, field.length_m / 2.0)
    sm, _nav = _sm(hl, ho)
    sm._init_search()

    assert _line_order(sm) == [1, 0, 2]          # orta → uzak → yakın


def test_order_directions_alternate_zigzag():
    """Yeni sırada ardışık line'ların yönleri TERS (↔) — yaw değişimi yok."""
    field = CompetitionField(COMPETITION_CORNERS)
    hl, ho = field.local_to_gps(-30.0, field.length_m / 2.0)
    sm, _nav = _sm(hl, ho)
    sm._init_search()

    pts = sm._search_points
    # Uzun kenar doğrultusundaki (y) hareket işaretleri ardışık ters olmalı.
    d0 = pts[1].y_m - pts[0].y_m
    d1 = pts[3].y_m - pts[2].y_m
    d2 = pts[5].y_m - pts[4].y_m
    assert d0 * d1 < 0 and d1 * d2 < 0


def test_order_uses_reference_when_home_missing():
    """home yoksa referans (launch) kullanılır; yine ORTA line başta."""
    field = CompetitionField(COMPETITION_CORNERS)
    rl, ro = field.local_to_gps(-30.0, field.length_m / 2.0)
    sm, _nav = _sm(None, None)                   # home yok
    sm.nav.fresh_position.return_value = MagicMock(latitude=rl, longitude=ro)
    sm._init_search()

    assert _line_order(sm)[0] == 1               # orta line ilk
