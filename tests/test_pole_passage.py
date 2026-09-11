"""POLE PASSAGE testleri — görev alanına girmeden önce iki direğin arkasından geçiş.

Kapsam:
  1. Sıralama başlangıca göre OTOMATİK (config sırasına güvenilmez): yakın → uzak.
  2. Geçiş, direk hattından POLE_CLEARANCE_M (8 m) yanal offsetli "arka" şeritten
     yapılır — İHA çubukların ÜZERİNDEN değil YANINDAN geçer.
  3. Her iki direğe de minimum mesafe >= 8 m (tüm geçiş polyline'ı boyunca).
  4. SAFE_POINT, uzak direğin 8 m ötesinde ve hat ÜZERİNDE.
  5. Farklı başlangıç konumunda sıralama ve şerit doğru.
  6. Eksik / aynı-nokta direk → görev abort (mission_failed + TASK_2_FAILED).
  7. Vision gate: pole geçişi boyunca target_detector.detect ÇAĞRILMAZ.
  8. SAFE_POINT'e ulaşmadan SEARCH başlamaz.
  9. SAFE_POINT'e varışta mevcut search line'larından EN YAKIN olanı seçilir.
"""
import logging
import math
from unittest.mock import MagicMock

import numpy as np

import config
import state_machine as sm_mod
from flight.navigation import haversine_m
from models import MissionState
from search.coverage_planner import _north_east_m
from state_machine import MissionStateMachine, _point_segment_dist

# Görev başlangıç bölgesi (gerçek saha) — direklerin güney-batısı.
START_SW = (38.3594208, 38.2611720)
# Direklerin kuzey-doğusu (sıralama tersine dönmeli).
START_NE = (38.3615000, 38.2635000)

_POLE_ORIGIN = config.POLE_1     # local metre çerçevesi için sabit referans


def _make_sm(start_lat, start_lon):
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=9.0, latitude=start_lat, longitude=start_lon,
        heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)
    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.rtl_controller = MagicMock()   # gerçek MAVLink RTL çağrısını engelle
    return sm, nav


def _local(p):
    """GPS → (north, east) metre, POLE_1 referanslı."""
    return _north_east_m(_POLE_ORIGIN[0], _POLE_ORIGIN[1], p[0], p[1])


def _polyline_min_dist_to_pole(pole, waypoints):
    """Polyline'ın (waypoints) bir direğe en yakın mesafesi (metre)."""
    px, py = _local(pole)
    best = float("inf")
    for i in range(len(waypoints) - 1):
        x1, y1 = _local(waypoints[i])
        x2, y2 = _local(waypoints[i + 1])
        best = min(best, _point_segment_dist(px, py, x1, y1, x2, y2))
    return best


def _perp_side_of_pole_line(p):
    """p'nin direk hattına göre işaretli dik konumu (t)."""
    d1, d2 = config.POLE_1, config.POLE_2
    nn, ne = _north_east_m(d1[0], d1[1], d2[0], d2[1])
    length = math.hypot(nn, ne)
    u = (nn / length, ne / length)
    v = (-u[1], u[0])
    n2, e2 = _north_east_m(d1[0], d1[1], p[0], p[1])
    return n2 * v[0] + e2 * v[1]


def _area_centroid():
    cs = list(config.COMPETITION_CORNERS.values())
    return (sum(c[0] for c in cs) / len(cs), sum(c[1] for c in cs) / len(cs))


# ── 1) Sıralama + 8 m azami yaklaşma (ÜZERİNDEN değil YANINDAN) ───────────── #
def test_pole_passage_offset_lane_clears_poles():
    sm, _nav = _make_sm(*START_SW)
    assert sm._init_pole_passage() is True

    wps = sm._pole_waypoints
    assert len(wps) == 3
    # Tüm geçiş boyunca her iki direğe de en az 8 m mesafe (çarpışma yok).
    assert _polyline_min_dist_to_pole(config.POLE_1, wps) >= config.POLE_CLEARANCE_M - 0.05
    assert _polyline_min_dist_to_pole(config.POLE_2, wps) >= config.POLE_CLEARANCE_M - 0.05
    # İlk waypoint hattan 8 m yanda (direğin üstünde DEĞİL).
    assert abs(haversine_m(*config.POLE_1, *wps[0]) - config.POLE_CLEARANCE_M) < 0.05


def test_offset_on_back_side_away_from_area():
    """Offset, alanın OLMADIĞI tarafa (= arka) uygulanır."""
    sm, _nav = _make_sm(*START_SW)
    sm._init_pole_passage()
    t_wp = _perp_side_of_pole_line(sm._pole_waypoints[0])
    t_area = _perp_side_of_pole_line(_area_centroid())
    assert t_wp * t_area < 0, "arka şerit alan tarafında olmamalı"


# ── 2) SAFE_POINT hattın ucunda (uzak direğin 8 m ötesi, hat üzerinde) ─────── #
def test_safe_point_on_pole_line_beyond_far_pole():
    sm, _nav = _make_sm(*START_SW)
    sm._init_pole_passage()
    safe = sm._pole_safe_point
    far = config.POLE_2                      # START_SW'da uzak direk D2
    assert abs(haversine_m(*far, *safe) - config.POLE_CLEARANCE_M) < 0.05
    # Hattın üzerinde (dik offset yok) → near→far ile far→safe paralel.
    nn, ne = _north_east_m(*config.POLE_1, *config.POLE_2)
    fn, fe = _north_east_m(far[0], far[1], safe[0], safe[1])
    assert abs(nn * fe - ne * fn) < 0.05


# ── 3) Config sırası ters verilse bile sıralama doğru ─────────────────────── #
def test_pole_order_auto_when_config_reversed(monkeypatch):
    monkeypatch.setattr(sm_mod, "POLE_1", config.POLE_2)
    monkeypatch.setattr(sm_mod, "POLE_2", config.POLE_1)
    sm, _nav = _make_sm(*START_SW)
    assert sm._init_pole_passage() is True

    wps = sm._pole_waypoints
    # Yakın direk (fiziksel D1) girişte 8 m offsetli.
    assert abs(haversine_m(*config.POLE_1, *wps[0]) - config.POLE_CLEARANCE_M) < 0.05
    # Uzak direk (fiziksel D2) → SAFE onun 8 m ötesinde.
    assert abs(haversine_m(*config.POLE_2, *sm._pole_safe_point) - config.POLE_CLEARANCE_M) < 0.05
    assert _polyline_min_dist_to_pole(config.POLE_1, wps) >= config.POLE_CLEARANCE_M - 0.05
    assert _polyline_min_dist_to_pole(config.POLE_2, wps) >= config.POLE_CLEARANCE_M - 0.05


# ── 4) Farklı başlangıç konumu → ters sıralama ────────────────────────────── #
def test_pole_order_from_other_side():
    sm, _nav = _make_sm(*START_NE)
    assert sm._init_pole_passage() is True

    wps = sm._pole_waypoints
    # START_NE'de yakın direk D2, uzak D1.
    assert abs(haversine_m(*config.POLE_2, *wps[0]) - config.POLE_CLEARANCE_M) < 0.05
    assert abs(haversine_m(*config.POLE_1, *sm._pole_safe_point) - config.POLE_CLEARANCE_M) < 0.05
    assert _polyline_min_dist_to_pole(config.POLE_1, wps) >= config.POLE_CLEARANCE_M - 0.05
    assert _polyline_min_dist_to_pole(config.POLE_2, wps) >= config.POLE_CLEARANCE_M - 0.05


# ── 5) Eksik / geçersiz direk → abort ─────────────────────────────────────── #
def test_missing_pole_aborts(monkeypatch):
    monkeypatch.setattr(sm_mod, "POLE_1", None)
    sm, _nav = _make_sm(*START_SW)
    sm._do_pole_passage()
    assert sm.data.mission_failed is True
    assert sm.data.state == MissionState.TASK_2_FAILED


def test_identical_poles_abort(monkeypatch):
    monkeypatch.setattr(sm_mod, "POLE_2", config.POLE_1)
    sm, _nav = _make_sm(*START_SW)
    sm._do_pole_passage()
    assert sm.data.mission_failed is True
    assert sm.data.state == MissionState.TASK_2_FAILED


# ── 6) Vision gate: pole geçişinde hedef tespiti çalışmaz ─────────────────── #
def test_vision_gate_no_detection_during_pole_passage():
    sm, nav = _make_sm(*START_SW)
    sm.target_detector.detect = MagicMock()
    sm.data.state = MissionState.POLE_PASSAGE

    for _ in range(5):
        sm._do_pole_passage()

    sm.target_detector.detect.assert_not_called()
    assert sm.data.state == MissionState.POLE_PASSAGE
    nav.goto_global.assert_called()


# ── 7) SAFE_POINT'e ulaşmadan SEARCH başlamaz ─────────────────────────────── #
def test_search_not_started_before_safe_point():
    sm, _nav = _make_sm(*START_SW)
    sm.data.state = MissionState.POLE_PASSAGE
    for _ in range(5):
        sm._do_pole_passage()
    assert sm.data.state == MissionState.POLE_PASSAGE
    assert sm._search_points == []          # planner hiç kurulmadı


# ── 8) SAFE_POINT'e varışta en yakın search line seçilir ──────────────────── #
def test_finish_selects_nearest_search_line():
    sm, nav = _make_sm(*START_SW)
    sm._init_pole_passage()
    safe = sm._pole_safe_point
    assert safe is not None

    # Tüm waypoint'ler tamamlanmış say → finish tetiklenir.
    sm._pole_idx = len(sm._pole_waypoints)
    nav.position.return_value = MagicMock(
        relative_altitude=9.0, latitude=safe[0], longitude=safe[1], heading_deg=0.0,
    )
    sm._do_pole_passage()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    # Mevcut 3-line geometrisi korunur → 6 waypoint.
    assert len(sm._search_points) == 6
    assert sm._search_idx % 2 == 0

    # Seçilen line, SAFE_POINT'e gerçekten en yakın olan mı?
    px, py = sm._search_field.gps_to_local(*safe)
    pts = sm._search_points
    dists = [
        _point_segment_dist(px, py, pts[i].x_m, pts[i].y_m,
                            pts[i + 1].x_m, pts[i + 1].y_m)
        for i in range(0, len(pts) - 1, 2)
    ]
    assert sm._search_idx == 2 * dists.index(min(dists))
