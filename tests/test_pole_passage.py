"""POLE PASSAGE testleri — görev alanına girmeden önce iki direğin arkasından geçiş.

Kapsam:
  1. Sıralama başlangıca göre OTOMATİK (config sırasına güvenilmez): yakın → uzak.
  2. SAFE_POINT = uzak direk + POLE_CLEARANCE_M (8 m), hat doğrultusunda.
  3. Farklı başlangıç konumunda sıralama ve SAFE_POINT doğru.
  4. Eksik / aynı-nokta direk → görev abort (mission_failed + TASK_2_FAILED).
  5. Vision gate: pole geçişi boyunca target_detector.detect ÇAĞRILMAZ.
  6. SAFE_POINT'e ulaşmadan SEARCH başlamaz.
  7. SAFE_POINT'e varışta mevcut search line'larından EN YAKIN olanı seçilir.
"""
import logging
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


# ── 1) Başlangıca göre otomatik sıralama + 8 m ────────────────────────────── #
def test_pole_passage_order_and_clearance():
    sm, _nav = _make_sm(*START_SW)
    assert sm._init_pole_passage() is True

    near, far, safe_pt = sm._pole_waypoints
    # START_SW'da yakın direk POLE_1 (D1), uzak direk POLE_2 (D2).
    assert near == config.POLE_1
    assert far == config.POLE_2
    # SAFE_POINT, uzak direkten POLE_CLEARANCE_M kadar ötede (yatay).
    assert abs(haversine_m(*far, *safe_pt) - config.POLE_CLEARANCE_M) < 0.05


def test_safe_point_collinear_with_pole_line():
    """SAFE_POINT, iki direği birleştiren doğru ÜZERİNDE (hatta dik offset YOK)."""
    sm, _nav = _make_sm(*START_SW)
    sm._init_pole_passage()
    near, far, safe_pt = sm._pole_waypoints

    nn, ne = _north_east_m(near[0], near[1], far[0], far[1])
    fn, fe = _north_east_m(far[0], far[1], safe_pt[0], safe_pt[1])
    cross = nn * fe - ne * fn          # iki vektör paralel → ~0
    assert abs(cross) < 0.05


# ── 2) Config sırası ters verilse bile sıralama doğru ─────────────────────── #
def test_pole_order_auto_when_config_reversed(monkeypatch):
    monkeypatch.setattr(sm_mod, "POLE_1", config.POLE_2)
    monkeypatch.setattr(sm_mod, "POLE_2", config.POLE_1)
    sm, _nav = _make_sm(*START_SW)
    assert sm._init_pole_passage() is True

    near, far, safe_pt = sm._pole_waypoints
    # Config ters olsa da yakın/uzak başlangıca göre belirlenir.
    assert near == config.POLE_1
    assert far == config.POLE_2
    assert abs(haversine_m(*far, *safe_pt) - config.POLE_CLEARANCE_M) < 0.05


# ── 3) Farklı başlangıç konumu → ters sıralama ────────────────────────────── #
def test_pole_order_from_other_side():
    sm, _nav = _make_sm(*START_NE)
    assert sm._init_pole_passage() is True

    near, far, safe_pt = sm._pole_waypoints
    assert near == config.POLE_2
    assert far == config.POLE_1
    assert abs(haversine_m(*far, *safe_pt) - config.POLE_CLEARANCE_M) < 0.05


# ── 4) Eksik / geçersiz direk → abort ─────────────────────────────────────── #
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


# ── 5) Vision gate: pole geçişinde hedef tespiti çalışmaz ─────────────────── #
def test_vision_gate_no_detection_during_pole_passage():
    sm, nav = _make_sm(*START_SW)
    sm.target_detector.detect = MagicMock()
    sm.data.state = MissionState.POLE_PASSAGE

    for _ in range(5):
        sm._do_pole_passage()

    sm.target_detector.detect.assert_not_called()
    assert sm.data.state == MissionState.POLE_PASSAGE
    nav.goto_global.assert_called()


# ── 6) SAFE_POINT'e ulaşmadan SEARCH başlamaz ─────────────────────────────── #
def test_search_not_started_before_safe_point():
    sm, _nav = _make_sm(*START_SW)
    sm.data.state = MissionState.POLE_PASSAGE
    for _ in range(5):
        sm._do_pole_passage()
    assert sm.data.state == MissionState.POLE_PASSAGE
    assert sm._search_points == []          # planner hiç kurulmadı


# ── 7) SAFE_POINT'e varışta en yakın search line seçilir ──────────────────── #
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
