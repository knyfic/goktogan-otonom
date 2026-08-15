"""
CompetitionField — 4 GPS köşesinden SABİT local frame + dinamik launch search.

Gereksinimler:
  - Yarışma alanı GPS köşeleriyle (A,B,C,D — 30×100 m) tanımlanır.
  - İHA'nın runtime GPS konumu competition frame'e çevrilir (launch'a göre
    kod değişmez; alan dünya sabitidir).
  - Lawnmower pattern korunur; rota İHA'ya en yakın hattan başlar.
  - Tüm waypoint'ler alan içinde kalır; alan dışı launch reddedilir.
  - Alan orientation'ı kuzeye hizalı olmak zorunda değildir (0°/30°/45°/90°).
"""
import logging
import math
import random
from unittest.mock import MagicMock

import pytest

from config import SEARCH_MARGIN, SCAN_LINE_SPACING
from search.coverage_planner import (
    CompetitionField,
    LawnmowerPlanner,
    SearchPoint,
    reorder_search_from_drone,
)

_ORIGIN = (47.0, 8.0)


def _gps_from_origin(o_lat, o_lon, north_m, east_m):
    """Kuzey/doğu metre → GPS (coverage_planner.local_to_gps ile aynı model)."""
    radius = 6378137.0
    d_lat = north_m / radius
    d_lon = east_m / (radius * math.cos(math.radians(o_lat)))
    return o_lat + math.degrees(d_lat), o_lon + math.degrees(d_lon)


def _field(width=30.0, length=100.0, theta_deg=0.0, origin=_ORIGIN):
    """
    θ dönüşlü dikdörtgen köşeleri üretir.

    A = origin (alt-sol kavramsal). B = A + length·(cosθ, sinθ) kuzey/doğu,
    D = A + width·(−sinθ, cosθ) — yani B yönünün 90° saat yönünde diki.
    θ = 0 → kuzeye hizalı alan (B kuzeyde, D doğuda).
    """
    o_lat, o_lon = origin
    th = math.radians(theta_deg)
    b = _gps_from_origin(o_lat, o_lon,
                         length * math.cos(th), length * math.sin(th))
    d = _gps_from_origin(o_lat, o_lon,
                         -width * math.sin(th), width * math.cos(th))
    c = _gps_from_origin(
        o_lat, o_lon,
        length * math.cos(th) - width * math.sin(th),
        length * math.sin(th) + width * math.cos(th),
    )
    return CompetitionField({"A": origin, "B": b, "C": c, "D": d})


def _route(field, lat, lon, spacing=SCAN_LINE_SPACING, margin=SEARCH_MARGIN):
    """Planner + competition kaydırma + en yakın hattan başlatma (state machine ile aynı)."""
    planner = LawnmowerPlanner(field.width_m, field.length_m, spacing, margin)
    points = [SearchPoint(p.x_m + field.width_m / 2.0, p.y_m)
              for p in planner.generate()]
    lx, ly = field.gps_to_local(lat, lon)
    return reorder_search_from_drone(points, lx, ly)


# ── 1) Köşeler → local frame ─────────────────────────────────────────────── #
def test_corner_frame():
    f = _field(30.0, 100.0, 0.0)
    assert f.width_m == pytest.approx(30.0, abs=0.1)
    assert f.length_m == pytest.approx(100.0, abs=0.1)

    assert f.gps_to_local(*_ORIGIN) == pytest.approx((0.0, 0.0), abs=0.1)      # A
    b = _gps_from_origin(*_ORIGIN, 100.0, 0.0)
    assert f.gps_to_local(*b) == pytest.approx((0.0, 100.0), abs=0.1)          # B
    c = _gps_from_origin(*_ORIGIN, 100.0, 30.0)
    assert f.gps_to_local(*c) == pytest.approx((30.0, 100.0), abs=0.1)         # C
    d = _gps_from_origin(*_ORIGIN, 0.0, 30.0)
    assert f.gps_to_local(*d) == pytest.approx((30.0, 0.0), abs=0.1)           # D

    # local → GPS → local yuvarlak dönüşüm
    lat, lon = f.local_to_gps(15.0, 50.0)
    assert f.gps_to_local(lat, lon) == pytest.approx((15.0, 50.0), abs=0.1)


# ── 2) Ortadaki launch ────────────────────────────────────────────────────── #
def test_center_launch():
    f = _field(30.0, 100.0, 0.0)
    gps = f.local_to_gps(15.0, 50.0)
    lx, ly = f.gps_to_local(*gps)
    assert (lx, ly) == pytest.approx((15.0, 50.0), abs=0.1)
    assert f.contains(*gps)


# ── 3) Farklı launch noktaları — aynı kod adapte olur ─────────────────────── #
def test_various_launch_points():
    f = _field(30.0, 100.0, 0.0)
    planner = LawnmowerPlanner(f.width_m, f.length_m, SCAN_LINE_SPACING, SEARCH_MARGIN)
    expected = {(p.x_m + f.width_m / 2.0, p.y_m) for p in planner.generate()}

    for dx, dy in [(2, 5), (28, 5), (15, 50), (5, 90), (27, 95)]:
        gps = f.local_to_gps(dx, dy)
        route = _route(f, *gps)
        assert len(route) == len(expected), f"launch ({dx},{dy}) rota eksik"
        assert {(p.x_m, p.y_m) for p in route} == expected, \
            f"launch ({dx},{dy}) rota tüm noktaları içermeli"
        for p in route:
            assert f.contains_local(p.x_m, p.y_m)


# ── 4) 1000 rastgele launch — tüm waypoint'ler içeride ────────────────────── #
def test_1000_random_launches_all_inside():
    rng = random.Random(1234)
    f = _field(30.0, 100.0, 0.0)
    for _ in range(1000):
        dx = rng.uniform(0.01, 29.99)
        dy = rng.uniform(0.01, 99.99)
        gps = f.local_to_gps(dx, dy)
        route = _route(f, *gps)
        assert len(route) > 0
        for p in route:
            assert f.contains_local(p.x_m, p.y_m), f"waypoint dışarıda: {p}"


# ── 5) Kenar launch ───────────────────────────────────────────────────────── #
def test_edge_launch():
    f = _field(30.0, 100.0, 0.0)
    margin = SEARCH_MARGIN
    for dx, dy in [(margin + 0.01, 50.0),
                   (30.0 - margin - 0.01, 50.0),
                   (15.0, margin + 0.01),
                   (15.0, 100.0 - margin - 0.01)]:
        gps = f.local_to_gps(dx, dy)
        assert f.contains(*gps), f"kenar launch ({dx},{dy}) alan içinde olmalı"
        for p in _route(f, *gps):
            assert f.contains_local(p.x_m, p.y_m)


# ── 6) Köşe launch ────────────────────────────────────────────────────────── #
def test_corner_launches():
    f = _field(30.0, 100.0, 0.0)
    for dx, dy in [(0.1, 0.1), (29.9, 0.1), (0.1, 99.9), (29.9, 99.9)]:
        gps = f.local_to_gps(dx, dy)
        assert f.contains(*gps)
        for p in _route(f, *gps):
            assert f.contains_local(p.x_m, p.y_m), f"waypoint dışarıda: {p}"


# ── 7) Alan dışı launch reddedilir ────────────────────────────────────────── #
def test_outside_launch_rejected():
    f = _field(30.0, 100.0, 0.0)
    for dx, dy in [(-5.0, 50.0), (35.0, 50.0), (15.0, -5.0), (15.0, 105.0)]:
        gps = f.local_to_gps(dx, dy)
        assert not f.contains(*gps), f"alan dışı ({dx},{dy}) reddedilmeli"


# ── 8) Farklı alan orientation'ları ───────────────────────────────────────── #
@pytest.mark.parametrize("theta", [0.0, 30.0, 45.0, 90.0])
def test_rotated_fields(theta):
    f = _field(30.0, 100.0, theta)
    assert f.width_m == pytest.approx(30.0, abs=0.2)
    assert f.length_m == pytest.approx(100.0, abs=0.2)

    th = math.radians(theta)
    assert f.gps_to_local(*_ORIGIN) == pytest.approx((0.0, 0.0), abs=0.2)
    b = _gps_from_origin(*_ORIGIN, 100.0 * math.cos(th), 100.0 * math.sin(th))
    assert f.gps_to_local(*b) == pytest.approx((0.0, 100.0), abs=0.2)
    d = _gps_from_origin(*_ORIGIN, -30.0 * math.sin(th), 30.0 * math.cos(th))
    assert f.gps_to_local(*d) == pytest.approx((30.0, 0.0), abs=0.2)

    lat, lon = f.local_to_gps(15.0, 50.0)
    assert f.gps_to_local(lat, lon) == pytest.approx((15.0, 50.0), abs=0.2)

    # Döndürülmüş alanda da tüm rotalar içeride
    rng = random.Random(7)
    for _ in range(50):
        dx = rng.uniform(0.1, 29.9)
        dy = rng.uniform(0.1, 99.9)
        gps = f.local_to_gps(dx, dy)
        for p in _route(f, *gps):
            assert f.contains_local(p.x_m, p.y_m)


# ── State machine entegrasyonu ────────────────────────────────────────────── #
def test_init_search_outside_area_goes_rtl():
    from models import MissionState
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(latitude=48.0, longitude=8.0)  # alan dışı
    sm = MissionStateMachine(navigation=nav, camera=MagicMock(), payload=MagicMock(),
                             logger=logging.getLogger("t"))
    sm._init_search()
    assert sm.data.mission_failed is True
    assert sm.data.state == MissionState.RTL
    assert sm._search_field is None


def test_init_search_inside_area_builds_route():
    from config import COMPETITION_CORNERS
    from state_machine import MissionStateMachine

    a = COMPETITION_CORNERS["A"]
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(latitude=float(a[0]), longitude=float(a[1]))
    sm = MissionStateMachine(navigation=nav, camera=MagicMock(), payload=MagicMock(),
                             logger=logging.getLogger("t"))
    sm._init_search()
    assert sm._search_field is not None
    assert len(sm._search_points) > 0
    for p in sm._search_points:
        assert sm._search_field.contains_local(p.x_m, p.y_m), \
            f"waypoint alan dışında: ({p.x_m:.1f}, {p.y_m:.1f})"


def test_advance_search_uses_field_gps():
    """_advance_search, waypoint'i competition frame'den GPS'e çevirir ve
    arama irtifasını (TAKEOFF_ALTITUDE) iletir."""
    from config import TAKEOFF_ALTITUDE
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=_ORIGIN[0], longitude=_ORIGIN[1], heading_deg=0.0, relative_altitude=8.0,
    )
    sm = MissionStateMachine(navigation=nav, camera=MagicMock(), payload=MagicMock(),
                             logger=logging.getLogger("t"))
    # Alan içi basit bir rota kur
    from search.coverage_planner import CompetitionField
    f = _field(30.0, 100.0, 0.0)
    sm._search_field = f
    sm._search_points = [SearchPoint(5.0, 5.0), SearchPoint(5.0, 95.0)]
    sm._search_idx = 0

    sm._advance_search()

    expected_lat, expected_lon = f.local_to_gps(5.0, 5.0)
    nav.goto_global.assert_called_once()
    args = nav.goto_global.call_args[0]
    kwargs = nav.goto_global.call_args[1]
    assert args[0] == pytest.approx(expected_lat, abs=1e-6)
    assert args[1] == pytest.approx(expected_lon, abs=1e-6)
    # Arama irtifası = TAKEOFF_ALTITUDE (11.5m); yaw kurulmamışsa gönderilmez
    assert args[2] == pytest.approx(TAKEOFF_ALTITUDE)
    assert kwargs.get("yaw_deg") is None


# ── 9) SEARCH_SPEED_MPS config'ten gelir; yalnızca search'te geçerli ──────── #
def _make_sm(nav):
    from state_machine import MissionStateMachine

    return MissionStateMachine(navigation=nav, camera=MagicMock(),
                               payload=MagicMock(),
                               logger=logging.getLogger("t"))


def test_search_speed_config_drives_nav(monkeypatch):
    """SEARCH_SPEED_MPS değişince set_max_speed de değişmeli (8.0 → 12.0)."""
    import state_machine as sm_mod
    from config import COMPETITION_CORNERS, VERTICAL_SPEED_MPS

    a = COMPETITION_CORNERS["A"]
    for speed in (8.0, 12.0):
        monkeypatch.setattr(sm_mod, "SEARCH_SPEED_MPS", speed)
        nav = MagicMock()
        nav.fresh_position.return_value = MagicMock(
            latitude=float(a[0]), longitude=float(a[1]),
        )
        sm = _make_sm(nav)
        sm._init_search()
        assert nav.set_max_speed.call_count == 1
        args = nav.set_max_speed.call_args[0]
        assert args[0] == speed, \
            f"SEARCH_SPEED_MPS={speed} set_max_speed'e yansımalı"
        assert args[1] == VERTICAL_SPEED_MPS
        assert sm._search_points, "search rotası kurulmalı"


def test_search_speed_not_applied_before_search():
    """Search hızı yalnızca _init_search içinde uygulanır — takeoff öncesi yok."""
    nav = MagicMock()
    sm = _make_sm(nav)
    assert nav.set_max_speed.call_count == 0, \
        "kurulumda search hızı uygulanmamalı"


# ── 10) İlk search — takeoff referansı + TEK GEÇİŞ (kırmızı üçgen) ────────── #
def test_first_search_uses_takeoff_reference():
    """İlk arama (kırmızı üçgen): launch referansı kullanılır; rota kısa kenar
    ORTASINDAN karşı kısa kenara uzun eksen boyunca TEK GEÇİŞTİR (11.5m)."""
    from config import COMPETITION_CORNERS, TAKEOFF_ALTITUDE

    field = CompetitionField(COMPETITION_CORNERS)
    mid_x = field.width_m / 2.0
    length = field.length_m
    launch_lat, launch_lon = field.local_to_gps(mid_x, 50.0)
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon,
    )
    sm = _make_sm(nav)
    sm._init_search()
    # Referans = takeoff GPS
    assert sm._search_origin_lat == pytest.approx(launch_lat, abs=1e-9)
    assert sm._search_origin_lon == pytest.approx(launch_lon, abs=1e-9)
    # TEK GEÇİŞ rotası: (orta, 0) → (orta, L) — kısa kenar ortasından uzun eksen
    assert [(p.x_m, p.y_m) for p in sm._search_points] == \
        [(mid_x, 0.0), (mid_x, length)]
    # Arama irtifası = TAKEOFF_ALTITUDE (11.5m)
    assert nav.set_max_speed.call_count == 1
    # Yaw: uzun eksen yönü — SITL alanı kuzeye hizalı olduğundan 0° (kısa kenara dik)
    assert sm._search_yaw_deg == pytest.approx(0.0)


def test_first_search_launch_in_upper_half_yaw_180():
    """Launch uzun eksenin üst yarısındaysa EN YAKIN kısa kenar üst kenardır;
    yaw 180° çevrilir (uzak kenara bakacak şekilde)."""
    from config import COMPETITION_CORNERS

    field = CompetitionField(COMPETITION_CORNERS)
    mid_x = field.width_m / 2.0
    length = field.length_m
    launch_lat, launch_lon = field.local_to_gps(10.0, length * 0.8)   # üst yarı (y=80)
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon,
    )
    sm = _make_sm(nav)
    sm._init_search()
    # y=80 > L/2 → en yakın kısa kenar ÜST kenar (y=L) → başlangıç (orta, L)
    assert [(p.x_m, p.y_m) for p in sm._search_points] == \
        [(mid_x, length), (mid_x, 0.0)]
    assert sm._search_yaw_deg == pytest.approx(180.0)


def test_advance_search_sends_yaw():
    """Tek geçiş yaw hedefi (kısa kenara dik) goto_global'e iletilir."""
    nav = MagicMock()
    nav.position.return_value = MagicMock(
        latitude=_ORIGIN[0], longitude=_ORIGIN[1], heading_deg=0.0, relative_altitude=8.0,
    )
    sm = _make_sm(nav)
    f = _field(30.0, 100.0, 0.0)
    sm._search_field = f
    sm._search_points = [SearchPoint(15.0, 5.0), SearchPoint(15.0, 95.0)]
    sm._search_idx = 0
    sm._search_yaw_deg = 0.0   # SITL alanı kuzeye hizalı → 0°

    sm._advance_search()

    kwargs = nav.goto_global.call_args[1]
    assert kwargs.get("yaw_deg") == 0.0


# ── 11) İkinci search — ilk payload drop referansı ────────────────────────── #
def _enter_second_search(sm, drop_lat, drop_lon, nav):
    """İrtifa beklemesini tamamlanmış sayıp _do_ascend_after_first_release çalıştır."""
    from config import TAKEOFF_ALTITUDE

    sm._alt_waiting = True
    sm._alt_target = TAKEOFF_ALTITUDE
    nav.position.return_value = MagicMock(
        relative_altitude=TAKEOFF_ALTITUDE, latitude=drop_lat, longitude=drop_lon,
    )
    sm._do_ascend_after_first_release()


def test_second_search_uses_first_drop_reference():
    """İkinci arama (mavi altıgen): DROP konumu referans alınır; aynı TEK GEÇİŞ
    geometrisi kurulur (en yakın kısa kenar ortası → karşı kısa kenar)."""
    from config import COMPETITION_CORNERS
    from models import MissionState, Target, TargetColor, TargetType

    field = CompetitionField(COMPETITION_CORNERS)
    mid_x = field.width_m / 2.0
    length = field.length_m
    takeoff_lat, takeoff_lon = field.local_to_gps(15.0, 50.0)
    drop_lat, drop_lon = field.local_to_gps(27.0, 90.0)

    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=takeoff_lat, longitude=takeoff_lon,
    )
    sm = _make_sm(nav)
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=drop_lat, longitude=drop_lon, confidence=0.9,
    )
    _enter_second_search(sm, drop_lat, drop_lon, nav)

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    # Referans DROP konumu olmalı — TAKEOFF değil
    assert sm._search_origin_lat == pytest.approx(drop_lat, abs=1e-9)
    assert sm._search_origin_lon == pytest.approx(drop_lon, abs=1e-9)
    assert sm._search_origin_lat != pytest.approx(takeoff_lat, abs=1e-9)
    assert sm._search_origin_lon != pytest.approx(takeoff_lon, abs=1e-9)
    # İkinci arama da TEK GEÇİŞ: drop (27,90) üst yarıda → (orta, L) → (orta, 0)
    assert [(p.x_m, p.y_m) for p in sm._search_points] == \
        [(mid_x, length), (mid_x, 0.0)]
    assert sm._search_yaw_deg == pytest.approx(180.0)


@pytest.mark.parametrize("dx,dy", [(2.0, 5.0), (15.0, 50.0), (28.0, 95.0)])
def test_second_search_different_drop_positions(dx, dy):
    """Farklı drop konumları → en yakın kısa kenarın ORTASINDAN TEK GEÇİŞ."""
    from config import COMPETITION_CORNERS
    from models import MissionState, Target, TargetColor, TargetType

    field = CompetitionField(COMPETITION_CORNERS)
    mid_x = field.width_m / 2.0
    length = field.length_m
    drop_lat, drop_lon = field.local_to_gps(dx, dy)

    nav = MagicMock()
    sm = _make_sm(nav)
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=drop_lat, longitude=drop_lon, confidence=0.9,
    )
    _enter_second_search(sm, drop_lat, drop_lon, nav)

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    assert sm._search_origin_lat == pytest.approx(drop_lat, abs=1e-9)
    assert sm._search_origin_lon == pytest.approx(drop_lon, abs=1e-9)
    # En yakın kısa kenarın ORTASI → karşı kısa kenar (tek geçiş)
    near_edge_y = 0.0 if dy <= length / 2.0 else length
    assert [(p.x_m, p.y_m) for p in sm._search_points] == \
        [(mid_x, near_edge_y), (mid_x, length - near_edge_y)]
    # Tüm waypoint'ler yarışma alanı içinde kalmalı
    for p in sm._search_points:
        assert field.contains_local(p.x_m, p.y_m), \
            f"ikinci search waypoint'i alan dışında: ({p.x_m:.1f}, {p.y_m:.1f})"


# ── 12) Heading bağımsızlığı ──────────────────────────────────────────────── #
def test_search_route_heading_independent():
    """İHA heading'i değişse bile search rotası aynı kalmalı (dünya sabiti frame)."""
    from config import COMPETITION_CORNERS

    field = CompetitionField(COMPETITION_CORNERS)
    ref_lat, ref_lon = field.local_to_gps(15.0, 50.0)

    routes = []
    for heading in (0.0, 90.0, 180.0, 270.0):
        nav = MagicMock()
        nav.fresh_position.return_value = MagicMock(
            latitude=ref_lat, longitude=ref_lon, heading_deg=heading,
        )
        sm = _make_sm(nav)
        sm._init_search()
        routes.append([(p.x_m, p.y_m) for p in sm._search_points])

    assert routes[0] == routes[1] == routes[2] == routes[3]


# ── 13) İlk drop konumu yoksa sessizce yanlış başlangıç yapma ─────────────── #
def test_second_search_missing_drop_goes_rtl():
    from models import MissionState

    nav = MagicMock()
    sm = _make_sm(nav)
    sm.data.first_target = None
    _enter_second_search(sm, 0.0, 0.0, nav)

    assert sm.data.mission_failed is True
    assert sm.data.state == MissionState.RTL
    assert sm._search_points == []
    assert sm._search_field is None


# ── 14) Search hızı her search girişinde aynı ─────────────────────────────── #
def test_first_and_second_search_use_same_speed(monkeypatch):
    """SEARCH #1 ve SEARCH #2 aynı SEARCH_SPEED_MPS değerini kullanmalı."""
    import state_machine as sm_mod
    from config import COMPETITION_CORNERS, TAKEOFF_ALTITUDE
    from models import Target, TargetColor, TargetType

    monkeypatch.setattr(sm_mod, "SEARCH_SPEED_MPS", 8.0)
    field = CompetitionField(COMPETITION_CORNERS)
    launch_lat, launch_lon = field.local_to_gps(15.0, 50.0)
    drop_lat, drop_lon = field.local_to_gps(27.0, 90.0)

    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon,
    )
    sm = _make_sm(nav)

    # SEARCH #1 (launch referansı)
    sm._init_search()
    first_speed = nav.set_max_speed.call_args[0][0]
    assert first_speed == 8.0

    # SEARCH #2 (ilk drop referansı)
    nav.set_max_speed.reset_mock()
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=drop_lat, longitude=drop_lon, confidence=0.9,
    )
    _enter_second_search(sm, drop_lat, drop_lon, nav)
    second_speed = nav.set_max_speed.call_args[0][0]
    assert second_speed == 8.0
    assert first_speed == second_speed


def test_return_search_restores_speed_first_target():
    """Birinci hedef aranırken yaklaşma fazından dönüş → SEARCH_SPEED_MPS geri gelir."""
    import state_machine as sm_mod
    from config import APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS
    from models import MissionState

    nav = MagicMock()
    sm = _make_sm(nav)
    sm.data.state = MissionState.SEARCHING_FIRST_TARGET
    sm.data.first_target = None

    # Yaklaşma fazı hızı düşürdü (senaryo simülasyonu)
    nav.set_max_speed(APPROACH_SPEED_MPS, VERTICAL_SPEED_MPS)
    nav.set_max_speed.reset_mock()

    sm._return_search()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    args = nav.set_max_speed.call_args[0]
    assert args[0] == sm_mod.SEARCH_SPEED_MPS
    assert args[1] == VERTICAL_SPEED_MPS


def test_return_search_restores_speed_second_target():
    """İkinci hedef aranırken hizalama fazından dönüş → SEARCH_SPEED_MPS geri gelir."""
    import state_machine as sm_mod
    from config import ALIGN_SPEED_MPS, COMPETITION_CORNERS, VERTICAL_SPEED_MPS
    from models import MissionState, Target, TargetColor, TargetType

    field = CompetitionField(COMPETITION_CORNERS)
    drop_lat, drop_lon = field.local_to_gps(27.0, 90.0)

    nav = MagicMock()
    sm = _make_sm(nav)
    sm.data.first_target = Target(
        target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=drop_lat, longitude=drop_lon, confidence=0.9,
    )
    sm.data.state = MissionState.SEARCHING_SECOND_TARGET

    # Hizalama fazı hızı düşürdü (0.8 m/s — ikinci search'teki yavaşlama senaryosu)
    nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
    nav.set_max_speed.reset_mock()

    sm._return_search()

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    args = nav.set_max_speed.call_args[0]
    assert args[0] == sm_mod.SEARCH_SPEED_MPS
    assert args[1] == VERTICAL_SPEED_MPS
