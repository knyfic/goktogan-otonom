"""
CompetitionField + FOV tabanlı 3-line search planner testleri.

Yeni geometri (30x100 varsayımı YOK):
  - Kenar uzunlukları GERÇEK GPS köşelerinden hesaplanır.
  - Uzun kenarlara paralel SEARCH_FOV_WIDTH_M (12.980 m) tabanlı 3 search line.
  - İlk/son line uzun kenarlardan FOV/2 (6.490 m) İÇERİDE — kamera alan dışını
    gereksiz görmez, görüşün dış sınırı alan kenarına oturur.
  - Orta line iki sınır arasını dengeler.
  - Line uçları gerçek dörtgen sınırıyla kesişir (alan dışı waypoint YOK).
  - Sıra zig-zag (→ ← →) — yaw değişimi gerektirmez.
  - Bozuk/trapez dörtgenler ve GPS perturbasyonları desteklenir.
"""
import logging
import math
import random
from unittest.mock import MagicMock

import pytest

from config import SEARCH_FOV_WIDTH_M
from search.coverage_planner import (
    CompetitionField,
    LawnmowerPlanner,
    SearchPoint,
    point_in_polygon,
)

_ORIGIN = (37.9334810, 34.6174975)


def _gps_from_origin(o_lat, o_lon, north_m, east_m):
    """Kuzey/doğu metre → GPS (coverage_planner.local_to_gps ile aynı model)."""
    radius = 6378137.0
    d_lat = north_m / radius
    d_lon = east_m / (radius * math.cos(math.radians(o_lat)))
    return o_lat + math.degrees(d_lat), o_lon + math.degrees(d_lon)


def _field(width=30.0, length=100.0, theta_deg=0.0, origin=_ORIGIN):
    """θ dönüşlü DİKDÖRTGEN köşeleri üretir (ideal alanlar için)."""
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


def _quad(local_corners, theta_deg=0.0, origin=_ORIGIN):
    """Local (x: genişlik, y: uzunluk) köşelerini θ döndürmeli GPS'e çevirir.

    _field ile AYNI model: y → kuzey, x → doğu (θ=0). Bozuk/trapez dörtgenler
    üretmek için kullanılır (dikdörtgen OLMAK zorunda değildir).
    """
    o_lat, o_lon = origin
    th = math.radians(theta_deg)
    pts = []
    for (x, y) in local_corners:
        north = y * math.cos(th) - x * math.sin(th)
        east = y * math.sin(th) + x * math.cos(th)
        pts.append(_gps_from_origin(o_lat, o_lon, north, east))
    return CompetitionField({"A": pts[0], "B": pts[1], "C": pts[2], "D": pts[3]})


def _gps_corners(field):
    """Local köşeleri GPS'e geri çevirir (pertürbasyon testleri için)."""
    return {k: field.local_to_gps(*loc) for k, loc in field.corner_local().items()}


def _line_transversal_offsets(field, fov=SEARCH_FOV_WIDTH_M, count=3):
    """Her line'ın transversal konumu (ilk uzun kenardan, GERÇEK metre) + alan genişliği."""
    d = field.search_direction_local()
    t = (-d[1], d[0])
    scale_t = math.sqrt(1.0 + 2.0 * field.axis_cos * t[0] * t[1])
    p_vals = [x * t[0] + y * t[1] for x, y in field.corner_local().values()]
    p_min, p_max = min(p_vals), max(p_vals)
    width = (p_max - p_min) * scale_t
    offsets = []
    for (s, e) in field.search_lines_local(fov, count):
        pm = ((s[0] + e[0]) / 2.0) * t[0] + ((s[1] + e[1]) / 2.0) * t[1]
        offsets.append((pm - p_min) * scale_t)
    return sorted(offsets), width


# ── 1) Köşeler → local frame (korunan temel geometri) ─────────────────────── #
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


def test_center_launch():
    f = _field(30.0, 100.0, 0.0)
    gps = f.local_to_gps(15.0, 50.0)
    lx, ly = f.gps_to_local(*gps)
    assert (lx, ly) == pytest.approx((15.0, 50.0), abs=0.1)
    assert f.contains(*gps)


# ── 2) Kenar uzunlukları / uzun-kısa tespiti (varsayım YOK) ───────────────── #
def test_edge_lengths_ideal_30x100():
    f = _field(30.0, 100.0, 0.0)
    edges = f.edge_lengths_m()
    assert edges["AB"] == pytest.approx(100.0, abs=0.2)
    assert edges["CD"] == pytest.approx(100.0, abs=0.2)
    assert edges["BC"] == pytest.approx(30.0, abs=0.2)
    assert edges["DA"] == pytest.approx(30.0, abs=0.2)
    long_names, short_names = f.long_short_edges()
    assert sorted(long_names) == ["AB", "CD"]
    assert sorted(short_names) == ["BC", "DA"]


def test_edge_lengths_real_competition_corners():
    """GERÇEK COMPETITION_CORNERS: kenar uzunlukları ölçülür; uzun/kısa tespiti
    tutarlıdır — config değerine sabitlenmez (alan değişirse test de çalışır)."""
    from config import COMPETITION_CORNERS

    f = CompetitionField(COMPETITION_CORNERS)
    edges = f.edge_lengths_m()
    long_names, short_names = f.long_short_edges()
    assert len(long_names) == 2 and len(short_names) == 2
    # Uzun kenarlar gerçekten kısadan uzun olmalı
    assert all(edges[n] > edges[s] for n in long_names for s in short_names)
    # length_m/width_m, AB/BC kenarlarıyla tutarlı olmalı
    assert f.length_m == pytest.approx(edges["AB"], rel=0.01)
    assert f.width_m == pytest.approx(edges["BC"], rel=0.01)


def test_edge_lengths_distorted_quad_no_30x100_assumption():
    """28/35/102/112 benzeri bozuk dörtgen: uzun/kısa GERÇEK ölçümden bulunur."""
    f = _quad([(0, 0), (0, 102), (28, 105), (35, 3)])
    edges = f.edge_lengths_m()
    assert edges["AB"] == pytest.approx(102.0, abs=0.5)
    assert edges["BC"] == pytest.approx(28.2, abs=0.5)
    assert edges["CD"] == pytest.approx(102.2, abs=0.5)
    assert edges["DA"] == pytest.approx(35.1, abs=0.5)
    long_names, short_names = f.long_short_edges()
    assert all(edges[n] >= 90.0 for n in long_names)
    assert all(edges[n] <= 40.0 for n in short_names)


# ── 3) FOV tabanlı 3 search line ──────────────────────────────────────────── #
def test_search_lines_ideal_30x100_positions():
    """Test A — ideal 30x100: 3 line; transversal konumlar 6.490 / 15.000 / 23.510."""
    f = _field(30.0, 100.0, 0.0)
    lines = f.search_lines_local(SEARCH_FOV_WIDTH_M, line_count=3)
    assert len(lines) == 3
    half = SEARCH_FOV_WIDTH_M / 2.0
    xs = sorted((s[0] + e[0]) / 2.0 for s, e in lines)
    assert xs == pytest.approx([half, 15.0, 30.0 - half], abs=0.05)
    # Line uçları KISA kenarlarda (y=0 ve y=100) — alan dışına taşmaz
    for (s, e) in lines:
        assert any(abs(s[1] - y) < 0.1 for y in (0.0, 100.0))
        assert any(abs(e[1] - y) < 0.1 for y in (0.0, 100.0))
        assert abs(s[1] - e[1]) > 99.0
        assert e[0] == pytest.approx(s[0], abs=0.01)   # uzun kenara paralel


def test_search_lines_ideal_coverage():
    """Test E — coverage: ilk line'ın FOV sınırı bir uzun kenarda, son line'ınki
    karşı uzun kenarda — alan dışı gereksiz görüş YOK, boşluk da YOK."""
    f = _field(30.0, 100.0, 0.0)
    half = SEARCH_FOV_WIDTH_M / 2.0
    lines = f.search_lines_local(SEARCH_FOV_WIDTH_M, line_count=3)
    xs = sorted((s[0] + e[0]) / 2.0 for s, e in lines)
    assert xs[0] - half == pytest.approx(0.0, abs=0.05)
    assert xs[-1] + half == pytest.approx(30.0, abs=0.05)
    for i in range(len(xs) - 1):
        assert xs[i] + half >= xs[i + 1] - half - 1e-6   # overlap/temas


def test_search_lines_distorted_quad():
    """Test B — 28/35/102/112 benzeri bozuk dörtgen: 3 line, uzun kenar
    doğrultusunda, ilk/son line 6.490 m içeride, uçlar sınırda, dışarıda yok."""
    f = _quad([(0, 0), (0, 102), (28, 105), (35, 3)])
    half = SEARCH_FOV_WIDTH_M / 2.0
    lines = f.search_lines_local(SEARCH_FOV_WIDTH_M, line_count=3)
    assert len(lines) == 3
    poly = list(f.corner_local().values())
    for (s, e) in lines:
        assert point_in_polygon(*s, poly)
        assert point_in_polygon(*e, poly)
    # Line'lar uzun kenar doğrultusunda
    d = f.search_direction_local()
    for (s, e) in lines:
        v = (e[0] - s[0], e[1] - s[1])
        n = math.hypot(*v)
        assert n > 1.0
        assert abs((v[0] * d[0] + v[1] * d[1]) / n) > 0.99
    # İlk/son line uzun kenarlardan FOV/2 içeride; orta line arayı dengeler
    offsets, width = _line_transversal_offsets(f)
    assert offsets[0] == pytest.approx(half, abs=0.1)
    assert offsets[1] == pytest.approx(width / 2.0, abs=0.1)
    assert offsets[-1] == pytest.approx(width - half, abs=0.1)


def test_search_lines_distorted_coverage():
    """Bozuk dörtgende de coverage: dış sınırlar alan kenarlarına oturur."""
    f = _quad([(0, 0), (0, 102), (28, 105), (35, 3)])
    half = SEARCH_FOV_WIDTH_M / 2.0
    offsets, width = _line_transversal_offsets(f)
    assert offsets[0] - half == pytest.approx(0.0, abs=0.1)
    assert offsets[-1] + half == pytest.approx(width, abs=0.1)
    for i in range(len(offsets) - 1):
        assert offsets[i] + half >= offsets[i + 1] - half - 1e-6


def test_search_lines_trapezoid():
    """Test C — karşılıklı kenarları paralel OLMAYAN dörtgen: çökmez, 3 line,
    line'lar içeride, uçlar sınırda."""
    f = _quad([(0, 0), (0, 100), (30, 98), (32, 0)])   # üst/alt kenarlar kaymış
    lines = f.search_lines_local(SEARCH_FOV_WIDTH_M, line_count=3)
    assert len(lines) == 3
    poly = list(f.corner_local().values())
    for (s, e) in lines:
        assert point_in_polygon(*s, poly)
        assert point_in_polygon(*e, poly)


def test_search_lines_gps_perturbation_stable():
    """Test D — köşelere küçük GPS hataları: çökmez, 3 line kalır, içeride kalır."""
    rng = random.Random(42)
    base = _field(30.0, 100.0, 0.0)
    base_corners = _gps_corners(base)
    for _ in range(20):
        corners = {
            k: (lat + rng.uniform(-1e-6, 1e-6), lon + rng.uniform(-1e-6, 1e-6))
            for k, (lat, lon) in base_corners.items()
        }
        f = CompetitionField(corners)
        lines = f.search_lines_local(SEARCH_FOV_WIDTH_M, line_count=3)
        assert len(lines) == 3
        poly = list(f.corner_local().values())
        for (s, e) in lines:
            assert point_in_polygon(*s, poly)
            assert point_in_polygon(*e, poly)


@pytest.mark.parametrize("theta", [0.0, 30.0, 45.0, 90.0])
def test_rotated_fields(theta):
    """Döndürülmüş alanlar: 3 line üretilir, waypoint'ler içeride kalır."""
    f = _field(30.0, 100.0, theta)
    poly = list(f.corner_local().values())
    route = LawnmowerPlanner(f, SEARCH_FOV_WIDTH_M).generate()
    assert len(route) == 6
    for p in route:
        assert point_in_polygon(p.x_m, p.y_m, poly)


def test_planner_zigzag_order():
    """Waypoint sırası zig-zag: line1 →, line2 ←, line3 → (yaw değişimi yok)."""
    f = _field(30.0, 100.0, 0.0)
    pts = LawnmowerPlanner(f, SEARCH_FOV_WIDTH_M).generate()
    assert len(pts) == 6
    # P1-P2 (line1), P3-P4 (line2), P5-P6 (line3): her line sabit x'te (uzun kenara paralel)
    for i in range(0, 6, 2):
        assert pts[i].x_m == pytest.approx(pts[i + 1].x_m, abs=0.01)
    assert len({round(p.x_m, 3) for p in pts}) == 3     # 3 farklı line
    # Yönler: line1 ile line3 aynı, line2 TERS (→ ← →)
    y1 = pts[0].y_m - pts[1].y_m
    y2 = pts[2].y_m - pts[3].y_m
    y3 = pts[4].y_m - pts[5].y_m
    assert y1 * y2 < 0
    assert y1 * y3 > 0


def test_planner_debug_summary():
    """Debug çıktısı kenar uzunluklarını, FOV'u, line sayısını ve GPS uçlarını içerir."""
    f = _field(30.0, 100.0, 0.0)
    s = LawnmowerPlanner(f, SEARCH_FOV_WIDTH_M).debug_summary()
    assert "Detected edge lengths:" in s
    assert "Long edges:" in s and "Short edges:" in s
    assert "Search FOV width: 12.980 m" in s
    assert "Search half FOV: 6.490 m" in s
    assert "Search line count: 3" in s
    assert "Line 1:" in s and "Line 2:" in s and "Line 3:" in s
    assert "start=" in s and "end=" in s


def test_outside_launch_rejected():
    """contains() gerçek dış noktaları reddeder (frame doğrulaması)."""
    f = _field(30.0, 100.0, 0.0)
    for dx, dy in [(-5.0, 50.0), (35.0, 50.0), (15.0, -5.0), (15.0, 105.0)]:
        gps = f.local_to_gps(dx, dy)
        assert not f.contains(*gps), f"alan dışı ({dx},{dy}) reddedilmeli"


# ── State machine entegrasyonu ────────────────────────────────────────────── #
def _make_sm(nav):
    from state_machine import MissionStateMachine

    return MissionStateMachine(navigation=nav, camera=MagicMock(),
                               payload=MagicMock(),
                               logger=logging.getLogger("t"))


def test_init_search_outside_area_builds_route():
    """İHA alan DIŞINDA başlasa bile search kurulur — abort/RTL YOK."""
    from models import MissionState

    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(latitude=48.0, longitude=8.0)  # alan dışı
    sm = _make_sm(nav)
    sm._init_search()
    assert sm.data.mission_failed is False
    assert sm.data.state != MissionState.RTL
    assert sm._search_field is not None
    assert len(sm._search_points) == 6    # 3 line × 2 uç
    poly = list(sm._search_field.corner_local().values())
    for p in sm._search_points:
        assert point_in_polygon(p.x_m, p.y_m, poly)


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
    assert len(sm._search_points) == 6
    poly = list(sm._search_field.corner_local().values())
    for p in sm._search_points:
        assert point_in_polygon(p.x_m, p.y_m, poly), \
            f"waypoint alan dışında: ({p.x_m:.1f}, {p.y_m:.1f})"


def test_init_search_outside_field_still_builds_route():
    """İHA alan dışında (batıda) başlasa bile FOV tabanlı 3 line kurulur."""
    from config import COMPETITION_CORNERS
    from models import MissionState
    from state_machine import MissionStateMachine

    field = CompetitionField(COMPETITION_CORNERS)
    west_lat, west_lon = field.local_to_gps(-50.0, field.length_m / 2.0)
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(latitude=west_lat, longitude=west_lon)
    sm = MissionStateMachine(navigation=nav, camera=MagicMock(), payload=MagicMock(),
                             logger=logging.getLogger("t"))
    sm._init_search()
    assert sm.data.state != MissionState.RTL
    assert sm.data.mission_failed is False
    assert len(sm._search_points) == 6


def test_first_search_uses_takeoff_reference():
    """İlk arama: launch referansı kullanılır; rota FOV tabanlı 3 line'dır."""
    from config import COMPETITION_CORNERS

    field = CompetitionField(COMPETITION_CORNERS)
    launch_lat, launch_lon = field.local_to_gps(
        field.width_m * 0.4, field.length_m * 0.4,
    )
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon,
    )
    sm = _make_sm(nav)
    sm._init_search()
    assert sm._search_origin_lat == pytest.approx(launch_lat, abs=1e-9)
    assert sm._search_origin_lon == pytest.approx(launch_lon, abs=1e-9)
    assert len(sm._search_points) == 6
    assert nav.set_max_speed.call_count == 1
    # Yaw: uzun kenar GENEL doğrultusu — line değişimlerinde dönme YOK (sabit)
    assert sm._search_yaw_deg == pytest.approx(field.search_direction_deg())


def test_init_search_yaw_is_long_edge_direction():
    """Search yaw'ı uzun kenar doğrultusunda SABİTTİR — konumdan bağımsız."""
    from config import COMPETITION_CORNERS

    field = CompetitionField(COMPETITION_CORNERS)
    launch_lat, launch_lon = field.local_to_gps(
        field.width_m * 0.9, field.length_m * 0.5,
    )
    nav = MagicMock()
    nav.fresh_position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon,
    )
    sm = _make_sm(nav)
    sm._init_search()
    assert sm._search_yaw_deg == pytest.approx(field.search_direction_deg())
    # 6 waypoint = 3 line; hiçbir line değişimi 180° yaw gerektirmez
    # (yaw tek değerdir ve _advance_search her goto'ya aynı değeri iletir)
    nav.position.return_value = MagicMock(
        latitude=launch_lat, longitude=launch_lon, heading_deg=0.0, relative_altitude=8.0,
    )
    sm._search_field = field
    sm._search_idx = 0
    nav.reset_mock()
    sm._advance_search()
    assert nav.goto_global.call_args[1].get("yaw_deg") == pytest.approx(sm._search_yaw_deg)


def test_advance_search_sends_yaw():
    """Tek geçiş yaw hedefi goto_global'e iletilir."""
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
    f = _field(30.0, 100.0, 0.0)
    sm._search_field = f
    sm._search_points = [SearchPoint(5.0, 5.0), SearchPoint(5.0, 95.0)]
    sm._search_idx = 0

    sm._advance_search()

    expected_lat, expected_lon = f.local_to_gps(5.0, 5.0)
    nav.goto_global.assert_called_once()
    args = nav.goto_global.call_args[0]
    assert args[0] == pytest.approx(expected_lat, abs=1e-6)
    assert args[1] == pytest.approx(expected_lon, abs=1e-6)
    assert args[2] == pytest.approx(TAKEOFF_ALTITUDE)


# ── Search hızı ───────────────────────────────────────────────────────────── #
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


# ── İlk / ikinci search referansı ─────────────────────────────────────────── #
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
    """İkinci arama: DROP konumu referans alınır; rota yine FOV tabanlı 3 line."""
    from config import COMPETITION_CORNERS
    from models import MissionState, Target, TargetColor, TargetType

    field = CompetitionField(COMPETITION_CORNERS)
    takeoff_lat, takeoff_lon = field.local_to_gps(12.0, 5.0)
    drop_lat, drop_lon = field.local_to_gps(
        field.width_m * 0.9, field.length_m * 0.5,
    )

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
    assert sm._search_origin_lat == pytest.approx(drop_lat, abs=1e-9)
    assert sm._search_origin_lon == pytest.approx(drop_lon, abs=1e-9)
    assert sm._search_origin_lat != pytest.approx(takeoff_lat, abs=1e-9)
    assert sm._search_origin_lon != pytest.approx(takeoff_lon, abs=1e-9)
    assert len(sm._search_points) == 6
    assert sm._search_yaw_deg == pytest.approx(field.search_direction_deg())


@pytest.mark.parametrize("dx,dy", [(2.0, 5.0), (10.0, 8.0), (28.0, 14.0)])
def test_second_search_different_drop_positions(dx, dy):
    """Farklı drop konumları → rota AYNI (konumdan bağımsız 3 line), içeride."""
    from config import COMPETITION_CORNERS
    from models import MissionState, Target, TargetColor, TargetType

    field = CompetitionField(COMPETITION_CORNERS)
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
    assert len(sm._search_points) == 6
    poly = list(field.corner_local().values())
    for p in sm._search_points:
        assert point_in_polygon(p.x_m, p.y_m, poly), \
            f"ikinci search waypoint'i alan dışında: ({p.x_m:.1f}, {p.y_m:.1f})"


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


def test_first_and_second_search_use_same_speed(monkeypatch):
    """SEARCH #1 ve SEARCH #2 aynı SEARCH_SPEED_MPS değerini kullanmalı."""
    import state_machine as sm_mod
    from config import COMPETITION_CORNERS
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
    """İlk hedef aranırken hizalama fazından dönüş → SEARCH_SPEED_MPS geri gelir."""
    import state_machine as sm_mod
    from config import ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS
    from models import MissionState

    nav = MagicMock()
    sm = _make_sm(nav)
    sm.data.first_target = None
    sm.data.state = MissionState.APPROACHING_TARGET

    nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
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

    nav.set_max_speed(ALIGN_SPEED_MPS, VERTICAL_SPEED_MPS)
    nav.set_max_speed.reset_mock()

    sm._return_search()

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    args = nav.set_max_speed.call_args[0]
    assert args[0] == sm_mod.SEARCH_SPEED_MPS
    assert args[1] == VERTICAL_SPEED_MPS


# ── Geriye dönük kısa-kenar geometrisi (korunan API) ───────────────────────── #
def test_short_edges_wide_field_are_width_edges():
    """30x100 alan: genişlik (30m) KISA → kısa kenar orta noktaları y=0 ve y=L."""
    f = _field(30.0, 100.0, 0.0)
    assert f.short_edge_midpoints_local()[0] == pytest.approx((15.0, 0.0))
    assert f.short_edge_midpoints_local()[1] == pytest.approx((15.0, 100.0))


def test_short_edges_tall_field_are_length_edges():
    """Uzunluğu kısa olan alan (100x30): kısa kenarlar x=0 ve x=W (A-B ve C-D)."""
    f = _field(100.0, 30.0, 0.0)
    assert f.short_edge_midpoints_local()[0] == pytest.approx((0.0, 15.0))
    assert f.short_edge_midpoints_local()[1] == pytest.approx((100.0, 15.0))


def test_real_competition_corners_short_edges():
    """GERÇEK COMPETITION_CORNERS: kısa kenar orta noktaları, ilgili köşe
    çiftlerinin GPS ortasına denk gelir (hangi kenarların kısa olduğu config'ten
    hesaplanır — config değerine sabitlenmez)."""
    from config import COMPETITION_CORNERS

    field = CompetitionField(COMPETITION_CORNERS)
    _, short_names = field.long_short_edges()
    mids = [field.local_to_gps(*m) for m in field.short_edge_midpoints_local()]
    for name in short_names:
        a = COMPETITION_CORNERS[name[0]]
        b = COMPETITION_CORNERS[name[1]]
        expected = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        ok = any(
            abs(lat - expected[0]) < 5e-5 and abs(lon - expected[1]) < 5e-5
            for (lat, lon) in mids
        )
        assert ok, f"{name} orta noktası köşe çiftinin ortasına denk gelmeli"
