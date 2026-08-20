"""
size_filter modülü testleri — GSD tabanlı fiziksel boyut filtresi.

Senaryolar:
  * 11.5 m'de beklenen üçgen (1 m kenar → ~64 px / ~1790 px²)
  * 11.5 m'de beklenen altıgen (2 m kenar → ~129 px / ~42960 px²)
  * farklı irtifalarda GSD değişimi (doğrusal ölçek)
  * alan toleransı altı / üstü reddi
  * altitude bilinmiyorsa mutlak eşikler (eski davranış)
"""
import math

import pytest

from config import (
    VISION_CAMERA_GEOMETRY,
    VISION_SIZE_FILTER,
    VISION_TARGETS,
)
from vision.size_filter import SizeFilter

_GEO = VISION_CAMERA_GEOMETRY
_SF = SizeFilter(_GEO, VISION_SIZE_FILTER)
_TRI = VISION_TARGETS["RED_TRIANGLE"]
_HEX = VISION_TARGETS["BLUE_HEXAGON"]

_H_REF = float(_GEO["gsd_ref_altitude_m"])
_GSD_REF = float(_GEO["gsd_ref_m_per_px"])
_FRAME_AREA = 960 * 352


# ── GSD ───────────────────────────────────────────────────────────────────── #

def test_gsd_at_reference_altitude():
    assert _SF.gsd_at(_H_REF) == pytest.approx(_GSD_REF, rel=1e-6)


def test_gsd_scales_linearly_with_altitude():
    assert _SF.gsd_at(2 * _H_REF) == pytest.approx(2 * _GSD_REF, rel=1e-6)
    assert _SF.gsd_at(_H_REF / 2) == pytest.approx(_GSD_REF / 2, rel=1e-6)


def test_reference_gsd_matches_field_measurement():
    """11.5 m'de 960 px ≈ 14.93 m zemin → GSD ≈ 0.01555 m/px."""
    ground_width_m = 14.93
    gsd = ground_width_m / 960.0
    assert gsd == pytest.approx(0.01555, abs=0.0005)


# ── beklenen boyutlar ─────────────────────────────────────────────────────── #

def test_expected_triangle_at_11_5m():
    edge_px = SizeFilter.expected_edge_px(1.0, _SF.gsd_at(_H_REF))
    assert edge_px == pytest.approx(64, rel=0.05)  # 1m / 0.01555 ≈ 64 px
    area_px = SizeFilter.expected_area_px("TRIANGLE", 1.0, _SF.gsd_at(_H_REF))
    expected = (math.sqrt(3) / 4) * 1.0 ** 2 / (_GSD_REF ** 2)
    assert area_px == pytest.approx(expected, rel=1e-6)
    assert area_px == pytest.approx(1790, rel=0.10)


def test_expected_hexagon_at_11_5m():
    edge_px = SizeFilter.expected_edge_px(2.0, _SF.gsd_at(_H_REF))
    assert edge_px == pytest.approx(129, rel=0.05)  # 2m / 0.01555 ≈ 129 px
    area_px = SizeFilter.expected_area_px("HEXAGON", 2.0, _SF.gsd_at(_H_REF))
    expected = (3 * math.sqrt(3) / 2) * 2.0 ** 2 / (_GSD_REF ** 2)
    assert area_px == pytest.approx(expected, rel=1e-6)
    assert area_px == pytest.approx(42960, rel=0.10)


def test_expected_size_scales_with_altitude():
    """İrtifa arttıkça beklenen piksel alanı azalır (GSD² ile)."""
    a_low = SizeFilter.expected_area_px("HEXAGON", 2.0, _SF.gsd_at(_H_REF))
    a_high = SizeFilter.expected_area_px("HEXAGON", 2.0, _SF.gsd_at(2 * _H_REF))
    assert a_high == pytest.approx(a_low / 4, rel=1e-6)


# ── alan toleransı ────────────────────────────────────────────────────────── #

def test_accept_triangle_within_tolerance_at_reference():
    expected = SizeFilter.expected_area_px("TRIANGLE", 1.0, _SF.gsd_at(_H_REF))
    tol = tuple(float(v) for v in _TRI["area_tolerance"])
    assert _SF.accept(expected, "TRIANGLE", 1.0, _H_REF, _FRAME_AREA, tol) is True
    assert _SF.accept(expected * tol[0], "TRIANGLE", 1.0, _H_REF, _FRAME_AREA, tol) is True
    assert _SF.accept(expected * tol[1], "TRIANGLE", 1.0, _H_REF, _FRAME_AREA, tol) is True


def test_reject_area_below_tolerance():
    tol = tuple(float(v) for v in _TRI["area_tolerance"])
    expected = SizeFilter.expected_area_px("TRIANGLE", 1.0, _SF.gsd_at(_H_REF))
    assert _SF.accept(expected * (tol[0] - 0.05), "TRIANGLE", 1.0, _H_REF,
                      _FRAME_AREA, tol) is False


def test_reject_area_above_tolerance():
    tol = tuple(float(v) for v in _HEX["area_tolerance"])
    expected = SizeFilter.expected_area_px("HEXAGON", 2.0, _SF.gsd_at(_H_REF))
    assert _SF.accept(expected * (tol[1] + 0.05), "HEXAGON", 2.0, _H_REF,
                      _FRAME_AREA, tol) is False


def test_reject_below_absolute_min_area():
    """Altitude yokken bile mutlak alt sınır uygulanır."""
    tol = (0.6, 1.4)
    assert _SF.accept(5.0, "TRIANGLE", 1.0, None, _FRAME_AREA, tol) is False


def test_reject_blob_filling_frame():
    """Kadrajın %75'inden büyük blob (altitude bilinmese bile) reddedilir."""
    tol = (0.6, 1.4)
    huge = VISION_SIZE_FILTER["max_blob_fraction"] * _FRAME_AREA + 1.0
    assert _SF.accept(huge, "HEXAGON", 2.0, None, _FRAME_AREA, tol) is False


def test_accept_without_altitude_uses_absolute_rules():
    """Altitude bilinmiyorsa (eski detect(frame) çağrısı) makul alan kabul edilir."""
    tol = (0.6, 1.4)
    assert _SF.accept(2000.0, "TRIANGLE", 1.0, None, _FRAME_AREA, tol) is True


def test_altitude_not_hardcoded():
    """11.5 m sabit sayı kodda YOK — config'ten gelir."""
    assert _SF._h_ref == float(_GEO["gsd_ref_altitude_m"])
