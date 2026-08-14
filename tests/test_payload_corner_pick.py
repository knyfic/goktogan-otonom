"""
Payload alımı — düşük irtifada köşe/kenar tespit seçimi testleri.

Kural (PAYLOAD_CORNER_PICK_ALT_M = 0.5m altı):
  - Normalde %15 kenar marjı (köşe eleme) uygulanır: payload ortada aranır.
  - 0.5m'nin ALTINDA köşe eleme kaldırılır; "mavi puan (color_purity) + alan"
    skoru EN YÜKSEK olan aday gerçek payload kabul edilir.
  - Bu seçim yalnızca düşük irtifada geçerlidir; üstündeki davranış değişmez.
"""
import logging
from types import SimpleNamespace
from unittest.mock import MagicMock

from config import (
    PAYLOAD_CORNER_PICK_ALT_M,
    PAYLOAD_HEIGHT,
    PAYLOAD_WIDTH,
)
from vision.geometry import expected_payload_pixels


def _sm():
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=2.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
    )
    camera = MagicMock()
    camera.latest.return_value = None
    return MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )


def _candidate(sm, altitude_m, cx, cy, area_factor, purity):
    """Beklenen alana göre normalize boyutlu sahte payload adayı."""
    exp_w, exp_h = expected_payload_pixels(
        PAYLOAD_WIDTH, PAYLOAD_HEIGHT, altitude_m,
        sm.position_controller.camera,
    )
    exp_area = exp_w * exp_h
    return SimpleNamespace(
        width_px=exp_w * 0.7, height_px=exp_h * 0.7,
        area_px=exp_area * area_factor,      # min_area (0.10x) ile max_area (9x) arası
        center_x=float(cx), center_y=float(cy),
        color_purity=purity,
    )


def test_low_alt_includes_corner_and_picks_highest_score():
    """0.5m altında köşedeki büyük+mavi aday, ortadaki küçük adaya tercih edilir."""
    sm = _sm()
    alt = PAYLOAD_CORNER_PICK_ALT_M - 0.2   # eşiğin altı (0.3m)

    corner_big = _candidate(sm, alt, cx=80.0, cy=90.0, area_factor=0.8, purity=0.90)
    center_small = _candidate(sm, alt, cx=960.0, cy=352.0, area_factor=0.25, purity=0.50)

    result = sm._filter_payload_results([center_small, corner_big], alt)

    assert result == [corner_big], "düşük irtifada köşe+büyük+mavi seçilmeli"


def test_low_alt_picks_larger_area_when_purity_equal():
    """Aynı mavi puanda daha BÜYÜK alanlı aday seçilir (gerçek payload daha büyüktür)."""
    sm = _sm()
    alt = PAYLOAD_CORNER_PICK_ALT_M - 0.2

    corner_small = _candidate(sm, alt, cx=60.0, cy=70.0, area_factor=0.3, purity=0.8)
    corner_large = _candidate(sm, alt, cx=1800.0, cy=1000.0, area_factor=0.9, purity=0.8)

    result = sm._filter_payload_results([corner_small, corner_large], alt)

    assert result == [corner_large], "eşit mavi puanda alanı büyük olan seçilmeli"


def test_above_low_alt_still_excludes_corner():
    """0.5m ÜSTÜNDE köşe eleme korunur — mevcut davranış değişmez."""
    sm = _sm()
    alt = PAYLOAD_CORNER_PICK_ALT_M + 1.0   # 1.5m (verify bandı dışı)

    corner = _candidate(sm, alt, cx=80.0, cy=90.0, area_factor=0.8, purity=0.90)
    center = _candidate(sm, alt, cx=960.0, cy=352.0, area_factor=0.8, purity=0.90)

    result = sm._filter_payload_results([corner, center], alt)

    assert corner not in result, "köşe aday yüksek irtifada elenmeli"
    assert center in result


def test_low_alt_threshold_config():
    """Eşik config'ten okunur (varsayılan 0.5m)."""
    assert PAYLOAD_CORNER_PICK_ALT_M == 0.5
