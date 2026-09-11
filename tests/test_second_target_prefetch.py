"""2. hedef (KIRMIZI ÜÇGEN) ÖN KAYIT + arama devam testleri.

Yeni davranış:
  1. İlk arama (MAVİ ALTIGEN) sırasında KIRMIZI ÜÇGEN PASİF izlenir; yalnızca
     KONUMU önceden kaydedilir — drop/alignment/state TETİKLENMEZ.
  2. İlk hedef tamamlanınca (kalkış sonrası) ön kayıtlı 2. hedefe ARAMA
     YAPILMADAN doğrudan gidilir.
  3. Ön kayıt yoksa 2. arama ilk search'te KALDIĞI YERDEN devam eder
     (line 1'e geri dönüp geçilmiş yerler tekrar taranmaz).
"""
import logging
from unittest.mock import MagicMock

import numpy as np

from config import SECOND_TARGET_PREFETCH_FRAMES, TAKEOFF_ALTITUDE, ALTITUDE_TOLERANCE_M
from models import MissionState, Target, TargetColor, TargetType


class _D:
    """Basit sahte Detection."""

    def __init__(self, color, shape, conf, cx=960.0, cy=352.0):
        self.color = color
        self.shape = shape
        self.confidence = conf
        self.center_x = cx
        self.center_y = cy
        self.area_px = 200.0
        self.bbox_width_px = 60.0
        self.bbox_height_px = 60.0


def _sm(state=MissionState.SEARCHING_FIRST_TARGET, altitude=TAKEOFF_ALTITUDE):
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=altitude, latitude=38.35942, longitude=38.26117,
        heading_deg=0.0, battery_voltage_v=24.8, battery_percent=85,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)
    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = state
    return sm, nav


def _blue_released():
    return Target(target_id=1, shape=TargetType.HEXAGON, color=TargetColor.BLUE,
                  latitude=38.36, longitude=38.262, confidence=0.9,
                  payload_released=True)


# ── 1) Pasif kırmızı kaydı: drop/state tetiklenmez ────────────────────────── #
def test_passive_red_recorded_without_triggering_drop():
    sm, nav = _sm()
    sm.target_detector = MagicMock()
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.9)
    sm.target_detector.detect.return_value = [d_red]
    sm.target_detector.stable_detection.return_value = None
    sm.target_detector.time_locked_color = None
    sm.target_detector.time_locked_shape = None
    sm._advance_search = MagicMock()

    for _ in range(SECOND_TARGET_PREFETCH_FRAMES):
        sm._do_search_target()

    pf = sm._prefetched_second_target
    assert pf is not None
    assert pf.color == TargetColor.RED and pf.shape == TargetType.TRIANGLE
    # Hedef kadraj merkezinde → zemin GPS'i = İHA GPS'i.
    assert abs(pf.latitude - 38.35942) < 1e-6
    assert abs(pf.longitude - 38.26117) < 1e-6
    # Aktif akış DEĞİŞMEDİ:
    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET
    assert sm.data.pending_detection is None
    assert sm.data.second_target is None, "ön kayıt second_target'a yazılmamalı (ilk drop BLUE'da)"


def test_no_prefetch_before_threshold():
    sm, _nav = _sm()
    sm.target_detector = MagicMock()
    sm.target_detector.detect.return_value = [_D(TargetColor.RED, "TRIANGLE", 0.9)]
    sm.target_detector.stable_detection.return_value = None
    sm.target_detector.time_locked_color = None
    sm.target_detector.time_locked_shape = None
    sm._advance_search = MagicMock()

    sm._do_search_target()          # tek frame → henüz kayıt yok
    assert sm._prefetched_second_target is None


# ── 2) Ön kayıtlı 2. hedefe doğrudan gidiş ────────────────────────────────── #
def test_ascend_goes_directly_to_prefetched_target():
    sm, nav = _sm(state=MissionState.ASCENDING_AFTER_FIRST_RELEASE)
    sm.data.first_target = _blue_released()
    sm._prefetched_second_target = Target(
        target_id=2, shape=TargetType.TRIANGLE, color=TargetColor.RED,
        latitude=38.3605, longitude=38.2625, confidence=0.9,
    )
    sm._alt_waiting = True
    sm._alt_target = TAKEOFF_ALTITUDE

    sm._do_ascend_after_first_release()

    assert sm.data.state == MissionState.APPROACHING_TARGET
    assert sm._locked_color == TargetColor.RED
    assert sm._locked_shape == "TRIANGLE"
    assert sm._target_rough_lat == 38.3605
    assert sm._target_rough_lon == 38.2625


# ── 3) Ön kayıt yoksa: arama KALDIĞI YERDEN devam ─────────────────────────── #
def test_ascend_resumes_second_search_from_same_line():
    sm, nav = _sm(state=MissionState.ASCENDING_AFTER_FIRST_RELEASE)
    sm.data.first_target = _blue_released()
    sm._prefetched_second_target = None
    sm._search_idx = 3                     # line 2 ortasında kalmış
    sm._search_resume_lat = 1.5
    sm._search_resume_lon = 2.5
    sm._alt_waiting = True
    sm._alt_target = TAKEOFF_ALTITUDE

    sm._do_ascend_after_first_release()

    assert sm.data.state == MissionState.SEARCHING_SECOND_TARGET
    # Mevcut 3-line geometrisi yeniden kurulur ama ilerleme KORUNUR.
    assert len(sm._search_points) == 6
    assert sm._search_idx == 3, "geçilen line'lar tekrar taranmamalı — kaldığı yerden devam"
    assert sm._search_resume_lat == 1.5 and sm._search_resume_lon == 2.5


# ── 4) detection_ground_gps: piksel→zemin GPS yönü ────────────────────────── #
def test_detection_ground_gps_direction():
    sm, nav = _sm(altitude=9.0)
    _c = 1920 / 2.0
    _r = 704 / 2.0
    # heading=0 (kuzey). Hedef kadrajda SAĞDA → doğuya offset (lon artar).
    gps = sm.position_controller.detection_ground_gps(_c + 100, _r, 1920, 704, 9.0)
    assert gps is not None
    assert gps[1] > 38.26117
    assert abs(gps[0] - 38.35942) < 1e-9

    # heading=90 (doğu). Hedef kadrajda SAĞDA → güneye offset (lat azalır).
    nav.position.return_value = MagicMock(
        relative_altitude=9.0, latitude=38.35942, longitude=38.26117,
        heading_deg=90.0,
    )
    gps2 = sm.position_controller.detection_ground_gps(_c + 100, _r, 1920, 704, 9.0)
    assert gps2[0] < 38.35942
