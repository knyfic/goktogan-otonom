"""
Hedef kilit mekanizması testleri.

Kullanıcı gereksinimleri:
  1. İlk hedef: sağlıklı görülen İLK tür kilitlenir, ona yönelinir; diğer tür
     (örn. mavi altıgen) umurunda olmaz — taramayı da engellemez.
  2. İkinci hedef: birinci hedefin TERSİ olan tür aranır; başka tür kabul edilmez.
  3. İki tür aynı anda görülürse EN SAĞLIKLI (en yüksek güven) olan birinci
     hedef olur; diğeri ikinci hedef kalır.
"""
import logging
import time
from unittest.mock import MagicMock

import numpy as np

from models import MissionState, Target, TargetColor, TargetType


class _D:
    """Basit sahte Detection (renk/şekil/güven yeterli)."""

    def __init__(self, color, shape, conf, cx=960.0, cy=352.0):
        self.color = color
        self.shape = shape
        self.confidence = conf
        self.center_x = cx
        self.center_y = cy
        self.area_px = 100.0
        self.bbox_width_px = 50.0
        self.bbox_height_px = 50.0


# ── 1) Detector zaman kilidi: ikisi birden görülürse en sağlıklısı ─────────── #
def _detector_with_first_seen(*types):
    from vision.target_detector import TargetDetector

    det = TargetDetector()
    now = time.monotonic()
    det._first_seen = {t: now - 2.0 for t in types}   # eşiği aşmış (≥1s)
    return det


def test_time_lock_picks_healthiest_when_both_visible():
    det = _detector_with_first_seen((TargetColor.RED, "TRIANGLE"),
                                    (TargetColor.BLUE, "HEXAGON"))
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.65)
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)

    det.update_time_lock([d_red, d_blue])

    assert det.time_locked_color == TargetColor.BLUE, "en sağlıklı tür birinci hedef olmalı"
    assert det.time_locked_shape == "HEXAGON"


def test_time_lock_single_mature_type_locks():
    det = _detector_with_first_seen((TargetColor.RED, "TRIANGLE"))
    det.update_time_lock([_D(TargetColor.RED, "TRIANGLE", 0.7)])
    assert det.time_locked_color == TargetColor.RED
    assert det.time_locked_shape == "TRIANGLE"


def test_time_lock_requires_duration():
    from vision.target_detector import TargetDetector

    det = TargetDetector()
    now = time.monotonic()
    det._first_seen = {(TargetColor.RED, "TRIANGLE"): now - 0.2}   # eşik dolmamış
    det.update_time_lock([_D(TargetColor.RED, "TRIANGLE", 0.9)])
    assert det.time_locked_color is None


def test_time_lock_single_visible_type_after_other_vanishes():
    """Önce mavi görünüp sonra kaybolursa kırmızı kilitlenir (ilk SAĞLIKLI tür)."""
    from vision.target_detector import TargetDetector

    det = TargetDetector()
    now = time.monotonic()
    det._first_seen = {(TargetColor.RED, "TRIANGLE"): now - 2.0}   # mavi kayboldu
    det.update_time_lock([_D(TargetColor.RED, "TRIANGLE", 0.8)])
    assert det.time_locked_color == TargetColor.RED


# ── 2) State machine: ikinci search beklenen türü zorlar ───────────────────── #
def _search_sm(state, first_target, detections, locked=None, stable=None):
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=8.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
        battery_voltage_v=24.8, battery_percent=85,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = state
    sm.data.first_target = first_target
    sm._locked_color, sm._locked_shape = (locked or (None, None))
    sm.target_detector = MagicMock()
    sm.target_detector.detect.return_value = detections
    sm.target_detector.stable_detection.return_value = stable
    # MagicMock varsayılanı truthy MagicMock döndürür — gerçek detektörde
    # kilit olmadan bunlar None'dır.
    sm.target_detector.time_locked_color = None
    sm.target_detector.time_locked_shape = None
    return sm, nav


def test_second_search_accepts_only_expected_target_type():
    """SEARCH #2: birinci (RED üçgen) + geçersiz (BLUE üçgen) elenir;
    yalnızca beklenen (BLUE altıgen) stable_detection'a gider."""
    first = Target(target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
                   latitude=1.0, longitude=1.0, confidence=0.9)
    d_first = _D(TargetColor.RED, "TRIANGLE", 0.9)
    d_bad = _D(TargetColor.BLUE, "TRIANGLE", 0.8)      # geçersiz kombinasyon
    d_expected = _D(TargetColor.BLUE, "HEXAGON", 0.85)

    sm, _nav = _search_sm(MissionState.SEARCHING_SECOND_TARGET, first,
                          [d_first, d_bad, d_expected], stable=d_expected)

    sm._do_search_target()

    args = sm.target_detector.stable_detection.call_args[0]
    assert len(args[0]) == 1 and args[0][0] is d_expected, \
        "yalnızca beklenen tür (BLUE altıgen) aday olmalı"
    assert sm.data.state == MissionState.TARGET_DETECTED


def test_second_search_first_target_alone_does_not_trigger():
    """SEARCH #2: yalnızca birinci hedef görülüyorsa tespit YOK — tarama devam."""
    first = Target(target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
                   latitude=1.0, longitude=1.0, confidence=0.9)
    d_first = _D(TargetColor.RED, "TRIANGLE", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_SECOND_TARGET, first,
                          [d_first], stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    assert sm.data.state != MissionState.TARGET_DETECTED, "birinci hedef 2. hedef olamaz"
    sm._advance_search.assert_called()
    call_args = sm.target_detector.stable_detection.call_args
    assert call_args[0][0] == [] and call_args[1].get("search_mode") is True


# ── 2b) İLK arama YALNIZCA KIRMIZI ÜÇGEN arar (görev planı) ──────────────── #
def test_first_search_accepts_only_red_triangle():
    """SEARCH #1 (kırmızı üçgen araması): mavi altıgen elenir; yalnızca
    kırmızı üçgen stable_detection'a gider ve görevi tetikler."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.85)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [d_blue, d_red], stable=d_red)

    sm._do_search_target()

    args = sm.target_detector.stable_detection.call_args[0]
    assert len(args[0]) == 1 and args[0][0] is d_red, \
        "yalnızca kırmızı üçgen (RED TRIANGLE) aday olmalı — mavi altıgen elenir"
    assert sm.data.state == MissionState.TARGET_DETECTED


def test_first_search_blue_hexagon_alone_does_not_trigger():
    """SEARCH #1: yalnızca mavi altıgen görülürse tespit YOK — tarama devam.
    Mavi altıgen ilk hedef değildir; görevi TETİKLEMEZ."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [d_blue], stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    assert sm.data.state != MissionState.TARGET_DETECTED, \
        "mavi altıgen ilk aramada görevi tetiklememeli"
    sm._advance_search.assert_called()
    call_args = sm.target_detector.stable_detection.call_args
    assert call_args[0][0] == [] and call_args[1].get("search_mode") is True


def test_first_search_blue_hexagon_does_not_pollute_time_lock():
    """SEARCH #1: mavi altıgen görüntüde sürekli olsa bile zaman kilidini
    kirletemez — update_time_lock filtrelenmiş (kırmızı üçgen) listeyi alır."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [d_blue], stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    # Mavi altıgen zaman kilidi listesine GİRMEMİŞ olmalı
    lock_call = sm.target_detector.update_time_lock.call_args
    assert lock_call is not None
    assert lock_call[0][0] == [], \
        "mavi altıgen ilk aramada zaman kilidini kirletmemeli"


# ── 3) Kilitliyken diğer tür taramayı durdurmaz ────────────────────────────── #
def test_locked_search_ignores_other_target_type():
    """Kilit RED üçgende: yalnızca BLUE altıgen görülürse tarama DEVAM eder."""
    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [_D(TargetColor.BLUE, "HEXAGON", 0.9)],
                          locked=(TargetColor.RED, "TRIANGLE"), stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    sm._advance_search.assert_called(), "kilit dışı tür taramayı engellememeli"


def test_locked_search_waits_for_locked_type():
    """Kilit RED üçgende: RED tespiti kararlı olmadan önce tarama DURUR (stabilize)."""
    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [_D(TargetColor.RED, "TRIANGLE", 0.7)],
                          locked=(TargetColor.RED, "TRIANGLE"), stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    sm._advance_search.assert_not_called(), "kilitli tür stabilize olurken durulmalı"


def test_unlocked_search_waits_for_stabilization():
    """Kilit yokken geçerli tür (kırmızı üçgen) tespit edildiyse tarama durur
    (stabilize için)."""
    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [_D(TargetColor.RED, "TRIANGLE", 0.9)],
                          locked=(None, None), stable=None)
    sm._advance_search = MagicMock()

    sm._do_search_target()

    sm._advance_search.assert_not_called()
