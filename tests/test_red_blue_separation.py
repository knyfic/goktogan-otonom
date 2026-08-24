"""
RED/BLUE hedef ayrımı regression testleri.

Görev kuralı (KESİN):
  1. Önce RED TRIANGLE aranır ve RED akışı TAMAMLANIR.
  2. Ancak bundan sonra BLUE HEXAGON aranabilir.

Bug: RED detection flicker yaptığında (detected → not detected → detected)
ALIGN fazında BLUE HEXAGON yanlışlıkla AKTİF hedef oluyordu (drone BLUE'ya
yöneliyor, hatta BLUE'yu first_target olarak kaydedebiliyordu).

Kök neden (state_machine.py):
  - _filter_active_detections: kilitli tür (RED) yokken 'locked' listesi boş
    olunca diğer türleri (BLUE dahil) döndürüyordu.
  - _pick_locked_detection: kilitli tür listede yoksa detections[0]'ı
    (BLUE olabilir) döndürüyordu.

Kural: BLUE'ya geçiş YALNIZCA RED mission completion flag'i gerçekleşince
(first_target=RED + payload_released) mümkündür. "RED detected == False" tek
başına RED'in tamamlandığı anlamına gelmez — flicker ile aynı şeydir.
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


def _search_sm(state, first_target, detections, locked=None, stable=None):
    """SEARCH state'i için gerçek SM; detektör mock'lu."""
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
    sm.target_detector.time_locked_color = None
    sm.target_detector.time_locked_shape = None
    return sm, nav


def _align_sm():
    """ALIGNING_TARGET (kilit RED üçgen) için gerçek SM; detektör mock'lu."""
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=5.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
        battery_voltage_v=24.8, battery_percent=85,
    )
    camera = MagicMock()
    camera.latest.return_value = np.zeros((704, 1920, 3), dtype=np.uint8)

    sm = MissionStateMachine(
        navigation=nav, camera=camera, payload=MagicMock(),
        logger=logging.getLogger("t"),
    )
    sm.data.state = MissionState.ALIGNING_TARGET
    sm._align_phase = "HORIZONTAL"
    sm._locked_color, sm._locked_shape = TargetColor.RED, "TRIANGLE"
    sm.data.pending_detection = _D(TargetColor.RED, "TRIANGLE", 0.85)
    sm._last_align_cmd_t = 0.0
    sm._last_align_status_log_t = time.monotonic()
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = False
    sm.target_detector = MagicMock()
    return sm, nav


# ── 1) SEARCH_RED: flicker sırasında BLUE asla aktif hedef olamaz ──────────── #
def _fake_stable_detection(n_required: int = 3):
    """Gerçek stable_detection mantığını taklit eder (state_machine'i değil,
    vision/target_detector.py): aynı tipte N ARDIŞIK frame gerekir; farklı tip
    görülürse sayacı sıfırlar (flicker stabilize olamaz — MIN_STABLE_DETECTIONS)."""
    counter: dict = {}

    def fake_stable(dets, **kw):
        if not dets:
            counter.clear()
            return None
        first = dets[0]   # filtrelenmiş listede tek tip kalır
        key = (first.color, first.shape)
        for k in list(counter):
            if k != key:
                del counter[k]
        counter[key] = counter.get(key, 0) + 1
        return first if counter[key] >= n_required else None

    return fake_stable


def test_red_detection_flicker_does_not_activate_blue():
    """SEARCH_RED: RED detection sequence True,True,False,True,False,False,
    True,True,False,True boyunca (hiçbir zaman 3 ardışık yok → stabilize olamaz):
      • mission_state SEARCHING_FIRST_TARGET olarak kalır (SEARCH_BLUE'ya geçmez)
      • aktif hedef (pending/lock) ASLA BLUE olmaz
    RED yokken görünen BLUE altıgen yalnızca YOKSAYILIR."""
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.8)
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)
    sequence = [True, True, False, True, False, False, True, True, False, True]

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None, [], stable=None)
    sm._advance_search = MagicMock()
    sm.target_detector.stable_detection.side_effect = _fake_stable_detection(3)

    for present in sequence:
        sm.target_detector.detect.return_value = (
            [d_red] if present else [d_blue]   # RED yokken mavi görünüyor
        )
        sm._do_search_target()

        assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET, \
            "RED flicker sırasında state SEARCH_RED'de kalmalı (SEARCH_BLUE'ya geçiş yok)"
        pd = sm.data.pending_detection
        if pd is not None:
            assert pd.color == TargetColor.RED and pd.shape == "TRIANGLE", \
                "aktif hedef yalnızca RED TRIANGLE olabilir"
        assert sm._locked_color in (None, TargetColor.RED), \
            "kilit asla BLUE olamaz"


def test_red_stable_detection_still_triggers_mission():
    """Kontrol: RED gerçekten 3 ardışık frame kararlı görülürse mission NORMAL
    şekilde TARGET_DETECTED'e geçer (flicker düzeltmesi RED aramasını bozmaz)."""
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.8)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None, [], stable=None)
    sm._advance_search = MagicMock()
    sm.target_detector.stable_detection.side_effect = _fake_stable_detection(3)
    sm.target_detector.detect.return_value = [d_red]

    for _ in range(3):   # 3 ardışık RED frame
        sm._do_search_target()

    assert sm.data.state == MissionState.TARGET_DETECTED, \
        "kararlı RED detection görevi tetiklemeli (mevcut davranış korunur)"
    assert sm.data.pending_detection.color == TargetColor.RED


# ── 2) ALIGN (RED): flicker'da BLUE'ya yönelme YOK — gerçek bug ────────────── #
def test_red_align_flicker_does_not_switch_active_target_to_blue():
    """ALIGN (RED kilitli): RED flicker yapıp görüntüde yalnızca BLUE varsa:
      • pending_detection ASLA BLUE olmaz (RED olarak kalır)
      • yabancı türe yönelim komutu gönderilmez (_no_detection_count artar)
    (Bu, _filter_active_detections + _pick_locked_detection'ın BLUE'yu
    sızdırdığı gerçek bug senaryosudur.)"""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9, cx=800.0, cy=300.0)

    sm, _nav = _align_sm()
    sm.target_detector.detect.return_value = [d_blue]   # RED flicker — yalnızca mavi

    sm._do_align_target()

    pd = sm.data.pending_detection
    assert pd is not None and pd.color == TargetColor.RED and pd.shape == "TRIANGLE", \
        "pending_detection RED'de kalmalı — BLUE'ya değişmemeli"
    assert sm._no_detection_count == 1, \
        "kilitli tür yoksa 'tespit yok' sayacı artmalı (BLUE'ya yönelim yok)"


def test_red_align_ignores_blue_even_in_mixed_frame():
    """ALIGN (RED kilitli): RED + BLUE AYNI karedeyken bile aktif hedef RED'dir."""
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.8, cx=900.0, cy=340.0)
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.95, cx=700.0, cy=280.0)

    sm, _nav = _align_sm()
    sm.target_detector.detect.return_value = [d_blue, d_red]   # ikisi de görünüyor

    sm._do_align_target()

    pd = sm.data.pending_detection
    assert pd is not None and pd.color == TargetColor.RED, \
        "karışık karede aktif hedef RED olmalı — BLUE'ya atlanamaz"
    assert sm._no_detection_count == 0


# ── 3) BLUE yalnızca RED tamamlandıktan SONRA aktifleşebilir ───────────────── #
def test_blue_activates_only_after_red_completion():
    """RED gerçekten tamamlandıktan (first_target=RED TRIANGLE +
    payload_released) sonra SEARCHING_SECOND_TARGET'te BLUE HEXAGON normal
    şekilde aday olur ve görevi tetikler (RED→BLUE geçişi korunur)."""
    first = Target(target_id=1, shape=TargetType.TRIANGLE, color=TargetColor.RED,
                   latitude=1.0, longitude=1.0, confidence=0.9, payload_released=True)
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_SECOND_TARGET, first,
                          [d_blue], stable=d_blue)

    sm._do_search_target()

    args = sm.target_detector.stable_detection.call_args[0]
    assert len(args[0]) == 1 and args[0][0] is d_blue, \
        "RED tamamlandıktan sonra BLUE hexagon aday olabilmeli"
    assert sm.data.state == MissionState.TARGET_DETECTED


def test_red_not_detected_alone_never_means_completion():
    """'RED detected == False' RED'in TAMAMLANDIĞI anlamına gelmez: RED henüz
    bulunamamışken BLUE görünse bile state SEARCH_RED'de kalır, TARGET_DETECTED'e
    geçilmez (yalnızca kararlı RED tespiti görevi tetikler)."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [d_blue], stable=None)
    sm._advance_search = MagicMock()

    for _ in range(5):   # BLUE sürekli görünüyor, RED hiç yok
        sm._do_search_target()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET, \
        "RED yokluğu RED tamamlanması değildir — BLUE görevi tetikleyemez"
