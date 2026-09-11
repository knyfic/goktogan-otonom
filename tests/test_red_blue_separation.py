"""
BLUE/RED hedef ayrımı regression testleri.

Görev kuralı (KESİN — yeni sıra):
  1. Önce BLUE HEXAGON aranır ve BLUE akışı TAMAMLANIR.
  2. Ancak bundan sonra RED TRIANGLE aranabilir.

Bug: ilk hedef (BLUE) detection'ı flicker yaptığında (detected → not detected →
detected) ALIGN fazında diğer tür (RED TRIANGLE) yanlışlıkla AKTİF hedef
olabiliyordu (drone RED'e yöneliyor, hatta RED'i first_target olarak
kaydedebiliyordu).

Kök neden (state_machine.py):
  - _filter_active_detections: kilitli tür (BLUE) yokken 'locked' listesi boş
    olunca diğer türleri (RED dahil) döndürüyordu.
  - _pick_locked_detection: kilitli tür listede yoksa detections[0]'ı
    (RED olabilir) döndürüyordu.

Kural: RED'e geçiş YALNIZCA BLUE mission completion flag'i gerçekleşince
(first_target=BLUE + payload_released) mümkündür. "BLUE detected == False" tek
başına BLUE'un tamamlandığı anlamına gelmez — flicker ile aynı şeydir.
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


def _align_sm(color=TargetColor.BLUE, shape="HEXAGON"):
    """ALIGNING_TARGET (kilit = ilk hedef türü) için gerçek SM; detektör mock'lu.

    İrtifa 10m: BLUE için şekil-kilit irtifası 8m olduğundan tracking_mode
    kapalı kalır → gerçek detection/filtre yolu test edilir.
    """
    from state_machine import MissionStateMachine

    nav = MagicMock()
    nav.position.return_value = MagicMock(
        relative_altitude=10.0, latitude=1.0, longitude=1.0, heading_deg=0.0,
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
    sm._locked_color, sm._locked_shape = color, shape
    sm.data.pending_detection = _D(color, shape, 0.85)
    sm._last_align_cmd_t = 0.0
    sm._last_align_status_log_t = time.monotonic()
    sm._no_detection_count = 0
    sm._aligned_frame_count = 0
    sm.position_controller = MagicMock()
    sm.position_controller.align_detection.return_value = False
    sm.target_detector = MagicMock()
    return sm, nav


# ── 1) SEARCH_BLUE: flicker sırasında RED asla aktif hedef olamaz ──────────── #
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


def test_blue_detection_flicker_does_not_activate_red():
    """SEARCH_BLUE: BLUE detection sequence True,True,False,True,False,False,
    True,True,False,True boyunca (hiçbir zaman 3 ardışık yok → stabilize olamaz):
      • mission_state SEARCHING_FIRST_TARGET olarak kalır (SEARCH_RED'e geçmez)
      • aktif hedef (pending/lock) ASLA RED olmaz
    BLUE yokken görünen RED üçgen yalnızca YOKSAYILIR."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.8)
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.9)
    sequence = [True, True, False, True, False, False, True, True, False, True]

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None, [], stable=None)
    sm._advance_search = MagicMock()
    sm.target_detector.stable_detection.side_effect = _fake_stable_detection(3)

    for present in sequence:
        sm.target_detector.detect.return_value = (
            [d_blue] if present else [d_red]   # BLUE yokken kırmızı görünüyor
        )
        sm._do_search_target()

        assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET, \
            "BLUE flicker sırasında state SEARCH_BLUE'da kalmalı (SEARCH_RED'e geçiş yok)"
        pd = sm.data.pending_detection
        if pd is not None:
            assert pd.color == TargetColor.BLUE and pd.shape == "HEXAGON", \
                "aktif hedef yalnızca BLUE HEXAGON olabilir"
        assert sm._locked_color in (None, TargetColor.BLUE), \
            "kilit asla RED olamaz"


def test_blue_stable_detection_still_triggers_mission():
    """Kontrol: BLUE gerçekten 3 ardışık frame kararlı görülürse mission NORMAL
    şekilde TARGET_DETECTED'e geçer (flicker düzeltmesi BLUE aramasını bozmaz)."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.8)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None, [], stable=None)
    sm._advance_search = MagicMock()
    sm.target_detector.stable_detection.side_effect = _fake_stable_detection(3)
    sm.target_detector.detect.return_value = [d_blue]

    for _ in range(3):   # 3 ardışık BLUE frame
        sm._do_search_target()

    assert sm.data.state == MissionState.TARGET_DETECTED, \
        "kararlı BLUE detection görevi tetiklemeli (mevcut davranış korunur)"
    assert sm.data.pending_detection.color == TargetColor.BLUE


# ── 2) ALIGN (BLUE): flicker'da RED'e yönelme YOK — gerçek bug ─────────────── #
def test_blue_align_flicker_does_not_switch_active_target_to_red():
    """ALIGN (BLUE kilitli): BLUE flicker yapıp görüntüde yalnızca RED varsa:
      • pending_detection ASLA RED olmaz (BLUE olarak kalır)
      • yabancı türe yönelim komutu gönderilmez (_no_detection_count artar)
    (Bu, _filter_active_detections + _pick_locked_detection'ın RED'i
    sızdırdığı gerçek bug senaryosudur.)"""
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.9, cx=800.0, cy=300.0)

    sm, _nav = _align_sm()
    sm.target_detector.detect.return_value = [d_red]   # BLUE flicker — yalnızca kırmızı

    sm._do_align_target()

    pd = sm.data.pending_detection
    assert pd is not None and pd.color == TargetColor.BLUE and pd.shape == "HEXAGON", \
        "pending_detection BLUE'da kalmalı — RED'e değişmemeli"
    assert sm._no_detection_count == 1, \
        "kilitli tür yoksa 'tespit yok' sayacı artmalı (RED'e yönelim yok)"


def test_blue_align_ignores_red_even_in_mixed_frame():
    """ALIGN (BLUE kilitli): BLUE + RED AYNI karedeyken bile aktif hedef BLUE'dur."""
    d_blue = _D(TargetColor.BLUE, "HEXAGON", 0.8, cx=900.0, cy=340.0)
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.95, cx=700.0, cy=280.0)

    sm, _nav = _align_sm()
    sm.target_detector.detect.return_value = [d_red, d_blue]   # ikisi de görünüyor

    sm._do_align_target()

    pd = sm.data.pending_detection
    assert pd is not None and pd.color == TargetColor.BLUE, \
        "karışık karede aktif hedef BLUE olmalı — RED'e atlanamaz"
    assert sm._no_detection_count == 0


# ── 3) RED yalnızca BLUE tamamlandıktan SONRA aktifleşebilir ───────────────── #
def test_red_activates_only_after_blue_completion():
    """BLUE gerçekten tamamlandıktan (first_target=BLUE HEXAGON +
    payload_released) sonra SEARCHING_SECOND_TARGET'te RED TRIANGLE normal
    şekilde aday olur ve görevi tetikler (BLUE→RED geçişi korunur)."""
    first = Target(target_id=1, shape=TargetType.HEXAGON, color=TargetColor.BLUE,
                   latitude=1.0, longitude=1.0, confidence=0.9, payload_released=True)
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_SECOND_TARGET, first,
                          [d_red], stable=d_red)

    sm._do_search_target()

    args = sm.target_detector.stable_detection.call_args[0]
    assert len(args[0]) == 1 and args[0][0] is d_red, \
        "BLUE tamamlandıktan sonra RED üçgen aday olabilmeli"
    assert sm.data.state == MissionState.TARGET_DETECTED


def test_blue_not_detected_alone_never_means_completion():
    """'BLUE detected == False' BLUE'un TAMAMLANDIĞI anlamına gelmez: BLUE henüz
    bulunamamışken RED görünse bile state SEARCH_BLUE'da kalır, TARGET_DETECTED'e
    geçilmez (yalnızca kararlı BLUE tespiti görevi tetikler)."""
    d_red = _D(TargetColor.RED, "TRIANGLE", 0.9)

    sm, _nav = _search_sm(MissionState.SEARCHING_FIRST_TARGET, None,
                          [d_red], stable=None)
    sm._advance_search = MagicMock()

    for _ in range(5):   # RED sürekli görünüyor, BLUE hiç yok
        sm._do_search_target()

    assert sm.data.state == MissionState.SEARCHING_FIRST_TARGET, \
        "BLUE yokluğu BLUE tamamlanması değildir — RED görevi tetikleyemez"
